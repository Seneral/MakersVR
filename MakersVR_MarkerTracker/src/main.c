/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "main.h"

#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_dma.h"
//#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_exti.h"

#include <string.h>
#include <stdint.h>
//#include <stdbool.h>

#include "usbd.h"

#define UART_BAUD_RATE			115200	// 57600
#define UART_TX_TIMEOUT			(10 * 72000000 / UART_BAUD_RATE)	// Wait 10 bauds to be sure
#define UART_RX_BUFFER_SIZE		512
#define UART_RX_BUFFER_SPACE	16									// Space for usb header to be prepended
#define UART_PING_INTERVAL		1000								// Interval in which marker detectors are pinged
#define UART_COMM_TIMEOUT		5000								// Comm timeout at which connection is reset
#define USB_WRITE_BUFFER_TIME	10									// Estimate of how many us we need to write USB buffer


/* Structures */

// UART State
enum CommState {
	CommNoCon = 0,
	CommID = 1,
	CommACK = 2,
	CommReady = CommID | CommACK,
	Stall = CommReady | 4
};
struct DataPacket {
	bool unhandled;
	uint8_t *data;
	uint_fast16_t len;
};
struct PortState {
	enum CommState commState;
	struct DataPacket data;
	TimePoint lastComm;
	uint_fast16_t cmd;
};


/* Function Prototypes */

// Setup function prototypes
static void LL_Init(void);
static void SystemClock_Config(void);
static void GPIO_Init(void);
static void UART_Init(void);
static void EXTI_Init(void);

// Utility
static inline void toggle() { GPIOC->ODR ^= LL_GPIO_PIN_13; }
static inline void blink() { toggle(); LL_mDelay(100); toggle(); }

// USB
static usbd_respond usbd_control_respond(usbd_device *usbd, usbd_ctlreq *req);
static usbd_respond usbd_control_receive(usbd_device *usbd, usbd_ctlreq *req);

// UART
static void UART_Send(uint_fast8_t port, const uint8_t* data, uint_fast8_t len);
static inline void UART_HoldPacket(uint_fast8_t port);
static inline void UART_ReleasePacket(uint_fast8_t port);
static inline void UART_NextPacket(uint_fast8_t port);
static inline void UART_NextPackets();
static inline void UART_ResetPort(uint_fast8_t port);


/* Variables */

volatile uint32_t msCounter = 0;
static volatile bool isStreaming;
static uint8_t sendBuffer[64];
TimePoint startup = { 0, 0 };
TimePoint lastPing = { 0, 0 };

// Debug
#ifdef DEBUG
volatile uint_fast16_t debugPos = 0;
volatile uint_fast16_t debugOffset = 0;
uint8_t debugBuffer[DEBUG_BUFFER_SIZE];
#endif

// Constants
const uint_fast8_t usbHeaderLength = 8;

// UART State
static struct PortState portStates[4]; // First one is invalid, just so we can index with UART 1,2,3
// UART Buffers
static uint8_t UART1_RX_DMA_Buffer[UART_RX_BUFFER_SPACE+UART_RX_BUFFER_SIZE];
static uint8_t UART2_RX_DMA_Buffer[UART_RX_BUFFER_SPACE+UART_RX_BUFFER_SIZE];
static uint8_t UART3_RX_DMA_Buffer[UART_RX_BUFFER_SPACE+UART_RX_BUFFER_SIZE];
// Hardware mapping
static USART_TypeDef * const UART[] = { NULL, USART1, USART2, USART3 };
static const uint32_t DMA_Channels[] = { 0, LL_DMA_CHANNEL_5, LL_DMA_CHANNEL_6, LL_DMA_CHANNEL_3 };
// Fixed UART Messages
static const uint8_t msg_id_opp[] = u8"MakersVR_MarkerDetector";
static const uint8_t msg_id_own_full[] = u8"#I16MakersVR_MarkerTracker"; // Assert length of payload in hex
static const uint8_t msg_ack[] = u8"#ACK";
static const uint8_t msg_nak[] = u8"#NAK";
static const uint8_t msg_start[] = u8"#+00";
static const uint8_t msg_stop[] = u8"#-00";
static const uint8_t msg_ping[] = u8"#P00";

/* Functions */

int main(void)
{
	// Setup clocks and interrupts (e.g. systick)
	LL_Init();
	SystemClock_Config();

	// Initialize GPIO
	GPIO_Init();
	EXTI_Init();

	// Init state
	memset(portStates, 0, sizeof(portStates));

	// Init UART
	UART_Init();

	// Init libusb_stm32 USB device stack
	usbd_impl_init((usbd_callbacks){ usbd_control_respond, usbd_control_receive });

	// startup sequence
	SetTimePoint(&startup);
	DEBUG_STR("/S");
	GPIOC->ODR |= LL_GPIO_PIN_13;
	delayUS(10000); // 10ms
	GPIOC->ODR &= ~LL_GPIO_PIN_13;
	
	// Send out Identification over UART
	UART_Send(1, msg_id_own_full, sizeof(msg_id_own_full)-1);
	UART_Send(2, msg_id_own_full, sizeof(msg_id_own_full)-1);
	UART_Send(3, msg_id_own_full, sizeof(msg_id_own_full)-1);

	TimePoint cur;
	int iteration = 0;
	while (1)
	{
		SetTimePoint(&cur);

		if (isStreaming)
		{ // Send packets

			// Determine time in USB frame
			uint_fast16_t frameTime = (cur.us + 1000 - lastSOF.us) % 1000;
			struct USBPortState *usbPort = usbd_alt_interface? &isoPort : &intPort[0];
			uint_fast16_t portTime = (frameTime + 1000 - usbPort->framePos)%1000;

			for (uint_fast8_t i = 1; i < 4; i++)
			{
				if ((portStates[i].commState & CommReady) == CommReady && portStates[i].data.unhandled)
				{ // Send unhandled UART packet out to USB
					// TODO: Fix freezes that seem to occur here, until then, just drop frames that couldn't be send off immediately 
					if (false && portTime < 1000-USB_WRITE_BUFFER_TIME*i && usbPort->bufferPos == 0 && usbPort->bufferSz > portStates[i].data.len+usbHeaderLength)
					{ // Port is ready to send, with time left to write buffer
						//DEBUG_CHARR('/', hex[portStates[i].data.len>>4], hex[portStates[i].data.len&15], ':', '0' + portTime/100, '0' + (portTime % 100) / 10, '0' + portTime%10);
						// Overwrite now unused UART header with USB header
						uint8_t *packet = portStates[i].data.data - usbHeaderLength;
						uint8_t *hdrPos = packet;
						*(hdrPos++) = '0' + i-1;
						*(hdrPos++) = 'B';
						*(hdrPos++) = hex[portStates[i].data.len>>4];
						*(hdrPos++) = hex[portStates[i].data.len&15];
						*(hdrPos++) = ':';
						*(hdrPos++) = '0' + portTime / 100;
						*(hdrPos++) = '0' + (portTime % 100) / 10;
						*(hdrPos++) = '0' + portTime % 10;
						// Send usb packet
						usbd_impl_send(packet, portStates[i].data.len + usbHeaderLength);
						// Release UART packet to wait for the next one
						UART_ReleasePacket(i);
						// Mark packet as handled
						portStates[i].data.len = 0;
						portStates[i].data.unhandled = false;
					}
				}
			}
		}

		// Occasionally check timeouts
		if (iteration++ % 1000 != 0) continue;

		// Check UART timeouts
		for (uint_fast8_t i = 1; i < 4; i++)
		{
			if (portStates[i].commState == CommNoCon) continue;
			TimeSpan timeSinceLastComm = GetTimeSpanMS(&cur, &portStates[i].lastComm);
			if (timeSinceLastComm > UART_COMM_TIMEOUT)
			{ // Reset Comm after silence
				UART_Send(i, msg_nak, sizeof(msg_nak)-1);
				UART_ResetPort(i);
				DEBUG_CHARR('/', '0'+i, 'T', 'M', 'O');
			}
		}

		// Check ping
		TimeSpan timeSinceLastPing = GetTimeSpanMS(&cur, &lastPing);
		if (timeSinceLastPing > UART_PING_INTERVAL)
		{ // Send ping occasionally (expecting a response)
			for (uint_fast8_t i = 1; i < 4; i ++)
			{
				if ((portStates[i].commState & CommReady) == CommReady)
					UART_Send(i, msg_ping, sizeof(msg_ping)-1);
			}
			lastPing = cur;
		}
	}
}


/* ------ USB Behaviour ------ */

/* Request from host expecting response on control endpoint (vendor-defined on the single interface) */
static usbd_respond usbd_control_respond(usbd_device *usbd, usbd_ctlreq *req)
{ // Check req->bRequest, req->wIndex and req->wValue for handling
	if (req->bRequest == 0x00)
	{ // Loopback test
		for (int i = 0; i < req->wLength; i++) req->data[i] = ~req->data[i];
		usbd->status.data_ptr = (void*)&req->data;
		usbd->status.data_count = req->wLength;
		if (req->wLength > 8) GPIOC->ODR &= ~LL_GPIO_PIN_13;
		if (req->wLength > 8 && req->data[1] > 0) GPIOC->ODR |= LL_GPIO_PIN_13;
		return usbd_ack;
	}
	if (req->bRequest == 0x01)
	{ // Request for debug data
#ifdef DEBUG
		if (debugOffset > 0)
		{ // Move debug buffer ahead to position 0
			memmove(debugBuffer, debugBuffer+debugOffset, debugPos-debugOffset);
			debugPos = debugPos-debugOffset;
			debugOffset = 0;
		}
		usbd->status.data_ptr = (void*)&debugBuffer;
		if (debugPos > req->wLength)
		{ // Only send the first set of bytes and set debugOffset to fix hole later
			usbd->status.data_count = req->wLength;
			debugOffset = req->wLength;
		}
		else
		{ // Else just send the whole buffer and reset
			usbd->status.data_count = debugPos;
			debugPos = 0;
		}
		return usbd_ack;
#endif
	}
	if (req->bRequest == 0x02)
	{ // Request to iterate connected Marker Detectors
		sendBuffer[0] = 3; // Maximum supported detectors
		sendBuffer[1] = portStates[1].commState;
		sendBuffer[2] = portStates[2].commState;
		sendBuffer[3] = portStates[3].commState;
		sendBuffer[4] = 0xFF;
		sendBuffer[5] = isStreaming;
		TimePoint cur;
		SetTimePoint(&cur);
		TimeSpan timeSinceStartupMS = GetTimeSpanUS(&cur, &startup) / 1000;
		sendBuffer[6] = timeSinceStartupMS / 1000; // Seconds
		sendBuffer[7] = (timeSinceStartupMS % 1000) * 1000 / 255; // Second in 1/255 steps
		usbd->status.data_ptr = (void*)&sendBuffer;
		usbd->status.data_count = 8;
		return usbd_ack;
	}
	return usbd_fail;
}

/* Request from host not expecting a response on control endpoint (vendor-defined on the single interface) */
static usbd_respond usbd_control_receive(usbd_device *usbd, usbd_ctlreq *req)
{ // Handle data in req->data of length req->wLength
	if (req->bRequest == 0x00)
	{ // Request to start streaming
		if (req->wLength >= 6)
		{ // Send included setup packet first
			sendBuffer[0] = '#';
			sendBuffer[1] = 'S';
			sendBuffer[2] = hex[req->wLength >> 4];
			sendBuffer[3] = hex[req->wLength & 0xF];
			memcpy(sendBuffer+4, req->data, req->wLength);
			if ((portStates[1].commState & CommReady) == CommReady)
				UART_Send(1, sendBuffer, 4+req->wLength);
			if ((portStates[2].commState & CommReady) == CommReady)
				UART_Send(2, sendBuffer, 4+req->wLength);
			if ((portStates[3].commState & CommReady) == CommReady)
				UART_Send(3, sendBuffer, 4+req->wLength);
			DEBUG_STR("/SETUP");
		}
		else
			DEBUG_STR("/NOSTP");
		// Then start streaming
		isStreaming = true;
		// Send connected Marker Detector start command
		for (uint_fast8_t i = 1; i < 4; i++)
		{
			if ((portStates[i].commState & CommReady) == CommReady)
				UART_Send(i, msg_start, sizeof(msg_start)-1);
		}
		DEBUG_STR("/STRON");
		return usbd_ack;
	}
	else if (req->bRequest == 0x01)
	{ // Request to stop streaming
		isStreaming = false;
		// Send connected Marker Detector stop command
		for (uint_fast8_t i = 1; i < 4; i ++)
		{
			if ((portStates[i].commState & CommReady) == CommReady)
				UART_Send(i, msg_stop, sizeof(msg_stop)-1);
		}
		DEBUG_STR("/STROFF");
		return usbd_ack;
	}
	else if (req->bRequest == 0x02)
	{ // Setup data (without start stream request)
		if (req->wLength >= 6)
		{
			sendBuffer[0] = '#';
			sendBuffer[1] = 'S';
			sendBuffer[2] = hex[req->wLength >> 4];
			sendBuffer[3] = hex[req->wLength & 0xF];
			memcpy(sendBuffer+4, req->data, req->wLength);
			if ((portStates[1].commState & CommReady) == CommReady)
				UART_Send(1, sendBuffer, 4+req->wLength);
			if ((portStates[2].commState & CommReady) == CommReady)
				UART_Send(2, sendBuffer, 4+req->wLength);
			if ((portStates[3].commState & CommReady) == CommReady)
				UART_Send(3, sendBuffer, 4+req->wLength);
			DEBUG_STR("/SETUP");
		}
		else
			DEBUG_STR("/NOSTP");
		
		return usbd_ack;
	}
	return usbd_ack;
}


/* ------ UART Behaviour ------ */

/** Process received data over UART */
static void UART_ProcessData(uint_fast8_t port, uint8_t* rcv, uint_fast16_t rcvSz)
{
	// Handle random zeros from unconnected UART ports
	if (rcvSz <= 5)
	{ // Count ratio of zeros
		uint_fast8_t zeros = 0;
		for (uint_fast8_t i = 0; i < rcvSz; i++)
			if (rcv[i] == 0) zeros++;
		if (zeros*3 >= rcvSz*2) return UART_NextPacket(port);
	}

	struct PortState *state = &portStates[port];
	SetTimePoint(&state->lastComm);
	while (true)
	{
		// Wait for next start of command
		while (state->cmd < rcvSz && rcv[state->cmd] != '#' && ++state->cmd < rcvSz);
		// Clear buffer if no command is left
		if (state->cmd >= rcvSz) return UART_NextPacket(port);
		// Assure at least packet header is received fully
		if (state->cmd+4 > rcvSz) return;

		uint_fast16_t cmd = state->cmd;
		if (rcv[cmd+1] == 'N' && rcv[cmd+2] == 'A' && rcv[cmd+3] == 'K')
		{ // NAK received
			DEBUG_CHARR('/', '0'+port, 'N', 'A', 'K');
			UART_ResetPort(port);
			return;
		}

		// Only relevant for packages, not ACK or NAK
		uint_fast8_t cmdID = rcv[cmd+1];
		uint_fast8_t a = rcv[cmd+2]-'0', b = rcv[cmd+3]-'0';
		a = a > 9? a-('A'-'9'+1) : a;
		b = b > 9? b-('A'-'9'+1) : b;
		uint_fast16_t dataCount = a*16 + b;
		uint_fast16_t dataStart = cmd+4;
		uint8_t *data = &rcv[dataStart];

		if ((state->commState & CommReady) != CommReady)
		{
			if (rcv[cmd+1] == 'A' && rcv[cmd+2] == 'C' && rcv[cmd+3] == 'K')
			{ // Received acknowledgement of own identity
				state->commState |= CommACK;
				state->cmd += 4;
				DEBUG_CHARR('/', '0'+port, 'A', 'C', 'K');
			}
			else if (cmdID == 'I' && !(state->commState & CommID) && (rcvSz - dataStart) <= dataCount)
			{ // Received proper identification, so send acknowledgment
				if (dataCount == sizeof(msg_id_opp)-1 && memcmp(data, msg_id_opp, dataCount) == 0)
				{ // Proper ID
					state->commState |= CommID;
					state->cmd += 4 + dataCount;
					DEBUG_CHARR('/', '0'+port, 'I', 'D', 'S');
					// Send acknowledgement and own identification if not yet acknowledged
					UART_Send(port, msg_ack, sizeof(msg_ack)-1);
					if (!(state->commState & CommACK))
						UART_Send(port, msg_id_own_full, sizeof(msg_id_own_full)-1);
				}
				else
				{ // Wrong ID, reset
					UART_Send(port, msg_nak, sizeof(msg_nak)-1);
					UART_ResetPort(port);
					DEBUG_CHARR('/', '0'+port, 'I', 'D', 'E');
					return;
				}
			}

			if ((state->commState & CommReady) == CommReady)
			{ // Finished comm setup, connection is ready
				DEBUG_CHARR('/', '0'+port, 'R', 'D', 'Y');
				UART_NextPacket(port);
				return;
			}
			return;
		}
		else
		{ // Received full command
			if (cmdID == 'B')
			{ // Identified marker packet

				// Make sure all data is there
				if (rcvSz - dataStart < dataCount) return;
				state->cmd += 4 + dataCount;

				// Calculate time left until next usb packet can be sent
				TimePoint cur;
				SetTimePoint(&cur);
				uint_fast16_t frameTime = (cur.us + 1000 - lastSOF.us) % 1000;
				struct USBPortState *usbPort = usbd_alt_interface? &isoPort : &intPort[0];
				uint_fast16_t portTime = (frameTime + 1000 - usbPort->framePos)%1000;

				//DEBUG_CHARR('/', hex[dataCount>>4], hex[dataCount&15], ':', '0' + portTime/100, '0' + (portTime % 100) / 10, '0' + portTime%10);

				if (portTime < 1000-USB_WRITE_BUFFER_TIME && usbPort->bufferPos == 0 && usbPort->bufferSz > dataCount+usbHeaderLength)
				{ // Plenty of time to write to buffer before ISO frame ends
					// Replace UART header with USB header
					uint_fast16_t header = dataStart-usbHeaderLength;
					uint_fast16_t pos = header;
					rcv[pos++] = '0' + port-1;
					rcv[pos++] = 'B';
					rcv[pos++] = hex[dataCount>>4];
					rcv[pos++] = hex[dataCount&15];
					rcv[pos++] = ':';
					rcv[pos++] = '0' + portTime / 100;
					rcv[pos++] = '0' + (portTime % 100) / 10;
					rcv[pos++] = '0' + portTime % 10;
					usbd_impl_send(&rcv[header], dataCount + usbHeaderLength);
					// Accept next UART packet
					UART_NextPacket(port);
				}
				else if (false)
				{ // Can't send immediately, schedule until after frame is over to send
					//DEBUG_CHARR('/', hex[dataCount>>4], hex[dataCount&15], ':', '0' + portTime/100, '0' + (portTime % 100) / 10, '0' + portTime%10);
					state->data.unhandled = true;
					state->data.data = data;
					state->data.len = dataCount;
					// Hold onto this UART packet
					UART_HoldPacket(port);
				}
				
				return;
			}
			if (cmdID == 'P')
			{ // Ping response
				// Nothing to do here, lastComm is already updated
				state->cmd += 4+dataCount;
			}
		}
	}
}




/* ------ INIT Behaviour ------ */

/** Low-level init */
static void LL_Init(void)
{
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	LL_GPIO_AF_DisableRemap_SWJ();

	NVIC_SetPriorityGrouping(0x00000003); // NVIC_PRIORITYGROUP_4

	// System interrupt init
	NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_SetPriority(BusFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_SetPriority(UsageFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_SetPriority(DebugMonitor_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
}

/** System Clock Configuration - 72MHz */
static void SystemClock_Config(void)
{
	// Configure flash latency (2 ticks required since 72MHz > 2*32MHz)
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

	// Configure HSE
	LL_RCC_HSE_Enable();
	while (!LL_RCC_HSE_IsReady());

	// Configure PLL to 72MHz
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
	LL_RCC_PLL_Enable();
	while (!LL_RCC_PLL_IsReady());

	// Set system clock
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1); // 72MHz
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2); // 36MHz
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1); // 72MHz
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

	// Set USB clock (72MHz / 1.5 = 48MHz)
	LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_PLL_DIV_1_5);

	// Configure systick
	LL_InitTick(72000000, 1000); // Period in ticks at which SysTick_Handler is called
	LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
	LL_SetSystemCoreClock(72000000);

	// Set TIM2 interrupts
	NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(TIM2_IRQn);

	// Initialise TIM2
	TIM2->CR1 &= ~TIM_CR1_CEN; // Disable Counter
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST; // Reset TIM2
	RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable internal clock for TIM2 (APB1 is at 72MHz)

	// Setup TIM2 for 1us counter	
	TIM2->PSC = 72-1; // Set Prescaler to 72 to get a 1MHz timer (1us interval)
	TIM2->ARR = 1000-1; // Set Auto-Reload to count up to 1000 to get a 1ms interval timer
	TIM2->CR1 |= TIM_CR1_ARPE; // Enable Auto-Reload Preload
	TIM2->EGR |= TIM_EGR_UG; // Trigger update event to load values
	
	// Start TIM2
	TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
	TIM2->CR1 |= TIM_CR1_CEN; // Enable Counter
	
	// Now TIM2_IRQHandler is called every 1ms, for us to update a 32Bit counter
	// Additionally, number of us within that 1ms frame can be read from TIM2->CNT
}
void TIM2_IRQHandler(void)
{
#if TIM_SR_UIF == 1 
	msCounter += TIM2->SR & TIM_SR_UIF;
#else
	if (TIM2->SR & TIM_SR_UIF)
		msCounter++;
#endif
	TIM2->SR = 0;
}

static void GPIO_Init(void)
{
	// Peripheral clock enable
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);

	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);

	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT_2MHz);
	LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_13, LL_GPIO_PULL_DOWN);
}

/** UART Initialization Function */
static void UART_Init(void)
{
	// We expect batches of up to around 300bytes on potentially all three UARTs at the same time, at a predictable, but high framerate of up to 90fps
	// Thus we use DMA to not waste CPU cycles, however not in circular mode since packet sizes are rather predictable, and normal mode is sufficient and easier
	// So, we use the IDLE interrupt to handle packets, once the full packet has arrived, disable DMA to prevent the new packet from being appended afterwards
	// Only after we process them we enable the DMA again to start loading in the next packet into the buffer from its beginning

	// TODO: Ditch DMA and just write bytes as they come into the apropriate USB buffer directly
	// Much lower latency, better distribution, less intermediate storage
	// And CPU is not used otherwise anyway, but DO keep the DMA version in case we ever do tracking on here!

	// Enable GPIO Clocks for all used GPIO Ports
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

	// DMA & UART Peripheral clock enable
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
	
	// Switch UART1 to alternative pin mapping on GPIOB
	LL_GPIO_AF_EnableRemap_USART1();

	struct UART_DMA_Setup {
		USART_TypeDef *usart;
		GPIO_TypeDef *GPIOx;
		uint32_t PinTX;
		uint32_t PinRX;
		uint32_t DMA_ch;
		uint8_t *rx_buffer;
		IRQn_Type irq;
	};
	const struct UART_DMA_Setup uarts[] = {
		{ // UART 1 (alternative configuration)
			.usart = USART1,
			.GPIOx = GPIOB,
			.PinTX = LL_GPIO_PIN_6,
			.PinRX = LL_GPIO_PIN_7,
			.DMA_ch = LL_DMA_CHANNEL_5,
			.rx_buffer = UART1_RX_DMA_Buffer+UART_RX_BUFFER_SPACE,
			.irq = USART1_IRQn
		},
		{ // UART 2
			.usart = USART2,
			.GPIOx = GPIOA,
			.PinTX = LL_GPIO_PIN_2,
			.PinRX = LL_GPIO_PIN_3,
			.DMA_ch = LL_DMA_CHANNEL_6,
			.rx_buffer = UART2_RX_DMA_Buffer+UART_RX_BUFFER_SPACE,
			.irq = USART2_IRQn
		},
		{ // UART 3
			.usart = USART3,
			.GPIOx = GPIOB,
			.PinTX = LL_GPIO_PIN_10,
			.PinRX = LL_GPIO_PIN_11,
			.DMA_ch = LL_DMA_CHANNEL_3,
			.rx_buffer = UART3_RX_DMA_Buffer+UART_RX_BUFFER_SPACE,
			.irq = USART3_IRQn
		}
	};

	// Common setup for all UARTs
	LL_GPIO_InitTypeDef GPIO_Pin_Init_TX;
	GPIO_Pin_Init_TX.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Pin_Init_TX.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_Pin_Init_TX.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_InitTypeDef GPIO_Pin_Init_RX;
	GPIO_Pin_Init_RX.Mode = LL_GPIO_MODE_FLOATING;
	LL_USART_InitTypeDef USART_InitStruct;
	USART_InitStruct.BaudRate = UART_BAUD_RATE;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;

	for (uint_fast8_t i = 0; i < sizeof(uarts)/sizeof(struct UART_DMA_Setup); i++)
	{
		struct UART_DMA_Setup u = uarts[i];

		// Init GPIO pins
		GPIO_Pin_Init_TX.Pin = u.PinTX;
		GPIO_Pin_Init_RX.Pin = u.PinRX;
		LL_GPIO_Init(u.GPIOx, &GPIO_Pin_Init_TX);
		LL_GPIO_Init(u.GPIOx, &GPIO_Pin_Init_RX);

		// Setup respective DMA Channel for UART RX
		LL_DMA_SetDataTransferDirection(DMA1, u.DMA_ch, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
		LL_DMA_SetChannelPriorityLevel(DMA1, u.DMA_ch, LL_DMA_PRIORITY_LOW);
		LL_DMA_SetMode(DMA1, u.DMA_ch, LL_DMA_MODE_NORMAL);
		LL_DMA_SetPeriphIncMode(DMA1, u.DMA_ch, LL_DMA_PERIPH_NOINCREMENT);
		LL_DMA_SetMemoryIncMode(DMA1, u.DMA_ch, LL_DMA_MEMORY_INCREMENT);
		LL_DMA_SetPeriphSize(DMA1, u.DMA_ch, LL_DMA_PDATAALIGN_BYTE);
		LL_DMA_SetMemorySize(DMA1, u.DMA_ch, LL_DMA_MDATAALIGN_BYTE);
		LL_DMA_SetPeriphAddress(DMA1, u.DMA_ch, (uint32_t)&u.usart->DR);
		LL_DMA_SetMemoryAddress(DMA1, u.DMA_ch, (uint32_t)u.rx_buffer);
		LL_DMA_SetDataLength(DMA1, u.DMA_ch, UART_RX_BUFFER_SIZE);

		// Apply configuration to UART
		LL_USART_Init(u.usart, &USART_InitStruct);
		LL_USART_ConfigAsyncMode(u.usart);
		LL_USART_EnableDMAReq_RX(u.usart);
		LL_USART_EnableIT_IDLE(u.usart);
	
		// Enable USART and DMA
		LL_DMA_EnableChannel(DMA1, u.DMA_ch);
		LL_USART_Enable(u.usart);
		
		// UART Interrupt (IDLE)
		NVIC_SetPriority(u.irq, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
		NVIC_EnableIRQ(u.irq);
	}
}

/** Send data over UART port */
static void UART_Send(uint_fast8_t port, const uint8_t* data, uint_fast8_t len)
{
	for (uint_fast8_t d = 0; d < len; d++)
	{
		LL_USART_TransmitData8(UART[port], data[d]);
		int timeout = 0;
		while (!LL_USART_IsActiveFlag_TXE(UART[port]) && ++timeout < UART_TX_TIMEOUT);
		if (timeout >= UART_TX_TIMEOUT)
		{
			DEBUG_CHARR('/', '0'+port, 'T', 'X', 'E');
			return UART_ResetPort(port);
		}
	}
	int timeout = 0;
	while (!LL_USART_IsActiveFlag_TC(UART[port]) && ++timeout < UART_TX_TIMEOUT);
	if (timeout >= UART_TX_TIMEOUT)
	{
		DEBUG_CHARR('/', '0'+port, 'T', 'C', 'E');
		return UART_ResetPort(port);
	}
}

/** Control functions to handle UART packet processing */
static inline void UART_HoldPacket(uint_fast8_t port)
{
	LL_DMA_DisableChannel(DMA1, DMA_Channels[port]);
	while (LL_DMA_IsEnabledChannel(DMA1, DMA_Channels[port]));
	LL_DMA_SetDataLength(DMA1, DMA_Channels[port], UART_RX_BUFFER_SIZE);
}
static inline void UART_ReleasePacket(uint_fast8_t port)
{
	portStates[port].cmd = 0;
	LL_DMA_EnableChannel(DMA1, DMA_Channels[port]);
}
static inline void UART_NextPacket(uint_fast8_t port)
{
	UART_HoldPacket(port);
	UART_ReleasePacket(port);
}
static inline void UART_NextPackets()
{
	UART_NextPacket(1);
	UART_NextPacket(2);
	UART_NextPacket(3);
}
static inline void UART_ResetPort(uint_fast8_t port)
{
	memset(&portStates[port], 0, sizeof(struct PortState));
	UART_NextPacket(port);
}

/** USART global interrupt handlers */
static inline void USART_IRQHandler(uint_fast8_t port, USART_TypeDef *usart, uint32_t DMA_Channel, uint8_t *rx_buffer)
{
	if (LL_USART_IsActiveFlag_IDLE(usart))
	{
		LL_USART_ClearFlag_IDLE(usart);
		UART_ProcessData(port, rx_buffer, UART_RX_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, DMA_Channel));
	}
}
void USART1_IRQHandler() { USART_IRQHandler(1, USART1, LL_DMA_CHANNEL_5, UART1_RX_DMA_Buffer+UART_RX_BUFFER_SPACE); }
void USART2_IRQHandler() { USART_IRQHandler(2, USART2, LL_DMA_CHANNEL_6, UART2_RX_DMA_Buffer+UART_RX_BUFFER_SPACE); }
void USART3_IRQHandler() { USART_IRQHandler(3, USART3, LL_DMA_CHANNEL_3, UART3_RX_DMA_Buffer+UART_RX_BUFFER_SPACE); }


/** EXTernal Interrupt Setup */

// TODO: Will be used for camera sync, not yet done

// PA0, PA1, PA4
static void EXTI_Init(void)
{
	// Setup input pins
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT_2MHz);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_13, LL_GPIO_PULL_DOWN);

	uint32_t linesMask = LL_EXTI_LINE_0 | LL_EXTI_LINE_1 | LL_EXTI_LINE_4;	
	EXTI->IMR |= linesMask; // Enable interrupt generation
	EXTI->RTSR |= linesMask; // Set to trigger on rising edge

	AFIO->EXTICR[0] &= ~(0x1111 << 0); // Select port A
	AFIO->EXTICR[0] &= ~(0x1111 << 4); // Select port A
	AFIO->EXTICR[1] &= ~(0x1111 << 0); // Select port A
}

void EXTI0_IRQHandler()
{
	if (EXTI->PR & LL_EXTI_LINE_0)
	{ // Interrupt pending
		EXTI->PR |= LL_EXTI_LINE_0; // Reset
	}
}
void EXTI1_IRQHandler()
{
	if (EXTI->PR & LL_EXTI_LINE_1)
	{ // Interrupt pending
		EXTI->PR |= LL_EXTI_LINE_1; // Reset
	}
}
void EXTI4_IRQHandler()
{
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
	if (EXTI->PR & LL_EXTI_LINE_4)
	{ // Interrupt pending
		EXTI->PR |= LL_EXTI_LINE_4; // Reset
	}
}





/** This function handles Non maskable interrupt. */
void NMI_Handler(void){}

/** This function handles Hard fault interrupt. */
void HardFault_Handler(void)
{
	DEBUG_STR("/HDF");
	while (1);
}

/** This function handles Memory management fault. */
void MemManage_Handler(void)
{
	DEBUG_STR("/MEM");
	while (1);
}

/** This function handles Prefetch fault, memory access fault. */
void BusFault_Handler(void)
{
	DEBUG_STR("/BUS");
	while (1);
}

/** This function handles Undefined instruction or illegal state. */
void UsageFault_Handler(void)
{
	DEBUG_STR("/USG");
	while (1);
}

/** This function handles System service call via SWI instruction. */
void SVC_Handler(void){}

/** This function handles Debug monitor. */
void DebugMon_Handler(void){}

/** This function handles Pendable request for system service. */
void PendSV_Handler(void){}

/** This function handles System tick timer. */
void SysTick_Handler(void){}