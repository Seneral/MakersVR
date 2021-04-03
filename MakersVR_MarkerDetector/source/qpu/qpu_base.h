#ifndef QPU_BASE_H
#define QPU_BASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>

#include "mailbox.h"
#include "qpu_registers.h"

/* QPU Access Base
	Provides easy access to all functionality of the QPU.
	
	Create a QPU access base using QPU_initBase and QPU_destroyBase.
	It provides direct access to the QPU V3D registers using the peripheral map.
	It also contains required information about the host.
	
	Contains a GPU buffer implementation for passing information buffers to the QPU.
	
	Requires sudo privileges for memmap (mailbox.h)
*/

#define BUS_TO_PHYS(x) ((x)&~0xC0000000)

#define QPU_USE_VC4_L2_CACHE	1				// Use L2 Cache or not
#define QPU_NO_FLUSH			1
#define QPU_TIMEOUT				2000 // ms

/* Dual pointer representation in VideoCore and ARM space */
typedef struct QPU_PTR {
	uint32_t vc;
	union {
		void *vptr;
		char *cptr;
		float *fptr;
		uint32_t *uptr;
	} arm;
} QPU_PTR;

/* Host information about QPU which can vary between RaspberryPi models */
typedef struct QPU_HOST {
	uint32_t mem_flg;
	uint32_t mem_map;
	uint32_t peri_addr;
	uint32_t peri_size;
} QPU_HOST;

/* Generic buffer in GPU memory, allocated through mailbox */
typedef struct QPU_BUFFER {
	uint32_t mb; // mailbox handle
	uint32_t handle;
	uint32_t size;
	QPU_PTR ptr;
} QPU_BUFFER;

/* QPU interface containing host information and direct QPU access through peripheral registers */
typedef struct QPU_BASE {
	QPU_HOST host;
	volatile uint32_t *peripherals; // Registers of peripherals for direct QPU register access
	uint32_t mb;
} QPU_BASE;

/* Get board-specific information about the QPU host integration */
int qpu_getHostInformation(QPU_HOST *host);

/* Allocate the buffer of desired size in GPU memory */
int qpu_allocBuffer(QPU_BUFFER *buffer, QPU_BASE *base, uint32_t size, uint32_t align);
/* Lock buffer to make buffer->ptr->arm.*ptr accessible */
void qpu_lockBuffer(QPU_BUFFER *buffer);
/* Unlock buffer to make buffer->ptr->arm.*ptr inaccessible */
void qpu_unlockBuffer(QPU_BUFFER *buffer);
/* Unmap buffer from ARM side and release buffer in GPU memory */
void qpu_releaseBuffer(QPU_BUFFER *buffer);

/* Initializes QPU access base */
int qpu_initBase(QPU_BASE *base);
/* Destroy QPU access base and clean up resources */
void qpu_destroyBase (QPU_BASE *base);


#ifdef __cplusplus
}
#endif

#endif
