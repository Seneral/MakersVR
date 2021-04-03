#include <dlfcn.h>

#include "qpu_base.h"

/* QPU Access Base
	Provides easy access to all functionality of the QPU.

	Create a QPU access base using QPU_initBase and QPU_destroyBase.
	It provides direct access to the QPU V3D registers using the peripheral map.
	It also contains required information about the host.

	Contains a GPU buffer implementation for passing information buffers to the QPU.

	Requires sudo privileges for memmap (mailbox.h)
*/

/* Allocate the buffer of desired size in GPU memory */
int qpu_allocBuffer(QPU_BUFFER *buffer, QPU_BASE *base, uint32_t size, uint32_t align)
{
	buffer->mb = base->mb;
	buffer->size = size;

	// Allocate GPU memory
	buffer->handle = mem_alloc(buffer->mb, buffer->size, align, base->host.mem_flg);
	if (!buffer->handle) return -1;

	// Get VC adress - stays constant over lifetime
	buffer->ptr.vc = mem_lock(buffer->mb, buffer->handle);
	mem_unlock(buffer->mb, buffer->handle);

	// Map into ARM space and get ARM adress - stays constant over lifetime
	buffer->ptr.arm.vptr = mapmem(BUS_TO_PHYS(buffer->ptr.vc + base->host.mem_map), buffer->size);
	if (!buffer->ptr.arm.vptr)
	{ // Release GPU buffer
		mem_free(buffer->mb, buffer->handle);
		return -2;
	}

	return 0;
}

/* Lock buffer to make buffer->ptr->arm.*ptr accessible */
void qpu_lockBuffer(QPU_BUFFER *buffer)
{
	//buffer->ptr.vc == mem_lock(buffer->mb, buffer->handle));
	mem_lock(buffer->mb, buffer->handle);
}

/* Unlock buffer to make buffer->ptr->arm.*ptr inaccessible */
void qpu_unlockBuffer(QPU_BUFFER *buffer)
{
	mem_unlock(buffer->mb, buffer->handle);
}

/* Unmap buffer from ARM side and release buffer in GPU memory */
void qpu_releaseBuffer(QPU_BUFFER *buffer)
{
	// Unmap ARM memory
	unmapmem(buffer->ptr.arm.vptr, buffer->size);
	// Free GPU memory
	mem_free(buffer->mb, buffer->handle);
}


/* Initializes QPU access base */
int qpu_initBase(QPU_BASE *base)
{
	// Get host information (peripheral adresses, etc)
	if (qpu_getHostInformation(&base->host)) return -1;

	// Map peripheral registers for direct QPU access
	base->peripherals = (volatile uint32_t *)mapmem(base->host.peri_addr, base->host.peri_size);
	if (!base->peripherals) return -2;

	// Open mailbox for GPU memory allocation
	base->mb = mbox_open();
	if (base->mb <= 0)
	{
		unmapmem((void *)base->peripherals, base->host.peri_size);
		return -3;
	}
	return 0;
}

/* Destroy QPU access base and clean up resources */
void qpu_destroyBase (QPU_BASE *base)
{
	// Unmap peripheral registers
    unmapmem((void*)base->peripherals, base->host.peri_size);
}


/* Get board-specific information about the QPU host integration */
int qpu_getHostInformation(QPU_HOST *host)
{
	/*
	BCM2835 "GPU_FFT" release 3.0
	Copyright (c) 2015, Andrew Holme.
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
		* Redistributions of source code must retain the above copyright
		notice, this list of conditions and the following disclaimer.
		* Redistributions in binary form must reproduce the above copyright
		notice, this list of conditions and the following disclaimer in the
		documentation and/or other materials provided with the distribution.
		* Neither the name of the copyright holder nor the
		names of its contributors may be used to endorse or promote products
		derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
	*/

	// Pi 1 defaults
	host->peri_addr = 0x20000000;
	host->peri_size = 0x01000000;
	host->mem_flg = QPU_USE_VC4_L2_CACHE? 0xC : 0x4;
	host->mem_map = QPU_USE_VC4_L2_CACHE? 0x0 : 0x20000000; // Pi 1 only

	// Load bcmhost lib at runtime to allow distributing compiled program on multiple pis
	void *handle = dlopen("libbcm_host.so", RTLD_LAZY);
	if (!handle) return -1;

	// Load relevant function
	unsigned (*bcm_host_get_sdram_address)     (void);
	unsigned (*bcm_host_get_peripheral_address)(void);
	unsigned (*bcm_host_get_peripheral_size)   (void);
	*(void **) (&bcm_host_get_sdram_address)      = dlsym(handle, "bcm_host_get_sdram_address");
	*(void **) (&bcm_host_get_peripheral_address) = dlsym(handle, "bcm_host_get_peripheral_address");
	*(void **) (&bcm_host_get_peripheral_size)    = dlsym(handle, "bcm_host_get_peripheral_size");

	if (bcm_host_get_sdram_address && bcm_host_get_sdram_address() != 0x40000000)
	{ // Pi 2?
		host->mem_flg = 0x4; // ARM cannot see VC4 L2 on Pi 2
		host->mem_map = 0x0;
	}

	// Fetch QPU host information
	if (bcm_host_get_peripheral_address)
	{
		host->peri_addr = bcm_host_get_peripheral_address();
	}
	if (bcm_host_get_peripheral_size)
	{
		host->peri_size = bcm_host_get_peripheral_size();
	}

	dlclose(handle);
	return 0;
}
