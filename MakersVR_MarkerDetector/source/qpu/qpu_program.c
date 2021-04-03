#include <stdio.h>
#include <string.h> // memset

#include "qpu_program.h"
#include "qpu_info.h"

/* QPU Program
	Simple interface for a General purpose program

	After creation, you need to lock the progmem buffer and fill in the progmem code and uniforms
	Currently, exeuting on multiple GPUs will execute them all with the same uniforms
*/

/* Initializes QPU program using base. Provide progmem size requirements through memsize.
 * If program is a general purpose program, messageSize should be > 2 to accommodate for code and uniforms.
 * Else messageSize should be 0 as QPU V3D registers are used for execution instead of a mailbox message. */
int qpu_initProgram(QPU_PROGRAM *program, QPU_BASE *base, QPU_PROGMEM progmem)
{
	// Allocate GPU memory for progmem (code, uniforms, control lists, etc)
	uint32_t progmemSize = progmem.codeSize + progmem.uniformsSize * 4 + progmem.messageSize * 4;
	int status = qpu_allocBuffer(&program->progmem_buffer, base, progmemSize, 4096);
	if (status != 0) return status;
	// Although not currently locked, addresses are already valid and will stay the same

	// Store ARM and VC side addresses
	QPU_PTR pmstart = program->progmem.start = program->progmem_buffer.ptr;
	program->progmem.code = pmstart;
	program->progmem.codeSize = progmem.codeSize;
	program->progmem.uniforms.vc = pmstart.vc + progmem.codeSize;
	program->progmem.uniforms.arm.vptr = pmstart.arm.vptr + progmem.codeSize;
	program->progmem.uniformsSize = progmem.uniformsSize;
	program->progmem.message.vc = pmstart.vc + progmem.codeSize + progmem.uniformsSize*4;
	program->progmem.message.arm.vptr = pmstart.arm.vptr + progmem.codeSize + progmem.uniformsSize*4;
	program->progmem.messageSize = progmem.messageSize;

	// Lock progmem buffer to make base->progmem accessible
	qpu_lockBuffer(&program->progmem_buffer);

	// Reset progmem to all zeros
	memset(program->progmem.start.arm.vptr, 0x0, program->progmem_buffer.size);

	if (progmem.messageSize > 1)
	{ // Setup messages
		program->progmem.message.arm.uptr[0] = program->progmem.uniforms.vc;
		program->progmem.message.arm.uptr[1] = program->progmem.code.vc;
	}

	// Unlock progmem - base->progmem can't be accessed anymore
	qpu_unlockBuffer(&program->progmem_buffer);

	return 1;
}

/* Destroy QPU program and clean up resources */
void qpu_destroyProgram (QPU_PROGRAM *program)
{
	// Destroy progmem in GPU memory
	qpu_releaseBuffer(&program->progmem_buffer);
}

/* QPU execute program using direct access to QPU V3D registers. */
int qpu_executeProgramDirect (QPU_PROGRAM *program, QPU_BASE *base, int numInst, int unifLength, int unifStride, QPU_PerformanceState *perfState)
{
	base->peripherals[V3D_DBCFG] = 0; // Disallow IRQ
	base->peripherals[V3D_DBQITE] = 0; // Disable IRQ
	base->peripherals[V3D_DBQITC] = -1; // Resets IRQ flags

	// Clear caches - L2, TMU, uniforms, instructions
	base->peripherals[V3D_L2CACTL] = (1<<2); // Clear L2 cache
	base->peripherals[V3D_SLCACTL] = 0b1111<<24 | 0b1111<<16 | 0b1111<<8 | 0b1111<<0;

	// Note QPU user program numbers to determine when all our instances finished
	int qpuQueued = (base->peripherals[V3D_SRQCS] & 0b111111);
	int qpuFinished = (base->peripherals[V3D_SRQCS] >> 16) & 0xFF;
	int qpuWaitCount = (qpuQueued + qpuFinished + numInst) % 256;
	//base->peripheral[V3D_SRQCS] = (1<<7) | (1<<8) | (1<<16); // Reset error bit and counts

//	if (qpuWaitCount < qpuFinished)
//		printf("QPU executing %d programs; waiting for %d with %d queued and %d already finished! \n", numInst, qpuWaitCount, qpuQueued, qpuFinished);

	for (int q = 0; q < numInst; q++)
	{
		// Maximum number of queued requests reached - wait until queue has space
		long long cnt = 0;
		while((base->peripherals[V3D_SRQCS] & 0b111111) == 16)
		{
			cnt++;
			if (cnt % 10000 == 0)
			{
				qpu_logErrors(base);
//				qpu_logStalls(base);
			}
			if (cnt % 10000 == 0 && perfState != NULL)
				qpu_updatePerformance(base, perfState);
			if (cnt % 100000 == 0)
			{
				printf("QPU stalled - queued %d / %d! \n", q, numInst);
				if (perfState != NULL) qpu_logPerformance(perfState);
				return -1;
			}
		}
		// Queue new program instance
		base->peripherals[V3D_SRQUA] = program->progmem.uniforms.vc + q * unifStride * 4;
		base->peripherals[V3D_SRQUL] = unifLength;
		base->peripherals[V3D_SRQPC] = program->progmem.code.vc;
	}

	// Wait for all instances to complete
	long long cnt = 0;
	int executed = 0;
	while ((executed = (base->peripherals[V3D_SRQCS] >> 16) & 0xFF) != qpuWaitCount)
	{
		cnt++;
		if (cnt % 100000 == 0)
		{
			qpu_logErrors(base);
//			qpu_logStalls(base);
		}
		if (cnt % 100000 == 0 && perfState != NULL)
			qpu_updatePerformance(base, perfState);
		if (cnt % 1000000 == 0)
		{
			printf("QPU stalled - waiting to execute %d / %d! \n", executed > qpuWaitCount? (qpuWaitCount + 256 - executed) : (qpuWaitCount - executed), numInst);
			if (perfState != NULL) qpu_logPerformance(perfState);
			return -1;
		}
	}

	return 0;
}

/* QPU execute general purpose program using mailbox. */
int qpu_executeProgramMailbox (QPU_PROGRAM *program, QPU_BASE *base, int numQPUs)
{
	return execute_qpu(base->mb, numQPUs, program->progmem.message.vc, QPU_NO_FLUSH, QPU_TIMEOUT);
}

int qpu_executeProgram (QPU_PROGRAM *program, QPU_BASE *base, int numQPUs)
{
	int retCode;
	if (program->progmem.messageSize < 2)
		retCode = qpu_executeProgramDirect(program, base, numQPUs, program->progmem.uniformsSize, 0, NULL);
	else
		retCode = qpu_executeProgramMailbox(program, base, numQPUs);

	return retCode;
}

/* Copies code from supplied buffer into program memory. */
void qpu_setProgramCode(QPU_PROGRAM *program, const char *code, unsigned int length)
{
	qpu_lockBuffer(&program->progmem_buffer);
	memcpy(&program->progmem.code.arm.cptr, code, length);
	qpu_unlockBuffer(&program->progmem_buffer);
}

/* Loads code from the file directly into program memory. */
int qpu_loadProgramCode(QPU_PROGRAM *program, const char *filename)
{
	FILE *file = fopen(filename, "rb");
	fseek(file, 0, SEEK_END);
	unsigned int fileLength = ftell(file);
	if (fileLength > program->progmem.codeSize)
	{
		printf("Unable to fit %d bytes of code into the %d bytes of code buffer!", fileLength, program->progmem.codeSize);
		fclose(file);
		return -1;
	}
	rewind(file);

	qpu_lockBuffer(&program->progmem_buffer);
	fread(program->progmem.code.arm.cptr, fileLength, 1, file);
	qpu_unlockBuffer(&program->progmem_buffer);
	fclose(file);
	return 0;
}

/* Returns file size of specified code file. Can be used to fit progmem code size perfectly. */
unsigned int qpu_getCodeSize(const char *filename)
{
	FILE *file = fopen(filename, "rb");
	fseek(file, 0, SEEK_END);
	unsigned int fileLength = ftell(file);
	fclose(file);
	return fileLength;
}
