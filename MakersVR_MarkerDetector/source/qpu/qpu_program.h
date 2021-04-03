#ifndef QPU_PROGRAM_H
#define QPU_PROGRAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "qpu_base.h"
#include "qpu_info.h"

/* QPU Program
	Simple interface for a General purpose program

	After creation, you need to lock the progmem buffer and fill in the progmem code and uniforms
	Currently, exeuting on multiple GPUs will execute them all with the same uniforms
*/

/* Program memory representation as stored in GPU memory (using QPU_BUFFER).
 * Usually mapped to ARM space for access.
 * Stores all stuff the QPU needs to know in order to execute our QPU program. */
typedef struct QPU_PROGMEM {
    QPU_PTR start; // Start of progmem
    QPU_PTR code;
	uint32_t codeSize;
    QPU_PTR uniforms;
	uint32_t uniformsSize;
    QPU_PTR message; // Instruction messages send to the QPU (later control lists?)
	uint32_t messageSize;
} QPU_PROGMEM;

/* QPU interface containing information, our buffers and program memory as well as direct QPU access through peripheral registers */
typedef struct QPU_PROGRAM {
	QPU_BUFFER progmem_buffer; // progmem buffer in GPU memory
	QPU_PROGMEM progmem; // ARM side progmem mapped from GPU memory; need to lock buffer before access
} QPU_PROGRAM;



/* Initializes QPU program using base. Provide progmem size requirements through memsize.
 * If program is a general purpose program, messageSize should be > 2 to accommodate for code and uniforms.
 * Else messageSize should be 0 as QPU V3D registers are used for execution instead of a mailbox message. */
int qpu_initProgram(QPU_PROGRAM *program, QPU_BASE *base, QPU_PROGMEM progmem);

/* Destroy QPU program and clean up resources */
void qpu_destroyProgram (QPU_PROGRAM *program);

/* QPU execute program depending on how it was initialized. */
int qpu_executeProgram (QPU_PROGRAM *program, QPU_BASE *base, int numInst);

/* QPU execute program using direct access to QPU V3D registers. */
int qpu_executeProgramDirect (QPU_PROGRAM *program, QPU_BASE *base, int numInst, int unifLength, int unifStride, QPU_PerformanceState *perfState);

/* QPU execute general purpose program using mailbox. */
int qpu_executeProgramMailbox (QPU_PROGRAM *program, QPU_BASE *base, int numQPUs);

/* Copies code from supplied buffer into program memory. */
void qpu_setProgramCode(QPU_PROGRAM *program, const char *code, unsigned int length);

/* Loads code from the file directly into program memory. */
int qpu_loadProgramCode(QPU_PROGRAM *program, const char *filename);

/* Returns file size of specified code file. Can be used to fit progmem code size perfectly. */
unsigned int qpu_getCodeSize(const char *filename);

#ifdef __cplusplus
}
#endif

#endif
