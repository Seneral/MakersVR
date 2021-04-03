#ifndef QPU_REGISTERS_H
#define QPU_REGISTERS_H

// These are the adresses of the V3D registers
// V3D spec: VideoCoreIV-AG100-R.pdf @ https://docs.broadcom.com/docs/12358545

/* Identification and Configuration Block (Specs) */
#define V3D_IDENT0	(0xC00000 >> 2) // V3D Identification 0 (V3D block identity)
#define V3D_IDENT1	(0xC00004 >> 2) // V3D Identification 1 (V3D Configuration A)
#define V3D_IDENT2	(0xC00008 >> 2) // V3D Identification 1 (V3D Configuration B)

/* Cache Control */
#define V3D_SCRATCH	(0xC00010 >> 2) // Scratch Register (Read/Write registers for general purposes)
#define V3D_L2CACTL	(0xC00020 >> 2) // L2 Cache Control
#define V3D_SLCACTL	(0xC00024 >> 2) // Slices Cache Control

/* Pipeline Interrupts */
#define V3D_INTCTL	(0xC00030 >> 2) // Interrupt Control
#define V3D_INTENA	(0xC00034 >> 2) // Interrupt Enables
#define V3D_INTDIS	(0xC00038 >> 2) // Interrupt Disables

/* V3D Control List Block */
#define V3D_CT0CS	(0xC00100 >> 2) // Control List Executor Thread 0 Control and Status
#define V3D_CT1CS	(0xC00104 >> 2) // Control List Executor Thread 1 Control and Status
#define V3D_CT0EA	(0xC00108 >> 2) // Control List Executor Thread 0 End Adress
#define V3D_CT1EA	(0xC0010C >> 2) // Control List Executor Thread 1 End Adress
#define V3D_CT0CA	(0xC00110 >> 2) // Control List Executor Thread 0 Current Adress
#define V3D_CT1CA	(0xC00114 >> 2) // Control List Executor Thread 1 Current Adress
#define V3D_CT00RA0	(0xC00118 >> 2) // Control List Executor Thread 0 Return Adress
#define V3D_CT01RA0	(0xC0011C >> 2) // Control List Executor Thread 1 Return Adress
#define V3D_CT0LC	(0xC00120 >> 2) // Control List Executor Thread 0 List Counter
#define V3D_CT1LC	(0xC00124 >> 2) // Control List Executor Thread 1 List Counter
#define V3D_CT0PC	(0xC00128 >> 2) // Control List Executor Thread 0 Primitive List Counter
#define V3D_CT1PC	(0xC0012C >> 2) // Control List Executor Thread 1 Primitive List Counter

/* V3D Pipelines (Binning & Rendering) Block */
#define V3D_PCS		(0xC00130 >> 2) // V3D Pipeline Control and Status
#define V3D_BFC		(0xC00134 >> 2) // Binning Mode Flush Count
#define V3D_RFC		(0xC00138 >> 2) // Rendering Mode Frame Count
#define V3D_BPCA	(0xC00300 >> 2) // Current Address of Binning Memory Pool
#define V3D_BPCS	(0xC00304 >> 2) // Remaining Size of Binning Memory Pool
#define V3D_BPOA	(0xC00308 >> 2) // Address of Overspill Binning Memory Block
#define V3D_BPOS	(0xC0030C >> 2) // Size of Overspill Binning Memory Block
#define V3D_BXCF	(0xC00310 >> 2) // Binner Debug

/* QPU Scheduler Block */
#define V3D_SQRSV0	(0xC00410 >> 2) // Reserve QPUs 0-7
#define V3D_SQRSV1	(0xC00414 >> 2) // Reserve QPUs 8-15
#define V3D_SQCNTL	(0xC00418 >> 2) // QPU Scheduler Control
#define V3D_SRQPC	(0xC00430 >> 2) // QPU User Program Request Program Address
#define V3D_SRQUA	(0xC00434 >> 2) // QPU User Program Request Uniforms Address
#define V3D_SRQUL	(0xC00438 >> 2) // QPU User Program Request Uniforms Length
#define V3D_SRQCS	(0xC0043C >> 2) // QPU User Program Request Control and Status

/* VPM Memory */
#define V3D_VPACNTL	(0xC00500 >> 2) // VPM Allocator Control
#define V3D_VPMBASE	(0xC00504 >> 2) // VPM base (user) memory reservation

/* Performance Counter - 0 <= x <= 15*/
#define V3D_PCTRC	(0xC00670 >> 2) // Performance Counter Clear
#define V3D_PCTRE	(0xC00674 >> 2) // Performance Counter Enables
#define V3D_PCTR(x) ((0xC00680 + 0x8*x) >> 2) // Performance Counter Count x
#define V3D_PCTRS(x) ((0xC00684 + 0x8*x) >> 2) // Performance Counter Mapping x

/* No idea FEP debug */
#define V3D_DBGE	(0xC00F00 >> 2) // PSE Error Signals
#define V3D_FDBGO	(0xC00F04 >> 2) // FEP Overrun Error Signals
#define V3D_FDBGB	(0xC00F08 >> 2) // FEP Interface Ready and Stall Signals, FEP Busy Signals
#define V3D_FDBGR	(0xC00F0C >> 2) // FEP Internal Ready Signals
#define V3D_FDBGS	(0xC00F10 >> 2) // FEP Internal Stall Input Signals
#define V3D_ERRSTAT	(0xC00F20 >> 2) // Miscellaneous Error Signals (VPM, VDW, VCD, VCM, L2C)

/* (Partially undocumented) General Purpose Interrupts */
#define V3D_DBCFG	(0xC00e00 >> 2)
#define V3D_DBQITE	(0xC00e2C >> 2)
#define V3D_DBQITC	(0xC00e30 >> 2)

#endif