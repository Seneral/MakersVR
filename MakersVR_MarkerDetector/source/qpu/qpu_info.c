#define __USE_MINGW_ANSI_STDIO 
#include <stdio.h>

#include "qpu_info.h"

void qpu_getUserProgramInfo(QPU_UserProgramInfo *info, QPU_BASE *base)
{
	info->QPURQCC = (base->peripherals[V3D_SRQCS] >> 16) & 0xFF;
	info->QPURQCM = (base->peripherals[V3D_SRQCS] >> 8) & 0xFF;
	info->QPURQERR = (base->peripherals[V3D_SRQCS] >> 7) & 1;
	info->QPURQL = (base->peripherals[V3D_SRQCS] >> 0) & 0x3F;
	info->VPMURSV = (base->peripherals[V3D_VPMBASE] >> 0) & 0x1F;
	info->VPMURSV_V = info->VPMURSV * 4;
}
void qpu_get3DPipelineInfo(QPU_3DPipelineInfo *info, QPU_BASE *base)
{
	info->BMOOM = (base->peripherals[V3D_PCS] >> 8) & 1;
	info->RMBUSY = (base->peripherals[V3D_PCS] >> 3) & 1;
	info->RMACTIVE = (base->peripherals[V3D_PCS] >> 2) & 1;
	info->BMBUSY = (base->peripherals[V3D_PCS] >> 1) & 1;
	info->BMACTIVE = (base->peripherals[V3D_PCS] >> 0) & 1;
	info->BMFCT = (base->peripherals[V3D_BFC] >> 0) & 0xFF;
	info->RMFCT = (base->peripherals[V3D_RFC] >> 0) & 0xFF;
}
bool qpu_getL2CacheState(QPU_BASE *base)
{
	return (base->peripherals[V3D_L2CACTL] >> 0) & 1;
}

int qpu_getQPUInterruptFlags(QPU_BASE *base)
{
	return (base->peripherals[V3D_DBQITC] >> 0) & 0xFFFF;
}
void qpu_resetQPUInterruptFlags(QPU_BASE *base)
{
	base->peripherals[V3D_DBQITC] |= 0xFFFF;

}
void qpu_get3DInterrupts(QPU_3DInterrupts *info, QPU_BASE *base)
{
	info->INT_SPILLUSE = (base->peripherals[V3D_INTCTL] >> 3) & 1;
	info->INT_OUTOMEM = (base->peripherals[V3D_INTCTL] >> 2) & 1;
	info->INT_FLDONE = (base->peripherals[V3D_INTCTL] >> 1) & 1;
	info->INT_FRDONE = (base->peripherals[V3D_INTCTL] >> 0) & 1;
}

void qpu_getHWIdent(QPU_HWIdent *info, QPU_BASE *base)
{
	info->TVER = (base->peripherals[V3D_IDENT0] >> 24) & 0xFF;
	info->IDSTR[0] = (base->peripherals[V3D_IDENT0] >> 	0) & 0xFF;
	info->IDSTR[1] = (base->peripherals[V3D_IDENT0] >> 	8) & 0xFF;
	info->IDSTR[2] = (base->peripherals[V3D_IDENT0] >> 16) & 0xFF;
	info->IDSTR[3] = 0;
}
void qpu_getHWConfiguration(QPU_HWConfiguration *info, QPU_BASE *base)
{
	uint32_t ident1 = base->peripherals[V3D_IDENT1];
	uint32_t ident2 = base->peripherals[V3D_IDENT2];
	info->VPMSZ = (ident1 >> 28) & 0xF;
	if (info->VPMSZ == 0) info->VPMSZ = 16;
	info->VPMSZ_V = info->VPMSZ * 16;
	info->HDRT = (ident1 >> 24) & 0xF;
	info->NSEM = (ident1 >> 16) & 0xFF;
	info->TUPS = (ident1 >> 12) & 0xF;
	info->QUPS = (ident1 >> 8) & 0xF;
	info->NSLC = (ident1 >> 4) & 0xF;
	info->QPU_NUM = info->QUPS * info->NSLC;
	info->REV = (ident1 >> 0) & 0xF;

	info->TLBDB = (ident2 >> 8) & 0xF;
	info->TLBSZ = (ident2 >> 4) & 0xF;
	info->TLB_X = info->TLBSZ == 0? 32 : 64;
	info->TLB_Y = info->TLBSZ == 2? 64 : 32;
	info->VRISZ = (ident2 >> 0) & 0xF;
}
void qpu_debugHW(QPU_BASE *base)
{
	QPU_HWIdent id;
	qpu_getHWIdent(&id, base);
	QPU_HWConfiguration hw;
	qpu_getHWConfiguration(&hw, base);
	printf("%s HW: %dK VPM memory (%d vectors); %d slices @ (%d TMUs, %d QPUs); Tile Buffer %dx%d \n",
		id.IDSTR, hw.VPMSZ, hw.VPMSZ*16, hw.NSLC, hw.TUPS, hw.QUPS, hw.TLB_X, hw.TLB_Y);
}

int qpu_getReservationSetting(QPU_BASE *base, int qpu)
{
	return (base->peripherals[qpu > 7? V3D_SQRSV1 : V3D_SQRSV0] >> (4*(qpu%8))) & 0xF;
}
void qpu_setReservationSetting(QPU_BASE *base, int qpu, int set)
{
	base->peripherals[qpu > 7? V3D_SQRSV1 : V3D_SQRSV0]
		&= ~(0xF << (4*(qpu%8)));
	base->peripherals[qpu > 7? V3D_SQRSV1 : V3D_SQRSV0]
		|= (set & 0xF) << (4*(qpu%8));
}
void qpu_logReservationSettings(QPU_BASE *base)
{
	int res[12];
	for(int i = 0; i < 12; i++)
		res[i] = qpu_getReservationSetting(base, i);

	// Log
	printf("QPUs 1-4: %d | %d | %d | %d\n", res[0], res[1], res[2], res[3]);
	printf("QPUs 5-8: %d | %d | %d | %d\n", res[4], res[5], res[6], res[7]);
	printf("QPUs 9-12: %d | %d | %d | %d\n", res[8], res[9], res[10], res[11]);

//	printf("QPU 5:%d, 6:%d, 7:%d, 8:%d");
//	printf("QPU 9:%d, 10:%d, 11:%d, 12:%d");
}

void qpu_setupPerformanceCounters(QPU_BASE *base, QPU_PerformanceState *state)
{
	base->peripherals[V3D_PCTRC] = 0xFFFF;
	base->peripherals[V3D_PCTRE] = 0x80000000 | 0xFFFF; // Only need first 11

	base->peripherals[V3D_PCTRS( 0)] = 13;
	base->peripherals[V3D_PCTRS( 1)] = 14;
	base->peripherals[V3D_PCTRS( 2)] = 15;
	base->peripherals[V3D_PCTRS( 3)] = 16;
	base->peripherals[V3D_PCTRS( 4)] = 17;

	base->peripherals[V3D_PCTRS( 5)] = 24;
	base->peripherals[V3D_PCTRS( 6)] = 25;
	base->peripherals[V3D_PCTRS( 7)] = 26;
	base->peripherals[V3D_PCTRS( 8)] = 27;
	base->peripherals[V3D_PCTRS( 9)] = 28;
	base->peripherals[V3D_PCTRS(10)] = 29;

	for (int i = 0; i < 11; i++)
	{
		state->rawLast[i] = 0;
		state->rawAccum[i] = 0;
		state->diffAccum[i] = 0;
	}
	state->clockRateMin = state->clockRateMax = 0;
}
void qpu_updatePerformance(QPU_BASE *base, QPU_PerformanceState *state)
{
	for (int i = 0; i < 11; i++)
	{
		uint32_t raw = base->peripherals[V3D_PCTR(i)];
		uint32_t diff = state->rawLast[i] > raw? (uint32_t)(raw + ((uint64_t)2 << 32) - state->rawLast[i]) : (raw - state->rawLast[i]);
		state->rawLast[i] = raw;
		state->rawAccum[i] += diff;
		state->diffAccum[i] += diff;
	}

	// Get clock frequency
	uint32_t cr = getClockRate(base->mb, 5);
	if (state->clockRateMin == 0)
	{
//		state->clockRateMin = cr;
		state->clockRateMin = state->clockRateMax = cr;
	}
	else
	{
//		state->clockRateMax = cr;
		if (state->clockRateMin > cr) state->clockRateMin = cr;
		if (state->clockRateMax < cr) state->clockRateMax = cr;
	}

	// Get SOC temperature
	state->tempSOC = getTemperature(base->mb);
}
void qpu_compilePerformance(QPU_PerformanceState *state, QPU_Performance *perf, bool diff)
{
	uint64_t *perfData = diff? state->diffAccum : state->rawAccum;

	// Note: Divide clocks by 4 to get actual instructions (QPU needs 4 clock cycle for a 16-way instruction)
	perf->clkIdle = perfData[ 0] / 4;
	perf->clkVert = perfData[ 1] / 4;
	perf->clkFrag = perfData[ 2] / 4;
	perf->clkInst = perfData[ 3] / 4;
	perf->clkTMUStall = perfData[ 4] / 4;
	perf->numTMUProc = perfData[ 5] / 4;
	perf->numTMUMiss = perfData[ 6] * 4;
	perf->clkVDWStall = perfData[ 7] / 4;
	perf->clkVCDStall = perfData[ 8] / 4;
	perf->numL2CHits = perfData[ 9];
	perf->numL2CMiss = perfData[10];

	if (diff)
	{
		for (int i = 0; i < 11; i++)
			state->diffAccum[i] = 0;
	}
}
void qpu_logPerformance (QPU_PerformanceState *state)
{
	// Compile performance report
	QPU_Performance accum, diff;
	qpu_compilePerformance(state, &accum, false);
	qpu_compilePerformance(state, &diff, true);

	// Interpret report
	uint64_t totalCycles = diff.clkVert + diff.clkFrag + diff.clkIdle; // Verified validity with clock rate and time
	uint64_t loadCycles = diff.clkVert + diff.clkFrag; // Valid, although sometimes other processes seem to take up vert cycles even though no instructions are issued
	uint64_t stallCycles = diff.clkTMUStall + diff.clkVDWStall + diff.clkVCDStall;
	//uint64_t loadCycles = diff.clkInst + stallCycles;
	double qpuLoadPerc = (double)loadCycles / totalCycles * 100.0;
	double qpuUsedLoadPerc = (double)loadCycles / totalCycles * 100.0 * 12 / state->qpusUsed;
	double qpuStallPerc = (double)stallCycles / loadCycles * 100.0;
	double avgTMUStalls = (double)diff.clkTMUStall / (diff.numTMUProc+1);
	double TMUCachedPerc = ((double)diff.numTMUProc - diff.numTMUMiss) / (diff.numTMUProc+1) * 100.0;
	double TMUUsageInter = (double)loadCycles / (diff.numTMUProc+1);
	// only works for 1 TMU per 2 QPUs, as in a RPi VC4; for every second QPU 20 cycles could be used for TMU access
	// That means with caching more than 100% bandwidth could be used in this metric, but whatever
	double VDWStallPerc = (double)diff.clkVDWStall / loadCycles * 100.0;
	double VCDStallPerc = (double)diff.clkVCDStall / loadCycles * 100.0;
	double L2CMissPerc = (double)diff.numL2CMiss / (diff.numL2CHits + diff.numL2CMiss) * 100.0;

	// Log performance using excerpts of report
//	printf("QPU Performance:\n");
	printf("     Cycles: %llu idle | %llu vert | %llu frag | %llu instructions\n", diff.clkIdle, diff.clkVert, diff.clkFrag, diff.clkInst);
	printf("     Clock %uMHz - %uMHz | %.01f%% total load | %.01f%% load for used QPUs | %.1f°C SoC temp\n", state->clockRateMin/1000000, state->clockRateMax/1000000, qpuLoadPerc, qpuUsedLoadPerc, state->tempSOC/1000.0f);
	printf("     %llu instructions | %llu load cycles | %.01f%% stalls\n", diff.clkInst, loadCycles, qpuStallPerc);
//	printf("     TMU: %llu stalls | %llu processed | %llu cache misses\n", diff.clkTMUStall, diff.numTMUProc, diff.numTMUMiss);
	printf("     TMU: %.1f avg access interval | %.1f avg stall cycles | %.1f%% cached\n", TMUUsageInter, avgTMUStalls, TMUCachedPerc);
//	printf("     VPM: %llu VDW stall cycles | %llu VCD stall cycles\n", diff.clkVDWStall, diff.clkVCDStall);
	printf("     VPM: %.1f%% VDW stall | %.1f%% VCD stall (avg of program cycles)\n", VDWStallPerc, VCDStallPerc);
//	printf("     L2C: %llu hits | %llu misses\n", diff.numL2CHits, diff.numL2CMiss);
	printf("     L2C: %.2f%% miss (%llu hits | %llu misses)\n", L2CMissPerc, diff.numL2CHits, diff.numL2CMiss);

	state->clockRateMin = state->clockRateMax = 0;

}

int qpu_logErrors(QPU_BASE *base)
{
	int hadError = 0;

	//  Miscellaneous Error Signals (VPM, VDW, VCD, VCM, L2C)
	int errstat = base->peripherals[V3D_ERRSTAT] & 0xFFFF;
	hadError |= errstat & ~(1<<12);
	if (errstat & (1<<15)) printf("L2CARE: L2C AXI Receive Fifo Overrun error!\n");
	if (errstat & (1<<14)) printf("VCMBE: VCM error (binner)!\n");
	if (errstat & (1<<13)) printf("VCMRE: VCM error (renderer)!\n");
	// 12: IDLE
	if (errstat & (1<<11)) printf("VCDE: VCD error - FIFO pointers out of sync!\n");
	if (errstat & (1<<10)) printf("VDWE: VDW error - address overflows!\n");
	if (errstat & (1<< 9)) printf("VPMEAS: VPM error - allocated size error!\n");
	if (errstat & (1<< 8)) printf("VPMEFNA: VPM error - free non-allocated!\n");
	if (errstat & (1<< 7)) printf("VPMEWNA: VPM error - write non-allocated!\n");
	if (errstat & (1<< 6)) printf("VPMERNA: VPM error - read non-allocated!\n");
	if (errstat & (1<< 5)) printf("VPMERR: VPM error - read range!\n");
	if (errstat & (1<< 4)) printf("VPMEWR: VPM error - write range!\n");
	if (errstat & (1<< 3)) printf("VPAERRGL: VPM Allocator error - renderer request greater than limit!\n");
	if (errstat & (1<< 2)) printf("VPAEBRGL: VPM Allocator error - binner request greater than limit!\n");
	if (errstat & (1<< 1)) printf("VPAERGS: VPM Allocator error - request too big!\n");
	if (errstat & (1<< 0)) printf("VPAEABB: VPM Allocator error - allocating base while busy!\n");

	// PSE Error Signals
	hadError |= errstat & (0x1F06);
	if (base->peripherals[V3D_DBGE] & (1<<20)) printf("IPD2_FPDUSED: error_ipd2_fpdused in PSE!\n");
	if (base->peripherals[V3D_DBGE] & (1<<19)) printf("IPD2_VALID: error_ipd2_valid in PSE!\n");
	if (base->peripherals[V3D_DBGE] & (1<<18)) printf("MULIP2: error_mulip2 in PSE!\n");
	if (base->peripherals[V3D_DBGE] & (1<<17)) printf("MULIP1: error_mulip1 in PSE!\n");
	if (base->peripherals[V3D_DBGE] & (1<<16)) printf("MULIP0: error_mulip0 in PSE!\n");
	if (base->peripherals[V3D_DBGE] & (1<< 2)) printf("VR1_B: error b reading VPM in PSE!\n");
	if (base->peripherals[V3D_DBGE] & (1<< 1)) printf("VR1_A: error a reading VPM in PSE!\n");

	// TODO: FEP Overrun Error Signals

	return hadError;
}

/*int qpu_logStalls(QPU_BASE *base)
{
	int hadStalls = 0;

	//  Miscellaneous Error Signals (VPM, VDW, VCD, VCM, L2C)
	int errstat = base->peripherals[V3D_ERRSTAT] & 0xFFFF;
	hadError |= errstat & ~(1<<12);
	if (errstat & (1<<15)) printf("L2CARE: L2C AXI Receive Fifo Overrun error!\n");
	if (errstat & (1<<14)) printf("VCMBE: VCM error (binner)!\n");
	if (errstat & (1<<13)) printf("VCMRE: VCM error (renderer)!\n");
	// 12: IDLE
	if (errstat & (1<<11)) printf("VCDE: VCD error - FIFO pointers out of sync!\n");
	if (errstat & (1<<10)) printf("VDWE: VDW error - address overflows!\n");
	if (errstat & (1<< 9)) printf("VPMEAS: VPM error - allocated size error!\n");

}
*/
