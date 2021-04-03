.include "vc4.qinc"

# Uniforms
.set srcPtr, ra0
.set tgtPtr, ra1
.set srcStride, rb0
.set tgtStride, rb1
.set lineIter, ra2			# Iterator over blocks (16 lines)
.set thresholdCO8, rb4
.set diffCO8, ra4
mov srcPtr, unif;
mov tgtPtr, unif;
mov srcStride, unif;
mov tgtStride, unif;
mov lineIter, unif;			# block count
mov ra5.8888i, unif;
mov diffCO8.8888i, unif;
mov thresholdCO8, ra5;

# Variables
.set vpmSetup, rb2
.set vdwSetup, rb3
.set mskAccum, ra3
.set sh1, ra5
.set sh2, ra6
.set sh3, ra7
.set sh2B, rb5
.set num4, rb6
.set bitMax, rb7

# Define variables storing the current headers
.func valReg(y, x)
	.assert b < 5 && b >= 0
	.assert l < 4 && l >= 0
	.assert y < 5 && y >= 0
	.assert x <= 2 && x >= 0
	ra17 + ((b*4*3 + (l-y)*3 + x + 15)%15)
.endf
.func minReg(n, x)
	.assert n <= 5  && n >= 2
	.assert x <= 2 && x >= 0
	rb20 + ((n-2)*3 + x)
.endf

# TODO: Generate vector mask to allow for any multiple of 8-wide columns (not just 16x8)

# Calculate base source of each tile column
mul24 r0, elem_num, 8;
add srcPtr, srcPtr, r0;

# Set mask parameters
ldi sh2, 16;
ldi sh3, 24;
mov sh1, 8; mov sh2B, sh2;
mov num4, 4;
ldi bitMax, 0x01010101;

# Start loading very first line
mov t0s, srcPtr; add srcPtr, srcPtr, 4;
nop;
mov t0s, srcPtr;

# Create VPM Setup
;mov r1, 5;
mul24 r1, qpu_num, r1;
ldi r0, vpm_setup(0, 1, h32(0));
add vpmSetup, r0, r1;

# Create VPM DMA Basic setup
shl r1, r1, 7; # Same VPM position, different place in register
ldi r0, vdw_setup_0(16, 5, dma_v32(0, 0));
add vdwSetup, r0, r1;

# Adjust values
ldi r0, 20;
mul24 tgtStride, tgtStride, r0; # Remove written bytes
;mov r0, 8;
sub srcStride, srcStride, r0; # Remove read bytes

# Init defaults
.lset b, 0
.lset l, 0
mov r0, 0; mov r1, -1; # Min for val, Max for min
mov valReg(4,0), r0; mov minReg(4,0), r1;
mov valReg(4,1), r0; mov minReg(4,1), r1;
mov valReg(4,2), r0; mov minReg(4,2), r1;
mov valReg(3,0), r0; mov minReg(3,0), r1;
mov valReg(3,1), r0; mov minReg(3,1), r1;
mov valReg(3,2), r0; mov minReg(3,2), r1;
mov valReg(2,0), r0; mov minReg(2,0), r1;
mov valReg(2,1), r0; mov minReg(2,1), r1;
mov valReg(2,2), r0; mov minReg(2,2), r1;
mov valReg(1,0), r0;
mov valReg(1,1), r0;
mov valReg(1,2), r0;

# Initiate mask accumulator for first iteration
mov mskAccum, 0;

:blockIter # Loop over blocks

	# Initiate VPM write and make sure last VDW finished
	read vw_wait; mov vw_setup, vpmSetup;

	.rep b, 5 # 5 Blocks of 32Bits each

		.rep l, 4 # 4 Lines of 8Bits each

			# Wait for current load and start next
			# Update column-wise minimum values for the first 4 columns
			add srcPtr, srcPtr, num4; ldtmu0
			mov valReg(0,0), r4;	v8min minReg(5,0), minReg(4,0), r4;
			mov t0s, srcPtr;		v8min minReg(4,0), minReg(3,0), r4;

			# Finish updating column-wise minimum for next iteration
#			v8min minReg(3,0), minReg(2,0), r4;
#			v8min minReg(2,0), valReg(1,0), r4;
			# Interleaved at the end of the loop accessing valReg(0,0) instead of r4

			# Wait for current load and start next
			# Update column-wise minimum values for the middle 4 columns
			add srcPtr, srcPtr, srcStride; ldtmu0
			mov valReg(0,1), r4;				v8min minReg(5,1), minReg(4,1), r4;
			mov t0s, srcPtr;					v8min minReg(4,1), minReg(3,1), r4;

			# Calculate 5x5 min for first four pixels
												shl r1, minReg(5,1), sh3;
												shr r0, minReg(5,0), sh1;
			v8adds r0, r0, r1; 					shl r1, minReg(5,1), sh2;
			v8min r2, r0, minReg(5,0);			shr r0, minReg(5,0), sh2;
			v8adds r0, r0, r1; 					shl r1, minReg(5,1), sh1;
			v8min r2, r0, r2;					shr r0, minReg(5,0), sh3;
			v8adds r0, r0, r1; 					v8min r2, r2, minReg(5,1);
			v8min r2, r0, r2;

			# Compute mask for 4 pixels at once
			# 0 if value < min+diff and value < threshold
												;shr r1, valReg(2,0), sh2B;
												shl r0, valReg(2,1), sh2B;
			v8adds r2, r2, diffCO8; 			v8min minReg(3,1), minReg(2,1), r4;
			v8min r2, r2, thresholdCO8; 		v8adds r0, r0, r1;
			v8subs r2, r0, r2;					v8min minReg(2,1), valReg(1,1), r4;

			# Write bit 0 to mask iff r2 accum byte is 0
			v8min r2, r2, bitMax;
			shl r2, r2, l*2+0; # Mask Pos from 0-7
				; mov r0, valReg(0,0);
			v8adds mskAccum, mskAccum, r2;
				; v8min minReg(3,0), minReg(2,0), r0;

			# Wait for current load and start next
			# Update column-wise minimum values for the last 4 columns
			add srcPtr, srcPtr, num4; ldtmu0
			mov valReg(0,2), r4;				v8min minReg(5,2), minReg(4,2), r4;
			mov t0s, srcPtr;					v8min minReg(4,2), minReg(3,2), r4;

			# Calculate 5x5 min for last four pixels
												shl r1, minReg(5,2), sh3;
												shr r0, minReg(5,1), sh1;
			v8adds r0, r0, r1; 					shl r1, minReg(5,2), sh2;
			v8min r2, r0, minReg(5,1);			shr r0, minReg(5,1), sh2;
			v8adds r0, r0, r1; 					shl r1, minReg(5,2), sh1;
			v8min r2, r0, r2;					shr r0, minReg(5,1), sh3;
			v8adds r0, r0, r1; 					v8min r2, r2, minReg(5,2);
			v8min r2, r0, r2;

			# Compute mask for 4 pixels at once
			# 0 if value < min+diff and value < threshold
												;shr r1, valReg(2,1), sh2B;
												shl r0, valReg(2,2), sh2B;
			v8adds r2, r2, diffCO8; 			v8min minReg(3,2), minReg(2,2), r4;
			v8min r2, r2, thresholdCO8; 		v8adds r0, r0, r1;
			v8subs r2, r0, r2;					v8min minReg(2,2), valReg(1,2), r4;

			# Write bit 0 to mask iff r2 accum byte is 0
			v8min r2, r2, bitMax;
				; mov r0, valReg(0,0);
			shl r2, r2, l*2+1; # Mask Pos from 0-7
				; v8min minReg(2,0), valReg(1,0), r0;

			.if l == 3 # Write to VPM
				v8adds vpm, mskAccum, r2; mov mskAccum, 0;
			.else # Continue accumulating
				v8adds mskAccum, mskAccum, r2;
			.endif

		.endr

	.endr

	# Initiate VDW from VPM to memory
	mov vw_setup, vdwSetup;
	ldi vw_setup, vdw_setup_1(0);
	mov vw_addr, tgtPtr;

	# Increase adress to next line
	;add tgtPtr, tgtPtr, tgtStride;

	# Line loop :blockIter
	sub.setf lineIter, lineIter, 1;
	brr.anynn -, :blockIter
	nop
	nop
	nop

# Read last two unused lines (outside of bounds)
ldtmu0
ldtmu0

mov.setf irq, nop;

nop; thrend
nop
nop