.include "vc4.qinc"

# Constants
.set BLK_SZ, 16	# Vector size in VPM - theoretically up to 64 out of 192 vectors adressable by user programs, but can only use DMA on 16 at a time
.set BLK_POS, 0	# Vector pos in VPM - doesn't really matter, could be chosen to not interfere with other processes, up to 48 is fine
# Results in a maximum of 1024 bytes moved per DMA call

# Variable setup
.set srcPtr, rb0
.set tgtPtr, ra0
.set srcStride, ra1
.set tgtStride, rb1
.set blkCnt, ra3	# Blocks of 1024 bytes
.set remLns, ra4	# Number of remaining 64 byte blocks (0 to 15)

# Read uniforms
mov srcPtr, unif;
mov tgtPtr, unif;
mov r0, unif;		# Read raw number of 64 byte blocks

# Create VPM DMA Read setup
ldi vr_setup, vdr_setup_0(3, 16, BLK_SZ, vdr_h32(1, 0, 0));

# Start loading first batch
mov vr_addr, srcPtr;

# Setup variables
shr blkCnt, r0, 4;	# Calculate number of 1024 byte blocks
and remLns, r0, 15;	# Get remainder of 64 byte blocks (modulo 16)
ldi srcStride, 16*4 * BLK_SZ;
ldi tgtStride, 16*4 * BLK_SZ;
add srcPtr, srcPtr, srcStride;

# Create VPM DMA Write setup
ldi vw_setup, vdw_setup_0(BLK_SZ, 16, dma_h32(0, 0));
ldi vw_setup, vdw_setup_1(0);

:blkFor # Loop over blocks of 1024 bytes

	# Start writing batch when loaded and last write finished
	read vw_wait; read vr_wait;
	mov vw_addr, tgtPtr; add tgtPtr, tgtPtr, tgtStride;

	# Start loading next batch immediately while writing
	mov vr_addr, srcPtr; add srcPtr, srcPtr, srcStride;

	# Loop over all blocks with 1 write and 1 load ongoing (maximum)
	sub.setf blkCnt, blkCnt, 1;
	brr.anynn -, :blkFor
	nop
	nop
	nop

# If we have any remaining lines (excess 64 byte blocks) to write
mov.setf r0, remLns;
brr.anynn -, :remLnsIf

# Setup VDW from VPM to memory with variable number of 64-byte blocks
ldi r0, vdw_setup_0(0, 16, dma_h32(0, 0)); # Note: block size NOT specified
ldi r1, 23;
shl r1, remLns, r1;		# Bring block size parameter in position
add vw_setup, r0, r1;	# Setup VDW

# Already got one excess read loading from loop
read vr_wait;

# Initiate VDW
mov vw_addr, tgtPtr;
read vw_wait;

:remLnsIf

mov.setf irq, nop;

nop; thrend
nop
nop
