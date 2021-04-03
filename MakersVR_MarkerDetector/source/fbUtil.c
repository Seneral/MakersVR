#include "fbUtil.h"

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

// For Framebuffer debug only
const char* typeLUT[] = { "Packed Pixels", "Planes", "Interleaved Planes", "Text", "VGA Planes", "FourCC" };
const char* visualLUT[] = { "Mono White", "Mono Black", "True Color", "Pseudo Color", "Direct Color", "Static Pseudo Color", "FourCC" };
const char* modeLUT[] = { "Other", "Non-Interlaced", "Interlaced", "Double", "Odd FLD First" };

void* lock_fb(int fbfd, long int fbsz)
{
	return mmap(0, fbsz, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
}
void unlock_fb(void *fbp, long int fbsz)
{
	munmap(fbp, fbsz);
}

void debug_fb_color(void *fbp, struct fb_var_screeninfo *vinfo, int pxPos, int pxCount)
{
	for (int u = pxPos; u < pxPos+pxCount; u++)
	{
		uint32_t pxCol = *(uint32_t*)(fbp + u * vinfo->bits_per_pixel / 8);
		printf("PX %d: R %d, G %d, B %d \n", pxPos,
			(pxCol >> vinfo->red.offset) & ((1 << vinfo->red.length) - 1),
			(pxCol >> vinfo->green.offset) & ((1 << vinfo->green.length) - 1),
			(pxCol >> vinfo->blue.offset) & ((1 << vinfo->blue.length) - 1));
	}
}
void debug_fb_hex(void *fbp, int pxPos, int pxCount)
{
	for (int u = pxPos; u < pxPos+pxCount; u++)
	{
		printf("%d: %#010x; ", u, ((uint32_t*)fbp)[u]);
		if ((u+1) % 8 == 0) printf("\n");
	}
	printf("\n");
}

int setupFrameBuffer(
	struct fb_var_screeninfo *orig_vinfo,
	struct fb_var_screeninfo *vinfo,
	struct fb_fix_screeninfo *finfo,
	bool debugFB)
{
	// Open the file for reading and writing
	int fbfd = open("/dev/fb0", O_RDWR);
	if (!fbfd) {
		printf("Error: cannot open framebuffer device.\n");
		return 0;
	}

	// Get variable screen information
	if (ioctl(fbfd, FBIOGET_VSCREENINFO, vinfo)) {
		printf("Error reading variable FrameBuffer information.\n");
		goto fberror;
	}

	// Store as back up
	memcpy(orig_vinfo, vinfo, sizeof(struct fb_var_screeninfo));

	// Modify and push vinfo
	//vinfo.bits_per_pixel = 16;
	if (ioctl(fbfd, FBIOPUT_VSCREENINFO, vinfo)) {
	    printf("Error setting variable information.\n");
		goto fberror;
	}

	// Get fixed screen information
	if (ioctl(fbfd, FBIOGET_FSCREENINFO, finfo)) {
		printf("Error reading fixed FrameBuffer information.\n");
		goto fberror;
	}

	if (debugFB)
	{
		printf("FrameBuffer: %s; %dx%d, %d bpp, %d bytes, Line Length %d!\n",
			finfo->id, vinfo->xres, vinfo->yres, vinfo->bits_per_pixel, finfo->smem_len, finfo->line_length);
		printf("   Type: %s, Visual: %s, Mode: %s!\n",
			typeLUT[finfo->type], visualLUT[finfo->visual], modeLUT[(vinfo->vmode&0b111) + 1]);
		printf("   Pixel Format: R: %d << %d; G: %d << %d; B: %d << %d! \n",
			vinfo->red.length, vinfo->red.offset, vinfo->green.length, vinfo->green.offset, vinfo->blue.length, vinfo->blue.offset);
	}

	return fbfd;

	fberror:
	close(fbfd);
	return 0;
}
