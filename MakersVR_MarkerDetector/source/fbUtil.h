#ifndef FBUTIL_H
#define FBUTIL_H

#include <linux/fb.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void* lock_fb(int fbfd, long int fbsz);
void unlock_fb(void *fbp, long int fbsz);

void debug_fb_color(void *fbp, struct fb_var_screeninfo *vinfo, int pxPos, int pxCount);
void debug_fb_hex(void *fbp, int pxPos, int pxCount);

int setupFrameBuffer(
	struct fb_var_screeninfo *orig_vinfo,
	struct fb_var_screeninfo *vinfo,
	struct fb_fix_screeninfo *finfo,
	bool debugFB);

#ifdef __cplusplus
}
#endif

#endif
