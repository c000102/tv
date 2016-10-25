#ifndef POINT_EXTRACTION_H
#define POINT_EXTRACTION_H

#include <math.h>
#include <string.h>
#include "calibration.h"

#define MAX_CORNERS         ((MM_U32)(1280*720))
#define PIX_WIN_SIZE		((MM_U16)(3))
#define PIX_IMG_SIZE		((MM_U08)(9))

CFUNC_BEGIN_DECLS

MM_S16  PixelAccuracy_new(
							MM_FLOAT *outFrontCorners,	/* [out] corner points */
							MM_U16 *nOutFront,			/* [out] number of points */
							MM_U08 *srcImgF,			/* [in] source image */
							MM_U16 iHEIGHT,				/* [in] height of image */
							MM_U16 iWidth,				/* [in] width of image */
							MM_FLOAT *inFrontCorners,	/* [im] corner points */
							MM_U16 nFront);				/* [in] number of points */
CFUNC_END_DECLS
#endif
