#ifndef FEATURE_EXTRACTION_H
#define FEATURE_EXTRACTION_H

#include "_hvsys.h"
#include "calibration.h"

#define PRL_MAX_CORNERS		((MM_U32)(MAX_ROI_SIZE) )
#define QUALITY_LEVEL       ((MM_FLOAT)(0.13f))
#define GF_BLOCK_SIZE       ((MM_U08)(3))
#define APARTURE_SIZE		((MM_U08)(3)) /* APARTURE_SIZE >= 3 */

CFUNC_BEGIN_DECLS

typedef  struct _POINTINFO
{
	MM_S32 x;
	MM_S32 y;
	MM_FLOAT CornerEig;
} POINTINFO;

typedef struct _CORNERINFO
{
	MM_U32 length;
	MM_U32 size;
	POINTINFO CornerList[PRL_MAX_CORNERS];
} CORNERINFO;


 MM_S16  GetCorner_new(
						 MM_FLOAT *outFrontCorners,		/* [output] corner points */
						 MM_U16 *nFront,				/* [output] unmber of corner points */
						 MM_U08 *srcImgF,				/* [input] source image */
						 MM_U16 iHEIGHT,				/* [input] height of image */
						 MM_U16 iWidth,					/* [input] width of image */
						 CALIBRATION_PARAM *param	/* calibration information */
 );

CFUNC_END_DECLS

#endif
