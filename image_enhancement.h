#ifndef IMAGE_ENHANCEMENT_H
#define IMAGE_ENHANCEMENT_H

#include "_hvsys.h"
#include "calibration.h"

CFUNC_BEGIN_DECLS

typedef struct _ROI_BOX
{
	MM_U16 stx;	/* x start point of ROI area */
	MM_U16 sty; /* y start point of ROI area */
	MM_U16 wdx; /* x ROI width */
	MM_U16 wdy;	/* y ROI height */
} ROI_BOX;


 void  Enhancement(
					MM_U08 outImgF1C1[],			/* [out] gray image */
					ROI_BOX ret_roi_box[],			/* [in] ROI information */
					MM_U08 srcImgFC4[],				/* [in] source image */
					MM_U16 iheight,					/* [in] height of image */
					MM_U16 iWidth,					/* [in] width of image */
					MM_U08 ID,						/* [in] camera number */
					MM_U08 CalibType,				/* [in] calibration type */
					CALIBRATION_PARAM *param);/* [in] calibration parameters */


 MM_S16  gray2binary_otsu(
						 MM_U08 res_Image[],	/* [out] binary iamge */
						 MM_U08 gray_image[],	/* [in] soruce iamge */
						 MM_U16 height,			/* [in] height of image */
						 MM_U16 width);			/* [in] width of image */

 void  erosion(
				 MM_U08 src_img[],		/* [in/out] image*/
				 MM_U16 iheight,		/* [in] height of image */
				 MM_U16 iWidth);		/* [in] weight of iamge */

CFUNC_END_DECLS

#endif


