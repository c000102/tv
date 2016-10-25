#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "image_enhancement.h"

CFUNC_BEGIN_DECLS

static MM_U08 crp_img[MAX_ROI_SIZE]; /* ROI iamge memory size */

static void  makeROI_RGBA_signle(
								MM_U08 output_img[],				/* [out] gray image */
								MM_U08 src[],					/* [in] source iamge(RGBA) */
								ROI_BOX box[],						/* [in] ROI imformation */
								MM_U16 height,						/* [in] height of image */
								MM_U16 width,						/* [in] width of image */
								MM_U08 ID,							/* [in] camera number */
								CALIBRATION_PARAM *param);	/* [in] calibration parameters */

static void  cvtC3toC1(
						MM_U08 dstC1[],			/* [out] gray image */
						MM_U08 srcC3[],	/* [in] source iamge at 3 chanel */
						MM_U16 iHEIGHT,			/* [in] height of image*/
						MM_U16 iWidth);			/* [in] width of image */

static void  gray2binary_unity(
								MM_U08 result_image[],		/* [out] binary image */
								MM_U08 gray_image[],	/* [in] gray image */
								MM_U16 height,				/* [in] height of image */
								MM_U16 width,				/* [in] width of image */
								MM_U16 threshold);			/* [in] binary thershold value */

static void factory_calibration_ROI_setting(
											ROI_BOX *roi,					/* [out] ROI imformation */
											CALIBRATION_PARAM *param,	/* [in] calibration parameters */
											MM_U08 ID);						/* [in] camera number */

static void service_calibration_ROI_setting(
											ROI_BOX *roi,					/* [out] ROI imformation */
											CALIBRATION_PARAM *param,	/* [in] calibration parameters */
											MM_U08 ID);						/* [in] camera number */

CFUNC_END_DECLS


// the main function of this module

void  Enhancement(  MM_U08 outImgF1C1[],			/* [out] gray image */
					ROI_BOX ret_roi_box[],			/* [in] ROI information */
					MM_U08 srcImgFC4[],				/* [in] source image */
					MM_U16 iheight,					/* [in] height of image */
					MM_U16 iWidth,					/* [in] width of image */
					MM_U08 ID,						/* [in] camera number */
					MM_U08 CalibType,				/* [in] calibration type */
					CALIBRATION_PARAM *param)	/* [in] calibration parameters */
{
	ROI_BOX roi_box[2];
	MM_U32 h = 0;
	MM_U32 w = 0;
	MM_FLOAT temp_roi[8] = { 0.0f, };

	memset(roi_box, 0x00, sizeof(ROI_BOX)*(MM_U32)(2));

	if (CalibType == FACTORY_CALIB) /* Factory calibration */
	{
		h = param->roi_height;
		w = param->roi_width;

		factory_calibration_ROI_setting(roi_box, param, ID);

		memset(outImgF1C1, 0x00, sizeof(MM_U08)*h*w*(MM_U08)(2));
		memset(crp_img, 0x00, sizeof(MM_U08)*h*w);

		makeROI_RGBA_signle(crp_img, srcImgFC4, &roi_box[0], iheight, iWidth + (MM_U08)(32), ID, param);
		memcpy(outImgF1C1, crp_img, sizeof(MM_U08) * h*w);

		memcpy(temp_roi, &param->roi_cropped[8], sizeof(MM_FLOAT) * 8);
		memcpy(param->roi_cropped, temp_roi, sizeof(MM_FLOAT) * 8);

		makeROI_RGBA_signle(crp_img, srcImgFC4, &roi_box[1], iheight, iWidth + (MM_U08)(32), ID, param);
		memcpy((outImgF1C1 + (h*w)), crp_img, sizeof(MM_U08) * h*w);

		memcpy(ret_roi_box, roi_box, sizeof(ROI_BOX)*(MM_U08)(2));

	}
	else /* Service calibiration */
	{
		service_calibration_ROI_setting(roi_box, param, ID);

		memset(outImgF1C1, 0x00, sizeof(MM_U08)*MAX_ROI_SIZE*(MM_U08)(2));
		memset(crp_img, 0x00, sizeof(MM_U08)*MAX_ROI_SIZE);

		memcpy(temp_roi, &param->roi_cropped[8], sizeof(MM_FLOAT) * 8);
		memcpy(param->service_roi_cropped, temp_roi, sizeof(MM_FLOAT) * 8);

		makeROI_RGBA_signle(crp_img, srcImgFC4, &roi_box[0], iheight, iWidth + (MM_U08)(32), ID, param);
		memcpy(outImgF1C1, crp_img, sizeof(MM_U08) * MAX_ROI_SIZE);

		memcpy(ret_roi_box, roi_box, sizeof(ROI_BOX)*(MM_U08)(1));
	}
	

}


static MM_U32 LUT[768]; /* gray image Lookup table memory */
static MM_U08 temp_crop_img[MAX_ROI_SIZE]; /* MISRA-C:2012  R.19.1 */

void  cvtC3toC1(MM_U08 dstC1[],			/* [out] gray image */
				MM_U08 srcC3[],	/* [in] source iamge at 3 chanel */
				MM_U16 iHEIGHT,			/* [in] height of image*/
				MM_U16 iWidth)			/* [in] width of image */
{
	MM_U32 n = (MM_U32)0;
	MM_U32 i = (MM_U32)0;
	MM_U32 b = (MM_U32)0;
	MM_U32 g = (MM_U32)0;
	MM_U32 r = (MM_U32)0;
	MM_U32 db = (MM_U32)0;
	MM_U32 dg = (MM_U32)0;
	MM_U32 dr = (MM_U32)0;

	n = (MM_U32)(iHEIGHT) * (MM_U32)(iWidth);
	r = ((MM_U32)(1) << (MM_U32)(13)); //13 = 14-1 : replaced

	db = (MM_U32)4899;
	dg = (MM_U32)9617;
	dr = (MM_U32)1868;
	
	for (i = 0; i < (MM_U32)(256); i++)
	{
		LUT[i] = b;
		LUT[i + (MM_U32)(256)] = g;
		LUT[i + (MM_U32)(512)] = r;

		b += (MM_U32)(db);
		g += (MM_U32)(dg);
		r += (MM_U32)(dr);
	}

	memset(temp_crop_img, 0x00, sizeof(MM_U08)*MAX_ROI_SIZE); /* MISRA-C:2012  R.19.1 */

	for (i = 0; i < n; i++)
	{
		//dstC1[i] = (MM_U08)((LUT[srcC3[0]] + 
		//	                 LUT[srcC3[(MM_U32)(1)] + (MM_U32)(256)] + 
		//	                 LUT[srcC3[2] + (MM_U32)(512)]) >> (MM_U08)(14));

		temp_crop_img[i] = (MM_U08)((LUT[srcC3[0]] +
									LUT[srcC3[(MM_U32)(1)] + (MM_U32)(256)] +
									LUT[srcC3[2] + (MM_U32)(512)]) >> (MM_U08)(14));

		srcC3 += (MM_U08)(3);
	}

	memcpy(dstC1, temp_crop_img, sizeof(MM_U08)*MAX_ROI_SIZE); /* MISRA-C:2012  R.19.1 */
}


//new
#define sub_ROI (0)

static MM_U08 tmp_img[MAX_ROI_SIZE * 3]; /* temporary image at 3 chanel */
void  makeROI_RGBA_signle(
							MM_U08 output_img[],				/* [out] gray image */
							MM_U08 src[],					/* [in] source iamge(RGBA) */
							ROI_BOX box[],						/* [in] ROI imformation */
							MM_U16 height,						/* [in] height of image */
							MM_U16 width,						/* [in] width of image */
							MM_U08 ID,							/* [in] camera number */
							CALIBRATION_PARAM *param)	/* [in] calibration parameters */
{
	MM_U32 y = (MM_U32)0;
	MM_U32 x = (MM_U32)0;
	MM_U32 xx = (MM_U32)0;
	MM_U32 yy = (MM_U32)0;
#if (sub_ROI)
	MM_S16 val1 = (MM_S16)0;
	MM_S16 val2 = (MM_S16)0;
#endif
	MM_U32 stPty = (MM_U32)0;
	MM_U32 stPtx = (MM_U32)0;
	MM_U32 dstheight = (MM_U32)0;
	MM_U32 dstWidth = (MM_U32)0;
	MM_U32 index = (MM_U32)0;
	MM_U32 Maxindex = (MM_U32)0;

	stPty = (box->sty);
	stPtx = (box->stx);
	dstheight = (box->wdy);
	dstWidth = (box->wdx);
	Maxindex = dstheight*dstWidth*(MM_U32)(3);

	memset(tmp_img, 0x00, sizeof(MM_U08)*dstheight*dstWidth*(MM_U32)(3));


	for (y = stPty; y<(MM_U32)(stPty + dstheight); y++) {
		for (x = stPtx; x<(MM_U32)(stPtx + dstWidth); x++) {

			yy = (y - stPty);
			xx = (x - stPtx);
			if (ID == (MM_U08)(0)) {

#if (sub_ROI)
				val1 = (MM_S16)(param->roi_cropped[1]) + (MM_S16)((MM_S16)(param->roi_cropped[0])*(MM_S16)(xx));
				val2 = (MM_S16)(param->roi_cropped[3]) + (MM_S16)((MM_S16)(param->roi_cropped[2])*(MM_S16)(xx));
#endif
				if (((yy < (MM_U32)(720)) && (yy >(MM_U32)(0))) && ((xx >(MM_U32)(0)) && (xx < (MM_U32)(1280))) && ((x < (MM_U32)(1280)) &&
					(x >(MM_U32)(0))) && ((y < (MM_U32)(720)) && (y >(MM_U32)(0))) && (/*(dstWidth <= (MM_U32)(CRP_WD)) &&*/ (dstWidth >(MM_U32)(0))) && (width <= (MM_U32)(1280 + 32))) {

					index = (MM_U32)(((MM_U32)(3)*((yy *dstWidth) + xx)));

					if ((index <= Maxindex) && ((index + (MM_U32)(1)) <= Maxindex) && ((index + (MM_U32)(2)) <= Maxindex)) {
#if (sub_ROI)
						tmp_img[index + (MM_U32)(0)] = (MM_U08)((((MM_S16)(yy) > val1) && ((MM_S16)(yy) < val2)) ? (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(3))] : (MM_U08)(0));
						tmp_img[index + (MM_U32)(1)] = (MM_U08)((((MM_S16)(yy) > val1) && ((MM_S16)(yy) < val2)) ? (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(2))] : (MM_U08)(0));
						tmp_img[index + (MM_U32)(2)] = (MM_U08)((((MM_S16)(yy) > val1) && ((MM_S16)(yy) < val2)) ? (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(1))] : (MM_U08)(0));
#else
						tmp_img[index + (MM_U32)(0)] = (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(3))];
						tmp_img[index + (MM_U32)(1)] = (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(2))];
						tmp_img[index + (MM_U32)(2)] = (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(1))];
#endif
					}
				}

			}
			else if (ID == (MM_U08)(1)) {

#if (sub_ROI)
				val1 = (MM_S16)(param->roi_cropped[5]) + (MM_S16)((MM_S16)(param->roi_cropped[4])*(MM_S16)(xx));
				val2 = (MM_S16)(param->roi_cropped[7]) + (MM_S16)((MM_S16)(param->roi_cropped[6])*(MM_S16)(xx));
#endif
				if (((yy < (MM_U32)(720)) && (yy >(MM_U32)(0))) && ((xx > (MM_U32)(0)) && (xx < (MM_U32)(1280))) && ((x < (MM_U32)(1280)) &&
					(x >(MM_U32)(0))) && ((y < (MM_U32)(720)) && (y >(MM_U32)(0))) && (/*(dstWidth <= (MM_U32)(CRP_WD)) &&*/ (dstWidth >(MM_U32)(0))) && (width <= (MM_U32)(1280 + 32))) {

					index = (MM_U32)(((MM_U32)(3)*((yy *dstWidth) + xx)));

					if ((index <= Maxindex) && ((index + (MM_U32)(1)) <= Maxindex) && ((index + (MM_U32)(2)) <= Maxindex)) {
#if (sub_ROI)
						tmp_img[index + (MM_U32)(0)] = (MM_U08)((((MM_S16)(yy) > val1) && ((MM_S16)(yy) < val2)) ? (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(3))] : (MM_U08)(0));
						tmp_img[index + (MM_U32)(1)] = (MM_U08)((((MM_S16)(yy) > val1) && ((MM_S16)(yy) < val2)) ? (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(2))] : (MM_U08)(0));
						tmp_img[index + (MM_U32)(2)] = (MM_U08)((((MM_S16)(yy) > val1) && ((MM_S16)(yy) < val2)) ? (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(1))] : (MM_U08)(0));
#else
						tmp_img[index + (MM_U32)(0)] = (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(3))];
						tmp_img[index + (MM_U32)(1)] = (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(2))];
						tmp_img[index + (MM_U32)(2)] = (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(1))];
#endif

					}

				}

			}
			else if (ID == (MM_U08)(2)) {

#if (sub_ROI)
				val1 = (MM_S16)(param->roi_cropped[9]) + (MM_S16)((MM_S16)(param->roi_cropped[8])*(MM_S16)(xx));
				val2 = (MM_S16)(param->roi_cropped[11]) + (MM_S16)((MM_S16)(param->roi_cropped[10])*(MM_S16)(xx));
#endif
				if (((yy < (MM_U32)(720)) && (yy >(MM_U32)(0))) && ((xx > (MM_U32)(0)) && (xx < (MM_U32)(1280))) && ((x < (MM_U32)(1280)) &&
					(x >(MM_U32)(0))) && ((y < (MM_U32)(720)) && (y >(MM_U32)(0))) && (/*(dstWidth <= (MM_U32)(CRP_WD)) &&*/ (dstWidth >(MM_U32)(0))) && (width <= (MM_U32)(1280 + 32))) {

					index = (MM_U32)(((MM_U32)(3)*((yy *dstWidth) + xx)));

					if ((index <= Maxindex) && ((index + (MM_U32)(1)) <= Maxindex) && ((index + (MM_U32)(2)) <= Maxindex)) {
#if (sub_ROI)
						tmp_img[index + (MM_U32)(0)] = (MM_U08)((((MM_S16)(yy) > val1) && ((MM_S16)(yy) < val2)) ? (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(3))] : (MM_U08)(0));
						tmp_img[index + (MM_U32)(1)] = (MM_U08)((((MM_S16)(yy) > val1) && ((MM_S16)(yy) < val2)) ? (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(2))] : (MM_U08)(0));
						tmp_img[index + (MM_U32)(2)] = (MM_U08)((((MM_S16)(yy) > val1) && ((MM_S16)(yy) < val2)) ? (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(1))] : (MM_U08)(0));
#else
						tmp_img[index + (MM_U32)(0)] = (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(3))];
						tmp_img[index + (MM_U32)(1)] = (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(2))];
						tmp_img[index + (MM_U32)(2)] = (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(1))];
#endif

					}

				}

			}
			else if (ID == (MM_U08)(3)) {
#if (sub_ROI)
				val1 = (MM_S16)(param->roi_cropped[13]) + (MM_S16)((MM_S16)(param->roi_cropped[12])*(MM_S16)(xx));
				val2 = (MM_S16)(param->roi_cropped[15]) + (MM_S16)((MM_S16)(param->roi_cropped[14])*(MM_S16)(xx));
#endif
				if (((yy < (MM_U32)(720)) && (yy >(MM_U32)(0))) && ((xx > (MM_U32)(0)) && (xx < (MM_U32)(1280))) && ((x < (MM_U32)(1280)) &&
					(x >(MM_U32)(0))) && ((y < (MM_U32)(720)) && (y >(MM_U32)(0))) && (/*(dstWidth <= (MM_U32)(CRP_WD)) &&*/ (dstWidth >(MM_U32)(0))) && (width <= (MM_U32)(1280 + 32))) {

					index = (MM_U32)(((MM_U32)(3)*((yy *dstWidth) + xx)));

					if ((index <= Maxindex) && ((index + (MM_U32)(1)) <= Maxindex) && ((index + (MM_U32)(2)) <= Maxindex)) {
#if (sub_ROI)
						tmp_img[index + (MM_U32)(0)] = (MM_U08)((((MM_S16)(yy) > val1) && ((MM_S16)(yy) < val2)) ? (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(3))] : (MM_U08)(0));
						tmp_img[index + (MM_U32)(1)] = (MM_U08)((((MM_S16)(yy) > val1) && ((MM_S16)(yy) < val2)) ? (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(2))] : (MM_U08)(0));
						tmp_img[index + (MM_U32)(2)] = (MM_U08)((((MM_S16)(yy) > val1) && ((MM_S16)(yy) < val2)) ? (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(1))] : (MM_U08)(0));
#else
						tmp_img[index + (MM_U32)(0)] = (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(3))];
						tmp_img[index + (MM_U32)(1)] = (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(2))];
						tmp_img[index + (MM_U32)(2)] = (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* width) + (MM_U32)(x))) + (MM_U32)(1))];
#endif
					}

				}
			}
			else {
			}

		} //ted
	}

	cvtC3toC1(output_img, tmp_img, (MM_U16)dstheight, (MM_U16)dstWidth);
}


static MM_U08 dst_img[MAX_ROI_SIZE];

void  erosion(
				MM_U08 src_img[],	/* [in/out] image*/
				MM_U16 iheight,		/* [in] height of image */
				MM_U16 iWidth)		/* [in] weight of iamge */
{
	MM_U16 h = (MM_U16)0;
	MM_U16 w = (MM_U16)0;
	MM_U08 vc = (MM_U08)0;
	MM_U16 v1 = (MM_U16)0;
	MM_U16 v2 = (MM_U16)0;
	MM_U16 v3 = (MM_U16)0;
	MM_U16 v4 = (MM_U16)0;
	MM_U16 v5 = (MM_U16)0;
	MM_U16 v6 = (MM_U16)0;
	MM_U16 v7 = (MM_U16)0;
	MM_U16 v8 = (MM_U16)0;
	MM_U08 cnt = (MM_U08)0;

	memset(dst_img, 0x00, sizeof(MM_U08)*MAX_ROI_SIZE);

	for (h = (MM_U08)(1); h < (iheight - (MM_U08)(1)); h++) {
		for (w = (MM_U08)(1); w < (iWidth - (MM_U08)(1)); w++) {
			if ((h >(iheight - (MM_U08)(2))) || (w >(iWidth - (MM_U08)(2)))) { break; }

			cnt = (MM_U08)(0);
			v8 = src_img[((h - (MM_U08)(1))*iWidth) + (w - (MM_U08)(1))]; v1 = src_img[((h - (MM_U08)(1))*iWidth) + w]; v2 = src_img[((h - (MM_U08)(1)) * iWidth) + (w + (MM_U08)(1))];
			v7 = src_img[(h   *iWidth) + (w - (MM_U08)(1))]; vc = src_img[(h   * iWidth) + w]; v3 = src_img[(h *iWidth) + (w + (MM_U08)(1))];
			v6 = src_img[((h + (MM_U08)(1))*iWidth) + (w - (MM_U08)(1))]; v5 = src_img[((h + (MM_U08)(1))*iWidth) + w]; v4 = src_img[((h + (MM_U08)(1))*iWidth) + (w + (MM_U08)(1))];

			if ((MM_U08)(0) == vc) {
				if (v8 != (MM_U08)(255)) { cnt++; } //else {cnt;}
				if (v1 != (MM_U08)(255)) { cnt++; } //else {cnt;}
				if (v2 != (MM_U08)(255)) { cnt++; } //else {cnt;}
				if (v7 != (MM_U08)(255)) { cnt++; } //else {cnt;}
				if (v3 != (MM_U08)(255)) { cnt++; } //else {cnt;}
				if (v6 != (MM_U08)(255)) { cnt++; } //else {cnt;}
				if (v5 != (MM_U08)(255)) { cnt++; } //else {cnt;}
				if (v4 != (MM_U08)(255)) { cnt++; } //else {cnt;}

			}
			dst_img[(h*iWidth) + w] = (cnt != (MM_U08)(8)) ? (MM_U08)(255) : (MM_U08)(0);
		}
	}
	memcpy(src_img, dst_img, sizeof(MM_U08)*MAX_ROI_SIZE);
}


static MM_S32 histData[256];
static MM_FLOAT histPData[256];
static MM_FLOAT histCData[256];
static MM_FLOAT sigma[256];
static MM_FLOAT f[256];

MM_S16  gray2binary_otsu(
						MM_U08 res_Image[],		/* [out] binary iamge */
						MM_U08 gray_image[],	/* [in] soruce iamge */
						MM_U16 height,			/* [in] height of image */
						MM_U16 width)			/* [in] width of image */
{
	MM_U16 i = (MM_U16)0;
	MM_U16 j = (MM_U16)0;
	MM_U08 pixel = (MM_U08)0;
	MM_S16 error_code = MM_TRUE;
	MM_FLOAT F = 0.0f;
	MM_FLOAT t = 0.0f;
	MM_FLOAT sum = 0.0f;
	MM_FLOAT maxSigma = 0.0f;
	MM_U16 threshold = (MM_U16)0;

	memset(histData, 0x00, sizeof(MM_S32)*(MM_U32)(256));
	memset(histPData, 0x00, sizeof(MM_FLOAT)*(MM_U32)(256));
	memset(histCData, 0x00, sizeof(MM_FLOAT)*(MM_U32)(256));
	memset(f, 0x00, sizeof(MM_FLOAT)*(MM_U32)(256));
	memset(sigma, 0x00, sizeof(MM_FLOAT)*(MM_U32)(256));

	for (i = 0; i< height; i++)
	{
		for (j = 0; j<width; j++)
		{
			pixel = (MM_U08)gray_image[(i*width) + j];
			if ((pixel == (MM_U08)(0)) || (pixel == (MM_U08)(255)))
			{
				continue;
			}
			histData[pixel]++;
		}
	}

	sum = (MM_FLOAT)(0.0f);

	for (i = 0; i<(MM_U16)(256); i++)
	{
		sum += (MM_FLOAT)((histData[i]));
	}

	for (i = 0; i<(MM_U16)(256); i++)
	{
		histPData[i] = (MM_FLOAT)(histData[i]) / (sum + (MM_FLOAT)(1));
	}

	for (i = 0; i<(MM_U16)(256); i++)
	{
		for (j = 0; j <= i; j++)
		{
			histCData[i] += histPData[j];
		}
	}

	for (i = 0; i<(MM_U16)(256); i++)
	{
		for (j = 0; j <= i; j++)
		{
			f[i] += ((MM_FLOAT)(j)* histPData[j]);
		}
	}

	F = f[255];
	t = (MM_FLOAT)(0.0);

	for (i = 0; i<(MM_U16)(256); i++)
	{
		if (histCData[i] < (MM_FLOAT)1)
		{
			t = (histCData[i] - (histCData[i] * histCData[i]));
		}

		if (t == 0.0f)
		{
			t = (MM_FLOAT)(0.000001f);
		}

		if ((F < (256.0f)) && (f[i] < (256.0f)) && (histCData[i] < (1.0f)) && ((t >= 0.000001f) && (t <= 1.0f)))
		{
			sigma[i] = (((F*histCData[i]) - f[i]) * ((F * histCData[i]) - f[i])) / t;
		}
	}

	maxSigma = (-99999999.0f);
	threshold = (MM_U16)(0);

	for (i = 0; i<(MM_U16)(256); i++)
	{
		if (sigma[i] > maxSigma)
		{
			threshold = i;
			maxSigma = sigma[i];
		}
	}

	//*otsu_threshold = threshold;
	gray2binary_unity(res_Image, gray_image, height, width, threshold);

	if (threshold == (MM_U16)(0))
	{
		error_code = MM_FALSE;

	}

	return error_code;
}


void  gray2binary_unity(
						MM_U08 result_image[],		/* [out] binary image */
						MM_U08 gray_image[],	/* [in] gray image */
						MM_U16 height,				/* [in] height of image */
						MM_U16 width,				/* [in] width of image */
						MM_U16 threshold)			/* [in] binary thershold value */
{
	MM_U16 i = (MM_U16)0;
	MM_U16 j = (MM_U16)0;

	memset(result_image, 0x00, sizeof(MM_U08)*height*width);

	for (i = 0; i<height; i++)
	{
		for (j = 0; j<width; j++)
		{
			if ((height != 1) && ((i == 0) || (j == 0) || (i == (height - 1)) || (j == (width - 1))))
			{
				result_image[(i*width) + j] = (MM_U08)(255);
			}
			else
			{
				if (gray_image[(i*width) + j] >= (MM_U08)(threshold))
				{
					result_image[(i*width) + j] = (MM_U08)(255);
				}
				else
				{
					result_image[(i*width) + j] = (MM_U08)(0);
				}
			}
		}//for(j)
	}//for(i)

}

void factory_calibration_ROI_setting(
									ROI_BOX *roi,					/* [out] ROI imformation */
									CALIBRATION_PARAM *param,	/* [in] calibration parameters */
									MM_U08 ID)						/* [in] camera number */
{
	// Front camera
	if (ID == (MM_U08)(0)) {
		/////////////////////////////////////
		roi[0].stx = (MM_U16)(param->roi_points[0]);	roi[0].sty = (MM_U16)(param->roi_points[1]);
		roi[0].wdx = (MM_U16)(param->roi_width);		roi[0].wdy = (MM_U16)(param->roi_height);
		roi[1].stx = (MM_U16)(param->roi_points[2]);	roi[1].sty = (MM_U16)(param->roi_points[3]);
		roi[1].wdx = (MM_U16)(param->roi_width);		roi[1].wdy = (MM_U16)(param->roi_height);
	}

	// Raer camera
	if (ID == (MM_U08)(1)) {
		////////////////////////////////////
		roi[0].stx = (MM_U16)(param->roi_points[4]);	roi[0].sty = (MM_U16)(param->roi_points[5]);
		roi[0].wdx = (MM_U16)(param->roi_width);		roi[0].wdy = (MM_U16)(param->roi_height);
		roi[1].stx = (MM_U16)(param->roi_points[6]);	roi[1].sty = (MM_U16)(param->roi_points[7]);
		roi[1].wdx = (MM_U16)(param->roi_width);		roi[1].wdy = (MM_U16)(param->roi_height);
	}

	// Left camera
	if (ID == (MM_U08)(2)) {
		////////////////////////////////////
		roi[0].stx = (MM_U16)(param->roi_points[8]);	roi[0].sty = (MM_U16)(param->roi_points[9]);
		roi[0].wdx = (MM_U16)(param->roi_width);		roi[0].wdy = (MM_U16)(param->roi_height);
		roi[1].stx = (MM_U16)(param->roi_points[10]);	roi[1].sty = (MM_U16)(param->roi_points[11]);
		roi[1].wdx = (MM_U16)(param->roi_width);		roi[1].wdy = (MM_U16)(param->roi_height);
	}

	// Right camera
	if (ID == (MM_U08)(3)) {
		////////////////////////////////////
		roi[0].stx = (MM_U16)(param->roi_points[12]);	roi[0].sty = (MM_U16)(param->roi_points[13]);
		roi[0].wdx = (MM_U16)(param->roi_width);		roi[0].wdy = (MM_U16)(param->roi_height);
		roi[1].stx = (MM_U16)(param->roi_points[14]);	roi[1].sty = (MM_U16)(param->roi_points[15]);
		roi[1].wdx = (MM_U16)(param->roi_width);		roi[1].wdy = (MM_U16)(param->roi_height);

	}
}

void service_calibration_ROI_setting(
									ROI_BOX *roi,					/* [out] ROI imformation */
									CALIBRATION_PARAM *param,	/* [in] calibration parameters */
									MM_U08 ID)						/* [in] camera number */
{
	// Front camera
	if (ID == (MM_U08)(0)) {
		/////////////////////////////////////
		roi[0].stx = (MM_U16)(param->service_roi_points[0]);	roi[0].sty = (MM_U16)(param->service_roi_points[1]);
		roi[0].wdx = (MM_U16)(param->service_roi_width);		roi[0].wdy = (MM_U16)(param->service_roi_height);
	}

	// Raer camera
	if (ID == (MM_U08)(1)) {
		////////////////////////////////////
		roi[0].stx = (MM_U16)(param->service_roi_points[2]);	roi[0].sty = (MM_U16)(param->service_roi_points[3]);
		roi[0].wdx = (MM_U16)(param->service_roi_width);		roi[0].wdy = (MM_U16)(param->service_roi_height);
	}

	// Left camera
	if (ID == (MM_U08)(2)) {
		////////////////////////////////////
		roi[0].stx = (MM_U16)(param->service_roi_points[4]);	roi[0].sty = (MM_U16)(param->service_roi_points[5]);
		roi[0].wdx = (MM_U16)(param->service_roi_width);		roi[0].wdy = (MM_U16)(param->service_roi_height);
	}

	// Right camera
	if (ID == (MM_U08)(3)) {
		////////////////////////////////////
		roi[0].stx = (MM_U16)(param->service_roi_points[6]);	roi[0].sty = (MM_U16)(param->service_roi_points[7]);
		roi[0].wdx = (MM_U16)(param->service_roi_width);		roi[0].wdy = (MM_U16)(param->service_roi_height);
	}
}

