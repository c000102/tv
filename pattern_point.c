#include "image_enhancement.h"
#include "point_extraction.h"
#include "feature_extraction.h"
#include "post_processing.h"
#include "pattern_point.h"

CFUNC_BEGIN_DECLS

static MM_U08 enhanImgC1[MAX_ROI_SIZE * 2];
static MM_U08 binimage[MAX_ROI_SIZE];
static MM_FLOAT srcPts[8];
static ROI_BOX roi_box1[2];

static MM_FLOAT fn_point[FTPT_SIZE];
static MM_FLOAT ft_pt[FTPT_SIZE];
static BORDERFOLLOW stBorderInfo[REGION_NUM];

static MM_S16  LR_Pattern(	MM_FLOAT *corPoints,				/* [out] corner points */
							MM_U08 *ImgC1,						/* [in] source image at 1 chanel */
							MM_U08 side,						/* [in] let or right pattern on image */
							CALIBRATION_PARAM *param);	/* [in] calibration parameters */

CFUNC_END_DECLS


MM_S16  find_pattern_points(MM_FLOAT *corPoints,				/* [out] corner points */
							MM_U08 *srcImgC4,					/* [in] source image at 4 chanel */
							MM_U08 ID,							/* [in] camear type */
							MM_U08 CalibType,					/* [in] calibration type(factory, service) */
							CALIBRATION_PARAM *cal_param)	/* [in] calibration parameters */
{
	MM_S16 ecodeL = (MM_S16)SUCCESS;
	MM_S16 ecodeR = (MM_S16)SUCCESS;
	MM_S16 ecode  = (MM_S16)SUCCESS;
	MM_U32 crp_img_szie = (MM_U32)0;


	crp_img_szie = (MM_U16)(cal_param->roi_height) * (MM_U16)(cal_param->roi_width);

	memset(roi_box1, 0x00, sizeof(ROI_BOX)*(MM_U08)(2));

	Enhancement(enhanImgC1, roi_box1, srcImgC4, HEIGHT, WIDTH, ID, CalibType, cal_param);

	ecodeL = LR_Pattern(corPoints, enhanImgC1, /*ft_pt,*/ LEFT_PATTERN, cal_param);

	if (CalibType == FACTORY_CALIB) /* factory calibration */
	{
		ecodeR = LR_Pattern(corPoints, enhanImgC1 + (crp_img_szie), /*ft_pt,*/ RIGHT_PATTERN, cal_param);

		if ((ecodeL == SUCCESS) && (ecodeR == SUCCESS)) /* succeeded */
		{
			ecode = SUCCESS;
		}
		else /* failed */
		{
			if ((ecodeL == ERROR_FEATURE_UNIT_CORNER) && (ecodeR == ERROR_FEATURE_UNIT_CORNER))
			{
				ecode = ERROR_FEATURE_PATTERN_CAMERA_FAULT;
			}
			else
			{
				if (is_feature_unit_error(ecodeL) && is_feature_unit_error(ecodeR))
				{
					ecode = ERROR_FEATURE_PATTERN_BOTH;
				}
				else if (is_feature_unit_error(ecodeL))
				{
					ecode = ERROR_FEATURE_PATTERN_LEFT;
				}
				else
				{
					ecode = ERROR_FEATURE_PATTERN_RIGHT;
				}
			}
		}
	}
	else  /* service calibration */
	{
		if (ecodeL == ERROR_FEATURE_UNIT_CORNER)
		{
			ecode = ERROR_FEATURE_PATTERN_CAMERA_FAULT;
		}
		else
		{
			if (is_feature_unit_error(ecodeL))
			{
				ecode = ERROR_FEATURE_PATTERN_SINGLE;
			}
			else
			{
				ecode = SUCCESS;
			}
		}
	}

	return ecode;
}

MM_S16  LR_Pattern(	MM_FLOAT *corPoints,				/* [out] corner points */
					MM_U08 *ImgC1,						/* [in] source image at 1 chanel */
					MM_U08 side,						/* [in] let or right pattern on image */
					CALIBRATION_PARAM *param)		/* [in] calibration parameters */
{
	MM_U08 i = (MM_U08)0;
	MM_S16 ecode = MM_TRUE;
	MM_U08 border_num = (MM_U08)0;
	MM_U16 fn_num = (MM_U16)0;
	MM_U16 point_cnt = (MM_U16)0;
	MM_U16 h = (MM_U16)0;
	MM_U16 w = (MM_U16)0;
	MM_U32 crp_img_szie = (MM_U32)0;

	h = (MM_U16)(param->roi_height);
	w = (MM_U16)(param->roi_width);
	crp_img_szie = (MM_U16)(param->roi_height) * (MM_U16)(param->roi_width);

	memset(binimage, 0x00, sizeof(MM_U08)*crp_img_szie);
	memset(fn_point, 0x00, sizeof(MM_FLOAT)*(FTPT_SIZE));
	memset(ft_pt, 0x00, sizeof(MM_FLOAT)*(FTPT_SIZE));

	memset(srcPts, 0x00, sizeof(MM_FLOAT)*(MM_U08)(8));
	memset(stBorderInfo, 0x00, sizeof(BORDERFOLLOW)*REGION_NUM);

	ecode = GetCorner_new(ft_pt, &point_cnt, ImgC1, h, w, param);

	if (ecode == MM_TRUE)
	{
		ecode = PixelAccuracy_new(fn_point, &fn_num, ImgC1, (MM_U16)(param->roi_height), (MM_U16)(param->roi_width), ft_pt, point_cnt);

		if (ecode == MM_TRUE)
		{
			gray2binary_otsu(binimage, ImgC1, (MM_U16)(param->roi_height), (MM_U16)(param->roi_width));
			erosion(binimage, (MM_U16)(param->roi_height), (MM_U16)(param->roi_width));
			ecode = borderFollowing(stBorderInfo, &border_num, binimage, (MM_U16)(param->roi_height), (MM_U16)(param->roi_width));

			if (ecode == MM_TRUE)
			{
				point_cnt = 0;
				memset(ft_pt, 0x00, sizeof(MM_FLOAT)*(FTPT_SIZE));
				ecode = calc_min_dist(ft_pt, &point_cnt, stBorderInfo, border_num, fn_point, fn_num);

				if (ecode == MM_TRUE)
				{
					fn_num = 0;
					memset(fn_point, 0x00, sizeof(MM_FLOAT)*(FTPT_SIZE));
					ecode = overlapPoints(fn_point, &fn_num, ft_pt, point_cnt);

					if (ecode == MM_TRUE)
					{
						point_cnt = 0;
						memset(ft_pt, 0x00, sizeof(MM_FLOAT)*(FTPT_SIZE));
						ecode = overlapPoints(ft_pt, &point_cnt, fn_point, fn_num);

						if (ecode == MM_TRUE)
						{
							fn_num = 0;
							memset(fn_point, 0x00, sizeof(MM_FLOAT)*(FTPT_SIZE));
							CirIntFR13x13(fn_point, &fn_num, ImgC1, (MM_U16)(param->roi_height), (MM_U16)(param->roi_width), ft_pt, point_cnt);

							if (fn_num == (MM_U16)(4))
							{
								for (i = (MM_U08)(0); i<(MM_U08)(4); i++) /* change position coordinates */
								{
									srcPts[((MM_U08)(2)*i) + (MM_U08)(0)] = (MM_FLOAT)(fn_point[((MM_U08)(2)*i) + (MM_U08)(0)] + (MM_FLOAT)(roi_box1[side].stx));
									srcPts[((MM_U08)(2)*i) + (MM_U08)(1)] = (MM_FLOAT)(fn_point[((MM_U08)(2)*i) + (MM_U08)(1)] + (MM_FLOAT)(roi_box1[side].sty));
								}

								change_point_order(&corPoints[side * 8], srcPts); /* point ordering  */

								ecode = SUCCESS;
							}
							else
							{
								ecode = ERROR_FEATURE_UNIT_POINT_NUM;
							}
						}
						else
						{
							ecode = ERROR_FEATURE_UNIT_OVERLAP;
						}
					}
					else
					{
						ecode = ERROR_FEATURE_UNIT_OVERLAP;
					}
				}
				else
				{
					ecode = ERROR_FEATURE_UNIT_CALC_MIN;
				}
			}
			else
			{
				ecode = ERROR_FEATURE_UNIT_BORDER_FOLLOWING;
			}
		}
		else
		{
			ecode = ERROR_FEATURE_UNIT_PIXEL_ACCURACY;
		}
	}
	else
	{
		ecode = ERROR_FEATURE_UNIT_CORNER;
	}


	return ecode;
}
