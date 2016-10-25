#include "calibration.h"
#include "pattern_point.h"

#include "describe_topview.h"


CFUNC_BEGIN_DECLS

CALIBRATION_PARAM all_one_cal_param; /* global declaration becasue of stack size on an embededd system */

MM_S16  check_swp_parameters(MM_U08 *msg_result,	
							 CAR_PARAM *car_param,	
							 CALIBRATION_PARAM *cal_param);

MM_S16  check_range_of_feature_points(MM_FLOAT *feature_points,
									  CALIBRATION_PARAM  *cal_param,
									  MM_U08 cal_type);

static MM_S16  check_swp_intrinsic_parameters(CALIBRATION_PARAM *cal_param);	
static MM_S16  check_swp_factory_parameters(CALIBRATION_PARAM *cal_param);	
static MM_S16  check_swp_service_parameters(CALIBRATION_PARAM *cal_param);	

static void  make_message_for_factory_pattern(	MM_U08 *msg_result,	
												MM_S16 ecode,	
												MM_U08 cam_idx);

static void regularize_world_coordinates(CALIBRATION_PARAM *cal_param, 
										 CAR_PARAM *car_param);

/*declaration of local functions for factory calibration */

static void  factory_input_for_pose_estimation(HvCameraData camData[],
												MM_FLOAT(feature_points[])[16],
												CALIBRATION_PARAM *cal_param);

static MM_S16  factory_output_from_pose_estimation(EXTRINSIC_PARAM *ext_param,
												 HvCameraData *cam_data);


static void  factory_default_when_error(EXTRINSIC_PARAM extrinsicParam[], 
	                                    MM_U08           msg_header,
										CALIBRATION_PARAM *cal_param);

static void  features_for_double_patterns(MM_U08 *front, 
										MM_U08 *rear, 
										MM_U08 *left, 
										MM_U08 *right, 
										MM_U16 height, 
										MM_U16 width, 
										MM_FLOAT (*pResult)[16],
										MM_U08  *msg_result,
										CALIBRATION_PARAM *param);

#define check_num (0)
#if check_num
void check_number(CALIBRATION_PARAM *param);
#endif

CFUNC_END_DECLS

/* --------------------------------------------------------------------------
                           factory calibration 
---------------------------------------------------------------------------- */


MM_S16  factory_calibration(MM_U08				*front_img,		  /* [in] front camera input, color space: RGBA, byte ordering: ABGR */
							MM_U08				*rear_img,		  /* [in] rear camera input */
							MM_U08				*left_img,		  /* [in] left camera input */
							MM_U08				*right_img,		  /* [in] right camera input */
							MM_U16				img_height,		  /* [in]  image height */
							MM_U16				img_width,		  /* [in]  image width */
							CAR_PARAM			*car_param,       /* [in]  car information */
							CALIBRATION_PARAM	*cal_param,		  /* [in]  information about calibraiton*/
							EXTRINSIC_PARAM		*ext_param,		  /* [out] camera extrinsic parameters */
							MM_U08				*msg_result)      /* [out] message for operation result*/
{
	HvCameraData cam_data[CAM_TOTAL];
	MM_FLOAT     feature_points[CAM_TOTAL][16];
	MM_S32		 pi_ecode = (MM_S32)HV_OK;
	MM_S32		 pm_ecode = (MM_S32)HV_OK;
	MM_S16       fo_ecode = (MM_S16)SUCCESS;
	MM_S16       fp_ecode = (MM_S16)SUCCESS;
	MM_S16       cam_idx  = (MM_S16)0;

	memset(ext_param, 0x00, sizeof(EXTRINSIC_PARAM)*(MM_U08)(CAM_TOTAL));
	memset(msg_result, 0x00, sizeof(MM_U08)*(MM_U08)(5));
	memset(cam_data, 0x00, sizeof(HvCameraData)*(MM_U08)CAM_TOTAL);
	memset((void*)feature_points, 0x00, sizeof(MM_FLOAT)*(MM_U08)64);

	/* set the type of calibration */
	msg_result[0] = (MM_U08)(msg_result[0] | FACTORY_CAL_TYPE_MASK);

#if (check_num)
	{
		MM_S16		 debug_ecode = (MM_S16)0;
		check_number(cal_param);
		debug_ecode = check_swp_parameters(msg_result, car_param, cal_param);
		PRINTF("\n==============================\n");
		PRINTF("debug ccode: %d\n", debug_ecode);
		PRINTF("\n==============================\n");
	}
#else
	check_swp_parameters(msg_result, car_param, cal_param);
#endif


	if(hvPositive(msg_result[0] & SWP_BIT_MASK) == (MM_U08)1) /* succeeded */
	{
		regularize_world_coordinates(cal_param, car_param);

		features_for_double_patterns(front_img,
									rear_img,
									left_img,
									right_img,
									(MM_U16)(cal_param->height),
									(MM_U16)(cal_param->width), 
									feature_points, 
									msg_result,
									cal_param);

		factory_input_for_pose_estimation(cam_data, feature_points, cal_param);

		for (cam_idx = CAM_FRONT; cam_idx <= CAM_RIGHT; cam_idx++)
		{
			if(hvPositive(msg_result[0] & CAM_BIT_MASK(cam_idx)) == (MM_U08)1) /* succeeded */
			{
				fp_ecode = check_range_of_feature_points(feature_points[cam_idx],cal_param, FACTORY_CALIB);

				if (hvSucceeded(fp_ecode))
				{
					pi_ecode = hvCalibInit(&cam_data[cam_idx]);

					if (hvSucceeded(pi_ecode))
					{
						pm_ecode = hvCalibApply(&cam_data[cam_idx], 1);

						if (hvSucceeded(pm_ecode))
						{
							fo_ecode = factory_output_from_pose_estimation(&ext_param[cam_idx], &cam_data[cam_idx]);

							/* TopView Display by YJ.Park 161013*/
							describe_topview(
								front_img,
								rear_img,
								left_img,
								right_img,
								(MM_U16)(cal_param->height),
								(MM_U16)(cal_param->width),
								cal_param,
								&cam_data[cam_idx],
								cam_idx);

							if (hvSucceeded(fo_ecode))
							{
								convert_IN_to_Fujitsu(&ext_param[cam_idx]);
								msg_result[0] = (MM_U08)(msg_result[0] | CAM_BIT_MASK(cam_idx));/* success message */
								msg_result[cam_idx + 1] = (MM_U08)(0x01);
							}
							else
							{
								msg_result[0] = (MM_U08)(msg_result[0] & ~CAM_BIT_MASK(cam_idx)); /* failure message */
								msg_result[cam_idx + 1] = (MM_U08)(0x06);
							}
						}
						else /* pose estimation error */
						{
							msg_result[0] = (MM_U08)(msg_result[0] & ~CAM_BIT_MASK(cam_idx)); /* failure message */
							msg_result[cam_idx + 1] = (MM_U08)(0x06);
						}
					}
					else  /* pose initialization error */
					{
						msg_result[0] = (MM_U08)(msg_result[0] & ~CAM_BIT_MASK(cam_idx)); /* failure message */
						msg_result[cam_idx + 1] = (MM_U08)(0x06);
					}

				}
				else /* check point range */
				{
					msg_result[0] = (MM_U08)(msg_result[0] & ~CAM_BIT_MASK(cam_idx)); /* failure message */
					msg_result[cam_idx + 1] = 0x04;
				}
			}
		}	
	}

	factory_default_when_error(ext_param, msg_result[0],cal_param);

	return (MM_S16)1; /* notify that the operantoin has been completed, not error.*/
}


MM_S16  check_range_of_feature_points(MM_FLOAT *feature_points, 
	                                  CALIBRATION_PARAM  *cal_param,  
	                                  MM_U08 cal_type)
{
	MM_S16 ecode = SUCCESS;
	MM_S16 i = (MM_S16)0;
	MM_S16 repeat_count = (MM_S16)0;

	repeat_count = (cal_type == FACTORY_CALIB) ? (MM_S16)cal_param->num_point : 
		                                         (MM_S16)cal_param->service_num_point;

	for (i = 0; i < repeat_count; i++)
	{
		if ((feature_points[2*i] <= 0.0f)       || 
			(feature_points[2*i] >= WIDTH)      ||
			(feature_points[2*i + 1] <= 0.0f)   ||
			(feature_points[2*i + 1] >= HEIGHT))
		{
			ecode = ERROR_FEATURE_UNIT_INVALID_POINT;
			break;
		}
	}
	return ecode;
}

void regularize_world_coordinates(CALIBRATION_PARAM *cal_param,/*[out]*/
								  CAR_PARAM *car_param)        /*[in]*/
{
	MM_S32 i = (MM_S32)0;
	MM_FLOAT difference = 0.0f;
	MM_FLOAT half_length = 0.0f;

	half_length = car_param->length / 2.0f;
	difference =  half_length - car_param->front_overhang;

	for (i = 0; i < 24; i++)
	{
		cal_param->world_coordinates[i][1] -= difference;
	}

	// 차량 중심의 world coordinates를 사용하면 원점을 이동시킬 필요 없기 때문에 초기화 한다.
	cal_param->oy = 0.0f;
}

HvCameraData cam_data[CAM_TOTAL];
HvCameraData one_cam_data;

void  factory_input_for_pose_estimation(HvCameraData camData[],
										MM_FLOAT (feature_points[])[16],
										CALIBRATION_PARAM *cal_param)
{
	MM_U08 i = (MM_U08)0;
	MM_FLOAT wpts[24][2];

	memset(wpts,0x00,sizeof(MM_FLOAT)*(MM_U08)24*(MM_U08)2);
	memset(cam_data, 0x00, sizeof(cam_data));
	memset(&one_cam_data, 0x00, sizeof(one_cam_data));

	cam_data[CAM_FRONT].f     = cal_param->focal_length;
	cam_data[CAM_FRONT].mu    = cal_param->mu;
	cam_data[CAM_FRONT].mv    = cal_param->mv;
	cam_data[CAM_FRONT].cx    = cal_param->cx;
	cam_data[CAM_FRONT].cy    = cal_param->cy;

	memcpy(cam_data[CAM_FRONT].k, cal_param->k,sizeof(MM_FLOAT)*5);
	memcpy(cam_data[CAM_FRONT].d, cal_param->d,sizeof(MM_FLOAT)*9);

	cam_data[CAM_FRONT].ssize_height   =	(MM_S32)(cal_param->height);
	cam_data[CAM_FRONT].ssize_width    =	(MM_S32)(cal_param->width);
	cam_data[CAM_FRONT].world_center.x =	0.0f;// cal_param->ox;
	cam_data[CAM_FRONT].world_center.y =	 0.0f;// cal_param->oy;
	cam_data[CAM_FRONT].numPoints      =	(MM_S32)(cal_param->num_point);

	memcpy(&one_cam_data, &cam_data[CAM_FRONT], sizeof(HvCameraData)); /* R19.1, ver2012 */
	memcpy(&cam_data[CAM_REAR],&one_cam_data,sizeof(HvCameraData));
	memcpy(&cam_data[CAM_LEFT],&one_cam_data,sizeof(HvCameraData));
	memcpy(&cam_data[CAM_RIGHT],&one_cam_data,sizeof(HvCameraData));

	cam_data[CAM_FRONT].camID = CAM_FRONT;
	cam_data[CAM_REAR].camID  = CAM_REAR;
	cam_data[CAM_LEFT].camID  = CAM_LEFT;
	cam_data[CAM_RIGHT].camID = CAM_RIGHT;

	memcpy(wpts, cal_param->world_coordinates,sizeof(MM_FLOAT)*24*2);

	/* world points for FRONT CAMERA */
	cam_data[CAM_FRONT].wpts[0].x = wpts[0][0];
	cam_data[CAM_FRONT].wpts[0].y = wpts[0][1];

	cam_data[CAM_FRONT].wpts[1].x = wpts[1][0];
	cam_data[CAM_FRONT].wpts[1].y = wpts[1][1];

	cam_data[CAM_FRONT].wpts[2].x = wpts[2][0];
	cam_data[CAM_FRONT].wpts[2].y = wpts[2][1];

	cam_data[CAM_FRONT].wpts[3].x = wpts[3][0];
	cam_data[CAM_FRONT].wpts[3].y = wpts[3][1];

	cam_data[CAM_FRONT].wpts[4].x = wpts[4][0];
	cam_data[CAM_FRONT].wpts[4].y = wpts[4][1];

	cam_data[CAM_FRONT].wpts[5].x = wpts[5][0];
	cam_data[CAM_FRONT].wpts[5].y = wpts[5][1];

	cam_data[CAM_FRONT].wpts[6].x = wpts[6][0];
	cam_data[CAM_FRONT].wpts[6].y = wpts[6][1];

	cam_data[CAM_FRONT].wpts[7].x = wpts[7][0];
	cam_data[CAM_FRONT].wpts[7].y = wpts[7][1];

	/* world points for REAR CAMERA */
	cam_data[CAM_REAR].wpts[0].x = wpts[22][0];
	cam_data[CAM_REAR].wpts[0].y = wpts[22][1];
	
	cam_data[CAM_REAR].wpts[1].x = wpts[23][0];
	cam_data[CAM_REAR].wpts[1].y = wpts[23][1];
	
	cam_data[CAM_REAR].wpts[2].x = wpts[20][0];
	cam_data[CAM_REAR].wpts[2].y = wpts[20][1];
	
	cam_data[CAM_REAR].wpts[3].x = wpts[21][0];
	cam_data[CAM_REAR].wpts[3].y = wpts[21][1];
	
	cam_data[CAM_REAR].wpts[4].x = wpts[18][0];
	cam_data[CAM_REAR].wpts[4].y = wpts[18][1];
	
	cam_data[CAM_REAR].wpts[5].x = wpts[19][0];
	cam_data[CAM_REAR].wpts[5].y = wpts[19][1];
	
	cam_data[CAM_REAR].wpts[6].x = wpts[16][0];
	cam_data[CAM_REAR].wpts[6].y = wpts[16][1];
	
	cam_data[CAM_REAR].wpts[7].x = wpts[17][0];
	cam_data[CAM_REAR].wpts[7].y = wpts[17][1];

	/* world points for LEFT CAMERA */
	cam_data[CAM_LEFT].wpts[0].x = wpts[11][0];
	cam_data[CAM_LEFT].wpts[0].y = wpts[11][1];
	
	cam_data[CAM_LEFT].wpts[1].x = wpts[ 8][0];
	cam_data[CAM_LEFT].wpts[1].y = wpts[ 8][1];
	
	cam_data[CAM_LEFT].wpts[2].x = wpts[ 9][0];
	cam_data[CAM_LEFT].wpts[2].y = wpts[ 9][1];
	
	cam_data[CAM_LEFT].wpts[3].x = wpts[10][0];
	cam_data[CAM_LEFT].wpts[3].y = wpts[10][1];
	
	cam_data[CAM_LEFT].wpts[4].x = wpts[ 3][0];
	cam_data[CAM_LEFT].wpts[4].y = wpts[ 3][1];
	
	cam_data[CAM_LEFT].wpts[5].x = wpts[ 0][0];
	cam_data[CAM_LEFT].wpts[5].y = wpts[ 0][1];
	
	cam_data[CAM_LEFT].wpts[6].x = wpts[ 1][0];
	cam_data[CAM_LEFT].wpts[6].y = wpts[ 1][1];
	
	cam_data[CAM_LEFT].wpts[7].x = wpts[ 2][0];
	cam_data[CAM_LEFT].wpts[7].y = wpts[ 2][1];

	/* world points for RIGHT CAMERA */
	cam_data[CAM_RIGHT].wpts[0].x = wpts[5][0];
	cam_data[CAM_RIGHT].wpts[0].y = wpts[5][1];
	
	cam_data[CAM_RIGHT].wpts[1].x = wpts[6][0];
	cam_data[CAM_RIGHT].wpts[1].y = wpts[6][1];
	
	cam_data[CAM_RIGHT].wpts[2].x = wpts[7][0];
	cam_data[CAM_RIGHT].wpts[2].y = wpts[7][1];
	
	cam_data[CAM_RIGHT].wpts[3].x = wpts[4][0];
	cam_data[CAM_RIGHT].wpts[3].y = wpts[4][1];
	
	cam_data[CAM_RIGHT].wpts[4].x = wpts[13][0];
	cam_data[CAM_RIGHT].wpts[4].y = wpts[13][1];
	
	cam_data[CAM_RIGHT].wpts[5].x = wpts[14][0];
	cam_data[CAM_RIGHT].wpts[5].y = wpts[14][1];
	
	cam_data[CAM_RIGHT].wpts[6].x = wpts[15][0];
	cam_data[CAM_RIGHT].wpts[6].y = wpts[15][1];
	
	cam_data[CAM_RIGHT].wpts[7].x = wpts[12][0];
	cam_data[CAM_RIGHT].wpts[7].y = wpts[12][1];

	/* copy feature points  */
	for (i=0; i<CAM_TOTAL; i++)
	{
		memcpy(cam_data[i].ipts, feature_points[i], sizeof(FLOAT)*16);

	}

	memcpy(camData, cam_data, sizeof(cam_data));
	
}


void  make_message_for_factory_pattern(MM_U08 *msg_result, /*[out]*/
									   MM_S16 ecode,       /*[in] */
									   MM_U08 cam_idx)     /*[in] */
{
	if (is_feature_pattern_error(ecode) == 1) /* failied */
	{
		/* setting one-byte header */
		msg_result[0] = (MM_U08)(msg_result[0] & ~CAM_BIT_MASK(cam_idx)); /* make the correspoing bit zero */
				
		/* setting one-byte body */
		if (ecode == ERROR_FEATURE_PATTERN_LEFT)
		{
			msg_result[cam_idx + 1] = (MM_U08)(0x02); /* left pattern error */
		}
		else if (ecode == ERROR_FEATURE_PATTERN_RIGHT)
		{
			msg_result[cam_idx + 1] = (MM_U08)(0x03);  /* right pattern error */
		}
		else if (ecode == ERROR_FEATURE_PATTERN_BOTH)
		{
			msg_result[cam_idx + 1] = (MM_U08)(0x04); /* both patterns error */
		}
		else
		{
			msg_result[cam_idx + 1] = (MM_U08)(0x07); /* both patterns error */
		}
	}
	else  /* succeeded */
	{
		msg_result[0] = (MM_U08)(msg_result[0] | CAM_BIT_MASK(cam_idx)); 
		msg_result[cam_idx + 1] = (MM_U08)0x01;
	}
}


/*maxium and minium of extrinsic parameter's values*/
#define MAX_VAL   (MM_FLOAT)(10000.0f)
#define MIN_VAL   (MM_FLOAT)(-10000.0f)
#define MAX_ANG   (MM_FLOAT)(360.0f)
#define MIN_ANG   (MM_FLOAT)(-360.0f)

MM_S16  check_range_of_extrinsic(EXTRINSIC_PARAM *one_ext_param)
{
	MM_S16 ecode = ERROR_OUT_OF_RANGE;

	if ((MIN_VAL < one_ext_param->x) && (one_ext_param->x < MAX_VAL) &&
		(MIN_VAL < one_ext_param->y) && (one_ext_param->y < MAX_VAL) &&
		(MIN_VAL < one_ext_param->z) && (one_ext_param->z < MAX_VAL) &&
		(MIN_ANG <= one_ext_param->pan) && (one_ext_param->pan <= MAX_ANG) &&
		(MIN_ANG <= one_ext_param->tilt) && (one_ext_param->tilt <= MAX_ANG) &&
		(MIN_ANG <= one_ext_param->rotate) && (one_ext_param->rotate <= MAX_ANG))
	{
		ecode = SUCCESS;
	}

	return ecode;

}


MM_S16  factory_output_from_pose_estimation(EXTRINSIC_PARAM *one_ext_param,	/* [out] */
										    HvCameraData *one_cam_data)		/* [in] */
{
	MM_S16  ecode = ERROR;

	ecode = check_range_of_extrinsic((EXTRINSIC_PARAM *)&one_cam_data->CamTransX);

	if (ecode == SUCCESS)
	{
		one_ext_param->x = one_cam_data->CamTransX;
		one_ext_param->y = one_cam_data->CamTransY;
		one_ext_param->z = one_cam_data->CamHeight;
		one_ext_param->pan = one_cam_data->PanAngle;
		one_ext_param->tilt = one_cam_data->TiltAngle;
		one_ext_param->rotate = one_cam_data->RotateAngle;

		ecode = SUCCESS;
	}
	else
	{
		memset(one_ext_param, 0x00, sizeof(EXTRINSIC_PARAM));
		ecode = ERROR_POSE_ESTIMATION;
	}

	return ecode;
}


void  features_for_double_patterns(MM_U08 *front_img,                 /* [in]  */
								   MM_U08 *rear_img,                  /* [in]  */
								   MM_U08 *left_img,                  /* [in]  */
								   MM_U08 *right_img,                 /* [in]  */
								   MM_U16 height,                     /* [in]  */
								   MM_U16 width,                      /* [in]  */   
								   MM_FLOAT (*feature_points)[16],    /* [out] */ 
								   MM_U08  *msg_result,               /* [out] */
								   CALIBRATION_PARAM *cal_param)	  /* [in]  */
{
	MM_S16 ecode = MM_TRUE;
	
	if ((front_img != NULL) && (rear_img != NULL) && (left_img != NULL) && 
		(right_img != NULL) && (msg_result != NULL) && (cal_param != NULL))
	{
		memset(feature_points, 0x00, sizeof(MM_FLOAT)*(MM_U08)CAM_TOTAL*(MM_U08)16);

		ecode = find_pattern_points(feature_points[CAM_FRONT], front_img, (MM_U08)CAM_FRONT, FACTORY_CALIB, cal_param);
		make_message_for_factory_pattern(msg_result, ecode, (MM_U08)CAM_FRONT);

		ecode = find_pattern_points(feature_points[CAM_REAR], rear_img, (MM_U08)CAM_REAR, FACTORY_CALIB, cal_param);
		make_message_for_factory_pattern(msg_result, ecode, (MM_U08)CAM_REAR);

		ecode = find_pattern_points(feature_points[CAM_LEFT], left_img, (MM_U08)CAM_LEFT, FACTORY_CALIB, cal_param);
		make_message_for_factory_pattern(msg_result, ecode, (MM_U08)CAM_LEFT);

		ecode = find_pattern_points(feature_points[CAM_RIGHT], right_img,(MM_U08)CAM_RIGHT, FACTORY_CALIB, cal_param);
		make_message_for_factory_pattern(msg_result, ecode, (MM_U08)CAM_RIGHT);
	}
}



void  factory_default_when_error(EXTRINSIC_PARAM   ext_param[], /*[out] */
								 MM_U08            msg_header,  /*[in]  */ 
							     CALIBRATION_PARAM *cal_param)  /*[in]  */
{
	if((msg_header & CAM_BIT_MASK(CAM_FRONT)) == (MM_U08)0 ) /* front camera failed */
	{
		if (cal_param->calibration_status[CAM_FRONT] == (MM_U08)0)
		{
			memcpy(&ext_param[0], &cal_param->default_front_extrinsic, sizeof(EXTRINSIC_PARAM));
		}
		else
		{
			memcpy(&ext_param[0], &cal_param->calibrated_front_extrinsic, sizeof(EXTRINSIC_PARAM));
		}
	}

	if((msg_header & CAM_BIT_MASK(CAM_REAR)) == (MM_U08)0 ) /* rear camera failed */
	{
		if (cal_param->calibration_status[CAM_REAR] == (MM_U08)0)
		{
			memcpy(&ext_param[1], &cal_param->default_rear_extrinsic, sizeof(EXTRINSIC_PARAM));
		}
		else
		{
			memcpy(&ext_param[1], &cal_param->calibrated_rear_extrinsic, sizeof(EXTRINSIC_PARAM));
		}
	}

	if((msg_header & CAM_BIT_MASK(CAM_LEFT)) == (MM_U08)0) /* left camera failed */
	{
		if (cal_param->calibration_status[CAM_LEFT] == (MM_U08)0)
		{
			memcpy(&ext_param[2], &cal_param->default_left_extrinsic, sizeof(EXTRINSIC_PARAM));
		}
		else
		{
			memcpy(&ext_param[2], &cal_param->calibrated_left_extrinsic, sizeof(EXTRINSIC_PARAM));
		}

	}

	if((msg_header & CAM_BIT_MASK(CAM_RIGHT)) == (MM_U08)0) /* right camera failed */
	{
		if (cal_param->calibration_status[CAM_RIGHT] == (MM_U08)0)
		{
			memcpy(&ext_param[3], &cal_param->default_right_extrinsic, sizeof(EXTRINSIC_PARAM));
		}
		else
		{
			memcpy(&ext_param[3], &cal_param->calibrated_right_extrinsic, sizeof(EXTRINSIC_PARAM));
		}
	}
}



void  convert_IN_to_Fujitsu(EXTRINSIC_PARAM *one_ext_param) /*[in/out]*/
{
	MM_FLOAT r02, r12, r20, r21, r22;
	MM_FLOAT pan_rad, tilt_rad, rot_rad;
	MM_FLOAT norm;

	pan_rad = (MM_FLOAT)hvDirToRad(one_ext_param->pan);
	tilt_rad = (MM_FLOAT)hvDirToRad(one_ext_param->tilt);
	rot_rad = (MM_FLOAT)hvDirToRad(one_ext_param->rotate);

	r02 = (MM_FLOAT)(-sin(pan_rad));
	r12 = (MM_FLOAT)(sin(tilt_rad)*cos(pan_rad));
	r20 = (MM_FLOAT)(cos(rot_rad)*sin(pan_rad)*cos(tilt_rad) + sin(rot_rad)*sin(tilt_rad));
	r21 = (MM_FLOAT)(sin(rot_rad)*sin(pan_rad)*cos(tilt_rad) - cos(rot_rad)*sin(tilt_rad));
	r22 = (MM_FLOAT)(cos(pan_rad)*cos(tilt_rad));

	norm = (MM_FLOAT)sqrt((r22*r22) / ((r20*r20) + (r21*r21)));
	tilt_rad = (MM_FLOAT)atan2(norm, 1.f);
	rot_rad = (MM_FLOAT)atan2(-r02, r12);
	pan_rad = (MM_FLOAT)atan2(-r20, -r21);

	one_ext_param->y = -one_ext_param->y;
	one_ext_param->z = -one_ext_param->z;
	one_ext_param->pan = (MM_FLOAT)hvRadToDirF(pan_rad);
	one_ext_param->tilt = (MM_FLOAT)hvRadToDirF(tilt_rad);
	one_ext_param->rotate = (MM_FLOAT)hvRadToDirF(rot_rad);
}


MM_S16  check_swp_parameters(MM_U08 *msg_result,				/* [out] error code message */
							 CAR_PARAM *car_param,			/* vehicle information */
							 CALIBRATION_PARAM *cal_param)	/* calibration parameters */
{
	MM_S16	  ecode = (MM_S16)SUCCESS;

	if ((msg_result != NULL) && (car_param != NULL) && (cal_param != NULL))
	{
		if ((car_param->length <= 0.0f)         || (car_param->length > 7000.0f)         ||
			(car_param->front_overhang <= 0.0f) || (car_param->front_overhang > 3500.0f) ||
			(car_param->wheelbase <= 0.0f)      || (car_param->wheelbase > 7000.0f)      ||
			(car_param->rear_overhang <= 0.0f)  || (car_param->rear_overhang > 3500.0f)  ||
			(car_param->width <= 0.0f)          || (car_param->width > 3000.0f)          ||
			(car_param->track <= 0.0f)          || (car_param->track > 3000.0f)          ||
			(car_param->height <= 0.0f)         || (car_param->height > 3000.0f)) /* failed */
		{
			ecode = ERROR_SWP1_CAR;
			msg_result[0] = (msg_result[0] & ~SWP_BIT_MASK);
		}
		else /* succeeded */
		{
			ecode = check_swp_intrinsic_parameters(cal_param);

			if (is_swp3_error(ecode) == 0) /* succeeded */
			{
				ecode = check_swp_factory_parameters(cal_param);

				if (is_swp3_error(ecode) == 0) /* succeeded */
				{
					ecode = check_swp_service_parameters(cal_param);

					if (is_swp3_error(ecode) == 0) /* succeeded */
					{
						msg_result[0] = (msg_result[0] | SWP_BIT_MASK);
					}
					else /* failed */
					{
						msg_result[0] = (msg_result[0] & ~SWP_BIT_MASK);
					}
				}
				else /* failed */
				{
					msg_result[0] = (msg_result[0] & ~SWP_BIT_MASK);
				}
			}
			else /* failed */
			{
				msg_result[0] = (msg_result[0] & ~SWP_BIT_MASK);
			}
		}
	}
	else
	{
		ecode = ERROR;
	}

	return ecode;
}



MM_S16  check_swp_intrinsic_parameters(CALIBRATION_PARAM *cal_param) /* [in]*/
{
	MM_S32 rcode = (MM_S32)SUCCESS;
	MM_S16 ecode = (MM_S16)SUCCESS;

	memset(&all_one_cal_param, 0xff, sizeof(CALIBRATION_PARAM));

	if ((cal_param->focal_length <= 0.0f) || (cal_param->focal_length > 10000.0f))
	{
		ecode = ERROR_SWP3_FOCAL_LENGTH;
	}
	else
	{
		if ((cal_param->mu <= 0.0f) || (cal_param->mv <= 0.0f) ||
			(cal_param->mu > 10000.0f)  || (cal_param->mv > 10000.0f))
		{
			ecode = ERROR_SWP3_CELL_SIZE;
		}
		else
		{
			if ((cal_param->cx < (640.0f - 3.0f)) || (cal_param->cx > (640.0f + 3.0f)) ||
				(cal_param->cy < (360.0f - 3.0f)) || (cal_param->cy > (360.0f + 3.0f)))
			{
				ecode = ERROR_SWP3_OPTICAL_AXIS;
			}
			else
			{
				rcode = memcmp(cal_param->k, all_one_cal_param.k, sizeof(MM_FLOAT) * (MM_U32)5);
				if (rcode == (MM_S32)0)
				{
					ecode = ERROR_SWP3_DISTORTION_K;
				}
				else
				{
					rcode = memcmp(cal_param->d, all_one_cal_param.d, sizeof(MM_FLOAT) * (MM_U32)9);
					if (rcode == (MM_S32)0)
					{
						ecode = ERROR_SWP3_DISTORTION_D;
					}
					else
					{
						if ((cal_param->height != (MM_U32)HEIGHT) || (cal_param->width != (MM_U32)WIDTH))
						{
							ecode = ERROR_SWP3_IMAGE_SIZE;
						}
					}
				}
			}
		}
	}

	return ecode;
}


MM_S16  check_swp_factory_parameters(CALIBRATION_PARAM *cal_param)
{
	MM_S32 rcode = (MM_S32)SUCCESS;
	MM_S16 ecode = (MM_S16)SUCCESS;
	MM_S16 ccode = (MM_S16)SUCCESS;

	memset(&all_one_cal_param, 0xff, sizeof(CALIBRATION_PARAM));

	if (cal_param->num_point != (MM_U32)8)
	{
		ecode = ERROR_SWP3_FACTORY_POINT_NUM;
	}
	else
	{
		rcode = memcmp(&cal_param->default_front_extrinsic,
					   &all_one_cal_param.default_front_extrinsic, sizeof(EXTRINSIC_PARAM));

		ccode = check_range_of_extrinsic(&cal_param->default_front_extrinsic);

		if ((rcode == (MM_S32)0)||(ccode == ERROR_OUT_OF_RANGE))
		{
			ecode = ERROR_SWP3_DEFAULT_EXTRINSIC;
		}
		else
		{
			rcode = memcmp(&cal_param->default_rear_extrinsic,
						  &all_one_cal_param.default_rear_extrinsic, sizeof(EXTRINSIC_PARAM));
			ccode = check_range_of_extrinsic(&cal_param->default_rear_extrinsic);

			if ((rcode == (MM_S32)0) || (ccode == ERROR_OUT_OF_RANGE))
			{
				ecode = ERROR_SWP3_DEFAULT_EXTRINSIC;
			}
			else
			{
				rcode = memcmp(&cal_param->default_left_extrinsic,
							   &all_one_cal_param.default_left_extrinsic, sizeof(EXTRINSIC_PARAM));
				ccode = check_range_of_extrinsic(&cal_param->default_left_extrinsic);

				if ((rcode == (MM_S32)0) || (ccode == ERROR_OUT_OF_RANGE))
				{
					ecode = ERROR_SWP3_DEFAULT_EXTRINSIC;
				}
				else
				{
					rcode = memcmp(&cal_param->default_right_extrinsic,
								   &all_one_cal_param.default_right_extrinsic, sizeof(EXTRINSIC_PARAM));
					ccode = check_range_of_extrinsic(&cal_param->default_right_extrinsic);

					if ((rcode == (MM_S32)0) || (ccode == ERROR_OUT_OF_RANGE))
					{
						ecode = ERROR_SWP3_DEFAULT_EXTRINSIC;
					}
					else
					{
						rcode = memcmp(cal_param->world_coordinates, 
							           all_one_cal_param.world_coordinates, sizeof(MM_FLOAT) * (MM_U32)24 * (MM_U32)2);
						if (rcode == (MM_S32)0)
						{
							ecode = ERROR_SWP3_WORLD_COORD;
						}
						else
						{
							rcode = memcmp(cal_param->roi_points, all_one_cal_param.roi_points, sizeof(MM_U32) * (MM_U32)16);
							if (rcode == (MM_S32)0)
							{
								ecode = ERROR_SWP3_FACTORY_ROI;
							}
						}
					}
				}
			}
		}
	}

	return ecode;
}



MM_S16  check_swp_service_parameters(CALIBRATION_PARAM *cal_param)
{
	MM_S32 rcode = (MM_S32)SUCCESS;
	MM_S16 ecode = (MM_S16)SUCCESS;

	memset(&all_one_cal_param, 0x00, sizeof(CALIBRATION_PARAM));

	if (cal_param->service_num_point != (MM_U32)4)
	{
		ecode = ERROR_SWP3_SERVICE_POINT_NUM;
	}
	else
	{
		if ((cal_param->biggest_square_width < 400.0f) || (cal_param->biggest_square_height < 400.0f) ||
			(cal_param->biggest_square_width > 800.0f) || (cal_param->biggest_square_height > 800.0f))
		{
			ecode = ERROR_SWP3_BIGGEST_SQUARE_SIZE;
		}
		else
		{
			rcode = memcmp(cal_param->service_roi_points, all_one_cal_param.service_roi_points, sizeof(MM_U32) * (MM_U32)8);

			if (rcode == (MM_S32)0)
			{
				ecode = ERROR_SWP3_SERVICE_ROI_POINTS;
			}
			else
			{
				if ((cal_param->service_roi_width == (MM_U32)0) || (cal_param->service_roi_width > (MM_U32)MAX_ROI_WIDTH) ||
					(cal_param->service_roi_height == (MM_U32)0) || (cal_param->service_roi_height > (MM_U32)MAX_ROI_HEIGHT))
				{
					ecode = ERROR_SWP3_SERVICE_ROI_SIZE;
				}
			} 
		} 
	}

	return ecode;
}

#if (check_num)
void check_number(CALIBRATION_PARAM *param)
{
	MM_U08 i = 0;

	/*MM_U08 *pf;
	pf = (MM_U08*)&param->focal_length;
	printf("%X %X %X %X\n", pf[0], pf[1], pf[2], pf[3]);*/


	// Intrinsic Camera Parameters
	PRINTF("Intrinsic Camera Paramers\n");
	PRINTF("%f\n", param->focal_length);
	PRINTF("%f\n", param->mu);
	PRINTF("%f\n", param->mv);
	PRINTF("%f\n", param->cx);
	PRINTF("%f\n", param->cy);

	// Disrottion Paramters
	PRINTF("\nDisrottion Paramters(K1 ~ K5)\n");
	PRINTF("%f\n", param->k[0]);
	PRINTF("%f\n", param->k[1]);
	PRINTF("%f\n", param->k[2]);
	PRINTF("%f\n", param->k[3]);
	PRINTF("%f\n", param->k[4]);

	PRINTF("\nDisrottion Paramters(d1 ~ d9)\n");
	PRINTF("%f\n", param->d[0]);
	PRINTF("%f\n", param->d[1]);
	PRINTF("%f\n", param->d[2]);
	PRINTF("%f\n", param->d[3]);
	PRINTF("%f\n", param->d[4]);
	PRINTF("%f\n", param->d[5]);
	PRINTF("%f\n", param->d[6]);
	PRINTF("%f\n", param->d[7]);
	PRINTF("%f\n", param->d[8]);

	// Image size
	PRINTF("\nImage size(height, width)\n");
	PRINTF("%d\n", param->height);
	PRINTF("%d\n", param->width);

	// Coordinates mapping information
	PRINTF("\nCoordinates mapping information(x, y)\n");
	PRINTF("%f\n", param->ox);
	PRINTF("%f\n", param->oy);

	// Points number
	PRINTF("\nPoints number\n");
	PRINTF("%d\n", param->num_point);

	// Default camera extrinsic
	PRINTF("\nDefault camera extrinsic(x, y, z, pan, tilt, rot)\n");
	PRINTF("%f\n", param->default_front_extrinsic.x);
	PRINTF("%f\n", param->default_front_extrinsic.y);
	PRINTF("%f\n", param->default_front_extrinsic.z);
	PRINTF("%f\n", param->default_front_extrinsic.pan);
	PRINTF("%f\n", param->default_front_extrinsic.tilt);
	PRINTF("%f\n", param->default_front_extrinsic.rotate);

	PRINTF("%f\n", param->default_rear_extrinsic.x);
	PRINTF("%f\n", param->default_rear_extrinsic.y);
	PRINTF("%f\n", param->default_rear_extrinsic.z);
	PRINTF("%f\n", param->default_rear_extrinsic.pan);
	PRINTF("%f\n", param->default_rear_extrinsic.tilt);
	PRINTF("%f\n", param->default_rear_extrinsic.rotate);

	PRINTF("%f\n", param->default_left_extrinsic.x);
	PRINTF("%f\n", param->default_left_extrinsic.y);
	PRINTF("%f\n", param->default_left_extrinsic.z);
	PRINTF("%f\n", param->default_left_extrinsic.pan);
	PRINTF("%f\n", param->default_left_extrinsic.tilt);
	PRINTF("%f\n", param->default_left_extrinsic.rotate);

	PRINTF("%f\n", param->default_right_extrinsic.x);
	PRINTF("%f\n", param->default_right_extrinsic.y);
	PRINTF("%f\n", param->default_right_extrinsic.z);
	PRINTF("%f\n", param->default_right_extrinsic.pan);
	PRINTF("%f\n", param->default_right_extrinsic.tilt);
	PRINTF("%f\n", param->default_right_extrinsic.rotate);

	// World points
	PRINTF("\nWorld points\n");
	for (i = 0; i<24; i++) {
		PRINTF("%12.6f\t", param->world_coordinates[i][0]);
		PRINTF("%12.6f\n", param->world_coordinates[i][1]);
	}

	// ROI start points
	PRINTF("\nROI Information 1st\n");
	for (i = 0; i<8; i++) {
		PRINTF("%3d\t", param->roi_points[2 * i + 0]);
		PRINTF("%3d\n", param->roi_points[2 * i + 1]);
	}

	// ROI cropping
	PRINTF("\nROI Information 2nd\n");
	for (i = 0; i<32; i++) {
		PRINTF("%f\n", param->roi_cropped[i]);
	}

	PRINTF("\nROI Image size\n");
	PRINTF("%d\n", param->roi_height);
	PRINTF("%d\n", param->roi_width);

	// Service Calibration Number of Points
	PRINTF("\nService Calibration Number of Points\n");
	PRINTF("%d\n", param->service_num_point);

	// Service Calibration Main Square on the Unit Pattern in mm
	PRINTF("\nMain Square on the Unit Pattern in mm\n");
	PRINTF("%f\n", param->biggest_square_height);
	PRINTF("%f\n", param->biggest_square_width);

	// Service Calibraton ROI start points
	PRINTF("\nROI Information 1st\n");
	for (i = 0; i<4; i++) {
		PRINTF("%3d\t", param->service_roi_points[2 * i + 0]);
		PRINTF("%3d\n", param->service_roi_points[2 * i + 1]);
	}

	// Service Calibraton ROI cropping
	PRINTF("\nROI Information 2nd\n");
	for (i = 0; i<16; i++) {
		PRINTF("%f\n", param->service_roi_cropped[i]);
	}

	// Service Calibration ROI size
	PRINTF("\nService Calibration ROI size\n");
	PRINTF("%d\n", param->service_roi_height);
	PRINTF("%d\n", param->service_roi_width);
}
#endif 
