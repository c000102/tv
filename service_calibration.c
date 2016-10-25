#include "calibration.h"
#include "pattern_point.h"


CFUNC_BEGIN_DECLS

extern MM_S16  check_range_of_feature_points(MM_FLOAT *feature_points, 
	                                         CALIBRATION_PARAM *cal_param,
											 MM_U08 cal_type);

extern MM_S16  check_range_of_extrinsic(EXTRINSIC_PARAM *one_ext_param);

extern MM_S16 check_swp_parameters(MM_U08 *msg_header, 
								  CAR_PARAM *car_param, 
								  CALIBRATION_PARAM *cal_param);

static void  update_roi_in_calibration_param(CALIBRATION_PARAM *cal_param);

static void  service_input_for_pose_estimaition(HvCameraData *one_cam_data,
												MM_FLOAT     *feature_points, 
												CALIBRATION_PARAM *cal_param, 
												MM_U08 cam_idx);

static MM_S16  service_output_from_pose_estimation(EXTRINSIC_PARAM *one_ext_param,
													HvCameraData* one_cam_data);

static void copy_existing_position(EXTRINSIC_PARAM *ext_param,
									CALIBRATION_PARAM *cal_param,
									HvCameraData* cam_data,
									MM_U08 cam_num);

void  service_default_when_error(EXTRINSIC_PARAM *one_ext_param,
								MM_U08 msg_header,
								MM_U08  cam_idx,
								CALIBRATION_PARAM *cal_param);

static void  features_for_signle_pattern(MM_U08 *img,						 
										MM_U16 height,						 
										MM_U16 width,						 
										MM_U08 cam_num,						 
										MM_FLOAT *list_points,               
										MM_U08   *result_message,			 
										CALIBRATION_PARAM *cal_param);
CFUNC_END_DECLS


/* ---------------------------------------------------------------------------
                           service calibration 
------------------------------------------------------------------------------ */


MM_S16  service_calibration(MM_U08				*img,			 /* [in] camera input, color space: RGBA, byte ordering: ABGR */
						   MM_U16				img_height,		 /* [in]  image height */
					       MM_U16				img_width,	     /* [in]  image width */
						   MM_U08				cam_idx,         /* [in]  camera index number */
						   CAR_PARAM			*car_param,      /* [in]  car information */
					       CALIBRATION_PARAM	*cal_param,		 /* [in]  information about calibraiton*/
					       EXTRINSIC_PARAM		*one_ext_param,	 /* [out] camera extrinsic parameters */
					       MM_U08				*msg_result)     /* [out] message for operation result*/
{
	HvCameraData	list_cam_data[CAM_TOTAL];
	MM_FLOAT		feature_points[16];
	MM_S32			pi_ecode   = (MM_S32)0;
	MM_S32			pm_ecode   = (MM_S32)0;
	MM_S16          so_ecode   = SUCCESS;
	MM_S16          fp_ecode   = SUCCESS;

	memset(list_cam_data, 0x00, sizeof(HvCameraData)*(MM_U08)CAM_TOTAL);
	memset(one_ext_param,0x00,sizeof(EXTRINSIC_PARAM));
	memset(msg_result,0x00,sizeof(MM_U08)*(MM_U08)(5));	
	memset((void*)feature_points,0x00,sizeof(MM_FLOAT)*(MM_U08)16);

	/* set the type of calibration */
	msg_result[0] = (MM_U08)(msg_result[0] | SERVICE_CAL_TYPE_MASK);

#if (0)
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

#if 0
	PRINTF("[%8.2f, %8.2f, %8.2f]\n",
		cal_param->calibrated_front_extrinsic.x, cal_param->calibrated_front_extrinsic.y, cal_param->calibrated_front_extrinsic.z);
	PRINTF("[%8.2f, %8.2f, %8.2f]\n",
		cal_param->calibrated_rear_extrinsic.x, cal_param->calibrated_rear_extrinsic.y, cal_param->calibrated_rear_extrinsic.z);
	PRINTF("[%8.2f, %8.2f, %8.2f]\n",
		cal_param->calibrated_left_extrinsic.x, cal_param->calibrated_left_extrinsic.y, cal_param->calibrated_left_extrinsic.z);
	PRINTF("[%8.2f, %8.2f, %8.2f]\n",
		cal_param->calibrated_right_extrinsic.x, cal_param->calibrated_right_extrinsic.y, cal_param->calibrated_right_extrinsic.z);

	PRINTF("service_calibration\n");
#endif

	if(hvPositive(msg_result[0] & SWP_BIT_MASK) == (MM_U08)1) /* succeeded */
	{
		update_roi_in_calibration_param(cal_param);

		features_for_signle_pattern(img, 
			  					    (MM_U16)(cal_param->height),
								    (MM_U16)(cal_param->width), 
								    cam_idx,
									feature_points,
								    msg_result,
								    cal_param);
		
		if ( hvPositive(msg_result[0] & CAM_BIT_MASK(cam_idx)) == (MM_U08)1)/* succeeded */
		{
			fp_ecode = check_range_of_feature_points(feature_points, cal_param, SERVICE_CALIB);
			
			if (hvSucceeded(fp_ecode))
			{
				service_input_for_pose_estimaition(&list_cam_data[cam_idx], feature_points, cal_param, cam_idx);

				pi_ecode = hvCalibInit(&list_cam_data[cam_idx]);
				
				if (hvSucceeded(pi_ecode))
				{
					pm_ecode = hvCalibApply(&list_cam_data[cam_idx], (MM_S32)1);
					
					if (hvSucceeded(pm_ecode)) // calibration success
					{
						so_ecode = service_output_from_pose_estimation(one_ext_param, &list_cam_data[cam_idx]);
						
						if (so_ecode == SUCCESS)
						{
							convert_IN_to_Fujitsu(one_ext_param);
							copy_existing_position(one_ext_param, cal_param, &list_cam_data[cam_idx], cam_idx);
							
							msg_result[0] = (MM_U08)(msg_result[0] | CAM_BIT_MASK(cam_idx));
							msg_result[cam_idx + 1] = (MM_U08)(0x01);
						}
						else /* out of range error*/
						{
							msg_result[0] = (MM_U08)(msg_result[0] & ~CAM_BIT_MASK(cam_idx));
							msg_result[cam_idx + 1] = (MM_U08)(0x06);
						}
					}
					else/* pose estimation error */
					{
						msg_result[0] = (MM_U08)(msg_result[0] & ~CAM_BIT_MASK(cam_idx));
						msg_result[cam_idx + 1] = (MM_U08)(0x06);
					}
				}
				else /* pose initialization error */
				{
					msg_result[0] = (MM_U08)(msg_result[0] & ~CAM_BIT_MASK(cam_idx));
					msg_result[cam_idx + 1] = (MM_U08)(0x06);
				}
			}
			else  /* check point range */
			{
				msg_result[0] = (MM_U08)(msg_result[0] & ~CAM_BIT_MASK(cam_idx));
				msg_result[cam_idx + 1] = 0x05;
			}
		}
	}

	service_default_when_error(one_ext_param, msg_result[0],cam_idx, cal_param);

	return (MM_S16)1; /*notify that the operation has been completed, not error*/
}


void  update_roi_in_calibration_param(CALIBRATION_PARAM *cal_param) /* [in/out] */
{
	MM_U32 temp_roi_point[8];
	MM_FLOAT temp_cropped[16];

	memset(temp_roi_point, 0x00, sizeof(MM_U32) * (MM_U32)8);
	memset(temp_cropped, 0x00, sizeof(MM_FLOAT) * (MM_U32)16);

	cal_param->roi_width = cal_param->service_roi_width;
	cal_param->roi_height = cal_param->service_roi_height;

	memcpy(temp_roi_point, cal_param->service_roi_points, sizeof(MM_U32) * (MM_U32)8); /* MISRA-C:2012 R.19.1 */
	memcpy(cal_param->roi_points, temp_roi_point,sizeof(MM_U32)*(MM_U32)8);

	memcpy(temp_cropped, cal_param->service_roi_cropped, sizeof(MM_FLOAT) * (MM_U32)16);  /* MISRA-C:2012 R.19.1 */
	memcpy(cal_param->roi_cropped,temp_cropped, sizeof(MM_FLOAT)*(MM_U32)16);
}



void  service_input_for_pose_estimaition(HvCameraData *one_cam_data,   /* [out] camera data for pose estimation */
										 MM_FLOAT     *feature_points,/* [in] four feature points */
										 CALIBRATION_PARAM *cal_param, /* [in] camera parameters */
										 MM_U08 cam_idx)			   /* [in] camera index */
{
	HvCameraData temp_one_cam_data;

	memset(&temp_one_cam_data, 0x00, sizeof(HvCameraData));

	temp_one_cam_data.camID = cam_idx;
	temp_one_cam_data.f     = cal_param->focal_length;
	temp_one_cam_data.mu    = cal_param->mu;
	temp_one_cam_data.mv    = cal_param->mv;
	temp_one_cam_data.cx    = cal_param->cx;
	temp_one_cam_data.cy    = cal_param->cy;
		
	memcpy(temp_one_cam_data.k,cal_param->k,sizeof(MM_FLOAT)*5);
	memcpy(temp_one_cam_data.d,cal_param->d,sizeof(MM_FLOAT)*9);

	temp_one_cam_data.ssize_height   = (MM_S32)(cal_param->height);
	temp_one_cam_data.ssize_width    = (MM_S32)(cal_param->width);
	temp_one_cam_data.world_center.x = cal_param->ox;
	temp_one_cam_data.world_center.y = cal_param->oy;
	temp_one_cam_data.numPoints      = cal_param->service_num_point;

	switch (cam_idx)
	{
	case CAM_FRONT:
		temp_one_cam_data.wpts[0].x = 0.0f;
		temp_one_cam_data.wpts[0].y = 0.0f;
		temp_one_cam_data.wpts[1].x = (cal_param->biggest_square_width);
		temp_one_cam_data.wpts[1].y = 0.0f;
		temp_one_cam_data.wpts[2].x = (cal_param->biggest_square_width);
		temp_one_cam_data.wpts[2].y = (cal_param->biggest_square_height);
		temp_one_cam_data.wpts[3].x = 0.0f;
		temp_one_cam_data.wpts[3].y = (cal_param->biggest_square_height);
		break;

	case CAM_REAR:
		temp_one_cam_data.wpts[0].x = (cal_param->biggest_square_width);
		temp_one_cam_data.wpts[0].y = (cal_param->biggest_square_height);
		temp_one_cam_data.wpts[1].x = 0.0f;
		temp_one_cam_data.wpts[1].y = (cal_param->biggest_square_height);
		temp_one_cam_data.wpts[2].x = 0.0f;
		temp_one_cam_data.wpts[2].y = 0.0f;
		temp_one_cam_data.wpts[3].x = (cal_param->biggest_square_width);
		temp_one_cam_data.wpts[3].y = 0.0f;
		break;

	case CAM_LEFT:
		temp_one_cam_data.wpts[0].x = 0.0f;
		temp_one_cam_data.wpts[0].y = (cal_param->biggest_square_height);
		temp_one_cam_data.wpts[1].x = 0.0f;
		temp_one_cam_data.wpts[1].y = 0.0f;
		temp_one_cam_data.wpts[2].x = (cal_param->biggest_square_width);
		temp_one_cam_data.wpts[2].y = 0.0f;
		temp_one_cam_data.wpts[3].x = (cal_param->biggest_square_width);
		temp_one_cam_data.wpts[3].y = (cal_param->biggest_square_height);
		break;

	case CAM_RIGHT:
		temp_one_cam_data.wpts[0].x = (cal_param->biggest_square_width);
		temp_one_cam_data.wpts[0].y = 0.0f;
		temp_one_cam_data.wpts[1].x = (cal_param->biggest_square_width);
		temp_one_cam_data.wpts[1].y = (cal_param->biggest_square_height);
		temp_one_cam_data.wpts[2].x = 0.0f;
		temp_one_cam_data.wpts[2].y = (cal_param->biggest_square_height);
		temp_one_cam_data.wpts[3].x = 0.0f;
		temp_one_cam_data.wpts[3].y = 0.0f;
		break;

	default:
		break;
	}

	/* image feature points */
	memcpy(&temp_one_cam_data.ipts, feature_points, sizeof(MM_FLOAT) * 8);
	memcpy(one_cam_data, &temp_one_cam_data, sizeof(HvCameraData));
}



MM_S16  service_output_from_pose_estimation(EXTRINSIC_PARAM *one_ext_param,
											HvCameraData* one_cam_data)
{
	MM_S16  ecode = ERROR;

	ecode = check_range_of_extrinsic((EXTRINSIC_PARAM *)&one_cam_data[0].CamTransX);

	if (ecode == SUCCESS)
	{
		one_ext_param->x = one_cam_data->CamTransX;
		one_ext_param->y = one_cam_data->CamTransY;
		one_ext_param->z = one_cam_data->CamHeight;
		one_ext_param->tilt = one_cam_data->TiltAngle;
		one_ext_param->pan = one_cam_data->PanAngle;
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


void  copy_existing_position(EXTRINSIC_PARAM *one_ext_param,
 							 CALIBRATION_PARAM *cal_param,
							 HvCameraData* cam_data,
							 MM_U08 cam_idx)
{
	
	switch (cam_idx)
	{
	case CAM_FRONT:
		if (cal_param->calibration_status[CAM_FRONT] == (MM_U08)0)
		{
			
			one_ext_param->x = cal_param->default_front_extrinsic.x;
			one_ext_param->y = cal_param->default_front_extrinsic.y;
			one_ext_param->z = cal_param->default_front_extrinsic.z;
		}
		else
		{
			
			one_ext_param->x = cal_param->calibrated_front_extrinsic.x;
			one_ext_param->y = cal_param->calibrated_front_extrinsic.y;
			one_ext_param->z = cal_param->calibrated_front_extrinsic.z;
		}
		break;
	case CAM_REAR:
		if (cal_param->calibration_status[CAM_REAR] == (MM_U08)0)
		{
			
			one_ext_param->x = cal_param->default_rear_extrinsic.x;
			one_ext_param->y = cal_param->default_rear_extrinsic.y;
			one_ext_param->z = cal_param->default_rear_extrinsic.z;
		}
		else
		{
			
			one_ext_param->x = cal_param->calibrated_rear_extrinsic.x;
			one_ext_param->y = cal_param->calibrated_rear_extrinsic.y;
			one_ext_param->z = cal_param->calibrated_rear_extrinsic.z;
			
		}
		break;
	case CAM_LEFT:
		if (cal_param->calibration_status[CAM_LEFT] == (MM_U08)0)
		{
			
			one_ext_param->x = cal_param->default_left_extrinsic.x;
			one_ext_param->y = cal_param->default_left_extrinsic.y;
			one_ext_param->z = cal_param->default_left_extrinsic.z;
		}
		else
		{
			
			one_ext_param->x = cal_param->calibrated_left_extrinsic.x;
			one_ext_param->y = cal_param->calibrated_left_extrinsic.y;
			one_ext_param->z = cal_param->calibrated_left_extrinsic.z;
		}
		break;
	case CAM_RIGHT:
		if (cal_param->calibration_status[CAM_RIGHT] == (MM_U08)0)
		{
			
			one_ext_param->x = cal_param->default_right_extrinsic.x;
			one_ext_param->y = cal_param->default_right_extrinsic.y;
			one_ext_param->z = cal_param->default_right_extrinsic.z;
		}
		else
		{
			
			one_ext_param->x = cal_param->calibrated_right_extrinsic.x;
			one_ext_param->y = cal_param->calibrated_right_extrinsic.y;
			one_ext_param->z = cal_param->calibrated_right_extrinsic.z;
		}
		break;
	default:
		
		break;
	}

}

void  features_for_signle_pattern(MM_U08 *img,							/*[in] input image */
							      MM_U16 height,						/*[in] height */
								  MM_U16 width,							/*[in] width */
								  MM_U08 cam_idx,						/*[in] camera index*/  
								  MM_FLOAT *list_points,				/*[out] feature points*/
	                              MM_U08   *msg_result,	  			    /*[out] message after calibration, 5 bytes*/
								  CALIBRATION_PARAM *cal_param)  /*[in] */
{
	MM_S16 ecode = MM_TRUE;

	if ((img != NULL) && (list_points != NULL) && (msg_result != NULL) && (cal_param != NULL))
	{
		memset(list_points, 0x00, sizeof(MM_FLOAT)*(MM_U08)16);

		ecode = find_pattern_points(list_points,
									img,
									(MM_U08)cam_idx,
									SERVICE_CALIB,
									cal_param);

		if (is_feature_pattern_error(ecode)) /* failed */
		{
			/* one-byte header */
			msg_result[0] = (MM_U08)(msg_result[0] & ~CAM_BIT_MASK(cam_idx)); /*make a corresponding bit  zero; */

			/* one-byte body */
			if (ecode == ERROR_FEATURE_PATTERN_CAMERA_FAULT)
			{
				msg_result[cam_idx + 1] = (MM_U08)(0x07);
			}
			else
			{
				msg_result[cam_idx + 1] = (MM_U08)(0x05);
			}
		}
		else /* succeeded */
		{
			/* one-byte header */
			msg_result[0] = (MM_U08)(msg_result[0] | CAM_BIT_MASK(cam_idx)); /* make a corresponding bit  one. */

			/* one-byte body */
			msg_result[cam_idx + 1] = (MM_U08)0x01;
		}
	}

}


void  service_default_when_error(EXTRINSIC_PARAM *one_ext_param, 
								 MM_U08 msg_header,
								 MM_U08  cam_idx,
							     CALIBRATION_PARAM *cal_param)
{
	if((msg_header & CAM_BIT_MASK(cam_idx)) == (MM_U08)0)  
	{
		switch (cam_idx)
		{
		case CAM_FRONT:
			if (cal_param->calibration_status[CAM_FRONT] == (MM_U08)0)
			{
				memcpy(one_ext_param, &cal_param->default_front_extrinsic, sizeof(EXTRINSIC_PARAM));
			}
			else
			{
				memcpy(one_ext_param, &cal_param->calibrated_front_extrinsic, sizeof(EXTRINSIC_PARAM));
			}
			break;
		case CAM_REAR:
			if (cal_param->calibration_status[CAM_REAR] == (MM_U08)0)
			{
				memcpy(one_ext_param, &cal_param->default_rear_extrinsic, sizeof(EXTRINSIC_PARAM));
			}
			else
			{
				memcpy(one_ext_param, &cal_param->calibrated_rear_extrinsic, sizeof(EXTRINSIC_PARAM));
			}
			break;
		case CAM_LEFT:
			if (cal_param->calibration_status[CAM_LEFT] == (MM_U08)0)
			{
				memcpy(one_ext_param, &cal_param->default_left_extrinsic, sizeof(EXTRINSIC_PARAM));
			}
			else
			{
				memcpy(one_ext_param, &cal_param->calibrated_left_extrinsic, sizeof(EXTRINSIC_PARAM));
			}
			break;
		case CAM_RIGHT:
			if (cal_param->calibration_status[CAM_RIGHT] == (MM_U08)0)
			{
				memcpy(one_ext_param, &cal_param->default_rear_extrinsic, sizeof(EXTRINSIC_PARAM));
			}
			else
			{
				memcpy(one_ext_param, &cal_param->calibrated_rear_extrinsic, sizeof(EXTRINSIC_PARAM));
			}
			break;
		default:
			break;
		}
	}
}

