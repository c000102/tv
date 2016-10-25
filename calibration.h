#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdio.h>
#include "_hvsys.h"


#define DEBUG_MODE	 (0)

/* define datatypes */

#ifndef DBL_EPSILON1
#define DBL_EPSILON1   (2.2204460492503131e-016) /* smallest such that 1.0+DBL_EPSILON != 1.0 */
#endif

#ifndef PI
#define PI  ((MM_FLOAT)(3.14159265359))
#endif

#define MARKER_NUM_MAX			((MM_U16)(10))

// lens distortion
#define	DIST_MAX_COUNT			((MM_U16)(91))
#define DIST_TABLE_SIZE			((MM_U16)(1))

#define CAM_FRONT				((MM_U16)(0))	/* front camera index */
#define CAM_REAR				((MM_U16)(1))	/* rear camera index */
#define CAM_LEFT				((MM_U16)(2))	/* left camera index */
#define CAM_RIGHT				((MM_U16)(3))	/* right camera index */
#define CAM_TOTAL				((MM_U16)(4))	/* total camera index */

#define MAX_POINTS				((MM_U16)(20)) /* maximum extracted points */
#define MIN_POINTS				((MM_U16)(4))  /* not used */

#define FACTORY_CALIB			((MM_U16)(1))  /* factory calibration runing */
#define SERVICE_CALIB			((MM_U16)(2))  /* service calibration runing */

#define HEIGHT					((MM_U16)(720))	/* height of image */
#define WIDTH					((MM_U16)(1280))/* width of image */
#define IMG_SIZE				((MM_U32)(HEIGHT * WIDTH))/* size of image */

#define MAX_ROI_WIDTH			((MM_U16)(500)) /* width of maximum ROI size*/
#define MAX_ROI_HEIGHT			((MM_U16)(500))	/* height of maximum ROI size*/
#define MAX_ROI_SIZE			((MM_U32)(MAX_ROI_WIDTH * MAX_ROI_HEIGHT))	/* maximum ROI size*/


#define FTPT_SIZE	((MM_U16)(1024)) /* maximum feature points */

#define LEFT_PATTERN (MM_U08)(0)	/* left of pattern in each camera image */
#define RIGHT_PATTERN (MM_U08)(1)	/* right of pattern in each camera image */

//////////  FOR POSE ESTIMATION  //////

#ifndef HANDLE
typedef void * HANDLE;
#endif




// Pointer stuff
typedef union tagHv32suf
{
	MM_S32 i;
	MM_U32 u;
	MM_FLOAT f;

} Hv32suf;

struct HvPoint
{
	MM_S32 x;
	MM_S32 y;
};

/* 2d points(float) */
typedef struct tagHvPoint2D32f
{
	MM_FLOAT x;	/* x coordinates of points */
	MM_FLOAT y;	/* y coordinates of points */
}HvPoint2D32f;

typedef struct tagHvPoint3D32f
{
	MM_FLOAT x;
	MM_FLOAT y;
	MM_FLOAT z;
}HvPoint3D32f;

typedef struct tagHvDstDatum
{
	MM_FLOAT Angle;           //  radian
	MM_FLOAT Distance;        //  mm
} HvDstDatum;


typedef struct tagHvDstData
{
	MM_S32        Width;
	MM_S32        Height;
	MM_FLOAT      DotPitchX;       //  mm
	MM_FLOAT      DotPitchY;       //  mm
	MM_S32        Count;
	HvDstDatum	  *Data;
} HvDstData;


typedef struct tagHvCapDiff
{
	MM_FLOAT Cx;        //
	MM_FLOAT Cy;        //
	MM_FLOAT Sx;        //
	MM_FLOAT Sy;        //
} HvCapDiff;



typedef struct tagHvCalDatum
{
	HvPoint2D32f	MkP[MARKER_NUM_MAX];         //--2014.09.05  
	HvPoint3D32f	MkV[MARKER_NUM_MAX];         //--2014.09.05  
	MM_FLOAT		MkDist[MARKER_NUM_MAX][MARKER_NUM_MAX];  //--2014.09.05  

	MM_S32			Mirror;         //  

	MM_FLOAT		CamTransX;      //  mm
	MM_FLOAT		CamTransY;      //  mm
	MM_FLOAT		CamHeight;      //  mm
	MM_FLOAT		DownAngle;      //  deg
	MM_FLOAT		RotateAngle;    //  deg
	MM_FLOAT		PanAngle;       //  deg

	HvDstData		*DData;         //  
	HvCapDiff		CDiff;          //
	MM_S32			Points;			//--2014.09.05
} HvCalDatum;


typedef struct tagHvDstTable
{
	MM_S32 RefCount;
	HvDstData* Data;
	MM_U08 Name[32];
} HvDstTable;


//typedef struct tagHvLinkData
//{
//	HvPoint3D32f   triP[MARKER_NUM_MAX];
//	HvPoint3D32f   triLCenter;
//	HvPoint3D32f   triRCenter;
//	MM_FLOAT  LinkLength;
//} HvLinkData;

/* extrinsic parameters */
typedef struct tagHvCamPose
{
	MM_FLOAT	CamTransX;      /* x position in mm*/
	MM_FLOAT	CamTransY;      /* y position in mm*/
	MM_FLOAT	CamHeight;      /* z position in mm*/
	MM_FLOAT	PanAngle;       /* pan rotation in deg */
	MM_FLOAT	TiltAngle;      /* tilt roation in deg */
	MM_FLOAT	RotateAngle;    /* rotate roation in deg*/

} HvCamPose;


//////////////////////////////

/* vehicle information */
typedef struct CAR_PARAM_TAG
{
	MM_U32   car_type;			/* vehicle type */
	MM_FLOAT length;			/* lenght of vehicle in mm */
	MM_FLOAT front_overhang;	/* front overhang of vehicle in mm */
	MM_FLOAT wheelbase;			/* wheel base of vehicle in mm*/
	MM_FLOAT rear_overhang;		/* rear overhang of vehicle in mm*/
	MM_FLOAT width;				/* width of vehicle in mm */
	MM_FLOAT track;				/* track of vehicle in mm */
	MM_FLOAT height;			/* height of vehicle in mm */

} CAR_PARAM;

/* extrinsic parameters */
typedef struct EXTRINSIC_PARAM_TAG
{
	MM_FLOAT x;			/* x position */
	MM_FLOAT y;			/* y position */
	MM_FLOAT z;			/* z position */
	MM_FLOAT pan;		/* pan rotation */
	MM_FLOAT tilt;		/* tilt roation */
	MM_FLOAT rotate;	/* rotate roation */

} EXTRINSIC_PARAM;

/* camera parameters for calibration */
typedef struct CALIBRATION_PARAM_TAG
{
   /* SWP3 data */
   MM_FLOAT focal_length;	/* camera focal length */
   MM_FLOAT mu;				/* camera sensor cell ratio for x pixel */
   MM_FLOAT mv;				/* camera sensor cell ratio for y pixel */
   MM_FLOAT cx;				/* camera optical x axis */
   MM_FLOAT cy;				/* camera optical y axis */
   MM_FLOAT k[5];			/* camera distortion parameters k */
   MM_FLOAT d[9];			/* camera distortion parameters k */
   MM_U32   height;			/* height of source image */
   MM_U32   width;			/* width of source image */
   MM_FLOAT ox;				/* world ponts of x shift variance */
   MM_FLOAT oy;				/* world ponts of y shift variance */
   MM_U32   num_point;		/* number of ponts */

   EXTRINSIC_PARAM default_front_extrinsic;	/* default front camera extrinsic parameters */
   EXTRINSIC_PARAM default_rear_extrinsic;	/* default rear camera extrinsic parameters */
   EXTRINSIC_PARAM default_left_extrinsic;	/* default left camera extrinsic parameters */
   EXTRINSIC_PARAM default_right_extrinsic;	/* default right camera extrinsic parameters */

   MM_FLOAT world_coordinates[24][2];		/* world points */
    
   MM_U32   roi_points[16];		/* roi points for factory calibration */
   MM_FLOAT roi_cropped[32];	/* roi points for factory calibraiotn */
   MM_U32   roi_width;			/* width of roi image */
   MM_U32   roi_height;			/* height of roi image */

   
   /* only for service calibraiton from the following */
   MM_U32   service_num_point;			/* number of ponts */
   MM_FLOAT biggest_square_width;		/* width of the main square on the unit pattern in mm */
   MM_FLOAT biggest_square_height;		/* height of the main square  on the unit pattern in mm  */
   MM_U32   service_roi_points[8];		/* roi points per image */
   MM_FLOAT service_roi_cropped[16];	/* roi points for service calibraiotn */
   MM_U32   service_roi_width;			/* width of roi image */
   MM_U32   service_roi_height;			/* height of roi image */

   /* extrinsic parameters calibrated, but not saved in SWP3 */
   EXTRINSIC_PARAM calibrated_front_extrinsic;	/* front camera extrinsic paramters calibrated */
   EXTRINSIC_PARAM calibrated_rear_extrinsic;	/* rear camera extrinsic paramters calibrated */
   EXTRINSIC_PARAM calibrated_left_extrinsic;	/* left camera extrinsic paramters calibrated */
   EXTRINSIC_PARAM calibrated_right_extrinsic;	/* right camera extrinsic paramters calibrated */
   MM_U08          calibration_status[4]; /* if the calibration succeeded at leat one time before, then 1, otherwise 0.*/

} CALIBRATION_PARAM;


/* 2d points(S16) */
typedef struct _NPOINTS
{
	MM_S16 x;	/* x coordinates of points */
	MM_S16 y;	/* y coordinates of points */
} NPOINTS;

/* 2d points(float) */
typedef struct _POINTS2DF
{  
	MM_FLOAT x;	/* x coordinates of points */
	MM_FLOAT y;	/* y coordinates of points */

} POINTS2DF;

/* 2d feature point  */
typedef struct _CORNER2DF
{
	MM_U16 MaxLength;					/* maximum feature poitns size */
	MM_U16 CornerSize;					/* current feature points size*/
	POINTS2DF Corner2DList[FTPT_SIZE];	/* memory of feature points */

} CORNER2DF2DF;



/* cmaera calibratin data */
typedef struct tagHvCameraData
{
	MM_S32		camID;			/* camera id */
	MM_FLOAT	f;				/* focal length */
	MM_FLOAT	mu;				/* pixels/mm */
	MM_FLOAT	mv;				/* pixels/mm */
	MM_FLOAT	cx;				/* x of optical center */
	MM_FLOAT	cy;				/* y of optical center */
	MM_FLOAT	k[5];			/* distortion parameters */
	MM_FLOAT	d[9];			/* undistortion parameters */

	MM_S32		ssize_width;	/* width of image */
	MM_S32		ssize_height;	/* height of image */
	MM_S32		offset_x;		/* not used */
	MM_S32		offset_y;		/* not used */

	MM_FLOAT	CamTransX;      /* mm */
	MM_FLOAT	CamTransY;      /* mm */
	MM_FLOAT	CamHeight;      /* mm */
	MM_FLOAT	PanAngle;       /* deg. */
	MM_FLOAT	TiltAngle;      /* deg. */
	MM_FLOAT	RotateAngle;    /* deg. */

	HvPoint2D32f world_center;   /* center of car in world coordinate */
	MM_S32		 marker_type;	 /* not used */
	MM_S32		 marker_length;	 /* not used */

	MM_S32		 numPoints;		   /* the number of points to be processed at once */
	HvPoint2D32f ipts[MAX_POINTS]; /* image points */
	HvPoint2D32f wpts[MAX_POINTS]; /* world points */

	MM_FLOAT t[3]; // translaction vector from camera coordinates
	MM_FLOAT R[9]; // rotation matrix from camera coordinates
} HvCameraData;

LIBCALIB_BEGIN_DECLS


INT hvCalibInit(HvCameraData cameraData[]);
INT hvCalibApply(HvCameraData CameraData[], INT cam_total);
void  convert_IN_to_Fujitsu(EXTRINSIC_PARAM *one_ext_param); /*[in/out]*/


///////////////////////////////////// call these funcitons //////////////////////////////////////

MM_S16 factory_calibration(MM_U08 *front_img,				 /* [in] front camera input, color space: RGBA, byte ordering: ABGR */
						MM_U08 *rear_img,					 /* [in] rear camera input */
						MM_U08 *left_img,					 /* [in] left camera input */
						MM_U08 *right_img,				     /* [in] right camera input */
						MM_U16 img_height,				     /* [in]  image height */
						MM_U16 img_width,				     /* [in]  image width */
						CAR_PARAM *car_param,                /* [in]  car information */
						CALIBRATION_PARAM *cal_param,		 /* [in]  information about calibraiton*/
						EXTRINSIC_PARAM   *ext_param,	     /* [out] camera extrinsic parameters */
						MM_U08            *result_message);  /* [out] message for operation result*/


MM_S16 service_calibration(MM_U08 *img,						 /* [in]  camera input, color space: RGBA, byte ordering: ABGR */  
			 			MM_U16 img_height,				     /* [in]  image height */
						MM_U16 img_width,				     /* [in]  image width */
						MM_U08 cam_num,                      /* [in]  camera number */
						CAR_PARAM *car_param,                /* [in]  car information */
						CALIBRATION_PARAM *cal_param,		 /* [in]  information about calibraiton*/
						EXTRINSIC_PARAM   *ext_param,	     /* [out] camera extrinsic parameters */
						MM_U08            *result_message);  /* [out] message for operation result*/

//////////////////////////////////////////////////////////////////////////////////////////////////

LIBCALIB_END_DECLS

#endif
