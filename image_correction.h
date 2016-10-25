#include <math.h>
#include <memory.h>
#include "calibration.h"

#pragma once

#ifdef __cplusplus
#define CAPI __cdecl
#define CNAME extern "C"
#else
#define CAPI
#define CNAME 
#endif


#ifndef TRUE
#define TRUE       (1)
#endif

#ifndef FALSE 
#define FALSE      (0)
#endif

#ifndef PI
#define PI         (MM_FLOAT)(3.14159265358979f)
#endif

#ifndef DEG2RAD
#define DEG2RAD    (MM_FLOAT)(PI/180.0f))
#endif

#ifndef RAD2DEG
#define RAD2DEG    ((MM_FLOAT)(180.0f/PI))
#endif

/* define image size */
#define IMG_H ((MM_U16)(720))
#define IMG_W ((MM_U16)(1280))

#define MM_SUCCESS							(MM_S16)(0)
#define MM_FAILURE							(MM_S16)(1)

#define DAT_NUM								(MM_U16)(91)		/*  the number of measured data (theta, distance) */

/* don't change the macro just below */
#define MAX_PAR								(MM_U16)(9)      /* the maximal number of unkouwn coefficients */
#define K_PAR								(MM_U16)(5)
#define D_PAR								(MAX_PAR)

#if defined(_WIN32) || defined(_WIN64)
//typedef signed char      MM_S08;
//typedef unsigned char    MM_U08;
//typedef signed __int16   MM_S16;
//typedef unsigned __int16 MM_U16;
//typedef signed __int32   MM_S32;
//typedef unsigned __int32 MM_U32;
//typedef float            MM_FLOAT;
//typedef double           MM_DOUBLE;
typedef void             MM_VOID;
#endif

//typedef struct tagHvPoint2D32f
//{
//	MM_FLOAT  x;
//	MM_FLOAT  y;
//
//} HvPoint2D32f;

typedef struct tagHvFishCameraData
{
	MM_U16      dimg_width; /* width of distorted image */
	MM_U16      dimg_height; /* height of distorted image */
	MM_FLOAT    dcx; /* horizontal coordinate of optical center in distorted images(pixel) */
	MM_FLOAT    dcy; /* vertical coordinate of optical center in distorted images(pixel) */
	MM_FLOAT    f;   /* focal length(mm) */
	MM_FLOAT    scw; /* width of sensor's cell(mm) */
	MM_FLOAT    sch; /* height of sensor's cell(mm) */

	MM_FLOAT    scale; /* scaling of the focal length when we makes an undistorted image */

	MM_U16      img_width; /* width of undistorted image */
	MM_U16      img_height; /* height of undistored image */
	MM_FLOAT	fm;	 /* focal length * (mu + mv) /2 */
	MM_FLOAT    fmu; /* focal length * mu */
	MM_FLOAT    fmv; /* focal length * mv */
	MM_FLOAT	mu;	 /* (1 / width of a sensor) */
	MM_FLOAT	mv;	 /* (1 / height of a sensor) */
	MM_FLOAT	cx;	 /* x coordinates of optical center */
	MM_FLOAT	cy;	 /* y coordinates of optical center */

	MM_FLOAT    lens[DAT_NUM]; /* distance(mm) based on incidence angle */
	MM_FLOAT	k[K_PAR]; /* curve-fitted by Levenburg-Marquart method(non-linearity optimization)  */
	MM_FLOAT	d[D_PAR]; /* curve-fitted by Levenburg-Marquart method(non-linearity optimization)  */

} HvFishCameraData;

CNAME MM_VOID CAPI image_correction_pt(
								HvPoint2D32f *upt,		/* [out] undistortion image points */
								HvPoint2D32f *ipt		/* [in] distortion image points */
);

CNAME MM_VOID CAPI image_correction_pt_bw(
								HvPoint2D32f *upt,	/* [out] undistortion image points */
								HvPoint2D32f *ipt	/* [in] distortion image points */
);

CNAME MM_VOID CAPI image_correction_img(
	MM_U08 *upt,		/* [out] undistortion image */
	MM_U08 *ipt,		/* [in] distortion image */
	MM_U16 height,		/* [in] height of image */
	MM_U08 width,		/* [in] widht image */
	HvCameraData *cam_data
);

CNAME MM_VOID CAPI initialize_camera_data(HvFishCameraData *param, MM_FLOAT focal_lengh_scale);


CNAME MM_VOID CAPI dpt2upt(HvPoint2D32f* upt,
	HvPoint2D32f* dpt,
	HvFishCameraData *cam);

CNAME MM_VOID CAPI upt2dpt(HvPoint2D32f* dpt,
	HvPoint2D32f* upt,
	HvFishCameraData *cam);


CNAME MM_S16 CAPI d2u_img_fw(MM_U08 *dst_img,
	MM_U16 dst_img_width,
	MM_U16 dst_img_height,
	MM_U08 *src_img,
	MM_U16 src_img_width,
	MM_U16 src_img_height,
	MM_U16 chl_num,
	HvFishCameraData *cam);

CNAME MM_S16 CAPI d2u_img_bw(MM_U08 *dst_img,
	MM_U16 dst_img_width,
	MM_U16 dst_img_height,
	MM_U08 *src_img,
	MM_U16 src_img_width,
	MM_U16 src_img_height,
	MM_U16 chl_num,
	HvFishCameraData *cam);

CNAME MM_S16 CAPI u2d_img_fw(MM_U08 *dst_img,
	MM_U16 dst_img_width,
	MM_U16 dst_img_height,
	MM_U08 *src_img,
	MM_U16 src_img_width,
	MM_U16 src_img_height,
	MM_U16 chl_num,
	HvFishCameraData *cam);

CNAME MM_S16 CAPI u2d_img_bw(MM_U08 *dst_img,
	MM_U16 dst_img_width,
	MM_U16 dst_img_height,
	MM_U08 *src_img,
	MM_U16 src_img_width,
	MM_U16 src_img_height,
	MM_U16 chl_num,
	HvFishCameraData *cam);