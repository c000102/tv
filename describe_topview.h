#ifndef DESCRIBE_TOPVIEW_H
#define DESCRIBE_TOPVIEW_H

#include "_hvsys.h"
#include "calibration.h"

#ifdef __cplusplus
#define CAPI __cdecl
#define CNAME extern "C"
#else
#define CAPI
#define CNAME 
#endif


/* define word */
#define IMG_H ((MM_U16)(720))
#define IMG_W ((MM_U16)(1280))

#ifndef SNG_EPSILON
#define SNG_EPSILON    (2.2204460492503131e-016) /* float */
#endif


CFUNC_BEGIN_DECLS

/* structure */

/* function */
MM_S16  describe_topview(	MM_U08				*front_img,		/* [in] front camera input, color space: RGBA, byte ordering: ABGR */
							MM_U08				*rear_img,		/* [in] rear camera input */
							MM_U08				*left_img,		/* [in] left camera input */
							MM_U08				*right_img,		/* [in] right camera input */
							MM_U16				img_height,		/* [in]  image height */
							MM_U16				img_width,		/* [in]  image width */
							CALIBRATION_PARAM	*cal_param,		/* [in]  information about calibraiton*/
							HvCameraData *cam_data,
							MM_S16 ID);


CNAME MM_S16 CAPI MatMul(MM_FLOAT *M, MM_FLOAT *A, MM_S32 m, MM_S32 n, MM_FLOAT *B, MM_S32 c, MM_S32 d);
CNAME MM_S16 CAPI MatInv3x3(MM_FLOAT *iM, MM_FLOAT *M);

CFUNC_END_DECLS

#endif
