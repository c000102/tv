#ifndef _HVSYS_H
#define _HVSYS_H

//#define HV_ARM9
//#define HV_WOER
//#define __LINUX__
// If compiled in C++ use C++ versions of standard headers

#ifdef __cplusplus
extern "C++"
{
#endif

#ifndef HV_WOER

//#include <malloc.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#endif

#ifdef __cplusplus
}
#endif


#if !((defined WIN32)||(defined WIN64))
#include "mm_types.h"
#include "system.h"
#else
#define PRINTF      (void)printf
#define MM_FALSE	((MM_S16)(0))
#define MM_TRUE		((MM_S16)(1))
#define PRINT_BEBUG(word) (void)printf(word);
#endif


#ifndef NULL
#define NULL 0
#endif

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

#if !defined(_SIZE_T) && !defined(_SIZE_T_DEFINED) && !defined(HV_ARM9)
#define _SIZE_T_DEFINED
typedef unsigned int size_t;
#endif

#if defined(linux) || defined (__LINUX__)
#define HV_LINUX
#endif

#if defined WIN32 || defined WIN64
#define HV_CDECL __cdecl
#define HV_STDCALL __stdcall
#else
#define HV_CDECL
#define HV_STDCALL
#endif

#ifndef HV_EXTERN_C
#ifdef __cplusplus
#define HV_EXTERN_C extern "C"
#define HV_DEFAULT(val) = val
#else
#define HV_EXTERN_C
#define HV_DEFAULT(val)
#endif
#endif

#ifndef HV_INLINE
#if defined __cplusplus
#define HV_INLINE inline
#elif (defined WIN32 || defined WIN64) && !defined __GNUC__
#define HV_INLINE __inline
#else
#define HV_INLINE static
#endif
#endif /* HV_INLINE */


#if (defined WIN32 || defined WIN64) && defined HVAPI_EXPORTS
#define HV_EXPORTS __declspec(dllexport)
#else
#define HV_EXPORTS
#endif

#ifndef HVAPI
#define HVAPI(rettype) HV_EXTERN_C HV_EXPORTS rettype HV_CDECL
#endif

#define HV_IMPL HV_EXTERN_C

// ImageCell API and callbacks calling convention
#if defined(HV_LINUX) || defined(HV_ARM9)
#define HV_API
#define HV_CALLBACK
#else
#define HV_API __stdcall
#define HV_CALLBACK __stdcall
#endif

//#if defined(HV_ARM9_)
//	extern void Uart_PutCh(INT data);
//#endif

#ifdef __cplusplus
#define LIBCALIB_BEGIN_DECLS extern "C" {
#define LIBCALIB_END_DECLS }
#define CFUNC_BEGIN_DECLS extern "C" {
#define CFUNC_END_DECLS }
#else
#define LIBCALIB_BEGIN_DECLS
#define LIBCALIB_END_DECLS
#define CFUNC_BEGIN_DECLS
#define CFUNC_END_DECLS
#endif


#if defined(_WIN32)||defined(_WIN64)
typedef signed char			MM_S08;
typedef char				MM_CHAR;
typedef unsigned char		MM_U08;
typedef signed short		MM_S16;
typedef unsigned short		MM_U16;
typedef signed int			MM_S32;
typedef unsigned int		MM_U32;
typedef float				MM_FLOAT;
typedef double				MM_DOUBLE;
typedef long long			MM_S64;
typedef unsigned long long  MM_U64;
#endif

/* for compatibility with pose estimation codes  */
#define	SBYTE		MM_S08
#define CHAR		MM_S08
#define BYTE		MM_U08
#define	SHORT		MM_S16
#define	USHORT		MM_U16
#define	INT			MM_S32
#define BOOL		MM_S32
#define	UINT		MM_U32
#define LONG		MM_S32
#define	FLOAT		MM_FLOAT
#define	DOUBLE		MM_DOUBLE


// MATH
#define hvAbs(x)			abs(x)
#define hvFabs(x)			(FLOAT)fabs(x)
#define hvMax(a, b)			((b) > (a) ? (b) : (a))
#define hvMin(a, b)			((b) < (a) ? (b) : (a))
#define hvRound(x)			((INT)((x) >= 0.0f ? (x) + 0.5f : (x) - 0.5f)) /* round macro */
#define hvRoundP(x)			((INT)((x) + 0.5f))                            /* round macro for positive whole number */ 
#define hvSqr(x)			((x) * (x))
#define hvCliping(data)		( (data) < 0 ? 0 : ( (data) > 255 ) ? 255 : ( data ) )
#define HV_CAST_8U(t)		(BYTE)(!((t) & ~255) ? (t) : (t) > 0 ? 255 : 0)
#define SWAP(a,b,t)			((t) = (a), (a) = (b), (b) = (t))
#define hvPositive(a)		(((a)>0)? 1:0) 

// PI
#define HV_PI				(3.1415926535897931f)
#define HV_SQRT_PI			(1.772454f)
#define HV_RAD_TO_DEG		(57.295780f)
#define HV_DEG_TO_RAD		(0.017453f)
#define hvRadToDirF(r)		((r) * HV_RAD_TO_DEG)
#define hvRadToDir(r)		 hvRound(hvRadToDirF(r))
#define hvDirToRad(d)		 ((d) * HV_DEG_TO_RAD)


// ERROR CODES FOR POSE ESTIMATION //
#define HV_OK                  ((MM_S16)(0))
#define HV_FAILED              ((MM_S16)(-1))
#define HV_OUT_OF_MEMORY       ((MM_S16)(-2))
#define HV_NOT_INITIALIZED     ((MM_S16)(-3))
#define HV_ARGUMENT_NULL       ((MM_S16)(-4))
#define HV_INVALID_ARGUMENT    ((MM_S16)(-5))
#define HV_NOT_IMPLEMENTED     ((MM_S16)(-9))
#define HV_SIZE_MISMATCH	   ((MM_S16)(-10))

#define hvFailed(result) ((result) < 0)
#define hvSucceeded(result) ((result) >= 0)


/****************************************************************************
*    definition of error codes  for factory and service calibration         *
****************************************************************************/
#define     CAM_BIT_MASK(cam_idx)				(((MM_U08)0x01) << (MM_U08)(cam_idx))
#define     SWP_BIT_MASK                        (((MM_U08)0x01) << (MM_U08)(4))
#define     FACTORY_CAL_TYPE_MASK               (((MM_U08)0x01) << (MM_U08)(5))
#define     SERVICE_CAL_TYPE_MASK               (((MM_U08)0x01) << (MM_U08)(6))


//ERROR CODES FOR COMMON						RANGE:     -1 ~ -499
#define     SUCCESS                               ((MM_S16)(0))  /* don't confuse TRUE or FALSE */
#define		ERROR								  ((MM_S16)(-1))
#define		ERROR_MEMORY_ALLOCATION				  ((MM_S16)(-100))
#define		ERROR_MEMORY_FREE					  ((MM_S16)(-110))
#define		ERROR_OUT_OF_RANGE					  ((MM_S16)(-200))

#define		is_failed(ecode) ((ecode) < 0)
#define		is_successed(ecode) ((ecode) >= 0)

//ERROR CODES FOR POSE ESTIMATION				RANGE: -1000 ~ -1999
#define     ERROR_POSE                             ((MM_S16)(-1000))
#define     ERROR_POSE_INITAIL                     ((MM_S16)(-1100)) 
#define     ERROR_POSE_ESTIMATION                  ((MM_S16)(-1200))
#define     ERROR_POSE_LAST                        ((MM_S16)(-1999)) 
#define     is_pose_error(ecode)				   ((ERROR_POSE_LAST<=(ecode))&& \
												   ((ecode)<= ERROR_POSE))

//ERROR CODES FOR SWP FILES						RANGE: -2000 ~ -2999 
#define     ERROR_SWP							   ((MM_S16)(-2000))

#define     ERROR_SWP1_CAR						   ((MM_S16)(-2100))
#define     ERROR_SWP1_CAR_LENGTH				   ((MM_S16)(-2101))
#define     ERROR_SWP1_CAR_FRONT_OVERHANG		   ((MM_S16)(-2102))
#define     ERROR_SWP1_CAR_WHEEL_BASE			   ((MM_S16)(-2103))
#define     ERROR_SWP1_CAR_REAR_OVERHANG		   ((MM_S16)(-2104))
#define     ERROR_SWP1_CAR_WIDTH				   ((MM_S16)(-2105))
#define     ERROR_SWP1_CAR_TRACK				   ((MM_S16)(-2106))
#define     ERROR_SWP1_CAR_HEIGHT				   ((MM_S16)(-2107))
#define     ERROR_SWP1_LAST						   ((MM_S16)(-2119))

#define     is_swp1_error(ecode)				   ((ERROR_SWP1_LAST<=(ecode))&& \
												   ((ecode)<= ERROR_SWP1_CAR))

#define     ERROR_SWP3                             ((MM_S16)(-2300))    
#define     ERROR_SWP3_FOCAL_LENGTH                ((MM_S16)(-2301))
#define     ERROR_SWP3_CELL_SIZE                   ((MM_S16)(-2302))
#define     ERROR_SWP3_OPTICAL_AXIS                ((MM_S16)(-2303))
#define     ERROR_SWP3_DISTORTION_K                ((MM_S16)(-2304))
#define     ERROR_SWP3_DISTORTION_D                ((MM_S16)(-2305))
#define     ERROR_SWP3_IMAGE_SIZE                  ((MM_S16)(-2306))
#define     ERROR_SWP3_FACTORY_POINT_NUM           ((MM_S16)(-2307)) 
#define     ERROR_SWP3_WORLD_OFFSET                ((MM_S16)(-2308))
#define     ERROR_SWP3_DEFAULT_EXTRINSIC           ((MM_S16)(-2309))
#define     ERROR_SWP3_WORLD_COORD                 ((MM_S16)(-2310)) 
#define     ERROR_SWP3_DISTANCE                    ((MM_S16)(-2311))
#define     ERROR_SWP3_FACTORY_ROI                 ((MM_S16)(-2312))  
#define     ERROR_SWP3_FACTORY_ROI_SIZE            ((MM_S16)(-2313))
#define     ERROR_SWP3_BIGGEST_SQUARE_SIZE         ((MM_S16)(-2314))
#define     ERROR_SWP3_SERVICE_ROI_POINTS          ((MM_S16)(-2315))
#define     ERROR_SWP3_SERVICE_POINT_NUM           ((MM_S16)(-2316)) 
#define     ERROR_SWP3_SERVICE_ROI_SIZE            ((MM_S16)(-2317))
#define     ERROR_SWP3_CALIBRATED_EXTRINSIC        ((MM_S16)(-2318)) 
#define     ERROR_SWP3_LAST                        ((MM_S16)(-2499))  
#define     is_swp3_error(ecode)				   ((ERROR_SWP3_LAST<=(ecode))&& \
                                                   ((ecode)<= ERROR_SWP3))

#define     ERROR_SWP_LAST                         ((MM_S16)(-2999))  
#define     is_swp_error(ecode)					   ((ERROR_SWP_LAST<=(ecode))&& \
                                                   ((ecode)<= ERROR_SWP))



//ERROR FOR FEATURE POINTS & PATTERNS                RANGE: -4000 ~ -4999
#define		ERROR_FEATURE						    ((MM_S16)(-4100))

#define     ERROR_FEATURE_UNIT                      ((MM_S16)(-4110))
#define     ERROR_FEATURE_UNIT_CORNER				((MM_S16)(-4111))
#define     ERROR_FEATURE_UNIT_PIXEL_ACCURACY       ((MM_S16)(-4112))
#define     ERROR_FEATURE_UNIT_BORDER_FOLLOWING		((MM_S16)(-4113))
#define     ERROR_FEATURE_UNIT_CALC_MIN				((MM_S16)(-4114))
#define     ERROR_FEATURE_UNIT_OVERLAP				((MM_S16)(-4115))
#define     ERROR_FEATURE_UNIT_POINT_NUM            ((MM_S16)(-4116)) 
#define     ERROR_FEATURE_UNIT_INVALID_POINT        ((MM_S16)(-4117)) 
#define     ERROR_FEATURE_UNIT_LAST                 ((MM_S16)(-4199))
#define     is_feature_unit_error(ecode)            ((ERROR_FEATURE_UNIT_LAST<=(ecode))&& \
												    ((ecode)<= ERROR_FEATURE_UNIT))  

#define     ERROR_FEATURE_PATTERN                   ((MM_S16)(-4300)) 
#define     ERROR_FEATURE_PATTERN_BOTH              ((MM_S16)(-4301)) 
#define     ERROR_FEATURE_PATTERN_LEFT              ((MM_S16)(-4302)) 
#define     ERROR_FEATURE_PATTERN_RIGHT             ((MM_S16)(-4303))  
#define     ERROR_FEATURE_PATTERN_SINGLE		    ((MM_S16)(-4304)) 
#define     ERROR_FEATURE_PATTERN_CAMERA_FAULT      ((MM_S16)(-4305))
#define     ERROR_FEATURE_PATTERN_LAST              ((MM_S16)(-4399)) 
#define     is_feature_pattern_error(ecode)         ((ERROR_FEATURE_PATTERN_LAST<=(ecode))&& \
												    ((ecode)<= ERROR_FEATURE_PATTERN))  

#define     ERROR_FEATURE_LAST                      ((MM_S16)(-4999))

#define     is_feature_error(ecode)					((ERROR_FEATURE_LAST<=(ecode))&& \
													((ecode)<= ERROR_FEATURE))


#endif
