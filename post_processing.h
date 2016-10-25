#ifndef POST_PROCESSING_H
#define POST_PROCESSING_H

#include <string.h>

#include "_hvsys.h"
#include "calibration.h"

#define MAX_BORDERSIZE  ((MM_U16)(800))
#define BORDER_NUM		((MM_U16)65000)
#define REGION_NUM		((MM_U08)30)
#define BORDER_THRES	((MM_U16)850)


CFUNC_BEGIN_DECLS

typedef struct _BORDERFOLLOW
{
	MM_U16  x[MAX_BORDERSIZE];
	MM_U16  y[MAX_BORDERSIZE];
	MM_U16  n;
} BORDERFOLLOW;


void  change_point_order(
							MM_FLOAT outPts[],	/* [out] output points */
							MM_FLOAT srcPts[]);	/* [in] input points */

MM_S16  calc_min_dist(
						MM_FLOAT ret_ftpoint[],			/* [out] output corner points */
						MM_U16 *ret_ftnum,				/* [out] number of output points */
						BORDERFOLLOW stBorderInfo[],	/* [in] border points */
						MM_U08 border_num,				/* [in] number of border*/
						MM_FLOAT ftpoint[],				/* [in] input corner points */
						MM_U16 ftnum);					/* [in] number of input points */


MM_S16 borderFollowing(
						BORDERFOLLOW stBorderInfo[],	/* [out] border points  */
						MM_U08 *border_num,				/* [out] number of border */
						MM_U08 gray[],					/* [in] source image */
						MM_U16 height,					/* [in] height of image */
						MM_U16 width);					/* [in] width of image */
						

MM_S16  CirIntFR13x13(
						MM_FLOAT fOutPits01[],		/* [out] corner points */
						MM_U16 *nOP01,				/* [out] number of points */
						MM_U08 grayImage[],			/* [in] source image */
						MM_U16 iheight,				/* [in] height of image */
						MM_U16 iWidth,				/* [in] width of image */
						MM_FLOAT fsigPoints01[],	/* [in] corner points */
						MM_U16 nP01);				/* [in] number of points */

MM_S16  overlapPoints(
						MM_FLOAT outPoints[],	/* [out] output corner points */
						MM_U16 *numOut,			/* [out] number of ouput points */
						MM_FLOAT inPoints[],	/* [in] input corner points */
						MM_U16 numIn);			/* [in] number of input points */

CFUNC_END_DECLS

#endif

