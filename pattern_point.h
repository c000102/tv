#ifndef PAATERN_POINT_H
#define PAATERN_POINT_H

#include <string.h>
#include <math.h>

#include "_hvsys.h"
#include "calibration.h"

CFUNC_BEGIN_DECLS

MM_S16  find_pattern_points(
							MM_FLOAT *corPoints,				/* [out] corner points */
							MM_U08 *srcImgC4,					/* [in] source image in 4 chanel */
							MM_U08 ID,							/* [in] camear type */
							MM_U08 CalibType,					/* [in] calibration type(factory, service) */
							CALIBRATION_PARAM *cal_param);/* [in] calibration parameters */
CFUNC_END_DECLS

#endif
