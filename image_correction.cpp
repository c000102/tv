#include "image_correction.h"

#define OpCv (1)

#if OpCv
#include "opencv2\opencv.hpp"
using namespace cv;
#endif

#if OpCv
static void uchar2matrgb_all(MM_U08 *in, Mat out_img);
#endif

static MM_U08 upt_img[IMG_H*IMG_W];
static HvDstTable DstTable[DIST_TABLE_SIZE] = { 0 };

MM_VOID CAPI image_correction_pt(
	HvPoint2D32f *upt,	/* [out] undistortion image points */
	HvPoint2D32f *ipt)	/* [in] distortion image points */
{

	HvFishCameraData cam; /* camera information inculude lens distortion data */
	HvPoint2D32f dipt;

	memset(&dipt, 0x00, sizeof(HvPoint2D32f));

	/* initialize and set a structure for camera parameters */
	memset(&cam, 0x00, sizeof(HvFishCameraData));
	initialize_camera_data(&cam, 1.0f);


	dipt.x = ipt->x - cam.cx;
	dipt.y = ipt->y - cam.cy;

	dpt2upt(upt, &dipt, &cam);

	upt->x = upt->x + cam.cx;
	upt->y = upt->y + cam.cy;

}

MM_VOID CAPI image_correction_pt_bw(
									HvPoint2D32f *dpt,	/* [out] distortion image points */
									HvPoint2D32f *upt)	/* [in]  undistortion image points */
{

	HvFishCameraData cam; /* camera information inculude lens distortion data */
	HvPoint2D32f dipt;
	HvPoint2D32f uipt;

	memset(&dipt, 0x00, sizeof(HvPoint2D32f));

	/* initialize and set a structure for camera parameters */
	memset(&cam, 0x00, sizeof(HvFishCameraData));
	initialize_camera_data(&cam, 1.0f);

	uipt.x = upt->x;
	uipt.y = upt->y;

	/* undistortion point to distortion point */
	upt2dpt(&dipt, &uipt, &cam);

	dpt->x = dipt.x;
	dpt->y = dipt.y;

}

MM_VOID CAPI image_correction_img(MM_U08 *upt, MM_U08 *ipt, MM_U16 height, MM_U08 width, HvCameraData *cam_data)
{

#if OpCv
	MM_U08 crp1[IMG_H*IMG_W];
	Mat crp1_img = Mat::zeros(IMG_H, IMG_W, CV_8UC1);
	Mat input_distorted_imgC3 = Mat::zeros(IMG_H, IMG_W, CV_8UC3);

	char name[100];

	memset(crp1, 0x00, sizeof(crp1));
	memcpy(crp1, ipt, sizeof(crp1));
	(void)uchar2matrgb_all(crp1, input_distorted_imgC3);

	sprintf_s(name, ".\\output_image\\upt_image.bmp");
	imwrite(name, input_distorted_imgC3);

#endif

	HvFishCameraData cam;

	Mat undistorted_imgC3;
	Mat back_distorted_imgC3;


	/* undistorted image */
	undistorted_imgC3 = Mat::zeros(input_distorted_imgC3.rows, input_distorted_imgC3.cols, CV_8UC3);
	back_distorted_imgC3 = Mat::zeros(input_distorted_imgC3.rows, input_distorted_imgC3.cols, CV_8UC3);

	memset((MM_U08*)undistorted_imgC3.data, 0x00, (input_distorted_imgC3.rows*input_distorted_imgC3.cols) * 3);
	memset((MM_U08*)back_distorted_imgC3.data, 0x00, (input_distorted_imgC3.rows*input_distorted_imgC3.cols) * 3);

	/* initialize and set a structure for camera parameters */
	memset(&cam, 0x00, sizeof(HvCameraData));
	initialize_camera_data(&cam, 1.0f);


	/* undistortion iamge */
	if (0) /* if you want the backward mapping, then you set 1. otherwise 1.*/
	{
		/* BACKWARD MAPPING(distorted image -> undistorted image) */
		d2u_img_bw(
			(MM_U08*)undistorted_imgC3.data,  /* output */
			undistorted_imgC3.cols,
			undistorted_imgC3.rows,
			(MM_U08*)input_distorted_imgC3.data, /* input */
			input_distorted_imgC3.cols,
			input_distorted_imgC3.rows,
			input_distorted_imgC3.channels(),
			&cam);

		/* BACKWARD MAPPING(undistorted image -> distorted image) */
		u2d_img_bw(
			(MM_U08*)back_distorted_imgC3.data, /* output */
			back_distorted_imgC3.cols,
			back_distorted_imgC3.rows,
			(MM_U08*)undistorted_imgC3.data, /* input */
			undistorted_imgC3.cols,
			undistorted_imgC3.rows,
			undistorted_imgC3.channels(),
			&cam);
	}
	else
	{
		/* FORWARD MAPPING(distorted image -> undistored image) */
		d2u_img_fw(
			(MM_U08*)undistorted_imgC3.data,  /* output */
			undistorted_imgC3.cols,
			undistorted_imgC3.rows,
			(MM_U08*)input_distorted_imgC3.data, /* input */
			input_distorted_imgC3.cols,
			input_distorted_imgC3.rows,
			input_distorted_imgC3.channels(),
			&cam);

		/* FORWARD MAPPING(undistorted image -> distorted image) */
		u2d_img_fw(
			(MM_U08*)back_distorted_imgC3.data, /* output */
			back_distorted_imgC3.cols,
			back_distorted_imgC3.rows,
			(MM_U08*)undistorted_imgC3.data, /* input */
			undistorted_imgC3.cols,
			undistorted_imgC3.rows,
			undistorted_imgC3.channels(),
			&cam);
	}

	/* output file */
#if (0)
	//imwrite(OUTPUT_FILE(input_distorted_image.bmp), input_distorted_imgC3);
	imwrite(".\\output_image\\undistorted_image.bmp", undistorted_imgC3);
	//imwrite(OUTPUT_FILE(back_distorted.bmp), back_distorted_imgC3);
#endif

	namedWindow("input distorted image", CV_WINDOW_AUTOSIZE);
	namedWindow("undistorted image", CV_WINDOW_AUTOSIZE);
	namedWindow("back distorted image", CV_WINDOW_AUTOSIZE);

	imshow("input distorted image", input_distorted_imgC3);
	imshow("undistorted image", undistorted_imgC3);
	imshow("back distorted image", back_distorted_imgC3);

	cvWaitKey(0);
}

MM_VOID CAPI initialize_camera_data(HvFishCameraData *camera, MM_FLOAT focal_lengh_scale)
{
	/* distorted image */
	camera->dimg_width = 1280;
	camera->dimg_height = 720;
	camera->dcx = (MM_FLOAT)camera->dimg_width / 2.0f;
	camera->dcy = (MM_FLOAT)camera->dimg_height / 2.0f;
	camera->f = 1.052720f; /* focal length(mm) */
	camera->scw = 0.0028f; /* width of sensor's cell(mm) */
	camera->sch = 0.0028f; /* height of sensor's cell(mm( */
	camera->scale = focal_lengh_scale; /* scaling the focal length when the undistorted image is made. */

									   /* undistorted image */
	camera->img_width = camera->dimg_width;
	camera->img_height = camera->dimg_height;
	camera->cx = (MM_FLOAT)camera->img_width / 2.0f; /* x-coordinate of the optical center */
	camera->cy = (MM_FLOAT)camera->img_height / 2.0f;/* y-coordinate of the optical center */
	camera->mu = 1.0f / camera->scw;
	camera->mv = 1.0f / camera->sch;
	camera->fm = (camera->scale * camera->f) * (camera->mu + camera->mv) / 2.0f;
	camera->fmu = (camera->scale * camera->f) * camera->mu;
	camera->fmv = (camera->scale * camera->f) * camera->mv;

	/* distortion coefficients */
	camera->k[0] = 1.05272000f;
	camera->k[1] = 0.07693000f;
	camera->k[2] = -0.05123000f;
	camera->k[3] = 0.01285000f;
	camera->k[4] = -0.00137000f;

	/* undistortion coefficients */
	camera->d[0] = 0.00000796f;
	camera->d[1] = 0.94941435f;
	camera->d[2] = 0.00640925f;
	camera->d[3] = -0.09443732f;
	camera->d[4] = 0.07513426f;
	camera->d[5] = -0.03511292f;
	camera->d[6] = 0.03167442f;
	camera->d[7] = -0.01777235f;
	camera->d[8] = 0.00363199f;
}

MM_VOID CAPI upt2dpt(	HvPoint2D32f* dpt,	/* [out] distorted floating-type point */
						HvPoint2D32f* upt,	/* [in] undistorted floating-type point */
						HvFishCameraData* cam) /* [in] camera information */
{
	MM_FLOAT ux = 0.0f;
	MM_FLOAT uy = 0.0f;
	MM_FLOAT ru = 0.0f; /* radius(distance) in an undistorted image */
	MM_FLOAT rd = 0.0f; /* radius(distance) in a distorted image  */
	MM_FLOAT th = 0.0f; /* theta in radian    */


	ux = (MM_FLOAT)(upt->x * cam->f / cam->fmu);
	uy = (MM_FLOAT)(upt->y * cam->f / cam->fmv);

	ru = (MM_FLOAT)sqrt(ux*ux + uy*uy);

	if ((2.0f / (cam->mu + cam->mv)) < ru)
	{
		th = (MM_FLOAT)atan(ru / cam->f);

		rd = (MM_FLOAT)(cam->k[0] * th +
			cam->k[1] * (MM_FLOAT)pow(th, 3) +
			cam->k[2] * (MM_FLOAT)pow(th, 5) +
			cam->k[3] * (MM_FLOAT)pow(th, 7) +
			cam->k[4] * (MM_FLOAT)pow(th, 9));

		dpt->x = ((rd / ru)* ux *cam->mu);
		dpt->y = ((rd / ru)* uy *cam->mv);
		
	}
	else
	{
		dpt->x = 0.0f;
		dpt->y = 0.0f;
	}
}



MM_VOID CAPI dpt2upt(	HvPoint2D32f* upt, /* [out] undistorted floatintg-type point */
						HvPoint2D32f* dpt, /* [in] distorted floating-type point, known value */
						HvFishCameraData* cam) /* [in] camera information */
{
	MM_FLOAT dx = 0.0f; /* x coordinate of distorted image */
	MM_FLOAT dy = 0.0f; /* y coordinate of distorted image */
	MM_FLOAT rd = 0.0f; /* radius(distance) in a distorted image  */
	MM_FLOAT ru = 0.0f; /* radius(distance) in an undistorted image */
	MM_FLOAT th = 0.0f; /* theta */


	dx = dpt->x / cam->mu;
	dy = dpt->y / cam->mv;
	rd = (MM_FLOAT)sqrt(dx*dx + dy*dy);

	th = (MM_FLOAT)(cam->d[0] +
		cam->d[1] * rd +
		cam->d[2] * (MM_FLOAT)pow(rd, 2) +
		cam->d[3] * (MM_FLOAT)pow(rd, 3) +
		cam->d[4] * (MM_FLOAT)pow(rd, 4) +
		cam->d[5] * (MM_FLOAT)pow(rd, 5) +
		cam->d[6] * (MM_FLOAT)pow(rd, 6) +
		cam->d[7] * (MM_FLOAT)pow(rd, 7) +
		cam->d[8] * (MM_FLOAT)pow(rd, 8));

	ru = cam->f * (MM_FLOAT)tan(th);

	if ((2.0f / (cam->mu + cam->mv)) < rd)
	{
		upt->x = (MM_FLOAT)((ru / rd)*dx / cam->f) * cam->fmu;
		upt->y = (MM_FLOAT)((ru / rd)*dy / cam->f) * cam->fmv;
	}
	else
	{
		upt->x = 0.0f;
		upt->y = 0.0f;
	}
}


////////////////////////////////////////////
// BACKWARD MAPPING
////////////////////////////////////////////

MM_S16 CAPI d2u_img_bw(	MM_U08 *dst_img,       /* [out] undistorted image */
						MM_U16 dst_img_width,  /* [in] width of the undistorted image */
						MM_U16 dst_img_height, /* [in] height of the undistorted image */
						MM_U08 *src_img,       /* [in] distorted image */
						MM_U16 src_img_width,  /* [in] width of the distorted image */
						MM_U16 src_img_height, /* [in] height of the distorted image */
						MM_U16 chl_num,        /* [in] the number of channels of distorted and undistorted ones  */
						HvFishCameraData *cam) 	  /* [in] camera information structure */
{
	MM_S16       x, y, c;
	MM_S16       sx, sy;
	HvPoint2D32f upt; /* undistorted-image points */
	HvPoint2D32f dpt; /* distorted-image points */


	for (y = 0; y < dst_img_height; y++)
	{
		for (x = 0; x < dst_img_width; x++)
		{
			dpt.x = 0.0f;
			dpt.y = 0.0f;

			upt.x = (MM_FLOAT)(x - cam->cx);
			upt.y = (MM_FLOAT)(y - cam->cy);

			upt2dpt(&dpt, &upt, cam); /* backward mapping */

			sx = (MM_S16)(dpt.x + cam->dcx + 0.5f);
			sy = (MM_S16)(dpt.y + cam->dcy + 0.5f);

			if (((0 <= sx) && (sx < src_img_width)) &&
				((0 <= sy) && (sy < src_img_height)))
			{
				for (c = 0; c < chl_num; c++)
				{
					dst_img[(y*dst_img_width + x)*chl_num + c] =
						src_img[(sy * src_img_width + sx)*chl_num + c];
				}
			}
		} /* end-inner_for */
	} /* end-outer_for */

	return (MM_S16)MM_SUCCESS;
}



MM_S16 CAPI u2d_img_bw(MM_U08 *dst_img,       /* [out] distorted image */
	MM_U16 dst_img_width,  /* [in] width of the distorted image */
	MM_U16 dst_img_height, /* [in] height of the distorted image */
	MM_U08 *src_img,       /* [in] undistorted image */
	MM_U16 src_img_width,  /* [in] width of the undistorted image */
	MM_U16 src_img_height, /* [in] height of the undistorted image */
	MM_U16 chl_num,        /* [in] the number of channels of distorted and undistorted ones */
	HvFishCameraData *cam)    /* [in] camera information structure */
{
	MM_S32		 x, y, c;
	MM_S32       sx, sy;
	MM_FLOAT     radius, max_radius;
	HvPoint2D32f upt; /* undistorted-image points */
	HvPoint2D32f dpt; /* distorted-image points */


	max_radius = (cam->dimg_height < cam->dimg_width) ?
		cam->dimg_width * 0.45f : cam->dimg_height*0.45f;

	for (y = 0; y < dst_img_height; y++)
	{
		for (x = 0; x < dst_img_width; x++)
		{
			upt.x = 0.0f;
			upt.y = 0.0f;
			dpt.x = (MM_FLOAT)(x - cam->dcx);
			dpt.y = (MM_FLOAT)(y - cam->dcy);

			radius = (MM_FLOAT)sqrt(dpt.x * dpt.x + dpt.y *dpt.y);

			if (radius < max_radius)
			{
				dpt2upt(&upt, &dpt, cam); /* backward mapping */

				sx = (MM_S32)(upt.x + cam->cx + 0.5f);
				sy = (MM_S32)(upt.y + cam->cy + 0.5f);


				if (((0 <= sx) && (sx < src_img_width)) &&
					((0 <= sy) && (sy < src_img_height)))
				{
					for (c = (MM_U16)0; c < chl_num; c++)
					{
						dst_img[(y*dst_img_width + x)*chl_num + c] =
							src_img[(sy*src_img_width + sx)*chl_num + c];
					}
				}
			}
		} /* end-inner-for */
	}  /* end - outer-for */

	return (MM_S16)MM_SUCCESS;
}


/////////////////////////////////////////
// FORWARD MAPPING
/////////////////////////////////////////

MM_S16 CAPI d2u_img_fw(MM_U08 *dst_img,       /*[out] undistorted image */
	MM_U16 dst_img_width,  /*[in] width of the undistorted image */
	MM_U16 dst_img_height, /*[in] height of the undistorted image */
	MM_U08 *src_img,       /*[in] distorted image */
	MM_U16 src_img_width,  /*[in] width of the distorted image */
	MM_U16 src_img_height, /*[in] height of the distorted image */
	MM_U16 chl_num,        /*[in] the number of channels of distorted and undistorted images */
	HvFishCameraData *cam) 	  /*[in] camera information structure */
{
	MM_S32       x, y, c;
	MM_S32       dx, dy;
	HvPoint2D32f upt; /* undistorted-image points */
	HvPoint2D32f dpt; /* distorted-image points */

	for (y = 0; y < src_img_height; y++)
	{
		for (x = 0; x < src_img_width; x++)
		{
			upt.x = 0.0f;
			upt.y = 0.0f;
			dpt.x = (MM_FLOAT)(x - cam->dcx);
			dpt.y = (MM_FLOAT)(y - cam->dcy);

			dpt2upt(&upt, &dpt, cam); /* forward mapping */

			dx = (MM_S32)(upt.x + cam->cx + 0.5f);
			dy = (MM_S32)(upt.y + cam->cy + 0.5f);

			if (((0 <= dx) && (dx < dst_img_width)) &&
				((0 <= dy) && (dy < dst_img_height)))
			{
				for (c = 0; c < chl_num; c++)
				{
					dst_img[(dy*dst_img_width + dx)*chl_num + c] =
						src_img[(y * src_img_width + x)*chl_num + c];
				}
			}
		} /* end-inner_for */
	} /* end-outer_for */

	return (MM_S16)MM_SUCCESS;
}



MM_S16 CAPI u2d_img_fw(MM_U08 *dst_img,       /* [out] distored image */
	MM_U16 dst_img_width,  /* [in] width of the distorted image */
	MM_U16 dst_img_height, /* [in] height of the distorted image */
	MM_U08 *src_img,       /* [in] undistorted image */
	MM_U16 src_img_width,  /* [in] width of the undistorted image */
	MM_U16 src_img_height, /* [in] height of the undistorted image */
	MM_U16 chl_num,        /* [in] the number of channels  */
	HvFishCameraData *cam)    /* [in] camera information structure */
{
	MM_S16		 x, y, c;
	MM_S16       dx, dy;
	HvPoint2D32f upt; /* undistorted-image points */
	HvPoint2D32f dpt; /* distorted-image points */

	for (y = 0; y < src_img_height; y++)
	{
		for (x = 0; x < src_img_width; x++)
		{
			dpt.x = 0.0f;
			dpt.y = 0.0f;
			upt.x = (MM_FLOAT)(x - cam->cx);
			upt.y = (MM_FLOAT)(y - cam->cy);

			upt2dpt(&dpt, &upt, cam); /* forward mapping */

			dx = (MM_S16)(dpt.x + cam->dcx + 0.5f);
			dy = (MM_S16)(dpt.y + cam->dcy + 0.5f);

			if (((0 <= dx) && (dx < dst_img_width)) &&
				((0 <= dy) && (dy < dst_img_height)))
			{
				for (c = (MM_U16)0; c < chl_num; c++)
				{
					dst_img[(dy*dst_img_width + dx)*chl_num + c] =
						src_img[(y * src_img_width + x)*chl_num + c];
				}
			}
		} /* end-inner-for */
	}  /* end - outer-for */

	return (MM_S16)MM_SUCCESS;
}



/* this code is the orignal source from Mr. Han */
/*
void hvFeatureUndistortion(HvPoint2D32f* dpt, HvPoint2D32f* upt, HvCameraData *settings)
{
MM_FLOAT x,y;
MM_FLOAT xu,yu;
MM_FLOAT rd,ru;
MM_FLOAT th;

x = (dpt->x - settings->cx)/settings->mu;
y = (dpt->y - settings->cy)/settings->mv;

rd = (float)sqrt(x*x + y*y);

th = settings->d[0] +
settings->d[1]*rd +
settings->d[2]*pow(rd,2) +
settings->d[3]*pow(rd,3) +
settings->d[4]*pow(rd,4) +
settings->d[5]*pow(rd,5) +
settings->d[6]*pow(rd,6) +
settings->d[7]*pow(rd,7) +
settings->d[8]*pow(rd,8);

ru = settings->f * tan(th);

if(rd == 0)
{
xu = 0;
yu = 0;
}
else
{
xu = x/rd*ru;
yu = y/rd*ru;
}

upt->x = xu;
upt->y = yu;
}

void hvFeatureDistortion(HvPoint2D32f* upt,HvPoint2D32f* dpt, HvCameraData * setting)
{
MM_FLOAT x;
MM_FLOAT y;
MM_FLOAT ru;
MM_FLOAT th;
MM_FLOAT rd;

x = ((MM_FLOAT)upt->x)/setting->f;
y = ((MM_FLOAT)upt->y)/setting->f;

ru = sqrt(x*x + y*y);
th = (MM_FLOAT)atan(ru);

rd = setting->k[0]*th +
setting->k[1]*pow(th,3) +
setting->k[2]*pow(th,5) +
setting->k[3]*pow(th,7) +
setting->k[4]*pow(th,9);

//dpt->x = rd/ru * x * settings->mu * 720/640 + settings->cx + (720-640)/2;

dpt->x = rd/ru * x * setting->mu + setting->cx;
dpt->y = rd/ru * y * setting->mv + setting->cy;
}
*/


#if OpCv
//ONLY FOR DEBUGGING
static void uchar2matrgb_all(MM_U08 *in, Mat out_img)
{
	MM_U16 i = 0, j = 0;
	for (i = 0; i < IMG_H; i++)
	{
		for (j = 0; j < IMG_W; j++)
		{
			out_img.at<uchar>(i, j * 3 + 0) = in[i*IMG_W + j];
			out_img.at<uchar>(i, j * 3 + 1) = in[i*IMG_W + j];
			out_img.at<uchar>(i, j * 3 + 2) = in[i*IMG_W + j];
		}
	}
} // ONLY FOR DEBUGGING
#endif

static void mtxPrint(char *str, MM_FLOAT *A, MM_S32 m, MM_S32 n)
{
	MM_S32 i = 0, j = 0;

	printf(str, m, n);

	for (i = 0; i<m; i++)
	{
		for (j = 0; j<n; j++)
		{
			if ((j%n) == 0)
			{
				printf("\n");
			}
			printf("%16.6lf ", A[i*n + j]);
		}
	}
	printf("\n\n");
}