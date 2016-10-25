#include "describe_topview.h"
#include "image_correction.h"

#define OpCv (1)

#if OpCv
#include "opencv2\opencv.hpp"
using namespace cv;
#endif

CFUNC_BEGIN_DECLS

/* local function */
static void  convert_RGBA2gray(
								MM_U08 output_img[],	/* [out] gray image */
								MM_U08 src[],			/* [in] source iamge(RGBA) */
								MM_U16 height,			/* [in] height of image */
								MM_U16 width,			/* [in] width of image */
								MM_U16 ID);				/* [in] camera number */

static void  cvtC3toC1(
						MM_U08 dstC1[],		/* [out] gray image */
						MM_U08 srcC3[],		/* [in] source iamge at 3 chanel */
						MM_U16 iHEIGHT,		/* [in] height of image*/
						MM_U16 iWidth);		/* [in] width of image */

static void dipt2wipt(
	HvPoint2D32f *wipt,		/* [out] world point on image(2D) */
	MM_U08 *img_value,		/* [out] intensity value on image */
	HvPoint2D32f *dipt,		/* [in] distortion points(2D) */
	MM_U08 *img,			/* [in] source image */
	HvCameraData *cam_data,	/* [in] information about calibraiton */
	MM_FLOAT tv_resolution
);

static void wipt2dipt(
	HvPoint2D32f *dipt,		/* [out] world point on image(2D) */
	HvPoint2D32f *wipt,		/* [in] distortion points(2D) */
	HvCameraData *cam_data,	/* [in] information about calibraiton */
	MM_FLOAT tv_resolution,
	MM_S16 ID
);

static void make_H_invH(
	MM_FLOAT *H,
	MM_FLOAT *invH,
	HvCameraData *cam_data
);

static void mtxPrint(char *str, MM_FLOAT *A, MM_S32 m, MM_S32 n);

CFUNC_END_DECLS

#if OpCv
	static void uchar2matrgb_all(MM_U08 *in, Mat out_img);
#endif

MM_U08 gray_img[IMG_H*IMG_W];
static MM_U08 upt_img[IMG_H*IMG_W];
static HvPoint2D32f wipt[4] = { { 0.0, }, };


MM_S16  describe_topview(	MM_U08				*front_img,		/* [in] front camera input, color space: RGBA, byte ordering: ABGR */
							MM_U08				*rear_img,		/* [in] rear camera input */
							MM_U08				*left_img,		/* [in] left camera input */
							MM_U08				*right_img,		/* [in] right camera input */
							MM_U16				img_height,		/* [in]  image height */
							MM_U16				img_width,		/* [in]  image width */
							CALIBRATION_PARAM	*cal_param,		/* [in]  information about calibraiton */
							HvCameraData *cam_data,
							MM_S16 ID)		  
{
	HvPoint2D32f dipt[4];
	MM_U08 img_value = 0;

	MM_CHAR name[256];
	MM_CHAR nameID[256];
	MM_U32 x = 0;
	MM_U32 y = 0;
	MM_U32 ux = 0;
	MM_U32 vy = 0;
	MM_U32 tv_cx = 0, tv_cy = 0;
	MM_U32 tv_height = 0, tv_width = 0;

	MM_FLOAT tv_resolution = 0.0f;

	memset(dipt, 0x00, sizeof(HvPoint2D32f) * 4);
	memset(nameID, 0x00, sizeof(MM_CHAR) * 256);

	tv_cy = 300;	/* top view of y center point in world coordinates */
	tv_cx = 400;	/* top view of x center point in world coordinates */
	tv_height = 1000;
	tv_width = 800;

#if (OpCv_)
	Mat src_img = Mat::zeros(IMG_H, IMG_W, CV_8UC3);

	for (int h = 0; h < IMG_H; h++)
	{
		for (int w = 0; w < IMG_W; w++)
		{
			src_img.at<uchar>(h, w * 3 + 0) = front_img[4 * (h*(IMG_W + 32) + w) + 3];
			src_img.at<uchar>(h, w * 3 + 1) = front_img[4 * (h*(IMG_W + 32) + w) + 2];
			src_img.at<uchar>(h, w * 3 + 2) = front_img[4 * (h*(IMG_W + 32) + w) + 1];
		}
	}
#endif
	Mat world_map = Mat::zeros(1000, 800, CV_8UC3);

	if (ID == (MM_S16)(0)) convert_RGBA2gray(gray_img, front_img, IMG_H, IMG_W, ID);
	if (ID == (MM_S16)(1)) convert_RGBA2gray(gray_img, rear_img, IMG_H, IMG_W, ID);
	if (ID == (MM_S16)(2)) convert_RGBA2gray(gray_img, left_img, IMG_H, IMG_W, ID);
	if (ID == (MM_S16)(3)) convert_RGBA2gray(gray_img, right_img, IMG_H, IMG_W, ID);


#if (0)//forward mapping
	tv_resolution = ((MM_FLOAT)(0.1f));
	circle(world_map, Point(tv_cx, tv_cy), 2, CV_RGB(255, 255, 255), -1);

	if (ID == 0)
	{
		//printf("\nditortion point\n");
		for (vy = 0; vy < img_height; vy++)
		{
			for (ux = 0; ux < img_width; ux++)
			{
				dipt[ID].x = (MM_FLOAT)(ux);
				dipt[ID].y = (MM_FLOAT)(vy);

				/*if (ux == 1201 && vy == 220)
				{
					printf("================================%4d\n", ux);
				}*/

				/* transite distortion image to world  points image */
				dipt2wipt(&wipt[ID], &img_value, &dipt[ID], gray_img, cam_data, tv_resolution);

				x = (MM_U32)(wipt[ID].x) + tv_cx;
				y = (MM_U32)(wipt[ID].y) + tv_cy;

				if ((x < tv_width && x > 0) && (y < tv_height && y>0))
				{
					//printf("[%5d, %5d] = %d\n", x, y, img_value);
					world_map.at<uchar>(y, 3 * x + 0) = img_value;
					world_map.at<uchar>(y, 3 * x + 1) = img_value;
					world_map.at<uchar>(y, 3 * x + 2) = img_value;
				}

			}
		}

		if (ID == 0) { strcpy(nameID, "00"); }
		//if (ID == 1) { strcpy(nameID, "01"); }
		//if (ID == 2) { strcpy(nameID, "02"); }
		//if (ID == 3) { strcpy(nameID, "03"); }

		sprintf_s(name, ".\\output_image\\%s_%3.1f_dtopview_fw.jpg", nameID, tv_resolution);
		imwrite(name, world_map);
	}

	/*if (ID == 3)
	{
		sprintf_s(name, ".\\output_image\\topview.bmp");
		imwrite(name, world_map);


		namedWindow("world_map", WINDOW_NORMAL);
		imshow("world_map", world_map);
		resizeWindow("world_map", 1000, 800);


		waitKey(0);
	}*/
#else //backword mapping

#define debugIMG (0)

	tv_resolution = ((MM_FLOAT)(10.0f));

	circle(world_map, Point(tv_cx, tv_cy), 2, CV_RGB(255, 255, 255), -1);

	if (ID == 0)
	{
		for (vy = 0; vy < tv_height; vy++)
		{
			for (ux = 0; ux < tv_width; ux++)
			{
				//printf("(%4d, %4d)\t", ux, vy);
#if (debugIMG)
				printf("\n\n================================\n");
				printf("top view in image coordinates: (%d, %d)\n", ux, vy);
#endif
				wipt[ID].x = (MM_FLOAT)(ux)-tv_cx;
				wipt[ID].y = (MM_FLOAT)(vy)-tv_cy;

				/* transite distortion image to world  points image */
				/*if (vy == 285) {
					printf("================================%4d\n", ux);
				}*/
#if (debugIMG)
				printf("top view in image center coordinates: (%f, %f)\n", wipt[ID].x, wipt[ID].y);
#endif
				wipt2dipt(&dipt[ID], &wipt[ID], cam_data, tv_resolution, ID);

				

				dipt[ID].x = dipt[ID].x + cam_data->cx;
				dipt[ID].y = dipt[ID].y + cam_data->cy;

				if ((dipt[ID].x == 0.0f) && (dipt[ID].y == 0.0f))
				{
					x = 0;
					y = 0;
				}
				else
				{
					x = (MM_U32)(dipt[ID].x + 0.5);
					y = (MM_U32)(dipt[ID].y + 0.5);
				}
				
#if (debugIMG)
				printf("distorted image point: (%d, %d)\n", x, y);
#endif

				if ((x < img_width && x > 0) && (y < img_height && y>0))
				{
					//printf("%d, %d = %d\n", x, y, img_value);
					img_value = gray_img[y*img_width+x];
					
					//printf("[%5d, %5d] = %d\n", x, y, img_value);
					world_map.at<uchar>(vy, 3 * ux + 0) = img_value;
					world_map.at<uchar>(vy, 3 * ux + 1) = img_value;
					world_map.at<uchar>(vy, 3 * ux + 2) = img_value;
				}

			}
		}

	}


	/* Display Top-view */
	if (ID == 0)
	{
		/*if (ID == 0) { strcpy(nameID, "00"); }
		if (ID == 1) { strcpy(nameID, "01"); }
		if (ID == 2) { strcpy(nameID, "02"); }
		if (ID == 3) { strcpy(nameID, "03"); }
		
		sprintf_s(name, ".\\output_image\\%s_%3.1f_dtopview_bw.jpg", nameID,tv_resolution);
		imwrite(name, world_map);*/


		namedWindow("world_map", WINDOW_NORMAL);
		imshow("world_map", world_map);
		resizeWindow("world_map", 1000, 800);


		waitKey(0);

		world_map.release();
	}

#endif

	return 0;
}

static MM_U08 tmp_img[IMG_H * IMG_W * 3]; /* temporary image at 3 chanel */
void  convert_RGBA2gray(
						MM_U08 output_img[],				/* [out] gray image */
						MM_U08 src[],					/* [in] source iamge(RGBA) */
						MM_U16 height,						/* [in] height of image */
						MM_U16 width,						/* [in] width of image */
						MM_U16 ID)							/* [in] camera number */
							
{
	MM_U32 y = (MM_U32)0;
	MM_U32 x = (MM_U32)0;
	MM_U32 xx = (MM_U32)0;
	MM_U32 yy = (MM_U32)0;

	MM_U32 index = (MM_U32)0;
	MM_U32 srcWidth = (MM_U32)0;

	srcWidth = width + 32;

	memset(tmp_img, 0x00, sizeof(MM_U08)*height*width*(MM_U32)(3));


	for (y = 0; y<(MM_U32)(height); y++) {
		for (x = 0; x<(MM_U32)(width); x++) {

			index = (MM_U32)(((MM_U32)(3)*((y *width) + x)));

			tmp_img[index + (MM_U32)(0)] = (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* srcWidth) + (MM_U32)(x))) + (MM_U32)(3))];
			tmp_img[index + (MM_U32)(1)] = (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* srcWidth) + (MM_U32)(x))) + (MM_U32)(2))];
			tmp_img[index + (MM_U32)(2)] = (MM_U08)src[(MM_U32)(((MM_U32)(4)*(((MM_U32)(y)* srcWidth) + (MM_U32)(x))) + (MM_U32)(1))];

		}
	}

	cvtC3toC1(output_img, tmp_img, height, width);

}

static MM_U32 LUT_data[768]; /* gray image Lookup table memory */
static MM_U08 temp_crop_img[IMG_H*IMG_W]; /* MISRA-C:2012  R.19.1 */

void  cvtC3toC1(MM_U08 dstC1[],			/* [out] gray image */
				MM_U08 srcC3[],	/* [in] source iamge at 3 chanel */
				MM_U16 iHEIGHT,			/* [in] height of image*/
				MM_U16 iWidth)			/* [in] width of image */
{
	MM_U32 n = (MM_U32)0;
	MM_U32 i = (MM_U32)0;
	MM_U32 b = (MM_U32)0;
	MM_U32 g = (MM_U32)0;
	MM_U32 r = (MM_U32)0;
	MM_U32 db = (MM_U32)0;
	MM_U32 dg = (MM_U32)0;
	MM_U32 dr = (MM_U32)0;

	//memset(LUT_data, 0x00, sizeof(MM_U32) * 768);

	n = (MM_U32)(iHEIGHT) * (MM_U32)(iWidth);
	r = ((MM_U32)(1) << (MM_U32)(13)); //13 = 14-1 : replaced

	db = (MM_U32)4899;
	dg = (MM_U32)9617;
	dr = (MM_U32)1868;

	for (i = 0; i < (MM_U32)(256); i++)
	{
		LUT_data[i] = b;
		LUT_data[i + (MM_U32)(256)] = g;
		LUT_data[i + (MM_U32)(512)] = r;

		b += (MM_U32)(db);
		g += (MM_U32)(dg);
		r += (MM_U32)(dr);
	}

	memset(temp_crop_img, 0x00, sizeof(MM_U08)*IMG_H*IMG_W); /* MISRA-C:2012  R.19.1 */

	for (i = 0; i < n; i++)
	{
		//dstC1[i] = (MM_U08)((LUT[srcC3[0]] + 
		//	                 LUT[srcC3[(MM_U32)(1)] + (MM_U32)(256)] + 
		//	                 LUT[srcC3[2] + (MM_U32)(512)]) >> (MM_U08)(14));

		temp_crop_img[i] = (MM_U08)((LUT_data[srcC3[0]] +
									 LUT_data[srcC3[(MM_U32)(1)] + (MM_U32)(256)] +
									 LUT_data[srcC3[2] + (MM_U32)(512)]) >> (MM_U08)(14));

		srcC3 += (MM_U08)(3);
	}

	memcpy(dstC1, temp_crop_img, sizeof(MM_U08)*IMG_H*IMG_W); /* MISRA-C:2012  R.19.1 */
}

static void dipt2wipt(
						HvPoint2D32f *wipt,		/* [out] world point on image(2D) */
						MM_U08 *img_value,		/* [out] intensity value on image */
						HvPoint2D32f *dipt,		/* [in] distortion points(2D) */
						MM_U08 *img,			/* [in] source image */
						HvCameraData *cam_data,	/* [in] information about calibraiton */
						MM_FLOAT tv_resolution
)
{
	HvPoint2D32f upt;	/* undistorted-image points */
	MM_FLOAT mat_H[9];
	MM_FLOAT mat_invH[9];

	MM_FLOAT vec_wpt[3];
	MM_FLOAT vec_ipt[3];

	MM_U32 x = 0;
	MM_U32 y = 0;

	memset(mat_H, 0x00, sizeof(MM_FLOAT) * 9);
	memset(mat_invH, 0x00, sizeof(MM_FLOAT) * 9);

	memset(vec_wpt, 0x00, sizeof(MM_FLOAT) * 3);
	memset(vec_ipt, 0x00, sizeof(MM_FLOAT) * 3);

	/* point based on image correction */
	image_correction_pt(&upt, dipt);

	/* make Homography */
	make_H_invH(mat_H, mat_invH, cam_data);


	vec_ipt[0] = upt.x;
	vec_ipt[1] = upt.y;
	vec_ipt[2] = 1.0f;
	MatMul(vec_wpt, mat_invH, 3, 3, vec_ipt, 3, 1);
	vec_wpt[0] = (vec_wpt[0] / vec_wpt[2]);
	vec_wpt[1] = (vec_wpt[1] / vec_wpt[2]) + 1389.24097f;
	vec_wpt[2] = vec_wpt[2] / vec_wpt[2];
	//mtxPrint("new world points", vec_wpt[i], 3, 1);

	wipt->x = vec_wpt[0] * tv_resolution;
	wipt->y = vec_wpt[1] * tv_resolution;

	x = (MM_U32)(dipt->x+0.5);
	y = (MM_U32)(dipt->y+0.5);

	*(img_value) = img[y * 1280 + x];

}

static void wipt2dipt(
	HvPoint2D32f *dipt,		/* [out] world point on image(2D) */
	HvPoint2D32f *wipt,		/* [in] distortion points(2D) */
	HvCameraData *cam_data,	/* [in] information about calibraiton */
	MM_FLOAT tv_resolution,
	MM_S16 ID
)
{
	HvPoint2D32f dpt;
	HvPoint2D32f upt;

	MM_FLOAT mat_H[9];
	MM_FLOAT mat_invH[9];

	MM_FLOAT vec_wpt[3];
	MM_FLOAT vec_uipt[3];

	MM_U32 x = 0;
	MM_U32 y = 0;

	MM_FLOAT dist = 0.0f;

	memset(&dpt, 0x00, sizeof(HvPoint2D32f));
	memset(&upt, 0x00, sizeof(HvPoint2D32f));

	memset(mat_H, 0x00, sizeof(MM_FLOAT) * 9);
	memset(mat_invH, 0x00, sizeof(MM_FLOAT) * 9);

	memset(vec_wpt, 0x00, sizeof(MM_FLOAT) * 3);
	memset(vec_uipt, 0x00, sizeof(MM_FLOAT) * 3);

	/* make Homography */
	make_H_invH(mat_H, mat_invH, cam_data);
	//mtxPrint("homography ", mat_H, 3, 3);

	vec_wpt[0] = (wipt->x*tv_resolution);
	vec_wpt[1] = (wipt->y*tv_resolution) - ((MM_FLOAT)(1389.24097f));
	vec_wpt[2] = ((MM_FLOAT)(1.0f));
#if (debugIMG)
	mtxPrint("world points", vec_wpt, 3, 1);
#endif
	dist = sqrtf((vec_wpt[0] * vec_wpt[0]) + (vec_wpt[0] * vec_wpt[0]));


	MatMul(vec_uipt, mat_H, 3, 3, vec_wpt, 3, 1);

	/*if (vec_uipt[1] < 0)
	{
		printf("check\n");
	}*/
#if (debugIMG)
	mtxPrint("undistortion points", vec_uipt, 3, 1);
#endif
	
	/*printf("sign: ");
	if (vec_uipt[0] < 0) { printf(" -"); }
	else { printf(" +"); }
	if (vec_uipt[1] < 0) { printf(" -"); }
	else { printf(" +"); }
	if (vec_uipt[2] < 0) { printf(" -"); }
	else { printf(" +"); }
	printf("\n");*/

	if ((vec_uipt[0] > 0)&& (vec_uipt[1] > 0)&&(vec_uipt[2] < 0))
	{
		vec_uipt[0] = (vec_uipt[0] / vec_uipt[2]);
		vec_uipt[1] = (vec_uipt[1] / vec_uipt[2]);
		vec_uipt[2] = (vec_uipt[2] / vec_uipt[2]);
#if (debugIMG)
		mtxPrint("undistortion points", vec_uipt, 3, 1);
#endif

		/* point based on image correction */
		//image_correction_pt(&upt, dipt);
		upt.x = vec_uipt[0] - cam_data->cx;
		upt.y = vec_uipt[1] - cam_data->cy;
#if (debugIMG)
		printf("undistortion point: [%f, %f]\n", upt.x, upt.y);
#endif
		image_correction_pt_bw(&dpt, &upt);
		//printf("  distortion: [%f, %f]\n", dpt.x, dpt.y);
		/*dipt->x = dpt.x + cam_data->cx;
		dipt->y = dpt.y + cam_data->cy;*/
		dipt->x = dpt.x;
		dipt->y = dpt.y;
#if (debugIMG)
		printf("  distortion point: [%f, %f]\n", dpt.x, dpt.y);
#endif
		
	}
	else
	{
		dpt.x = 0.0f;
		dpt.y = 0.0f;
	}

	
}

static void make_H_invH(
						MM_FLOAT *H,
						MM_FLOAT *invH,
						HvCameraData *cam_data
)
{
	MM_FLOAT mat_R[9];	/* extrinsic angle: pan(¥È), tilt(¥Õ), rot(¥×) */
	MM_FLOAT mat_R1[3];	/* 1st column vector of rotation matrix */
	MM_FLOAT mat_R2[3];	/* 2nd column vector of rotation matrix */
	MM_FLOAT mat_t[3];	/* translation vector */
	MM_FLOAT mat_K[9];

	MM_FLOAT mat_preH[9];

	memset(mat_K, 0x00, sizeof(MM_FLOAT) * 9);
	memset(mat_R, 0x00, sizeof(MM_FLOAT) * 9);
	memset(mat_R1, 0x00, sizeof(MM_FLOAT) * 3);
	memset(mat_R2, 0x00, sizeof(MM_FLOAT) * 3);
	memset(mat_t, 0x00, sizeof(MM_FLOAT) * 3);
	memset(mat_preH, 0x00, sizeof(MM_FLOAT) * 9);
	

	mat_K[0] = cam_data->f;	mat_K[1] = 0.0f;		mat_K[2] = 640.0f;
	mat_K[3] = 0.0f;		mat_K[4] = cam_data->f;	mat_K[5] = 360.0f;
	mat_K[6] = 0.0f;		mat_K[7] = 0.0f;		mat_K[8] = 1.0f;
	//mtxPrint("intrinsic matrix", mat_K, 3, 3);

	/* rotation matrix */
	memcpy(mat_R, cam_data->R, sizeof(MM_FLOAT) * 9);
	//mtxPrint("rotation matrix: ", mat_R, 3, 3);

	mat_R1[0] = mat_R[0];
	mat_R1[1] = mat_R[3];
	mat_R1[2] = mat_R[6];

	mat_R2[0] = mat_R[1];
	mat_R2[1] = mat_R[4];
	mat_R2[2] = mat_R[7];

	mat_t[0] = cam_data->t[0];
	mat_t[1] = cam_data->t[1];
	mat_t[2] = cam_data->t[2];
	//mtxPrint("translation matrix: ", mat_t, 3, 1);

	mat_preH[0] = mat_R1[0];	mat_preH[1] = mat_R2[0]; mat_preH[2] = mat_t[0];
	mat_preH[3] = mat_R1[1];	mat_preH[4] = mat_R2[1]; mat_preH[5] = mat_t[1];
	mat_preH[6] = mat_R1[2];	mat_preH[7] = mat_R2[2]; mat_preH[8] = mat_t[2];

	//mtxPrint("pre-homography matrix: ", mat_preH, 3, 3);

	/* gernerate homography */
	MatMul(H, mat_K, 3, 3, mat_preH, 3, 3);
	//mtxPrint("homography matrix: ", mat_H, 3, 3);

	/* inverse homography */
	MatInv3x3(invH, H);
	//mtxPrint("inverse homography matrix: ", mat_invH, 3, 3);
}

// M_md = A_mn * B_cd
MM_S16 CAPI MatMul(MM_FLOAT *M, MM_FLOAT *A, MM_S32 m, MM_S32 n, MM_FLOAT *B, MM_S32 c, MM_S32 d) // matrix multiplication
{
	MM_S32 i, j, k;
	MM_S16 error_code = MM_FAILURE;

	if (M && A && B && (n == c) && (m>0) && (n>0) && (c>0) && (d>0))
	{
		memset(M, 0x00, sizeof(MM_FLOAT)*m*d);

		for (i = 0; i < m; i++)
		{
			for (j = 0; j < d; j++)
			{
				for (k = 0; k < n; k++)
				{
					M[i*d + j] += (A[i*n + k] * B[k*d + j]);
				}
			}
		}
		error_code = MM_SUCCESS;
	}

	return error_code;
}

MM_S16 CAPI MatInv3x3(MM_FLOAT *iM, MM_FLOAT *M)
{
	MM_S16 error_code = MM_FAILURE;
	MM_FLOAT sign = 1.0;
	MM_FLOAT det = 0.0;


	if (iM && M)
	{
		// D= aei + bfg + cdh -ceg - bdi - afh

		det = M[0] * M[4] * M[8] + M[1] * M[5] * M[6] + M[2] * M[3] * M[7]
			- M[2] * M[4] * M[6] - M[1] * M[3] * M[8] - M[0] * M[5] * M[7];


		if (fabs(det) > SNG_EPSILON)
		{
			sign = (det > 0.0f) ? 1.0f : -1.0f;
			det = sign / det;

			iM[0] = (M[4] * M[8] - M[5] * M[7])*det; //  ei - fh
			iM[1] = (-M[1] * M[8] + M[2] * M[7])*det; // -bi + ch
			iM[2] = (M[1] * M[5] - M[2] * M[4])*det; //  bf - ce

			iM[3] = (-M[3] * M[8] + M[5] * M[6])*det; // -di + fg
			iM[4] = (M[0] * M[8] - M[2] * M[6])*det; //  ai - cg
			iM[5] = (-M[0] * M[5] + M[2] * M[3])*det; // -af + cd

			iM[6] = (M[3] * M[7] - M[4] * M[6])*det; //  dh - eg
			iM[7] = (-M[0] * M[7] + M[1] * M[6])*det; // -ah + bg
			iM[8] = (M[0] * M[4] - M[1] * M[3])*det; //  ae - bd

			error_code = MM_SUCCESS;
		}
	}

	return error_code;
}


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