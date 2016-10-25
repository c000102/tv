#include <time.h>
#include "opencv2\opencv.hpp"

#include "calibration.h"

using namespace cv;
using namespace std;

#define PROJECT (1) // 1: V-Project, 0: Y400
#define FACTORY_RUNNING (1) // 1: Factory calibration, 0: Service calibration

#ifdef _DEBUG
#pragma comment(lib, "opencv_ts300d.lib")
#pragma comment(lib, "opencv_world300d.lib")
#pragma comment(lib, "..//Release//libcalib.lib")
#else
#pragma comment(lib, "opencv_ts300.lib")
#pragma comment(lib, "opencv_world300.lib")
#pragma comment(lib, ".//lib//libcalib.lib") 
#endif

#if (PROJECT)
#define TEST_FILE(name) ".//test_image//"#name 
#define INPUT_FILE(name) ".//input_image//V//cixi//"#name 
#define OUTPUT_FILE(name) ".//output_image//"#name
#else
#define TEST_FILE(name) ".//test_image//"#name 
#define INPUT_FILE(name) ".//input_image//Y//"#name 
#define OUTPUT_FILE(name) ".//output_image//"#name
#endif

#if (PROJECT)
#define BINFILE ".//bin_file//V//CD_CX11_CH0_LH.bin"
#else
#define BINFILE ".//bin_file//Y//Local_Parameter_File_Sensor.bin"
#endif

void set_car_informaiton(CAR_PARAM *car_param);
void read_cal_info_file(CALIBRATION_PARAM *cal_param);
void check_number(CALIBRATION_PARAM *param);

void print_result_message(MM_U08* msg);
void PrintExtrinsic(EXTRINSIC_PARAM* ext_param, MM_U08 k);
void PrintExtrinsicComparison(EXTRINSIC_PARAM* ext, CALIBRATION_PARAM cal, MM_U08 k);


// 임베디드 입력영상의 형식을 맞추기 위함
Mat front_src_imgC3 = Mat::zeros(HEIGHT,WIDTH,CV_8UC3);
Mat rear_src_imgC3 = Mat::zeros(HEIGHT,WIDTH,CV_8UC3);
Mat left_src_imgC3 = Mat::zeros(HEIGHT,WIDTH,CV_8UC3);
Mat right_src_imgC3 = Mat::zeros(HEIGHT,WIDTH,CV_8UC3);
Mat single_src_imgC3 = Mat::zeros(HEIGHT,WIDTH,CV_8UC3);

Mat front_src_imgC4 = Mat::zeros(HEIGHT,WIDTH,CV_8UC4);
Mat rear_src_imgC4 = Mat::zeros(HEIGHT,WIDTH,CV_8UC4);
Mat left_src_imgC4 = Mat::zeros(HEIGHT,WIDTH,CV_8UC4);
Mat right_src_imgC4 = Mat::zeros(HEIGHT,WIDTH,CV_8UC4);
Mat single_src_imgC4 = Mat::zeros(HEIGHT,WIDTH,CV_8UC4);

MM_U08  front_img[(WIDTH+32)*HEIGHT*4];
MM_U08  rear_img[(WIDTH+32)*HEIGHT*4];
MM_U08  left_img[(WIDTH+32)*HEIGHT*4];
MM_U08  right_img[(WIDTH+32)*HEIGHT*4];
MM_U08  single_img[(WIDTH+32)*HEIGHT*4];

int main()
{
	clock_t           start_time, end_time;
	CAR_PARAM         car_param;
	CALIBRATION_PARAM cal_param;
	EXTRINSIC_PARAM   ext_param[CAM_TOTAL];
	MM_U08            msg_result[5];
	MM_S32			  h,w;

	memset(&car_param,0x00, sizeof(CAR_PARAM));
	memset(&cal_param, 0x00, sizeof(CALIBRATION_PARAM));
	memset(ext_param, 0x00, sizeof(EXTRINSIC_PARAM)*CAM_TOTAL);
	memset(msg_result, 0x00, sizeof(MM_U08) * 5);

	set_car_informaiton(&car_param);
	//set_cal_info_param(&cal_param); // from the hard code
	read_cal_info_file(&cal_param); // from a bin file

#if (0)
	memset(&cal_param.calibrated_front_extrinsic, 0xff, sizeof(EXTRINSIC_PARAM)); /* because of the initial value of fresh memory */
	memset(&cal_param.calibrated_rear_extrinsic, 0xff, sizeof(EXTRINSIC_PARAM));
	memset(&cal_param.calibrated_left_extrinsic, 0xff, sizeof(EXTRINSIC_PARAM));
	memset(&cal_param.calibrated_right_extrinsic, 0xff, sizeof(EXTRINSIC_PARAM));
#else
	memset(&cal_param.calibrated_front_extrinsic, 0x00, sizeof(EXTRINSIC_PARAM)); /* because of the initial value of fresh memory */
	memset(&cal_param.calibrated_rear_extrinsic, 0x00, sizeof(EXTRINSIC_PARAM));
	memset(&cal_param.calibrated_left_extrinsic, 0x00, sizeof(EXTRINSIC_PARAM));
	memset(&cal_param.calibrated_right_extrinsic, 0x00, sizeof(EXTRINSIC_PARAM));
#endif
	cal_param.calibration_status[CAM_FRONT] = 0;
	cal_param.calibration_status[CAM_REAR] = 0;
	cal_param.calibration_status[CAM_LEFT] = 0;
	cal_param.calibration_status[CAM_RIGHT] = 0;
	/* ------------------------------------------------------------------------- */

	//check_number(&cal_param);

	if(FACTORY_RUNNING)
	{
		//Error case: Lf(Left faliure), Rf, Bf, Cf(camera falut), CfW, CfR, CfG, CfB

		front_src_imgC3 = imread(INPUT_FILE(00_front.bmp),IMREAD_ANYCOLOR);
		//front_src_imgC3 = imread(TEST_FILE(.\\Front\\00_front_Lf.bmp), IMREAD_ANYCOLOR);

	    rear_src_imgC3  = imread(INPUT_FILE(01_rear.bmp),IMREAD_ANYCOLOR);
		//rear_src_imgC3 = imread(TEST_FILE(.\\Rear\\01_rear_Lf.bmp), IMREAD_ANYCOLOR);

		left_src_imgC3  = imread(INPUT_FILE(02_left.bmp),IMREAD_ANYCOLOR);
		//left_src_imgC3 = imread(TEST_FILE(.\\Left\\02_left_CfW.bmp), IMREAD_ANYCOLOR);

		right_src_imgC3 = imread(INPUT_FILE(03_right.bmp),IMREAD_ANYCOLOR);
		//right_src_imgC3 = imread(TEST_FILE(.\\Right\\03_right_Rf.bmp), IMREAD_ANYCOLOR);

		cvtColor(front_src_imgC3, front_src_imgC4, CV_RGB2RGBA);
		cvtColor(rear_src_imgC3, rear_src_imgC4, CV_RGB2RGBA);
		cvtColor(left_src_imgC3, left_src_imgC4, CV_RGB2RGBA);
		cvtColor(right_src_imgC3, right_src_imgC4, CV_RGB2RGBA);

		memset(front_img,0x00,sizeof(unsigned char)*(WIDTH+32)*HEIGHT*4);	
		memset(rear_img,0x00,sizeof(unsigned char)*(WIDTH+32)*HEIGHT*4);	
		memset(left_img,0x00,sizeof(unsigned char)*(WIDTH+32)*HEIGHT*4);	
		memset(right_img,0x00,sizeof(unsigned char)*(WIDTH+32)*HEIGHT*4);	

		for(h=0; h < HEIGHT; h++)
		{
			for(w=0; w < WIDTH; w++)
			{
				front_img[4*(h*(WIDTH+32)+w)+3] = front_src_imgC4.at<uchar>(h,4*w+0);//R
				front_img[4*(h*(WIDTH+32)+w)+2] = front_src_imgC4.at<uchar>(h,4*w+1);//G
				front_img[4*(h*(WIDTH+32)+w)+1] = front_src_imgC4.at<uchar>(h,4*w+2);//B
				front_img[4*(h*(WIDTH+32)+w)+0] = front_src_imgC4.at<uchar>(h,4*w+3);//A

				rear_img[4*(h*(WIDTH+32)+w)+3] = rear_src_imgC4.at<uchar>(h,4*w+0);//R
				rear_img[4*(h*(WIDTH+32)+w)+2] = rear_src_imgC4.at<uchar>(h,4*w+1);//G
				rear_img[4*(h*(WIDTH+32)+w)+1] = rear_src_imgC4.at<uchar>(h,4*w+2);//B
				rear_img[4*(h*(WIDTH+32)+w)+0] = rear_src_imgC4.at<uchar>(h,4*w+3);//A

				left_img[4*(h*(WIDTH+32)+w)+3] = left_src_imgC4.at<uchar>(h,4*w+0);//R
				left_img[4*(h*(WIDTH+32)+w)+2] = left_src_imgC4.at<uchar>(h,4*w+1);//G
				left_img[4*(h*(WIDTH+32)+w)+1] = left_src_imgC4.at<uchar>(h,4*w+2);//B
				left_img[4*(h*(WIDTH+32)+w)+0] = left_src_imgC4.at<uchar>(h,4*w+3);//A

				right_img[4*(h*(WIDTH+32)+w)+3] = right_src_imgC4.at<uchar>(h,4*w+0);//R
				right_img[4*(h*(WIDTH+32)+w)+2] = right_src_imgC4.at<uchar>(h,4*w+1);//G
				right_img[4*(h*(WIDTH+32)+w)+1] = right_src_imgC4.at<uchar>(h,4*w+2);//B
				right_img[4*(h*(WIDTH+32)+w)+0] = right_src_imgC4.at<uchar>(h,4*w+3);//A
			}
		}

		cal_param.calibrated_front_extrinsic = cal_param.default_front_extrinsic;
		cal_param.calibrated_rear_extrinsic = cal_param.default_rear_extrinsic;
		cal_param.calibrated_left_extrinsic = cal_param.default_left_extrinsic;
		cal_param.calibrated_right_extrinsic = cal_param.default_right_extrinsic;

		start_time = clock();
		factory_calibration(front_img,			
						    rear_img,	
					  	    left_img,	
						    right_img,	
						    HEIGHT,	
						    WIDTH,		
							&car_param,
							&cal_param,		
							ext_param,	  
							msg_result);
		end_time = clock();

		/*
		namedWindow("front image",CV_WINDOW_AUTOSIZE );
		namedWindow("rear image",CV_WINDOW_AUTOSIZE );
		namedWindow("left image",CV_WINDOW_AUTOSIZE );
		namedWindow("right image",CV_WINDOW_AUTOSIZE );
		
		imshow("front image",front_src_imgC3);
		imshow("rear image",rear_src_imgC3);
		imshow("left image",left_src_imgC3);
		imshow("right image",right_src_imgC3);
		
		waitKey(0); 
		*/

		print_result_message(msg_result);
		PrintExtrinsic(ext_param, CAM_TOTAL);
		PrintExtrinsicComparison(ext_param, cal_param, CAM_TOTAL);
		printf("spent time: %.3lf[ms]\n", (MM_DOUBLE)(end_time - start_time));

		destroyAllWindows();
		//system("pause");
	} 
	else
	{
		single_src_imgC3 = imread(INPUT_FILE(00_front.bmp),IMREAD_ANYCOLOR);
		//single_src_imgC3 = imread(TEST_FILE(.\\Service\\00_front_Lf.bmp), IMREAD_ANYCOLOR);
		//single_src_imgC3 = imread(TEST_FILE(.\\Service\\02_left_Lf.bmp), IMREAD_ANYCOLOR);
		cvtColor(single_src_imgC3, single_src_imgC4, CV_RGB2RGBA);
		memset(single_img,0x00,sizeof(unsigned char)*(WIDTH+32)*HEIGHT*4);	

		for(h=0; h < HEIGHT; h++)
		{
			for(w=0; w < WIDTH; w++)
			{
				single_img[4*(h*(WIDTH+32)+w)+3] = single_src_imgC4.at<uchar>(h,4*w+0);//R
				single_img[4*(h*(WIDTH+32)+w)+2] = single_src_imgC4.at<uchar>(h,4*w+1);//G
				single_img[4*(h*(WIDTH+32)+w)+1] = single_src_imgC4.at<uchar>(h,4*w+2);//B
				single_img[4*(h*(WIDTH+32)+w)+0] = single_src_imgC4.at<uchar>(h,4*w+3);//A
			}
		}
		
		start_time = clock();
		service_calibration(single_img,
							HEIGHT,
							WIDTH,	
							CAM_FRONT,  
							&car_param,
							&cal_param,	
							&ext_param[CAM_FRONT],
							msg_result); 
		end_time = clock();

		/*namedWindow("service image", CV_WINDOW_AUTOSIZE);
		imshow("service image",single_src_imgC3);
		waitKey(0);*/

		print_result_message(msg_result);
		PrintExtrinsic(ext_param, CAM_FRONT);
		printf("spent time: %.3lf[ms]\n", (MM_DOUBLE)(end_time - start_time));

		destroyAllWindows();
		system("pause");
	}

	return 0;
}


void set_car_informaiton(CAR_PARAM *car_param)
{
#if (PROJECT)
	//V-Project
	car_param->car_type =				0;
	car_param->length =			4522.576f;
	car_param->front_overhang =	0855.153f;
	car_param->wheelbase =		2734.205f;
	car_param->rear_overhang =	0919.278f;
	car_param->width =			1853.000f;
	car_param->track =			1587.600f;//front:1587.600, rear: 1586.400
	car_param->height =			1210.294f;//side hieght
#else
	//Y400
	car_param->car_type =				0;
	car_param->length =			4852.565f;
	car_param->front_overhang =	0897.402f;
	car_param->wheelbase =		2865.031f;
	car_param->rear_overhang =	1090.133f;
	car_param->width =			1918.206f;
	car_param->track =			1620.000f;
	car_param->height =			1778.033f;
#endif

	
}

void  read_cal_info_file(CALIBRATION_PARAM *cal_param)
{
	FILE *fp = NULL;
	errno_t error_no;
	MM_U16  margin_size;

	margin_size = sizeof(EXTRINSIC_PARAM) * CAM_TOTAL;

	error_no = fopen_s(&fp, BINFILE, "rb");

	if (error_no == (errno_t)0)
	{
		fread(cal_param, sizeof(CALIBRATION_PARAM) - margin_size, 1, fp);

		fclose(fp);
	}

}

void print_result_message(MM_U08* msg)
{
	printf(" 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", msg[0], msg[1], msg[2], msg[3],msg[4]);
}

void PrintExtrinsic(EXTRINSIC_PARAM* ext_param, MM_U08 k)
{
	int i=0;

	printf("\nextrinsic parameters\n");

	if (k==CAM_TOTAL)
	{
		for (i=0; i<4; i++)
		{
			printf("[%10.3f %10.3f %10.3f ", ext_param[i].x, ext_param[i]. y, ext_param[i].z);
			printf("%8.3f %8.3f %8.3f]\n", ext_param[i].pan, ext_param[i]. tilt, ext_param[i].rotate);
		}
		printf("\n");

	}
	else
	{
		printf("[%10.2f, %10.2f, %10.2f ", ext_param[i].x, ext_param[i].y, ext_param[i].z);
		printf("%8.2f, %8.2f, %8.2f]\n\n", ext_param[i].pan, ext_param[i].tilt, ext_param[i].rotate);
	}
}

void PrintExtrinsicComparison(EXTRINSIC_PARAM* ext, CALIBRATION_PARAM cal,MM_U08 k)
{
	printf("\nextrinsic parameters Comparison\n");

	if (k == CAM_TOTAL)
	{
		printf("[%10.2f, %10.2f, %10.2f, ", abs(ext[0].x)		- abs(cal.default_front_extrinsic.x), 
											abs(ext[0].y)		- abs(cal.default_front_extrinsic.y), 
											abs(ext[0].z)		- abs(cal.default_front_extrinsic.z));
		printf("%8.2f, %8.2f, %8.2f]\n",	abs(ext[0].pan)		- abs(cal.default_front_extrinsic.pan),
											abs(ext[0].tilt)	- abs(cal.default_front_extrinsic.tilt),
											abs(ext[0].rotate)	- abs(cal.default_front_extrinsic.rotate));

		printf("[%10.2f, %10.2f, %10.2f, ", abs(ext[1].x)		- abs(cal.default_rear_extrinsic.x),
											abs(ext[1].y)		- abs(cal.default_rear_extrinsic.y),
											abs(ext[1].z)		- abs(cal.default_rear_extrinsic.z));
		printf("%8.2f, %8.2f, %8.2f]\n",	abs(ext[1].pan)		- abs(cal.default_rear_extrinsic.pan),
											abs(ext[1].tilt)	- abs(cal.default_rear_extrinsic.tilt),
											abs(ext[1].rotate)	- abs(cal.default_rear_extrinsic.rotate));

		printf("[%10.2f, %10.2f, %10.2f, ", abs(ext[2].x)		- abs(cal.default_left_extrinsic.x),
											abs(ext[2].y)		- abs(cal.default_left_extrinsic.y),
											abs(ext[2].z)		- abs(cal.default_left_extrinsic.z));
		printf("%8.2f, %8.2f, %8.2f]\n",	abs(ext[2].pan)		- abs(cal.default_left_extrinsic.pan),
											abs(ext[2].tilt)	- abs(cal.default_left_extrinsic.tilt),
											abs(ext[2].rotate)	- abs(cal.default_left_extrinsic.rotate));

		printf("[%10.2f, %10.2f, %10.2f, ", abs(ext[3].x)		- abs(cal.default_right_extrinsic.x),
											abs(ext[3].y)		- abs(cal.default_right_extrinsic.y),
											abs(ext[3].z)		- abs(cal.default_right_extrinsic.z));
		printf("%8.2f, %8.2f, %8.2f]\n",	abs(ext[3].pan)		- abs(cal.default_right_extrinsic.pan),
											abs(ext[3].tilt)	- abs(cal.default_right_extrinsic.tilt),
											abs(ext[3].rotate)	- abs(cal.default_right_extrinsic.rotate));

		printf("\n");

	}
	else
	{
		printf("[%10.2f, %10.2f, %10.2f ", ext[0].x, ext[0].y, ext[0].z);
		printf("%8.2f, %8.2f, %8.2f]\n\n", ext[0].pan, ext[0].tilt, ext[0].rotate);
	}
}

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
