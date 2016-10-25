#include<string.h>
#include <math.h>
#include <stdlib.h>
#include "post_processing.h"
#include "image_enhancement.h"

CFUNC_BEGIN_DECLS

static MM_U08  getBandChannel( MM_U08 *lineBandOtsu,MM_U08 size);
static MM_S16  erosion1D(MM_U08 src_img[], MM_U16 iSize);

static MM_U08 winImg[169];
static MM_U08 radiusImg[169];
static MM_U08 lineBandImg[48];
static MM_U08 lineBandOtsuImg[48];

CFUNC_END_DECLS

MM_S16  CirIntFR13x13(MM_FLOAT fOutPits01[],	/* [out] corner points */
					  MM_U16 *nOP01,			/* [out] number of points */
					  MM_U08 grayImage[],		/* [in] source image */
					  MM_U16 iheight,			/* [in] height of image */
					  MM_U16 iWidth,			/* [in] width of image */
					  MM_FLOAT fsigPoints01[],	/* [in] corner points */
					  MM_U16 nP01)				/* [in] number of points */
					 
{
	MM_S16 error_code = MM_TRUE;
	MM_U08 pp = (MM_U08)0;
	MM_U08 cn = (MM_U08)0;
	MM_U08 i  = (MM_U08)0;
	MM_S08 h=(MM_S08)0;
	MM_S08 w=(MM_S08)0;
	MM_U08 outCnt = (MM_U08)0;
	MM_FLOAT meanV = 0.0f;
	MM_FLOAT varV = 0.0f;
	MM_FLOAT sumV = 0.0f;
	MM_U16 xx = (MM_U16)(0);
	MM_U16 yy = (MM_U16)(0);
	MM_U08 wWidth = (MM_U08)(13);
	MM_S08 maskSize = (MM_S08)(6);
	MM_U08 bandSize = (MM_U08)(48);

	
	memset(winImg,0x00, sizeof(MM_U08)* (MM_U08)(169));
	memset(radiusImg,0x00, sizeof(MM_U08)* (MM_U08)(169));
	memset(lineBandImg,0x00, sizeof(MM_U08)* (MM_U08)(48));
	memset(lineBandOtsuImg,0x00, sizeof(MM_U08)* (MM_U08)(48));		
 
	for (pp=(MM_U08)(0); pp<nP01; pp++) {
			
		xx = (MM_U16)(fsigPoints01[((MM_U08)(2)*pp)+(MM_U08)(0)]);
		yy = (MM_U16)(fsigPoints01[((MM_U08)(2)*pp)+(MM_U08)(1)]);

		if ( ((xx >= (MM_U16)(maskSize)) && (yy >= (MM_U16)(maskSize))) && ((yy <= (MM_U16)(iheight-(MM_U16)(maskSize))) && (xx <= (MM_U16)(iWidth-(MM_U16)(maskSize)))) )
		{
			for (h=(MM_S08)((MM_S08)(-1)*maskSize); h<=maskSize; h++) 
			{
				for (w=(-maskSize); w<=maskSize; w++) 
				{
					winImg[((h+maskSize)*(MM_S08)(wWidth))+(w+maskSize)] = (MM_U08)(grayImage[(((MM_S32)(yy)+(MM_S32)(h))*(MM_S32)(iWidth))+((MM_S32)(xx)+(MM_S32)(w))]);
				}
			}

			//clockwise
			lineBandImg[(MM_U08) (0)] = winImg[(MM_U08) (0)*wWidth+(MM_U08) (6)];
			lineBandImg[(MM_U08) (1)] = winImg[(MM_U08) (0)*wWidth+(MM_U08) (7)];
			lineBandImg[(MM_U08) (2)] = winImg[(MM_U08) (0)*wWidth+(MM_U08) (8)];
			lineBandImg[(MM_U08) (3)] = winImg[(MM_U08) (0)*wWidth+(MM_U08) (9)];
			lineBandImg[(MM_U08) (4)] = winImg[(MM_U08) (0)*wWidth+(MM_U08)(10)];
			lineBandImg[(MM_U08) (5)] = winImg[(MM_U08) (0)*wWidth+(MM_U08)(11)];
			lineBandImg[(MM_U08) (6)] = winImg[(MM_U08) (0)*wWidth+(MM_U08)(12)];
			lineBandImg[(MM_U08) (7)] = winImg[(MM_U08) (1)*wWidth+(MM_U08)(12)];
			lineBandImg[(MM_U08) (8)] = winImg[(MM_U08) (2)*wWidth+(MM_U08)(12)];
			lineBandImg[(MM_U08) (9)] = winImg[(MM_U08) (3)*wWidth+(MM_U08)(12)];
			lineBandImg[(MM_U08)(10)] = winImg[(MM_U08) (4)*wWidth+(MM_U08)(12)];
			lineBandImg[(MM_U08)(11)] = winImg[(MM_U08) (5)*wWidth+(MM_U08)(12)];
			lineBandImg[(MM_U08)(12)] = winImg[(MM_U08) (6)*wWidth+(MM_U08)(12)];
			lineBandImg[(MM_U08)(13)] = winImg[(MM_U08) (7)*wWidth+(MM_U08)(12)];
			lineBandImg[(MM_U08)(14)] = winImg[(MM_U08) (8)*wWidth+(MM_U08)(12)];
			lineBandImg[(MM_U08)(15)] = winImg[(MM_U08) (9)*wWidth+(MM_U08)(12)];
			lineBandImg[(MM_U08)(16)] = winImg[(MM_U08)(10)*wWidth+(MM_U08)(12)];
			lineBandImg[(MM_U08)(17)] = winImg[(MM_U08)(11)*wWidth+(MM_U08)(12)];
			lineBandImg[(MM_U08)(18)] = winImg[(MM_U08)(12)*wWidth+(MM_U08)(12)];
			lineBandImg[(MM_U08)(19)] = winImg[(MM_U08)(12)*wWidth+(MM_U08)(11)];
			lineBandImg[(MM_U08)(20)] = winImg[(MM_U08)(12)*wWidth+(MM_U08)(10)];
			lineBandImg[(MM_U08)(21)] = winImg[(MM_U08)(12)*wWidth+(MM_U08) (9)];
			lineBandImg[(MM_U08)(22)] = winImg[(MM_U08)(12)*wWidth+(MM_U08) (8)];
			lineBandImg[(MM_U08)(23)] = winImg[(MM_U08)(12)*wWidth+(MM_U08) (7)];
			lineBandImg[(MM_U08)(24)] = winImg[(MM_U08)(12)*wWidth+(MM_U08) (6)];
			lineBandImg[(MM_U08)(25)] = winImg[(MM_U08)(12)*wWidth+(MM_U08) (5)];
			lineBandImg[(MM_U08)(26)] = winImg[(MM_U08)(12)*wWidth+(MM_U08) (4)];
			lineBandImg[(MM_U08)(27)] = winImg[(MM_U08)(12)*wWidth+(MM_U08) (3)];
			lineBandImg[(MM_U08)(28)] = winImg[(MM_U08)(12)*wWidth+(MM_U08) (2)];
			lineBandImg[(MM_U08)(29)] = winImg[(MM_U08)(12)*wWidth+(MM_U08) (1)];
			lineBandImg[(MM_U08)(30)] = winImg[(MM_U08)(12)*wWidth+(MM_U08) (0)];
			lineBandImg[(MM_U08)(31)] = winImg[(MM_U08)(11)*wWidth+(MM_U08) (0)];
			lineBandImg[(MM_U08)(32)] = winImg[(MM_U08)(10)*wWidth+(MM_U08) (0)];
			lineBandImg[(MM_U08)(33)] = winImg[(MM_U08) (9)*wWidth+(MM_U08) (0)];
			lineBandImg[(MM_U08)(34)] = winImg[(MM_U08) (8)*wWidth+(MM_U08) (0)];
			lineBandImg[(MM_U08)(35)] = winImg[(MM_U08) (7)*wWidth+(MM_U08) (0)];
			lineBandImg[(MM_U08)(36)] = winImg[(MM_U08) (6)*wWidth+(MM_U08) (0)];
			lineBandImg[(MM_U08)(37)] = winImg[(MM_U08) (5)*wWidth+(MM_U08) (0)];
			lineBandImg[(MM_U08)(38)] = winImg[(MM_U08) (4)*wWidth+(MM_U08) (0)];
			lineBandImg[(MM_U08)(39)] = winImg[(MM_U08) (3)*wWidth+(MM_U08) (0)];
			lineBandImg[(MM_U08)(40)] = winImg[(MM_U08) (2)*wWidth+(MM_U08) (0)];
			lineBandImg[(MM_U08)(41)] = winImg[(MM_U08) (1)*wWidth+(MM_U08) (0)];
			lineBandImg[(MM_U08)(42)] = winImg[(MM_U08) (0)*wWidth+(MM_U08) (0)];
			lineBandImg[(MM_U08)(43)] = winImg[(MM_U08) (0)*wWidth+(MM_U08) (1)];
			lineBandImg[(MM_U08)(44)] = winImg[(MM_U08) (0)*wWidth+(MM_U08) (2)];
			lineBandImg[(MM_U08)(45)] = winImg[(MM_U08) (0)*wWidth+(MM_U08) (3)];
			lineBandImg[(MM_U08)(46)] = winImg[(MM_U08) (0)*wWidth+(MM_U08) (4)];
			lineBandImg[(MM_U08)(47)] = winImg[(MM_U08) (0)*wWidth+(MM_U08) (5)];
			/* meanV = 0.0f; */ 
			sumV = 0.0f;
			varV= 0.0f;
			
			for (i=(MM_U08)(0); i<bandSize; i++) {
				sumV = sumV + (MM_FLOAT)lineBandImg[i];
			}
			meanV = sumV/(MM_FLOAT)(bandSize);

			for (i=(MM_U08)(0); i<bandSize; i++) {
				varV = varV + ( (((MM_FLOAT)(lineBandImg[i]) - meanV)*(MM_FLOAT)(lineBandImg[i])) - meanV);
			}
			varV = varV/((MM_FLOAT)(bandSize)-(MM_FLOAT)(1));

			(void) gray2binary_otsu(lineBandOtsuImg,lineBandImg,1,bandSize);

			(void) erosion1D(lineBandOtsuImg,bandSize);

 			cn = getBandChannel(lineBandOtsuImg,bandSize);
			if ( (cn >= (MM_U08)(2)) && (varV > (MM_FLOAT)(50.0f)) ) {
				fOutPits01[((MM_U08)(2)*outCnt)+(MM_U08)(0)] = fsigPoints01[((MM_U08)(2)*pp)+(MM_U08)(0)];
				fOutPits01[((MM_U08)(2)*outCnt)+(MM_U08)(1)] = fsigPoints01[((MM_U08)(2)*pp)+(MM_U08)(1)];
				outCnt++;
			}

		}
		(*nOP01) = (MM_U16)outCnt;
	
		if(*nOP01 <= (MM_U16)(0)) {
			error_code = MM_FALSE;
		}
	}
	


	return error_code;
}

MM_U08  getBandChannel( MM_U08 *lineBandOtsu,
					   MM_U08 size)
{
	MM_U08 checked = (MM_U08)1;
	MM_U08 checkAll = (MM_U08)0;
	MM_U08 channel = (MM_U08)0;
	MM_U08 idx = (MM_U08)0;
	MM_U08 idx2 = (MM_U08)0;
	MM_U08 bChecked = (MM_U08)1;
	MM_U08 tempIdx = (MM_U08)0;
	MM_U08 diffIdx1 = (MM_U08)0;
	MM_U08 diffIdx2 = (MM_U08)0;
	MM_U08 bCounter[3] = { (MM_U08)0,(MM_U08)0,(MM_U08)0};
	MM_U08 n= (MM_U08)0;
	MM_U08 i= (MM_U08)0;

 	for (n=(MM_U08)(0); n<size; n++) {
		if(lineBandOtsu[n] == (MM_U08)(0)) {
			checkAll = (MM_U08)(1);
			if(checked == (MM_U08)(1)) {
				bCounter[0] += (MM_U08)(1);
				tempIdx = n;
			}
			if(checked == (MM_U08)(0)) {
				bCounter[1] += (MM_U08)(1);
				idx = n;
			}
			if(checked == (MM_U08)(2)) {
				bCounter[2] += (MM_U08)(1);
				idx2 = n;
			}
		} else {
			if(checkAll == (MM_U08)(1)) {
				if(checked == (MM_U08)(0)) {
					checked = (MM_U08)(2);
				}
				else if(checked == (MM_U08)(1)) {
					checked = (MM_U08)(0);
				}
				else if(checked == (MM_U08)(2))
				{
					checked = (MM_U08)(4);
				}
				else
				{
				}
			}
			checkAll = (MM_U08)(0);
		}
		if((lineBandOtsu[size-(MM_U08)(1)] == (MM_U08)(0)) && (lineBandOtsu[0] == (MM_U08)(0)))
		{
			if(lineBandOtsu[(size-(MM_U08)(1))-(MM_U08)(n)] == (MM_U08)(0)) {
				if(bChecked == (MM_U08)(1)) {
					bCounter[0] += (MM_U08)(1);
				}
			} else {
				bChecked = (MM_U08)(0);
			}
		}
		
	}
    
	if( (idx == (size-(MM_U08)(1))) && (lineBandOtsu[0]==(MM_U08)(0)) ) {
		bCounter[1] = (MM_U08)(0);
	}
	if( (idx2 == (size-(MM_U08)(1))) && (lineBandOtsu[0]==(MM_U08)(0))) {
		bCounter[2] = (MM_U08)(0);
	}

	diffIdx1 = (MM_U08)abs( (MM_S32)(((MM_S32)(idx) - (MM_S32)(bCounter[1])) - (MM_S32)(tempIdx)) );
	diffIdx2 = (MM_U08)abs( (MM_S32)(((MM_S32)(idx2)- (MM_S32)(bCounter[2])) -   (MM_S32)(idx)) );

	
	if(diffIdx1 <= (MM_U08)(1)) {
		bCounter[1] = (MM_U08)(0);
	}

	if(diffIdx2 <= (MM_U08)(1)) {
		bCounter[2] = (MM_U08)(0);
	}
	
	if ( bCounter[0] < (MM_U08)(2) ) {
		bCounter[0] = (MM_U08)(0);
	}

	if ( bCounter[1] < (MM_U08)(2) ) {
		bCounter[1] = (MM_U08)(0);
	}

	if ( bCounter[2] < (MM_U08)(2) ) {
		bCounter[2] = (MM_U08)(0);
	}

	if( (bCounter[2] == (MM_U08)(0)) && (bCounter[1] <= (MM_U08)(2)) ) {
		bCounter[1] = (MM_U08)(0);
	}

	for(i = (MM_U08)(0); i < (MM_U08)(3) ; i++)
	{
		if(bCounter[i]!= (MM_U08)(0)) {
			channel++;
		}
	}
	return channel;

}

static MM_FLOAT tmpPoints[50*2];

 MM_S16  overlapPoints(	MM_FLOAT outPoints[],	/* [out] output corner points */
						MM_U16 *numOut,			/* [out] number of ouput points */
						MM_FLOAT inPoints[],	/* [in] input corner points */
						MM_U16 numIn)			/* [in] number of input points */
{
	MM_S16 error_code = MM_TRUE;
	MM_U16 i=(MM_U16)0;
	MM_U16 j=(MM_U16)0;
	MM_U16 cnt1=(MM_U16)0;
	MM_U16 cnt2=(MM_U16)0;
	MM_FLOAT x1 = 0.0f;
	MM_FLOAT y1 = 0.0f;
	MM_FLOAT x2 = 0.0f;
	MM_FLOAT y2 = 0.0f;
	MM_FLOAT disV = 0.0f;
	MM_FLOAT thesholdV = 10.0f;	


	memset(tmpPoints,0x00, sizeof(MM_FLOAT)* (MM_U08)(50)*(MM_U08)(2));
		
	for (i=(MM_U16)(0); i<numIn; i++) {
		x1 = (MM_FLOAT)inPoints[((MM_U16)(2)*i)];
		y1 = (MM_FLOAT)inPoints[((MM_U16)(2)*i)+(MM_U16)(1)];
			
		for (j=(i+(MM_U16)(1)); j<numIn; j++) {
			if ( (x1 == 0.0f) || (y1 == 0.0f) ) {
				j = numIn;
				continue;
			}
				
			x2 = (MM_FLOAT)inPoints[((MM_U16)(2)*j)];
			y2 = (MM_FLOAT)inPoints[((MM_U16)(2)*j)+(MM_U16)(1)];
				
			//disV = (MM_FLOAT)sqrtf( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
			disV = (MM_FLOAT)sqrt((MM_FLOAT)((MM_FLOAT)(((MM_FLOAT)(x2-x1))*((MM_FLOAT)(x2-x1))) + (MM_FLOAT)(((MM_FLOAT)(y2-y1))*((MM_FLOAT)(y2-y1)))));
				
			if (disV < thesholdV) {
				inPoints[((MM_U16)(2)*i)]   = (x1 + x2)/(MM_FLOAT)(2);
				inPoints[((MM_U16)(2)*i) + (MM_U16)(1)] = (y1 + y2)/(MM_FLOAT)(2);

				inPoints[((MM_U16)(2)*j) + (MM_U16)(0)] = 0.0f;
				inPoints[((MM_U16)(2)*j) + (MM_U16)(1)] = 0.0f;
				
				cnt1++;
				break;
			}
		}
	}
	
	for (i=(MM_U16)(0); i<numIn; i++) {
		x1 = inPoints[((MM_U16)(2)*i)];
		y1 = inPoints[((MM_U16)(2)*i)+(MM_U16)(1)];
		if ( (x1 == 0.0f) || (y1 == 0.0f) ) {
			continue;
		}

		outPoints[((MM_U16)(2)*cnt2)]   = x1;
		outPoints[((MM_U16)(2)*cnt2)+(MM_U16)(1)] = y1;
		cnt2++;
	}
	*numOut = cnt2;

	if(*numOut <= (MM_U16)(0)) {
		error_code = MM_FALSE;
	}

	return error_code;
}

static MM_U08 visited[MAX_ROI_SIZE]; // original
static MM_U08 gray_ret[MAX_ROI_SIZE];
static MM_U16 xchain[BORDER_NUM];
static MM_U16 ychain[BORDER_NUM];
static NPOINTS nei[8] = {{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1},{0,1},{1,1}};
static MM_U32 meanX[BORDER_NUM];
static MM_U32 meanY[BORDER_NUM];

MM_S16 borderFollowing(	BORDERFOLLOW stBorderInfo[],	/* [out] border points  */
						MM_U08 *border_num,				/* [out] number of border */
						MM_U08 gray[],					/* [in] source image */
						MM_U16 height,					/* [in] height of image */
						MM_U16 width)					/* [in] width of image */

{
    MM_S16 error_code = MM_TRUE;
	MM_U16 i = (MM_U16)0;
	MM_U16 j = (MM_U16)0;
    MM_U16 x0 =(MM_U16)0;
    MM_U16 y0 =(MM_U16)0;
    MM_U16 k =(MM_U16)0;
    MM_U16 n =(MM_U16)0;
    MM_U16 x =(MM_U16)0;
    MM_U16 y =(MM_U16)0;
    MM_S16 x1=(MM_S16)0;
    MM_S16 y1=(MM_S16)0;
    MM_U16 border_count = (MM_U16)0;
    MM_U08 c0=(MM_U08)0;
    MM_U08 c1=(MM_U08)0;
    MM_U08 numberBorder = (MM_U08)0;   
    MM_U16 u = (MM_U16)0;
    MM_U16 v = (MM_U16)0; 
    MM_S16 u1 = (MM_S16)0;
    MM_S16 v1 = (MM_S16)0;
    MM_FLOAT dist = 0.0f;
    MM_FLOAT minDist = 0.0f;
    MM_U16  minIdx = (MM_U16)0;


	memset(visited,0x00,MAX_ROI_SIZE*sizeof(MM_U08));
	memset(gray_ret,0x00,MAX_ROI_SIZE*sizeof(MM_U08));
	   
	memset(xchain,0x00,sizeof(MM_U16)*BORDER_NUM);
	memset(ychain,0x00,sizeof(MM_U16)*BORDER_NUM);
	   
	memset(meanX,0x00,sizeof(MM_U32)*BORDER_NUM);
	memset(meanY,0x00,sizeof(MM_U32)*BORDER_NUM);
	   
	for (y = (MM_U16)(1); y < height-1; y++)
	{
		for (x = (MM_U16)(1); x < width-1; x++)
		{
			if( (y > (height-(MM_U16)(1))) || (x > (width-(MM_U16)(1))) ) {
				break;
			}
			   
			c0 = gray[(y*width)+x];
			c1 = gray[((y-(MM_U16)(1))*width) + x];
			   
			if ( ((c0 != c1) && (c0 == (MM_U08)(255))) && ((visited[(y*width)+x] == (MM_U08)(0))) )
			{
				border_count = (MM_U16)(0);
				   
				x0 = x;
				y0 = y;
				n = (MM_U16)(4);

				memset(meanX,0x00,sizeof(MM_U32)*BORDER_NUM);
				memset(meanY,0x00,sizeof(MM_U32)*BORDER_NUM);
				   
				do
				{
					for(k = (MM_U16)(0); k < (MM_U16)(8); k++)
					{
						u1 = ((MM_S16)(x)+nei[n].x);
						v1 = ((MM_S16)(y)+nei[n].y);
						u = (MM_U16)(u1);
						v = (MM_U16)(v1);
						   
						if( (u>= width) || (v >= height) ) {
							continue;
						}
						   
						if (gray[(v*width) + u] == c0) {
							break;
						}
						n = ((n+(MM_U16)(1))&(MM_U16)(7));
					}
					   
					if ( (k != (MM_U16)(8)) && ( (y <= (height-(MM_U16)(1))) && (x <= (width-(MM_U16)(1))) ) )
					{
						visited[(y*width)+x] = (MM_U08)(255);
						xchain[border_count] = x;
						ychain[border_count] = y;
						border_count++;
						   
						if(border_count >= BORDER_NUM) {
							break;
						}
						   
						x1 = ((MM_S16)(x) + nei[n].x);
						y1 = ((MM_S16)(y) + nei[n].y);
						   
						x = (MM_U16)(x1);
						y = (MM_U16)(y1);
						   
						n = (MM_U16)((n+(MM_U16)(5))%(MM_U16)(7));
					}
				   
				} while (!((x == x0) && (y == y0))); //END OF DO WHILE
				   
				if(k == (MM_U16)(8)) {
					continue;
				}
				   
				if((border_count < (MM_U16)(200)) || (border_count > BORDER_THRES)) {
					continue;
				}

				   
				for(k = (MM_U16)(0); k< border_count;k++)
				{				   
					stBorderInfo[numberBorder].x[k] = xchain[k];
					stBorderInfo[numberBorder].y[k] = ychain[k];
				}
				stBorderInfo[numberBorder].n = border_count;
				numberBorder += (MM_U08)(1);
			}
		}
	}
	*border_num = numberBorder;
	   
	for (j=0; j<numberBorder; j++)
	{
		for (i = (MM_U16)(0); i < stBorderInfo[j].n ; i++) 
		{
			x = stBorderInfo[j].x[i];
			y = stBorderInfo[j].y[i];
			   
			if( ((y > (height-(MM_U16)(1))) || (x > (width-(MM_U16)(1)))) ) {
				break;
			}
			   
			meanX[j] += (MM_U32)(x);
			meanY[j] += (MM_U32)(y);
			gray_ret[(y*width) + x] = (MM_U08)(255);
		}
		meanX[j] = meanX[j]/(MM_U32)(stBorderInfo[j].n);
		meanY[j] = meanY[j]/(MM_U32)(stBorderInfo[j].n);
	}

	dist = 0.0f;
	minDist = 100000.0f;
	minIdx = 0;
	for(i=0; i<numberBorder; i++) {
		dist = (MM_FLOAT)((meanX[i]-(MM_U32)(width/2))*(meanX[i]-(MM_U32)(width/2)) + (meanY[i]-(MM_U32)(height/2))*(meanY[i]-(MM_U32)(height/2)));
		dist = sqrtf(dist);
		//PRINTF("%02d dist: %f\n", i, dist);
		   
		if (minDist>dist) {
			minDist=dist;
			minIdx = i;
		}
	}
	   
	stBorderInfo[0].n = stBorderInfo[minIdx].n;
	for (j=0; j<stBorderInfo[0].n; j++)
	{
		stBorderInfo[0].x[j] = stBorderInfo[minIdx].x[j];
		stBorderInfo[0].y[j] = stBorderInfo[minIdx].y[j];
		// PRINTF("[%03d,%03d]   ",stBorderInfo[0].x[j], stBorderInfo[0].y[j]);
	}
	   
	//memset(gray,0x00,sizeof(MM_U08)*MAX_ROI_SIZE);
	//memcpy(gray,gray_ret,sizeof(MM_U08)*MAX_ROI_SIZE);
	   
	if(*border_num == (MM_U08)(0)) 
	{
		error_code = MM_FALSE;
	}
	
		 
   return error_code;
}



MM_S16  calc_min_dist(	MM_FLOAT ret_ftpoint[],			/* [out] output corner points */
						MM_U16 *ret_ftnum,				/* [out] number of output points */
						BORDERFOLLOW stBorderInfo[],	/* [in] border points */
						MM_U08 border_num,				/* [in] number of border*/
						MM_FLOAT ftpoint[],				/* [in] input corner points */
						MM_U16 ftnum)					/* [in] number of input points */
{
#if 1
	MM_S16 error_code = MM_TRUE;
	MM_U16 j = (MM_U16)0;
	MM_U16 k = (MM_U16)0;

	MM_FLOAT dist  = 0.0f;
	MM_FLOAT distT = 8.0f;
	MM_FLOAT x1    = 0.0f;
	MM_FLOAT y1    = 0.0f;
	MM_FLOAT x2    = 0.0f;
	MM_FLOAT y2    = 0.0f;
	MM_U16 cntpts  = (MM_U16)(0);

	MM_FLOAT Temp1 = 0.0f;
	MM_FLOAT Temp2 = 0.0f;
	MM_FLOAT Temp3 = 0.0f;


	memset(ret_ftpoint,0x00,sizeof(MM_FLOAT)*(MM_U16)(FTPT_SIZE));

	for (j = (MM_U08)(0); j < ftnum; j++)
	{
		x2 = (MM_FLOAT)ftpoint[((MM_U08)(2)*j) + (MM_U08)(0)];
		y2 = (MM_FLOAT)ftpoint[((MM_U08)(2)*j) + (MM_U08)(1)];

		for (k = 0; k < stBorderInfo[0].n; k++)
		{
			y1 = (MM_FLOAT)(stBorderInfo[0].y[k]);
			x1 = (MM_FLOAT)(stBorderInfo[0].x[k]);

			Temp1 = (((MM_FLOAT)(x2)-(MM_FLOAT)(x1)) * ((MM_FLOAT)(x2)-(MM_FLOAT)(x1)));
			Temp2 = (((MM_FLOAT)(y2)-(MM_FLOAT)(y1)) * ((MM_FLOAT)(y2)-(MM_FLOAT)(y1)));
			Temp3 = (Temp1 + Temp2);

			if (Temp3 < 5000000.0f)
			{
				dist = (MM_FLOAT)sqrt((MM_FLOAT)(Temp3));
			}

			if (dist < distT)
			{
				ret_ftpoint[((MM_U16)(2)*cntpts) + (MM_U16)(0)] = x2;
				ret_ftpoint[((MM_U16)(2)*cntpts) + (MM_U16)(1)] = y2;

				k = stBorderInfo[0].n;

				cntpts++;
			}
		}
	}

	*ret_ftnum = cntpts;

	if(*ret_ftnum <= (MM_U16)(0))
	{
		error_code = MM_FALSE;
	}
	

	return error_code;
#else
	MM_S16 error_code = MM_TRUE;
	MM_U08 j = 0;
	MM_U16 k = 0;

	MM_FLOAT dist = 0.0f;
	MM_FLOAT distT = 8.0f;
	MM_FLOAT x1 = 0.0f;
	MM_FLOAT y1 = 0.0f;
	MM_FLOAT x2 = 0.0f;
	MM_FLOAT y2 = 0.0f;
	MM_U16 cntpts = (MM_U16)(0);

	MM_FLOAT Temp1 = 0.0f;
	MM_FLOAT Temp2 = 0.0f;
	MM_FLOAT Temp3 = 0.0f;

#if (DEBUG_MODE)
	PRINTF("calc_min_dist(): start\n");
#endif

	if ((ret_ftpoint == NULL) || (ret_ftnum == NULL) || (stBorderInfo == NULL) || (border_num == (MM_U08)(0)) || (ftpoint == NULL) || (ftnum == (MM_U16)(0))) {
#if (DEBUG_MODE)
		PRINTF("calc_min_dist():Input Argument error\n");
#endif
		error_code = MM_FALSE;
	}
	else {
		memset(ret_ftpoint, 0x00, sizeof(MM_FLOAT)*(MM_U16)(100));

		for (j = (MM_U08)(0); j < ftnum; j++)
		{
			x2 = (MM_FLOAT)ftpoint[((MM_U08)(2)*j) + (MM_U08)(0)];
			y2 = (MM_FLOAT)ftpoint[((MM_U08)(2)*j) + (MM_U08)(1)];

			for (k = 0; k < stBorderInfo[0].n; k++)
			{
				y1 = (MM_FLOAT)(stBorderInfo[0].y[k]);
				x1 = (MM_FLOAT)(stBorderInfo[0].x[k]);

				Temp1 = (((MM_FLOAT)(x2)-(MM_FLOAT)(x1)) * ((MM_FLOAT)(x2)-(MM_FLOAT)(x1)));
				Temp2 = (((MM_FLOAT)(y2)-(MM_FLOAT)(y1)) * ((MM_FLOAT)(y2)-(MM_FLOAT)(y1)));
				Temp3 = (Temp1 + Temp2);

				if (Temp3 < 5000000.0f)
				{
					dist = (MM_FLOAT)sqrt((MM_FLOAT)(Temp3));
				}

				if (dist < distT)
				{
					ret_ftpoint[((MM_U16)(2)*cntpts) + (MM_U16)(0)] = x2;
					ret_ftpoint[((MM_U16)(2)*cntpts) + (MM_U16)(1)] = y2;

					k = stBorderInfo[0].n;

					cntpts++;
				}
			}
		}

		*ret_ftnum = cntpts;

		if (*ret_ftnum <= (MM_U16)(0))
		{
			error_code = MM_FALSE;
#if (DEBUG_MODE)
			PRINTF("calc_min_dist(): number of feature points error [%d]\n", error_code);
#endif
		}
	}

#if (DEBUG_MODE)
	PRINTF("calc_min_dist(): end [%d]\n\n", error_code);
#endif

	return error_code;
#endif
}


void  change_point_order(MM_FLOAT outPts[],	/* [out] output points */
						 MM_FLOAT srcPts[])	/* [in] input points */
{
   MM_FLOAT xSort[4]={0.0f,};
   MM_FLOAT ySort[4]={0.0f,};
   MM_U08 i = (MM_U08)0;
   MM_U08 j = (MM_U08)0;
   MM_FLOAT tmpX=0.0f, tmpY=0.0f;

   MM_FLOAT tempOut[8] = { 0.0f, };
  
	for (i = (MM_U08)(0); i < (MM_U08)(4); i++) 
	{
		xSort[i] = srcPts[((MM_U08)(2)*i) + (MM_U08)(0)];
		ySort[i] = srcPts[((MM_U08)(2)*i) + (MM_U08)(1)];
	}

	for (i = (MM_U08)(0); i < ((MM_U08)(4) - (MM_U08)(1)); i++) 
	{
		for (j = i; j < (MM_U08)(4); j++) 
		{
			if (ySort[i] > ySort[j]) 
			{
				tmpY = ySort[i];
				tmpX = xSort[i];

				ySort[i] = ySort[j];
				ySort[j] = tmpY;
				xSort[i] = xSort[j];
				xSort[j] = tmpX;
			}
		}
	}

	if (xSort[(MM_U08)(0)] > xSort[(MM_U08)(1)]) 
	{
		tempOut[(MM_U08)(0)] = xSort[(MM_U08)(1)];
		tempOut[(MM_U08)(1)] = ySort[(MM_U08)(1)];
		tempOut[(MM_U08)(2)] = xSort[(MM_U08)(0)];
		tempOut[(MM_U08)(3)] = ySort[(MM_U08)(0)];
	}
	else
	{
		tempOut[(MM_U08)(2)] = xSort[(MM_U08)(1)];
		tempOut[(MM_U08)(3)] = ySort[(MM_U08)(1)];
		tempOut[(MM_U08)(0)] = xSort[(MM_U08)(0)];
		tempOut[(MM_U08)(1)] = ySort[(MM_U08)(0)];
	}

	if (xSort[(MM_U08)(2)] > xSort[(MM_U08)(3)]) 
	{
		tempOut[(MM_U08)(4)] = xSort[(MM_U08)(2)];
		tempOut[(MM_U08)(5)] = ySort[(MM_U08)(2)];
		tempOut[(MM_U08)(6)] = xSort[(MM_U08)(3)];
		tempOut[(MM_U08)(7)] = ySort[(MM_U08)(3)];
	}
	else
	{
		tempOut[(MM_U08)(6)] = xSort[(MM_U08)(2)];
		tempOut[(MM_U08)(7)] = ySort[(MM_U08)(2)];
		tempOut[(MM_U08)(4)] = xSort[(MM_U08)(3)];
		tempOut[(MM_U08)(5)] = ySort[(MM_U08)(3)];
	}

	memcpy(outPts, tempOut, sizeof(tempOut));
}
  

static MM_U08 dst_img[48];

MM_S16  erosion1D(MM_U08 src_img[],
				  MM_U16 iSize)
{
    MM_U16 i=(MM_U16)0;
 //   MM_U08 vc=(MM_U08)0;
    MM_U16 v1=(MM_U16)0;
    MM_U16 v2=(MM_U16)0;
    MM_U08 cnt = (MM_U08)0;
    MM_S16 error_code = MM_TRUE;

	memset(dst_img,0x00,sizeof(MM_U08)*48);

	for (i = (MM_U08)(0); i < iSize; i++)
	{
		// if(i > (iSize-(MM_U08)(2)) ) { break;}
		if (i==0)
		{
			v1 = src_img[iSize-(MM_U08)(1)];
			v2 = src_img[i+(MM_U08)(1)];
				   
			//if ( vc == (MM_U08)(255)) { cnt++; } //else {cnt;}
			if ( v1 == (MM_U08)(0)) { cnt++; } //else {cnt;}
			if ( v2 == (MM_U08)(0)) { cnt++; } //else {cnt;}	
		}
		else if(i==iSize-(MM_U08)(1))
		{
			v1 = src_img[i-(MM_U08)(1)];
			v2 = src_img[0];
				   
			//if ( vc == (MM_U08)(255)) { cnt++; } //else {cnt;}
			if ( v1 == (MM_U08)(0)) { cnt++; } //else {cnt;}
			if ( v2 == (MM_U08)(0)) { cnt++; } //else {cnt;}	
		}
		else
		{
			cnt = (MM_U08)(0);
			v1 = src_img[i-(MM_U08)(1)];
			//vc = src_img[i];
			v2 = src_img[i+(MM_U08)(1)];
				   
			//if ( vc == (MM_U08)(255)) { cnt++; } //else {cnt;}
			if ( v1 == (MM_U08)(0)) { cnt++; } //else {cnt;}
			if ( v2 == (MM_U08)(0)) { cnt++; } //else {cnt;}	
		}
		dst_img[i] = (cnt != (MM_U08)(0)) ? (MM_U08)(0) : (MM_U08)(255);
                  
	}
	memcpy(src_img,dst_img,sizeof(MM_U08)*48);

   return error_code;
}

