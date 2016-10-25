#include "calibration.h"
#include "point_extraction.h"

CFUNC_BEGIN_DECLS
static CORNER2DF2DF cornersF;
static MM_FLOAT imagF[PIX_IMG_SIZE*PIX_IMG_SIZE];
static MM_U08 img[PIX_IMG_SIZE*PIX_IMG_SIZE];
static POINTS2DF cT,cI;

static void  corSubAcc(MM_U08 Image[], 
							   CORNER2DF2DF Corners[], 
							   MM_U16 cols, 
							   MM_U16 rows, 
							   MM_U16 WinSZ, 
							   MM_U16 MaxIter, 
							   MM_FLOAT Quality);

CFUNC_END_DECLS


 MM_S16  PixelAccuracy_new(	MM_FLOAT *outFrontCorners,	/* [out] corner points */
							MM_U16 *nOutFront,			/* [out] number of points */
							MM_U08 *srcImgF,			/* [in] source image */
							MM_U16 iHEIGHT,				/* [in] height of image */
							MM_U16 iWidth,				/* [in] width of image */
							MM_FLOAT *inFrontCorners,	/* [im] corner points */
							MM_U16 nFront)				/* [in] number of points */
{
	MM_U16 i=(MM_U16)0;
	MM_S16 error_code = MM_TRUE;
	

	cornersF.CornerSize = (MM_U16)0;
	cornersF.MaxLength = (MM_U16)0;

#if (DEBUG_MODE)
			PRINTF("PixelAccuracy_new(): start\n");
#endif

	if((srcImgF == NULL) || (iHEIGHT == (MM_U16)(0)) || (iWidth == (MM_U16)(0)) || 
	   (inFrontCorners == NULL) || (nFront == (MM_U16)(0)) || (outFrontCorners == NULL) || (nOutFront == NULL)) {
#if (DEBUG_MODE)
		PRINTF("PixelAccuracy_new() : input argument error\n");
#endif
		error_code = MM_FALSE;
	} else {
		memset(cornersF.Corner2DList,0x00,sizeof(POINTS2DF)*FTPT_SIZE);
		cornersF.CornerSize = nFront;
		
		for( i=(MM_U16)(0); i<(MM_U16)(nFront); i++) {
			if((nFront < FTPT_SIZE) &&  (i < FTPT_SIZE)) {
				cornersF.Corner2DList[(MM_U16)(i)].x = (MM_FLOAT)(inFrontCorners[((MM_U16)(2)*i)]);
				cornersF.Corner2DList[(MM_U16)(i)].y = (MM_FLOAT)(inFrontCorners[((MM_U16)(2)*i)+(MM_U16)(1)]);
			}
		}
		(void) corSubAcc(srcImgF, &cornersF, iWidth, iHEIGHT, PIX_WIN_SIZE, (MM_U16)(100), (MM_FLOAT)(0.001f));
		*(nOutFront) = (MM_U16)(cornersF.CornerSize);
	
		memset(outFrontCorners,0,sizeof(MM_FLOAT)* (cornersF.CornerSize)*(MM_U08)(2));
		for( i=(MM_U16)(0); i<(MM_U16)(cornersF.CornerSize); i++) {
			if((cornersF.CornerSize < FTPT_SIZE) && (i < FTPT_SIZE)) {
				outFrontCorners[((MM_U16)(2)*i)]   =			(MM_FLOAT)cornersF.Corner2DList[(MM_U16)(i)].x;
				outFrontCorners[((MM_U16)(2)*i)+(MM_U16)(1)] =  (MM_FLOAT)cornersF.Corner2DList[(MM_U16)(i)].y;		    
			}
		}

		if(cornersF.CornerSize <= (MM_U16)(1)) {
#if (DEBUG_MODE)
			PRINTF("PixelAccuracy_new(): corner point error [%d] \n", cornersF.CornerSize);
#endif
			error_code = MM_FALSE;
		} 

	}
#if (DEBUG_MODE)
			PRINTF("PixelAccuracy_new(): end [%d]\n\n", error_code);
#endif
	return error_code;

}

void  corSubAcc(MM_U08 Image[],
				CORNER2DF2DF Corners[],
				MM_U16 cols,
				MM_U16 rows,
				MM_U16 WinSZ,
				MM_U16 MaxIter,
				MM_FLOAT Quality)
{
	//printf("in corner SubPixel... \n");
	MM_U16 i =(MM_U16)0;
	MM_U16 j =(MM_U16)0;
	MM_U16 k =(MM_U16)0;
	MM_U16 PT_i=(MM_U16)0;
	MM_U16 count =(MM_U16)0;
	MM_FLOAT det=0.0f;
	MM_FLOAT scale=0.0f;
	MM_U16 winSize = 0;
	MM_U16 HalfWinSZ= 0; 
	MM_FLOAT eps = 0.0f;
	MM_U08 *subpix = NULL;
	MM_FLOAT y=0.0f;
	MM_FLOAT vy=0.0f;
	MM_FLOAT x=0.0f;
	MM_S32 iter = (MM_S32)0;
	MM_FLOAT err = 0.0f;
	POINTS2DF cI2 = {0.0f,0.0f};
	MM_FLOAT a = 0.0f;
	MM_FLOAT b = 0.0f;
	MM_FLOAT c = 0.0f;
	MM_FLOAT bb1 = 0.0f;
	MM_FLOAT bb2 = 0.0f; 
	MM_FLOAT py = 0.0f;
	MM_FLOAT m = 0.0f;
	MM_FLOAT tgx = 0.0f;
	MM_FLOAT tgy = 0.0f;
	MM_FLOAT gxx = 0.0f;
	MM_FLOAT gxy = 0.0f;
	MM_FLOAT gyy = 0.0f;
	MM_FLOAT px = 0.0f;
	MM_U08 leftV = (MM_U08)0;
	MM_U08 rightV= (MM_U08)0;
	MM_S16 flag = MM_TRUE;

	
	winSize = (WinSZ*(MM_U16)(2))+(MM_U16)(1);
	HalfWinSZ= winSize/(MM_U16)(2);
	eps = Quality*Quality;

	memset(imagF,0,sizeof(MM_FLOAT)*PIX_IMG_SIZE*PIX_IMG_SIZE);
	memset(img,0,sizeof(MM_U08)*PIX_IMG_SIZE*PIX_IMG_SIZE);
	
	if( Corners->CornerSize >= (MM_U32)(1)) {
		/* calculate mask */
	
		for( i = (MM_U16)(0); i < winSize; i++ )
		{
			y = (((MM_FLOAT)(i) - (MM_FLOAT)(WinSZ))/(MM_FLOAT)(WinSZ));
			vy = (MM_FLOAT)(exp((MM_FLOAT)(-y*y)));
		
			for( j = (MM_U16)(0); j < winSize; j++ )
			{
				x = (((MM_FLOAT)(j) - (MM_FLOAT)(WinSZ))/(MM_FLOAT)(WinSZ));
				//Mask->ImageDataF[i * winSize + j] = (float)(vy*expf(-x*x));
				imagF[(i*winSize)+j] = (MM_FLOAT)(vy*exp((MM_FLOAT)(-x*x)));
			}
		}

		/* do optimization loop for all the points */
		for(PT_i = (MM_U16)(0) ; PT_i < Corners->CornerSize; PT_i++)
		{
			iter=(MM_S32)(0);
			err=(MM_FLOAT)(0);
			if(PT_i < FTPT_SIZE) {
				cT = Corners->Corner2DList[PT_i];
			}
		
			cI=cT;
		
			do
			{
				/*cI2.x = (MM_FLOAT)(0.0f);*/
				/*cI2.y = (MM_FLOAT)(0.0f);*/
				a = (MM_FLOAT)(0.0f);
				b = (MM_FLOAT)(0.0f);
				c = (MM_FLOAT)(0.0f);
				bb1 = (MM_FLOAT)(0.0f);
				bb2 = (MM_FLOAT)(0.0f); 
				cI2.x = (MM_FLOAT)(0.0f);
				cI2.y = (MM_FLOAT)(0.0f);
				count = (MM_U16)(0);
				memset(img,0,sizeof(MM_U08)*PIX_IMG_SIZE*PIX_IMG_SIZE);	
				 /* calc derivatives */ // ...........?
				if( (cI.y < 300.0f) && (cI.y > 4.0f) && (cI.x < 250.0f) && (cI.x > 4.0f) ) {
					MM_FLOAT r0 = 0.0f, r1 = 0.0f, r2 = 0.0f, r3 = 0.0f, r4 = 0.0f;
					r0 = ((MM_FLOAT)(HalfWinSZ)+(MM_FLOAT)(1));
					r1 = ((MM_FLOAT)(cI.y) - (MM_FLOAT)(r0));
					r2 = ((MM_FLOAT)(cI.y) + (MM_FLOAT)(r0));
					r3 = ((MM_FLOAT)(cI.x) - (MM_FLOAT)(r0));
					r4 = ((MM_FLOAT)(cI.x) + (MM_FLOAT)(r0));

					for( i = (MM_U16)(r1); i <= (MM_U16)(r2); i++) {
						for(j = (MM_U16)(r3); j <= (MM_U16)(r4); j++) {
					
							if( ((j >= cols) ||   (i >= rows)) ) {
								continue;
							} else {
								
								if(count < (MM_U16)(81)) {
									img[count] = Image[j + (i*cols)];
								}

								if (count < (MM_U16)(255)) {
									count++;
								}
							}
						}
					}
				}
				subpix = &img[10];

			 
				k = (MM_U16)(0);
				for(i = (MM_U16)(0); i < winSize; i++)
				{
					py = ((MM_FLOAT)(i) - (MM_FLOAT)(WinSZ));
				
					for( j = (MM_U16)(0); j < winSize; j++)
					{
						m = imagF[k];
						tgx = ((MM_FLOAT)(subpix[j+(MM_U16)(1)]) - (MM_FLOAT)(subpix[j-(MM_U16)(1)]));
						tgy = ((MM_FLOAT)(subpix[j+winSize+(MM_U16)(2)]) - (MM_FLOAT)(subpix[j-winSize-(MM_U16)(2)]));
						gxx = tgx * tgx * m;
						gxy = tgx * tgy * m;
						gyy = tgy * tgy * m;
						px = ((MM_FLOAT)(j) - (MM_FLOAT)(WinSZ));

						a += gxx;
						b += gxy;
						c += gyy;
					
						bb1 += (gxx * px) + (gxy * py);
						bb2 += (gxy * px) + (gyy * py);		

						k++;
					}

					subpix += (MM_U08)(winSize + (MM_U16)(2));
				}
				if((a <=600000.0f) && (a >(-600000.0f)) && (b <= 600000.0f) && (b >(-600000.0f)) && (c <= (600000.0f)) && (c >(-600000.0f))) {
					det=(a*c)-(b*b);
				}
				if( fabs( det ) <= (MM_FLOAT)(DBL_EPSILON1*DBL_EPSILON1) ) {
					//break;
					flag = MM_FALSE;
				}
				else
				{
					// 2x2 matrix inversion
					if (det !=(MM_FLOAT)(0)) {
						scale=(MM_FLOAT)((MM_FLOAT)(1.0f)/det);
					}
					if((scale > (MM_FLOAT)(0)) && (scale < (MM_FLOAT)(1)) && (bb1 <= (MM_FLOAT)(600000)) && (bb1 >(MM_FLOAT)(-600000)) && (bb2 <= (MM_FLOAT)(600000)) && (bb2 >(MM_FLOAT)(-600000))) {
						cI2.x = (cI.x + (c*scale*bb1) - (b*scale*bb2));
						cI2.y = (cI.y - (b*scale*bb1) + (a*scale*bb2));
					}
					//err = (cI2.x - cI.x) * (cI2.x - cI.x) + (cI2.y - cI.y) * (cI2.y - cI.y);
					if((cI2.x > (MM_FLOAT)(0)) && (cI2.x < (MM_FLOAT)(300)) && (cI2.y > (MM_FLOAT)(0)) && (cI2.y < (MM_FLOAT)(250)) && (cI.x > (MM_FLOAT)(0)) &&
						(cI.x < (MM_FLOAT)(300)) && (cI.y > (MM_FLOAT)(0)) && (cI.y < (MM_FLOAT)(250))) {
						err = ((cI2.x - cI.x) * (cI2.x - cI.x)) + ((cI2.y - cI.y) * (cI2.y - cI.y));
					}
					cI = cI2;
					if((cI.x < 300.0f) && (cI.x > 0.0f) && (cI.y > 0.0f) && (cI.y < 250.0f)) {
						if( (cI.x < 0.0f) || (cI.x >= (MM_FLOAT)(cols)) || (cI.y < 0.0f) || (cI.y >= (MM_FLOAT)(rows)) ) {
							//break;
							flag = MM_FALSE;
						}
					}
				}
				//if( cI.x < 0 || cI.x >= cols || cI.y < 0 || cI.y >= rows )
					//break;
			
				++iter;
			}while( (iter < (MM_S32)(MaxIter)) && (err > eps) && (flag == MM_TRUE) );

			if((cI.x < 300.0f) && (cI.x > 0.0f) && (cI.y > 0.0f) && (cI.y < 250.0f)) {
				
				a = (MM_FLOAT)(fabs( (MM_FLOAT)( (MM_FLOAT)(cI.x) - (MM_FLOAT)(cT.x)))) ;
				b = (MM_FLOAT)(fabs( (MM_FLOAT)( (MM_FLOAT)(cI.y) - (MM_FLOAT)(cT.y))) );
				
				if(a < 256.0f) {
					leftV =  (MM_U08)(a); 
				}
				if(b < 256.0f) {
					rightV = (MM_U08)(b);
				}

				if( (leftV > (MM_U08)(WinSZ)) || (rightV > (MM_U08)((WinSZ))) ) {
					cI = cT;
				}
			}
			if(PT_i < FTPT_SIZE) {
				Corners->Corner2DList[PT_i] =  cI;
			}
		}
	}

}

