#include<string.h>
#include<math.h>
#include "calibration.h"
#include "feature_extraction.h"

CFUNC_BEGIN_DECLS

static CORNERINFO corner;
static CORNER2DF2DF frontCorners;

static MM_FLOAT Gx[MAX_ROI_SIZE];
static MM_FLOAT Gy[MAX_ROI_SIZE];
static MM_FLOAT dx2[MAX_ROI_SIZE];
static MM_FLOAT dy2[MAX_ROI_SIZE];
static MM_FLOAT dxy[MAX_ROI_SIZE];

static void  SobelGradient(MM_U08 srcImg[], MM_FLOAT gx[], MM_FLOAT gy[], MM_FLOAT Scale, CALIBRATION_PARAM *param);
static MM_FLOAT  CornerEigenValsVecs(MM_U08 src[], MM_FLOAT eigenv[], MM_U32  block_size, CALIBRATION_PARAM *param);
static MM_S16  Find_Feature_Points(MM_U08 *img,CORNER2DF2DF *OutCorners, MM_FLOAT qualityLevel, MM_U08 blockSize, CALIBRATION_PARAM *param);
static MM_FLOAT  CalMinEigValue(  MM_FLOAT  Dx2,  MM_FLOAT Dy2,  MM_FLOAT Dxy);
static void  thresholdToZero(MM_FLOAT srcImg[], MM_FLOAT MaxVal, MM_U32 h, MM_U32 w);
static void  Dilate(MM_FLOAT src[], MM_FLOAT dst[], MM_U16 hight, MM_U16 width);
static void calculate_corner(CORNER2DF2DF *OutCorners,CALIBRATION_PARAM *param);


CFUNC_END_DECLS


MM_S16  GetCorner_new(	MM_FLOAT *outFrontCorners,		/* [out] corner points */
						MM_U16 *nFront,					/* [out] unmber of corner points */
						MM_U08 *srcImgF,				/* [in] source image */
						MM_U16 iHEIGHT,					/* [in] height of image */
						MM_U16 iWidth,					/* [in] width of image */
						CALIBRATION_PARAM *param	/* calibration information */
)
{
	MM_S16 ecode = MM_TRUE;
	MM_U16 i=(MM_U16)0;

	frontCorners.CornerSize = (MM_S16)(0);
	frontCorners.MaxLength =  (MM_S16)(0);
	
	memset(frontCorners.Corner2DList,0x00,sizeof(POINTS2DF)*FTPT_SIZE);
		
	ecode = Find_Feature_Points(srcImgF, &frontCorners, (MM_FLOAT)(QUALITY_LEVEL),  GF_BLOCK_SIZE, param );

	if (ecode == MM_TRUE)
	{
		*nFront = hvMin((MM_U16)frontCorners.CornerSize, FTPT_SIZE);

		for (i = (MM_U16)(0); i < *nFront; i++)
		{
			outFrontCorners[((MM_U16)(2)*i)] = frontCorners.Corner2DList[(MM_U16)(i)].x;
			outFrontCorners[((MM_U16)(2)*i) + (MM_U16)(1)] = frontCorners.Corner2DList[(MM_U16)(i)].y;
		}
	}

	return ecode;
}


static MM_FLOAT eig[MAX_ROI_SIZE];
static MM_FLOAT dstEig[MAX_ROI_SIZE];


MM_S16  Find_Feature_Points(MM_U08 *img,
							CORNER2DF2DF *OutCorners,
							MM_FLOAT qualityLevel,
							MM_U08 blockSize,
							CALIBRATION_PARAM *param)
{
	MM_FLOAT min = 0.0f;
	MM_S16    error_code = MM_TRUE;

	memset(dstEig, 0x00, sizeof(MM_FLOAT)*MAX_ROI_SIZE);
	memset(&corner, 0x00, sizeof(CORNERINFO));

	corner.length = (MM_U32)(0);
	corner.size = PRL_MAX_CORNERS;

	memset(eig, 0, sizeof(MM_FLOAT)* MAX_ROI_SIZE);
	min = CornerEigenValsVecs(img, eig, (MM_U32)(blockSize), param);

	if (min != (MM_FLOAT)(0)) {
		min = (min*(qualityLevel));
	}

	thresholdToZero(eig, min, param->roi_height, param->roi_width);

	Dilate(eig, dstEig, (MM_U16)(param->roi_height), (MM_U16)(param->roi_width));

	calculate_corner(OutCorners, param);


	if (OutCorners->CornerSize == (MM_S16)(0))
	{
		error_code = MM_FALSE;
	}

	return error_code;
}



void calculate_corner(CORNER2DF2DF *OutCorners, CALIBRATION_PARAM *param)
{
	MM_S32 i = (MM_S32)0;
	MM_S32 x = (MM_S32)0;
	MM_S32 y = (MM_S32)0;
	MM_U32 cnt = (MM_U32)0;
	MM_FLOAT val = 0.0f;

	for (y = (MM_S32)(2); y< (MM_S32)((MM_S32)((MM_U16)(param->roi_height)) - (MM_S32)(2)); y++) 
	{
		for (x = (MM_S32)(2); x< (MM_S32)((MM_S32)((MM_U16)(param->roi_width)) - (MM_S32)(2)); x++) 
		{
			i = x + (y * (MM_S32)((MM_U16)(param->roi_width)));
			val = eig[i];

			if ((val != (MM_FLOAT)(0)) && (val == dstEig[i])) 
			{
				if (cnt < FTPT_SIZE) 
				{
					OutCorners->Corner2DList[(MM_U16)(cnt)].x = (MM_FLOAT)x;
					OutCorners->Corner2DList[(MM_U16)(cnt)].y = (MM_FLOAT)y;
				}
				else 
				{
					break;
				}

				cnt++;
			}
			else {
				eig[i] = (MM_FLOAT)(0.0f);
			}
		}
	}

	if (cnt != (MM_U32)(0))
	{
		corner.length = cnt - (MM_U32)(1);
	}
	else
	{
		corner.length = (MM_U32)(0);
	}
	OutCorners->CornerSize = (MM_U16)(corner.length);
}


MM_FLOAT  CornerEigenValsVecs(MM_U08 src[],
							  MM_FLOAT eigenv[],
							  MM_U32 block_size,
							  CALIBRATION_PARAM *param)
{
	MM_U08 tmpScale = (MM_U08)0, tmpV =(MM_U08)0;
	MM_U32 PixelIndex= (MM_U32)0;
	MM_U32 x = (MM_U32)0, y =(MM_U32)0, nY=(MM_U32)0, BlockHalfSz = (MM_U32)0;
	MM_FLOAT sumdx2 = 0.0f, sumdy2 = 0.0f, sumdxy = 0.0f;
	MM_FLOAT DX = 0.0f, DY = 0.0f, DXX = 0.0f, DYY = 0.0f, DXY = 0.0f;
	MM_FLOAT MinEigenVal = 0.0f, scale = 0.0f;
	
	memset(Gx,0x00, sizeof(MM_FLOAT)* MAX_ROI_SIZE);
	memset(Gy,0x00, sizeof(MM_FLOAT)* MAX_ROI_SIZE);
	memset(dx2,0x00, sizeof(MM_FLOAT)* MAX_ROI_SIZE);
	memset(dy2,0x00, sizeof(MM_FLOAT)* MAX_ROI_SIZE);
	memset(dxy,0x00, sizeof(MM_FLOAT)* MAX_ROI_SIZE);
	
	BlockHalfSz= (MM_U32)(block_size/(MM_U32)(2));
	
	tmpScale = APARTURE_SIZE;
	/* tmpScale = (tmpScale > (MM_U08)(0)) ? APARTURE_SIZE : (MM_U08)(3); */

	tmpV = (MM_U08)((MM_U08)(1) << (MM_U08)(( tmpScale - (MM_U08)(1))));

	scale = (MM_FLOAT)(tmpV ) * (MM_FLOAT)(block_size);

        scale *= (MM_FLOAT)(255.0f);
        scale =  (MM_FLOAT)(1.0f)/scale;

		SobelGradient( src, Gx,Gy, scale, param);

        for( y = BlockHalfSz; y < (MM_U32)(((MM_U16)(param->roi_height)) -BlockHalfSz); y++)
        {
                for( x = BlockHalfSz; x < (MM_U32)(((MM_U16)(param->roi_width)) -BlockHalfSz); x++)
                {
                      //  MM_U32 cols=0,Top=0, Left=0, TopLeft=0 ;
                        PixelIndex= ((MM_U32)((MM_U16)(param->roi_width))*y) +x;

                        DX = Gx[PixelIndex];
                        DY = Gy[PixelIndex];

                        DXX =DX*DX;
                        DYY =DY*DY;
                        DXY =DX*DY;

                        dx2[PixelIndex] = DXX;
                        dy2[PixelIndex] = DYY;
                        dxy[PixelIndex] = DXY;
                }

                if( y < (MM_U32)(BlockHalfSz+BlockHalfSz))
				{
					continue;
				}

                nY= y - BlockHalfSz;
                for( x = BlockHalfSz; x < (MM_U32)((MM_U16)(param->roi_width) -BlockHalfSz); x++)
                {
                        PixelIndex= ((MM_U32)((MM_U16)(param->roi_width))*nY) +x;
                        sumdx2= (MM_FLOAT)(0);
						sumdy2= (MM_FLOAT)(0);
						sumdxy= (MM_FLOAT)(0);

                        //new last time
                        sumdx2 += (dx2[((MM_U32)((MM_U16)(param->roi_width))*(nY-(MM_U32)(1)))+(x-(MM_U32)(1))] + dx2[((MM_U32)((MM_U16)(param->roi_width))*(nY-(MM_U32)(1)))+(x)] + dx2[((MM_U32)((MM_U16)(param->roi_width))*(nY-(MM_U32)(1)))+(x+(MM_U32)(1))] +
                                   dx2[((MM_U32)((MM_U16)(param->roi_width))*(nY))+(x-(MM_U32)(1))]   + dx2[((MM_U32)((MM_U16)(param->roi_width))*(nY))+(x)]   + dx2[((MM_U32)((MM_U16)(param->roi_width))*(nY))+(x+(MM_U32)(1))] +
                                   dx2[((MM_U32)((MM_U16)(param->roi_width))*(nY+(MM_U32)(1)))+(x-(MM_U32)(1))] + dx2[((MM_U32)((MM_U16)(param->roi_width))*(nY+(MM_U32)(1)))+(x)] + dx2[((MM_U32)((MM_U16)(param->roi_width))*(nY+(MM_U32)(1)))+(x+(MM_U32)(1))]);

                        sumdy2 += (dy2[((MM_U32)((MM_U16)(param->roi_width))*(nY-(MM_U32)(1)))+(x-(MM_U32)(1))] + dy2[((MM_U32)((MM_U16)(param->roi_width))*(nY-(MM_U32)(1)))+(x)] + dy2[((MM_U32)((MM_U16)(param->roi_width))*(nY-(MM_U32)(1)))+(x+(MM_U32)(1))] +
                                   dy2[((MM_U32)((MM_U16)(param->roi_width))*(nY))+(x-(MM_U32)(1))]   + dy2[((MM_U32)((MM_U16)(param->roi_width))*(nY))+(x)]   + dy2[((MM_U32)((MM_U16)(param->roi_width))*(nY))+(x+(MM_U32)(1))] +
                                   dy2[((MM_U32)((MM_U16)(param->roi_width))*(nY+(MM_U32)(1)))+(x-(MM_U32)(1))] + dy2[((MM_U32)((MM_U16)(param->roi_width))*(nY+(MM_U32)(1)))+(x)] + dy2[((MM_U32)((MM_U16)(param->roi_width))*(nY+(MM_U32)(1)))+(x+(MM_U32)(1))]);

                        sumdxy += (dxy[((MM_U32)((MM_U16)(param->roi_width))*(nY-(MM_U32)(1)))+(x-(MM_U32)(1))] + dxy[((MM_U32)((MM_U16)(param->roi_width))*(nY-(MM_U32)(1)))+(x)] + dxy[((MM_U32)((MM_U16)(param->roi_width))*(nY-(MM_U32)(1)))+(x+(MM_U32)(1))] +
                                   dxy[((MM_U32)((MM_U16)(param->roi_width))*(nY))+(x-(MM_U32)(1))]   + dxy[((MM_U32)((MM_U16)(param->roi_width))*(nY))+(x)]   + dxy[((MM_U32)((MM_U16)(param->roi_width))*(nY))+(x+(MM_U32)(1))] +
                                   dxy[((MM_U32)((MM_U16)(param->roi_width))*(nY+(MM_U32)(1)))+(x-(MM_U32)(1))] + dxy[((MM_U32)((MM_U16)(param->roi_width))*(nY+(MM_U32)(1)))+(x)] + dxy[((MM_U32)((MM_U16)(param->roi_width))*(nY+(MM_U32)(1)))+(x+(MM_U32)(1))]);
                        //end of new last time
                        ////////////////////////////////////
                        if((sumdx2== (MM_FLOAT)(0)) && (sumdy2== (MM_FLOAT)(0)) && (sumdxy== (MM_FLOAT)(0)))
                        { continue;}

                        eigenv[PixelIndex] = CalMinEigValue( sumdx2,sumdy2,sumdxy);
                        MinEigenVal =  (eigenv[PixelIndex]>MinEigenVal) ? eigenv[PixelIndex] : MinEigenVal;
                }
        }

        return MinEigenVal;
}



static MM_FLOAT xbuf[MAX_ROI_SIZE];
static MM_FLOAT ybuf[MAX_ROI_SIZE];

void  SobelGradient(MM_U08 srcImg[],
						  MM_FLOAT gx[],
						  MM_FLOAT gy[],
						  MM_FLOAT Scale,
						  CALIBRATION_PARAM *param)
{
	
	MM_U32 tmpV = (MM_U32)0;
	MM_U32 x = (MM_U32)0;
	MM_U32 y = (MM_U32)0;
	MM_U32 line_base1 = (MM_U32)0;
	MM_U32 line_base2 = (MM_U32)0;
	MM_U32 line_base3 = (MM_U32)0;
	MM_U32 pos = (MM_U32)0;

	MM_S32 first = (MM_S32)0;
	/* MM_S32 second = 0; */
	MM_S32 third = (MM_S32)0;
	
	memset((void *)xbuf,0x00,sizeof(MM_FLOAT)*param->roi_height*param->roi_width);
	memset((void *)ybuf,0x00,sizeof(MM_FLOAT)*param->roi_height*param->roi_width);
	
	for(y = (MM_U32)(1); y< (MM_U32)((MM_U32)((MM_U16)(param->roi_height)) - (MM_U32)(1)); y++)
	{
		line_base1 = (y*(MM_U32)((MM_U16)(param->roi_width)));
		for(x = (MM_U32)(1); x < (MM_U32)((MM_U32)((MM_U16)(param->roi_width)) - (MM_U32)(1)); x++)
		{
			pos = line_base1 + x;
			//xbuf[pos] =  (MM_FLOAT)(srcImg[(MM_U32)(pos+(MM_U32)(1))]-srcImg[(MM_U32)(pos-(MM_U32)(1))]);
			//ybuf[pos] =  (MM_FLOAT)(srcImg[pos-(MM_U32)(1)] +(srcImg[pos]<<(MM_U32)(1)) + srcImg[pos+(MM_U32)(1)]) * Scale;

			first = (MM_S32)srcImg[pos-(MM_U32)(1)];
			/* second = (MM_S32)srcImg[pos]; */
			third = (MM_S32)srcImg[pos+(MM_U32)(1)];

			xbuf[pos] =  ( (MM_FLOAT)(third) - (MM_FLOAT)(first) );

			tmpV = ((MM_U32)(srcImg[pos-(MM_U32)(1)]) +  ((MM_U32)(srcImg[pos])<<(MM_U32)(1)) + (MM_U32)(srcImg[pos+(MM_U32)(1)]));
			ybuf[pos] =  ((MM_FLOAT)(tmpV) * Scale);
			
		}
	}
	
	for(y = (MM_U32)(1); y < (MM_U32)((MM_U32)((MM_U16)(param->roi_height)) - (MM_U32)(1)); y++)
	{
		line_base1 = (y-(MM_U32)(1))*(MM_U32)((MM_U16)(param->roi_width));
		line_base2 = (y+(MM_U32)(0))*(MM_U32)((MM_U16)(param->roi_width));
		line_base3 = (y+(MM_U32)(1))*(MM_U32)((MM_U16)(param->roi_width));
		for(x = (MM_U32)(1); x < (MM_U32)((MM_U32)((MM_U16)(param->roi_width)) - (MM_U32)(1)); x++)
		{
			gx[line_base2+x] =  ((xbuf[line_base1+x] + (xbuf[line_base2+x]*(MM_FLOAT)(2))+ xbuf[line_base3+x] )*Scale);
			gy[line_base2+x] =   (ybuf[line_base3+x]-ybuf[line_base1+x]);
		}
	}

}

MM_FLOAT  CalMinEigValue(MM_FLOAT Dx2,
				 	     MM_FLOAT Dy2,
					     MM_FLOAT Dxy)
{
	MM_FLOAT fValue = 0.0f;

    if( (Dx2 == (MM_FLOAT)(0)) &&( Dy2 == (MM_FLOAT)(0)) && (Dxy == (MM_FLOAT)(0)))
	{
            fValue = (MM_FLOAT)(0.0f);
    }
	else
	{
		Dx2 *= (MM_FLOAT)(0.5f);
		Dy2 *= (MM_FLOAT)(0.5f);
		fValue = (MM_FLOAT)(Dx2 + Dy2) - (MM_FLOAT)(sqrt( (MM_FLOAT)(((Dx2 - Dy2)*(Dx2 - Dy2)) + (Dxy*Dxy))) ); 
	}
		
	return fValue; 
}

void  thresholdToZero(MM_FLOAT srcImg[],
					 MM_FLOAT MaxVal,
					 MM_U32 h, 
					 MM_U32 w)
{
	MM_U32 i = (MM_U32)0;

	for( i=(MM_U32)(0); i < MAX_ROI_SIZE; i++)
	{
		if((srcImg[i]== (MM_FLOAT)(0)) || (srcImg[i]>=MaxVal))
		{
			continue;
		}
		srcImg[i] = (MM_FLOAT)(0.0f);
	}
}

void  Dilate(MM_FLOAT src[], 
				   MM_FLOAT dst[], 
				   MM_U16 hight, 
				   MM_U16 width)
{
	MM_U32 x= (MM_U32)0;
	MM_U32 y = (MM_U32)0;
	MM_U32 nX= (MM_U32)0;
	MM_U32 nY= (MM_U32)0;
	MM_U32 halfWin = (MM_U32)0;
	MM_U32 Index= (MM_U32)0;
	MM_U32 BlockIndex= (MM_U32)0;
    MM_FLOAT val= 0.0f;
	MM_FLOAT blockVal=0.0f;
    MM_U32 rows = (MM_U32)0;
	MM_U32 cols = (MM_U32)0;

	halfWin =(MM_U32)(7/2);
	rows = (MM_U32)hight;
	cols = (MM_U32)width;
	
	memset(dst,0, sizeof(MM_FLOAT)* MAX_ROI_SIZE);

	for( y=halfWin; y< (MM_U32)(rows-halfWin); y++)
	{
        for( x=halfWin; x< (MM_U32)(cols-halfWin); x++)
        {
        Index = (y*cols) +  x ;
        val = src[Index];
        for( nY = (MM_U32)(y-halfWin); nY<= (y+ halfWin); nY++)
        {
            for( nX = (MM_U32)(x-halfWin); nX<= (x+ halfWin); nX++)
            {
                BlockIndex = (nY*cols) + nX;
                blockVal = src[BlockIndex];
                if((Index==BlockIndex)|| (blockVal<= val))
				{
					continue;
				}

                val = blockVal;
            }
        }
        dst[Index]=val;
        }
    }
}

