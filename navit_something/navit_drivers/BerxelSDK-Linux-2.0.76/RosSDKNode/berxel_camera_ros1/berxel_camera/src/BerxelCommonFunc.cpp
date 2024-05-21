#include <stdint.h>
#include <string.h>
#include <string>
#include <stdio.h>


#if defined(_WIN32)
#include <windows.h>
#include <mmsystem.h>
#else
#include <sys/time.h>
#endif


#include "BerxelCommonFunc.h"

#define MIN(a,b)            (((a) < (b)) ? (a) : (b))
#define MAX(a,b)            (((a) > (b)) ? (a) : (b))

#define MaxE(a,b)            (((a) > (b)) ? (a) : (b))
#define MinE(a,b)            (((a) < (b)) ? (a) : (b))

BerxelCommonFunc* BerxelCommonFunc::m_commonFunc = NULL;

BerxelCommonFunc::BerxelCommonFunc()
{


}


BerxelCommonFunc::~BerxelCommonFunc()
{
	if(m_commonFunc)
	{
		delete m_commonFunc;
		m_commonFunc = NULL;
	}
}

BerxelCommonFunc* BerxelCommonFunc::getInstance()
{

	if(m_commonFunc == NULL)
	{
		m_commonFunc = new BerxelCommonFunc();
	}
	return m_commonFunc;
}



#define MAX_DEPTH_HIST 100000





int BerxelCommonFunc::MakeColorTable(unsigned char *red, unsigned char *green, unsigned char *blue)
{
	const int terminal_value = 100;//100

								   //blue   = [0:255, full,  full,  255 : -1 : 0, zero,         zero];
								   //green  = [zero,  0:255, full,  full,         255 : -1 : 0, zero];
								   //red    = [zero,  zero,  0:255, full,         full,         255 : -1 : 0];
	int k = 0;
	for (int i = terminal_value; i < 256; i++, k++)
	{
		blue[k] = i;
		green[k] = 0;
		red[k] = 0;
	}
	for (int i = 0; i < 256; i++, k++)
	{
		blue[k] = 255;
		green[k] = i;
		red[k] = 0;
	}
	for (int i = 0; i < 127; i++, k++)
	{
		blue[k] = 255;
		green[k] = 255;
		red[k] = i * 2;
	}
	for (int i = 0; i < 127; i++, k++)
	{
		blue[k] = 255 - i * 2;
		green[k] = 255;
		red[k] = 255;
	}
	for (int i = 0; i < 256; i++, k++)
	{
		blue[k] = 0;
		green[k] = 255 - i;
		red[k] = 255;
	}
	for (int i = terminal_value; i < 256; i++, k++)
	{
		blue[k] = 0;
		green[k] = 0;
		red[k] = 255 - i + terminal_value;
	}
	return k;
}

void BerxelCommonFunc::MaxMin(const short* data, int len, short& max_value, short& min_value)
{
	const short *src = data;
	const short *src_end = data + len;
	max_value = *data;
	min_value = *data;
	while (src < src_end)
	{
		if (*src != 0)
		{
			if (max_value < *src)
			{
				max_value = *src;
			}
			else if (min_value > *src)
			{
				min_value = *src;
			}
		}
		src++;
	}
}

//在指定定义区间内，给深度图附不同的彩色信息
void BerxelCommonFunc::ImageScaleColor(uint8_t* color, uint16_t* depth, int height, int width, int range_min, int range_max, berxel::BerxelHawkPixelType pixelType)
{
	unsigned char red[256 * 6];
	unsigned char green[256 * 6];
	unsigned char blue[256 * 6];

	static uint16_t    s_depthOri[1280 *800];
	memcpy(s_depthOri, depth, width * height *2);

	/* 如何获取深度图精度
	1. pixelType == BERXEL_HAWK_PIXEL_TYPE_DEP_16BIT_12I_4D
		深度图共16位,前面12位为整数部分，单位是1mm，后面4位位小数部分，单位是0.0625mm

		uint16_t depthOri = pDepth[i];

		float  depthFront = depthOri >> 4;
		float  depthTail  = (depthOri & 0x000f)/16; 

		float depth =depthFront + depthTail;


	2. pxelType ==  BERXEL_HAWK_PIXEL_TYPE_DEP_16BIT_13I_3D 
		  
		深度图共16位,前面13位为整数部分，单位是1mm，后面3位位小数部分，单位是0.125mm

		uint16_t depthOri = pDepth[i];

		float  depthFront = depthOri >> 3;
		float  depthTail  = (depthOri & 0x0007)/8; 

		float depth =depthFront + depthTail;

	*/
		
		//获取深度图整数部分
	

	if(pixelType == berxel::BERXEL_HAWK_PIXEL_TYPE_DEP_16BIT_13I_3D)
	{
		for(uint32_t i = 0; i < width * height ; ++i)
		{
			s_depthOri[i] = s_depthOri[i] >> 3;
		}
	}
	else
	{
		for(uint32_t i = 0; i < width * height ; ++i)
		{
			s_depthOri[i] = s_depthOri[i] >> 4;
		}
	}
		

	int elem_cnt = MakeColorTable(red, green, blue);

	/*for (int i = 0; i < elem_cnt; i++)
	{
	printf("%d %d %d\n", red[i],green[i],blue[i]);
	}*/

	short max_value, min_value;
	MaxMin((const short *)s_depthOri, height*width, max_value, min_value);
	if (range_min != range_max)
	{
		min_value = range_min;
		max_value = range_max;
	}
	unsigned short range_value = MaxE(1, max_value - min_value);  //获取定义区间深度值差，最小设置为1

	const short *src = (const short *)s_depthOri;
	const short *src_end = (const short *)s_depthOri + height*width;
	unsigned char *dst = color;
	while (src < src_end)
	{
		short value = MaxE(min_value, MinE(max_value, (*src)));  //获取定义区间[min_value,max_value]的深度值
		if (value == 0)
		{
			dst[0] = 0;
			dst[1] = 0;
			dst[2] = 0;
		}
		else
		{
			int norm_value = (float)(value - min_value) / (float)range_value * (elem_cnt - 1) + 0.5;
			dst[0] = blue[norm_value];
			dst[1] = green[norm_value];
			dst[2] = red[norm_value];
		}
		src++;
		dst += 3;
	}
}


// Histogram view mode
void BerxelCommonFunc::covertHist(float* pHist, int histSize, uint16_t *psrcData, uint32_t srcwidth, uint32_t srcheight)
{
	const uint16_t* pDepth = (const uint16_t*)psrcData;
	unsigned int nPointsCount = 0;

	memset(pHist, 0, histSize*sizeof(float));

	int height = srcheight;
	int width  = srcwidth;

	
	for(int y = 0; y < height; ++y)
	{
		for(int x = 0; x < width; ++x, ++pDepth)
		{
			if(*pDepth != 0)
			{
				pHist[*pDepth]++;
				nPointsCount++;
			}
		}
	}

	for(int nIndex = 1; nIndex < histSize; ++nIndex)
	{
		pHist[nIndex] += pHist[nIndex-1];
	}

	if(nPointsCount)
	{
		for(int nIndex = 1; nIndex < histSize; ++nIndex)
		{
			pHist[nIndex] = (256 * (1.0f - (pHist[nIndex] / nPointsCount)));
		}
	}
}




void BerxelCommonFunc::convertDepthToRgbByHist(uint16_t* pDepth, RGB888* pRgb ,int width, int height,berxel::BerxelHawkPixelType pixelType)
{
	static float       s_depthHist[MAX_DEPTH_HIST];
	static uint16_t    s_depthOri[1280 *800];
	memcpy(s_depthOri, pDepth, width * height *2);

	/* 如何获取深度图精度
	1. pixelType == BERXEL_HAWK_PIXEL_TYPE_DEP_16BIT_12I_4D
		深度图共16位,前面12位为整数部分，单位是1mm，后面4位位小数部分，单位是0.0625mm

		uint16_t depthOri = pDepth[i];

		float  depthFront = depthOri >> 4;
		float  depthTail  = (depthOri & 0x000f)/16; 

		float depth =depthFront + depthTail;


	2. pxelType ==  BERXEL_HAWK_PIXEL_TYPE_DEP_16BIT_13I_3D 
		  
		深度图共16位,前面13位为整数部分，单位是1mm，后面3位位小数部分，单位是0.125mm

		uint16_t depthOri = pDepth[i];

		float  depthFront = depthOri >> 3;
		float  depthTail  = (depthOri & 0x0007)/8; 

		float depth =depthFront + depthTail;

	*/
		
		//获取深度图整数部分
	

	if(pixelType == berxel::BERXEL_HAWK_PIXEL_TYPE_DEP_16BIT_13I_3D)
	{
		for(uint32_t i = 0; i < width * height ; ++i)
		{
			s_depthOri[i] = s_depthOri[i] >> 3;
		}

	}
	else
	{
		for(uint32_t i = 0; i < width * height ; ++i)
		{
			s_depthOri[i] = s_depthOri[i] >> 4;
		}

	}
		
	//获取深度图整数部分

	

	covertHist(s_depthHist, MAX_DEPTH_HIST, (uint16_t *)s_depthOri, width, height);

	
	for(int i = 0; i < width * height; ++i) 
	{
		pRgb[i].r = s_depthHist[s_depthOri[i]];
		pRgb[i].g = pRgb[i].r;
		pRgb[i].b = 0;
	}




}


void BerxelCommonFunc::convertDepthToRGB(uint16_t* pDepth, RGB888* pRgb,int width , int height, berxel::BerxelHawkPixelType pixelType)
{
	//uint16_t* pde = (uint16_t*)pHawkFrame->getData();
	
	/* 如何获取深度图精度
	1. pixelType == BERXEL_HAWK_PIXEL_TYPE_DEP_16BIT_12I_4D
		 深度图共16位,前面12位为整数部分，单位是1mm，后面4位位小数部分，单位是0.0625mm

			uint16_t depthOri = pDepth[i];

			float  depthFront = depthOri >> 4;
			float  depthTail  = (depthOri & 0x000f)/16; 

			float depth =depthFront + depthTail;


	  2. pxelType ==  BERXEL_HAWK_PIXEL_TYPE_DEP_16BIT_13I_3D 
		  
		  深度图共16位,前面13位为整数部分，单位是1mm，后面3位位小数部分，单位是0.125mm

		  uint16_t depthOri = pDepth[i];

		  float  depthFront = depthOri >> 3;
		  float  depthTail  = (depthOri & 0x0007)/8; 

		  float depth =depthFront + depthTail;

	  */
		
		//获取深度图整数部分
	

	if(pixelType == berxel::BERXEL_HAWK_PIXEL_TYPE_DEP_16BIT_13I_3D)
	{
		for(uint32_t i = 0; i < width * height ; ++i)
		{	
			uint16_t depthOri = pDepth[i] >> 3;

			//将深度图转换为RGB
			pRgb[i].r = depthOri >> 3;
			pRgb[i].g = pRgb[i].r;
			pRgb[i].b = pRgb[i].r;
		} 
	}
	else
	{
		for(uint32_t i = 0; i < width * height ; ++i)
		{	
			uint16_t depthOri = pDepth[i] >> 4;

			//将深度图转换为RGB
			pRgb[i].r = depthOri >> 3;
			pRgb[i].g = pRgb[i].r;
			pRgb[i].b = pRgb[i].r;
		} 
	}

	

}
void BerxelCommonFunc::convertIrToRGB(uint16_t* pIr, RGB888* pRgb,  int width, int height)
{
	for(uint32_t i = 0; i < width * height; ++i)
	{
		pRgb[i].r = pIr[i] >> 2;
		pRgb[i].g = pRgb[i].r;
		pRgb[i].b = pRgb[i].r;
	} 

}



int32_t BerxelCommonFunc::takePhoto(const char* imageName,int index, const uint8_t* pframe, int width, int height)
{

	char bmpImagePath[128] = {0};

	sprintf(bmpImagePath,"%s_%d.bmp" ,imageName,  index);


	BMPHEADER bmfh; // bitmap file header
	BMPINFO bmih; // bitmap info header (windows)

	const int OffBits = 54;

	int32_t imagePixSize = width * height;

	memset(&bmfh, 0, sizeof(BMPHEADER));
	bmfh.bfReserved1 = 0;
	bmfh.bfReserved2 = 0;
	bmfh.bfType      = 0x4d42;
	bmfh.bfOffBits   = OffBits; // 头部信息54字节
	bmfh.bfSize      = imagePixSize * 3 + OffBits;

	memset(&bmih, 0, sizeof(BMPINFO));
	bmih.biSize      = 40; // 结构体大小为40
	bmih.biPlanes    = 1;
	bmih.biSizeImage = imagePixSize * 3;

	bmih.biBitCount    = 24;
	bmih.biCompression = 0;
	bmih.biWidth       = width;
	bmih.biHeight      = height;

	// rgb -> bgr
	RGB888* pRgb = (RGB888*)g_bmpColor;
	RGB888* pSrc = (RGB888*)pframe;
	int tmpindex1(0), tmpindex2(0);

	for(int i = 0; i < height; ++i)
	{
		tmpindex1 = i * width;
		tmpindex2 = (height - i - 1) * width;
		for(int j = 0; j < width; ++j)
		{
			pRgb[tmpindex1 + j].r = pSrc[tmpindex2 + j].b;
			pRgb[tmpindex1 + j].g = pSrc[tmpindex2 + j].g;
			pRgb[tmpindex1 + j].b = pSrc[tmpindex2 + j].r;
		}
	}

	char buf[128]= {0};
	std::string fullPath = bmpImagePath;

	FILE* pSaveBmp = fopen(fullPath.c_str(), "wb");
	if(NULL == pSaveBmp)
	{
		return -1;
	}

	fwrite(&bmfh, 8, 1, pSaveBmp);
	fwrite(&bmfh.bfReserved2, sizeof(bmfh.bfReserved2), 1, pSaveBmp);
	fwrite(&bmfh.bfOffBits, sizeof(bmfh.bfOffBits), 1, pSaveBmp);
	fwrite(&bmih, sizeof(BMPINFO), 1, pSaveBmp );
	fwrite(g_bmpColor, imagePixSize*3, 1, pSaveBmp);

	fclose(pSaveBmp);

	return 0;
}

void BerxelCommonFunc::convertDepthToCv16UC1(uint16_t* pDepth, uint16_t* pCv16UC1,  int width, int height,berxel::BerxelHawkPixelType pixelType)
{
	if(pixelType == berxel::BERXEL_HAWK_PIXEL_TYPE_DEP_16BIT_13I_3D)
	{
		for(uint32_t i = 0; i < width * height; ++i)
		{
			pCv16UC1[i] = pDepth[i] >> 3;
		}
	}
	else
	{
		for(uint32_t i = 0; i < width * height; ++i)
		{
			pCv16UC1[i] = pDepth[i] >> 4;
		}
	}
 
}

void BerxelCommonFunc::saveRawData(uint8_t* pRawData, int dataSize , string stringName, int index)
{

	char strRawDataName[128] =  {0};
	sprintf(strRawDataName,"%s_%d.raw" ,stringName.c_str(),  index);

	FILE* pFile = fopen(strRawDataName, "wb");
	if(pFile)
	{
		fwrite(pRawData, dataSize, 1, pFile);
		fclose(pFile);
		printf("save raw data  Success !\n");
	}
	else
	{
		printf("save raw data  Failed !\n");
	}

}







