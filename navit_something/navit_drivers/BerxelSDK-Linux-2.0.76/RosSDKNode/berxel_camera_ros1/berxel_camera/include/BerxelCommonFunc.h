#ifndef __BERXEL_COMMON_FUNC_H__
#define __BERXEL_COMMON_FUNC_H__

#include<stdint.h>
#include <string>
#include "BerxelHawkDefines.h"

#pragma pack (push, 1)

typedef struct
{
	uint8_t r;
	uint8_t g;
	uint8_t b;
}RGB888;


typedef struct _BMPHEADER
{
	uint16_t bfType;
	uint32_t bfSize;
	uint16_t bfReserved1;
	uint16_t bfReserved2;
	uint32_t bfOffBits;
} BMPHEADER;

typedef struct _BMPINFO
{
	uint32_t biSize;
	uint32_t biWidth;
	uint32_t biHeight;
	uint16_t biPlanes;
	uint16_t biBitCount;
	uint32_t biCompression;
	uint32_t biSizeImage;
	uint32_t biXPelsPerMeter;
	uint32_t biYPelsPerMeter;
	uint32_t biClrUsed;
	uint32_t biClrImportant;
} BMPINFO;

#pragma pack (pop)

using namespace std;

class BerxelCommonFunc
{
private:
	BerxelCommonFunc();

protected:
	   ~BerxelCommonFunc();

public: 	
	static BerxelCommonFunc* getInstance();

private:
	void MaxMin(const short* data, int len, short& max_value, short& min_value);
	int MakeColorTable(unsigned char *red, unsigned char *green, unsigned char *blue);

public:
	void ImageScaleColor(uint8_t* color, uint16_t* data, int height, int width, int range_min, int range_max, berxel::BerxelHawkPixelType pixelType);

	void covertHist(float* pHist, int histSize, uint16_t *psrcData, uint32_t srcwidth, uint32_t srcheight);
	void convertDepthToRGB(uint16_t* pDepth, RGB888* pRgb ,int width, int height, berxel::BerxelHawkPixelType pixelType);
	void convertDepthToRgbByHist(uint16_t* pDepth, RGB888* pRgb ,int width, int height, berxel::BerxelHawkPixelType pixelType);
	void convertIrToRGB(uint16_t* pIr, RGB888* pRgb ,int width, int height);
	int32_t takePhoto(const char* imageName,int index, const uint8_t* pframe, int width, int height);
	void saveRawData(uint8_t* pRawData, int dataSize , string stringName, int index);
	void convertDepthToCv16UC1(uint16_t* pDepth, uint16_t* pCv16UC1,  int width, int height, berxel::BerxelHawkPixelType pixelType);
private:
	static BerxelCommonFunc* m_commonFunc;
	int32_t g_bmpColor[1920*1080*3];
};





#endif