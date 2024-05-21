#ifndef __SUNNY_AV_DECODE_H__
#define __SUNNY_AV_DECODE_H__

typedef void* HSUNNYAVDEC;

typedef enum tagSunnyAVDecoderID
{
	SunnyAVDecoderID_H264,
	SunnyAVDecoderID_MJPEG,
}SunnyAVDecoderID;



HSUNNYAVDEC SunnyAVDecoder_Init(const SunnyAVDecoderID decoderID);
bool SunnyAVDecoder_Decode(HSUNNYAVDEC hdec, unsigned char* in_data, int in_len, int* width, int* height, unsigned char* out_buffer, int* out_len);
void SunnyAVDecoder_Uninit(HSUNNYAVDEC hdec);

#endif

