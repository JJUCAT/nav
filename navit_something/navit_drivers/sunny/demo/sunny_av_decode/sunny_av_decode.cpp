//#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <iostream>
#include <string.h>
#include <string>

#ifdef __cplusplus
extern "C" {
#endif

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
//#include <libswscale/swscale.h>

#ifdef __cplusplus
}
#endif
#include "sunny_av_decode.h"

#ifndef NULL
	#define NULL (0)
#endif


typedef struct tagSunnyDecData
{
	SunnyAVDecoderID decoderID;
	AVCodec *		codec;
	AVCodecContext*	contex;
	AVFrame *		picture;
	AVPacket		avpkt;
}SunnyDecData;


static void PrintAVFrame(AVFrame* pFrame)
{
	printf(">>  AVFrame: \n");
	for (int pos = 0; pos < AV_NUM_DATA_POINTERS; pos++)
	{
		printf("  >>  data[%d]=%p.\n", pos, pFrame->data[pos]);
		printf("  >>  linesize[%d]=%d.\n", pos, pFrame->linesize[pos]);
	}
	printf("  >>  width,height=%d,%d.\n", pFrame->width, pFrame->height);
	printf("  >>  format=%d.\n", pFrame->format);
	printf("  >>  key_frame=%d.\n", pFrame->key_frame);
	printf("  >>  pict_type=%d.\n", pFrame->pict_type);
	printf("  >>  pts=%ld.\n", pFrame->pts);
	printf("  >>  pkt_dts=%ld.\n", pFrame->pkt_dts);
	printf("  >>  quality=%d.\n", pFrame->quality);

}

//查找出所有00 00 00 01 这个NALU头位置信息，并且保存到指定文件中
static void LocateNaluHead(unsigned char* h264_buffer, const int h264_len, char* pSaveFile)
{
	if ((NULL == h264_buffer) || (6 > h264_len))
	{
		printf("invalid h264 data.\n");
		return;
	}

	FILE* fp = fopen(pSaveFile, "ab");
	if (NULL == fp)
	{
		printf("open file [%s] failed, errno=%d(%s).\n", pSaveFile, errno, strerror(errno));
		return;
	}

	const unsigned char szHead[4] = { 0x00, 0x00, 0x00, 0x01 };
	char szBufTmp[128] = { 0 };
	for (int offset = 0; offset < (h264_len - 6); offset++)
	{
		if (0 == memcmp(szHead, h264_buffer + offset, sizeof(szHead)))
		{
			sprintf(szBufTmp, "offset:0x%08x, data:%02x%02x%02x%02x %02x %02x ...\n", offset, h264_buffer[offset + 0], h264_buffer[offset + 1], h264_buffer[offset + 2], h264_buffer[offset + 3], h264_buffer[offset + 4], h264_buffer[offset + 5]);
			fwrite(szBufTmp, 1, strlen(szBufTmp), fp);
		}
	}

	fclose(fp);
}

static void ParseH264Data(unsigned char* h264_buffer, const int h264_len)
{
    LocateNaluHead(h264_buffer, h264_len, (char*)("./NaluHead.txt"));
}


static int PackingYUV420P(AVFrame* pFrame, void* pOutBuf)
{
	unsigned char* pYSrc = pFrame->data[0];
	unsigned char* pUSrc = pFrame->data[1];
	unsigned char* pVSrc = pFrame->data[2];

	int nDataLen = 0;
	unsigned char* pDest = (unsigned char*) pOutBuf;
	for (int i = 0; i < pFrame->height; i++)
	{
		memcpy(pDest, pYSrc, pFrame->width);
		pDest += pFrame->width;
		nDataLen += pFrame->width;
		pYSrc += pFrame->linesize[0];
	}
	for (int i = 0; i < pFrame->height / 2; i++)
	{
		memcpy(pDest, pUSrc, pFrame->width / 2);
		pDest += pFrame->width / 2;
		nDataLen += pFrame->width / 2;
		pUSrc += pFrame->linesize[1];
	}
	for (int i = 0; i < pFrame->height / 2; i++)
	{
		memcpy(pDest, pVSrc, pFrame->width / 2);
		pDest += pFrame->width / 2;
		nDataLen += pFrame->width / 2;
		pVSrc += pFrame->linesize[2];
	}

	return nDataLen;
}


static bool g_bInited = false;

HSUNNYAVDEC SunnyAVDecoder_Init(const SunnyAVDecoderID decoderID)
{
	if (!g_bInited)
	{
		//注册所有的编解码器
		void *opaque = NULL;
		av_demuxer_iterate(&opaque);
		//av_register_all();  //被弃用
		g_bInited = true;
	}

	enum AVCodecID id = AV_CODEC_ID_H264;
	switch (decoderID)
	{
	case SunnyAVDecoderID_H264: id = AV_CODEC_ID_H264; break;
	case SunnyAVDecoderID_MJPEG: id = AV_CODEC_ID_MJPEG; break;
	default:
		printf("decoder id:%d is not supported now.\n", decoderID);
		return NULL;
	}

	SunnyDecData *decData = (SunnyDecData*)malloc(sizeof(SunnyDecData));
	if(NULL == decData)
		return NULL;

	memset(decData, 0, sizeof(decData[0]));

	decData->decoderID = decoderID;
	av_init_packet(&decData->avpkt);
	decData->codec = avcodec_find_decoder(id);
	if(!decData->codec)
	{
		printf("can not find decoder.\n");
		free(decData);
		return NULL;
	}

	decData->contex = avcodec_alloc_context3(decData->codec);
	decData->picture = av_frame_alloc();
	decData->contex->coded_height = 0;
	decData->contex->coded_width = 0;
	decData->contex->width = 0;
	decData->contex->height = 0;
	if(decData->codec->capabilities&AV_CODEC_CAP_TRUNCATED)
		decData->contex->flags|= AV_CODEC_FLAG_TRUNCATED;

	if (avcodec_open2(decData->contex, decData->codec, NULL) < 0) 
	{
		if (decData->contex)
		{
			avcodec_free_context(&(decData->contex));
		}
		if (decData->picture)
		{
			av_frame_free(&(decData->picture));
		}
		free(decData);
		return NULL;
	}
	return (HSUNNYAVDEC)decData;
}

bool SunnyAVDecoder_Decode(HSUNNYAVDEC hdec, unsigned char* in_data, int in_len, int* width, int* height, unsigned char* out_buffer, int* out_len)
{
	//https://www.cnblogs.com/leisure_chn/p/10404502.html
	//linesize有可能填充，会比width还大一些.

	//https://blog.csdn.net/zhuweigangzwg/article/details/43734169
	//https://zhuanlan.zhihu.com/p/258734008
	//解码后的数据提取样例.

	SunnyDecData *decData = (SunnyDecData*)hdec;
	if (NULL == hdec)
	{
		return false;
	}

	//if (SunnyAVDecoderID_H264 == decData->decoderID)
	//{
	//	ParseH264Data(in_data, in_len);
	//}

	decData->avpkt.size = in_len;
	decData->avpkt.data = in_data;
	av_frame_unref(decData->picture);

	int ret = avcodec_send_packet(decData->contex, &(decData->avpkt));
	if (0 != ret)
	{
		printf("avcodec_send_packet failed, ret=%d.\n", ret);
		return false;
	}
	const int got_picture = avcodec_receive_frame(decData->contex, decData->picture);
	if (0 != got_picture)
	{
		printf("avcodec_receive_frame failed, ret=%d.\n", got_picture);
		return false;
	}

	//PrintAVFrame(decData->picture);

	*width = decData->picture->width;
	*height = decData->picture->height;
	switch (decData->picture->format)
	{
	case AV_PIX_FMT_YUVJ420P:
	case AV_PIX_FMT_YUV420P: *out_len = PackingYUV420P(decData->picture, (unsigned char*)out_buffer); break;
	default:
		printf("decData->picture->format=%d, to be continued.\n", decData->picture->format);
		return false;
	}

	return true;
}

void SunnyAVDecoder_Uninit(HSUNNYAVDEC hdec)
{
	SunnyDecData *decData = (SunnyDecData*)hdec;
	if (NULL == hdec)
	{
		return;
	}

	if(decData->contex)
	{
		avcodec_close(decData->contex);
		avcodec_free_context(&(decData->contex));
	}
	if(decData->picture)
	{
		av_frame_free(&(decData->picture));
	}
	free(decData);
}
