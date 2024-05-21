#ifndef KY102N_IMU_DRIVER_KY102N_PROTOCAL_H
#define KY102N_IMU_DRIVER_KY102N_PROTOCAL_H

#define IMU102N_HEAD1		0X5A
#define IMU102N_HEAD2		0X5A
#define IMU102NDATA_LEN		59
#define IMU102N_CRCINDEX	58

typedef struct imu102n
{
    union {
        float 	fGx;
        int32_t int_gx;
    }Gx;
    union {
        float 	fGy;
        int32_t int_gy;
    }Gy;
    union {
        float 	fGz;
        int32_t int_gz;
    }Gz;
    union {
        float 	fAx;
        int32_t int_ax;
    }Ax;
    union {
        float 	fAy;
        int32_t int_ay;
    }Ay;
    union {
        float 	fAz;
        int32_t int_az;
    }Az;
    union {
        float 	temperature;
        int32_t int_temperature;
    }temp;
    int tem;
} Imu102nStruct;

/*
typedef struct imu_txbuf1
{
    uint8_t imu_head[2];
    uint8_t id[2];
    uint8_t len;
    uint8_t data[32];
    uint8_t chcksum[2];
}imu_txbuf;*/
#endif //KY102N_IMU_DRIVER_KY102N_PROTOCAL_H
