#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "main.h"

// �Ϻ��������ٶ�9.7963
#define Gg 9.7963
#define WIT_DATA_SUM          11    //WIT�����ܳ�
#define WIT_HEADER            0x55  //WIT֡ͷ
#define WIT_ACC_HEADER        0x51	//WIT���ٶ�֡ͷ
#define WIT_ANGLE_HEADER      0x53	//WIT�Ƕ�֡ͷ
#define WIT_Quaternion_HEADER 0x59	//WIT��Ԫ��֡ͷ

typedef enum
{
	WIT_ACC,
	WIT_ANGLE,
	WIT_Quaternion,
	WIT_DATA_NUM,
}WIT_DATA_e;

typedef struct 
{
	short data[3];
}WIT_Angle_t;

typedef struct 
{
	short data[3];
}WIT_ACC_t;

typedef struct
{
	short data[4];
}WIT_Quaternion_t;


/*���Ӻ���*/
typedef struct
{
	void (*imu_hook)(uint32_t *imu);
}hook_t;

typedef struct
{
	double pitch;
	double roll;
	double yaw;

	double ACC_x;
	double ACC_y;
	double ACC_z;
		
	double q0,q1,q2,q3;
}__attribute__((packed)) IMU_data_t;


typedef struct 
{

	IMU_data_t data;

	WIT_Angle_t Angle;
	WIT_ACC_t ACC;
	WIT_Quaternion_t Quaternion;
	
	WIT_DATA_e hook_signal;

	/*�ص�����*/
	hook_t hook_fun[WIT_DATA_NUM];
	
	unsigned char *temp[WIT_DATA_NUM];
	
	unsigned char rx_buffer[255];
	unsigned char rx_origin_data;
		
}IMU931_t;


extern IMU931_t IMU931;


void IMU_Init(IMU931_t *imu);
void IMU_uart_callback(IMU931_t *imu);
void IMU_hook(IMU931_t *imu);
IMU_data_t * Get_IMU_Data(IMU931_t *imu);

#ifdef __cplusplus
}
#endif

#endif