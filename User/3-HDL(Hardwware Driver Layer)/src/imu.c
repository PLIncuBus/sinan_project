#include "imu.h"

IMU931_t IMU931;

static void IMU_ACC_hook(uint32_t *imu);
static void IMU_ANGLE_hook(uint32_t *imu);
static void IMU_Quaternion_hook(uint32_t *imu);


void * IMU_hook_fun[WIT_DATA_NUM] = {IMU_ACC_hook,IMU_ANGLE_hook,IMU_Quaternion_hook};

void IMU_Init(IMU931_t *imu)
{
	unsigned char i;
	for(i = 0 ; i < WIT_DATA_NUM ; i++)
	{
		imu->hook_fun[i].imu_hook = (void(*)(uint32_t *imu))IMU_hook_fun[i];
	}
	HAL_UART_Receive_IT(&huart1,&imu->rx_origin_data,1);
}

void IMU_uart_callback(IMU931_t *imu)
{	
	static uint8_t cnt;
	imu->rx_buffer[cnt++]=imu->rx_origin_data;
	//数据头不对，则重新开始寻找0x55数据头
	if (imu->rx_buffer[0]!=WIT_HEADER) 
	{
		cnt=0;
		return;
	}
	//数据不满11个，则返回
	if (cnt<WIT_DATA_SUM) {return;}
	else
	{
		if(imu->rx_buffer[1] == WIT_ACC_HEADER)	
		{
			imu->hook_signal = WIT_ACC ;
			memcpy(imu->temp[WIT_ACC],imu->rx_buffer,11);
		}
		else if(imu->rx_buffer[1] == WIT_ANGLE_HEADER) 
		{
			imu->hook_signal = WIT_ANGLE ;
			memcpy(imu->temp[WIT_ANGLE],imu->rx_buffer,11);
		}
		else if(imu->rx_buffer[1] == WIT_Quaternion_HEADER)
		{
			imu->hook_signal = WIT_Quaternion ;
			memcpy(imu->temp[WIT_Quaternion],imu->rx_buffer,11);
		}
		//清空缓存区
		cnt=0;
	}  
}

void IMU_hook(IMU931_t *imu)
{
		if(imu->hook_signal == WIT_ACC)
		{
			imu->hook_fun[WIT_ACC].imu_hook((uint32_t*)imu);
			imu->hook_signal = WIT_DATA_NUM;
		}
		else if(imu->hook_signal == WIT_ANGLE)
		{
			imu->hook_fun[WIT_ANGLE].imu_hook((uint32_t*)imu);
			imu->hook_signal = WIT_DATA_NUM;
		}
		else if(imu->hook_signal == WIT_Quaternion)
		{
			imu->hook_fun[WIT_Quaternion].imu_hook((uint32_t*)imu);
			imu->hook_signal = WIT_DATA_NUM;
		}
}

static void IMU_ACC_hook(uint32_t *imu)
{
	IMU931_t *local_imu = (IMU931_t  *)imu;
	memcpy(&local_imu->ACC.data,&local_imu->temp[WIT_ACC][2],8);
	local_imu->data.ACC_x  = (double)local_imu->ACC.data[0]/32768*16*Gg;
	local_imu->data.ACC_y  = (double)local_imu->ACC.data[1]/32768*16*Gg;
	local_imu->data.ACC_z  = (double)local_imu->ACC.data[2]/32768*16*Gg;
}

static void IMU_ANGLE_hook(uint32_t *imu)
{
	IMU931_t *local_imu = (IMU931_t  *)imu;
	memcpy(&local_imu->Angle.data,&local_imu->temp[WIT_ANGLE][2],8);
	local_imu->data.roll  = (double)local_imu->Angle.data[0]/32768*180;
	local_imu->data.pitch = (double)local_imu->Angle.data[1]/32768*180;
	local_imu->data.yaw   = (double)local_imu->Angle.data[2]/32768*180;	
}

static void IMU_Quaternion_hook(uint32_t *imu)
{
	IMU931_t *local_imu = (IMU931_t  *)imu;
	memcpy(&local_imu->Quaternion.data,&local_imu->temp[WIT_Quaternion][2],12);
	local_imu->data.q0  = (double)local_imu->Quaternion.data[0]/32768;
	local_imu->data.q1  = (double)local_imu->Quaternion.data[1]/32768;
	local_imu->data.q2  = (double)local_imu->Quaternion.data[2]/32768;
	local_imu->data.q3  = (double)local_imu->Quaternion.data[3]/32768;	
}


IMU_data_t * Get_IMU_Data(IMU931_t *imu)
{
	return &imu->data;
}
