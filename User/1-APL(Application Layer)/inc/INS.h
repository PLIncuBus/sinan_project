#ifndef __INS_H
#define __INS_H

extern "C"
{
	
	
#include "main.h"

	
#define INS_T 0.02
	
typedef struct
{
	double pitch;
	double roll;
	double yaw;

	double ACC_x;
	double ACC_y;
	double ACC_z;
		
	double q0,q1,q2,q3;
}__attribute__((packed)) INS_data_t;	


//自定义四元数
struct Quaternion {
    double w, x, y, z;
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(double aw, double ax, double ay, double az) : w(aw), x(ax), y(ay), z(az) {}
  };


//自定义向量
struct Vector3 {
	double x, y, z;
	Vector3() : x(0), y(0), z(0) {}
	Vector3(double ax, double ay, double az) : x(ax), y(ay), z(az) {}
};



void INS_Init(); //INS_data_t *ins
void INS_region_cal();//INS_data_t *ins





}

#endif