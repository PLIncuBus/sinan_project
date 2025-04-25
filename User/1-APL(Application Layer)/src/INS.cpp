#include "INS.h"
#include "matrix.h"


	float rotate[3 * 3];
	float acc[3 * 1];
// float acc_after_rotate[3 * 1];
// float acc_after_minus_Gg[3 * 1];
	float Gg_acc[3 * 1]={0,0,Gg};
//	float overall_axis_acc[3 * 1];

	float out[3];
 
 Vector3 INS_acc;
 Vector3 INS_last_v ;
 Vector3 INS_location;
 Vector3 INS_last_location;
 Vector3 INS_v_sum;

//typedef struct 
//{
//	
//		IMU_data_t * data;
//	
//}INS_t;

IMU_data_t *INS;



//重载运算符
Quaternion operator*(const Quaternion& q1, const Quaternion& q2)
{//四元数相乘，q1右乘q2
  Quaternion q1q2;
  q1q2.w = q2.w*q1.w - q2.x*q1.x - q2.y*q1.y - q2.z*q1.z;
  q1q2.x = q2.x*q1.w + q2.w*q1.x + q2.z*q1.y - q2.y*q1.z;
  q1q2.y = q2.y*q1.w - q2.z*q1.x + q2.w*q1.y + q2.x*q1.z;
  q1q2.z = q2.z*q1.w + q2.y*q1.x - q2.x*q1.y + q2.w*q1.z;
  return q1q2;
}
Vector3 operator*(const Vector3 & T_in,float T )
{
	Vector3 T_OUT;
	T_OUT.x = T*T_in.x;
	T_OUT.y = T*T_in.y;
	T_OUT.z = T*T_in.z;
	return T_OUT;
}
Vector3 operator-(const Vector3& q1, const Vector3& q2) {
	Vector3 q(q1.x - q2.x, q1.y - q2.y, q1.z - q2.z);
	return q;
}
Vector3 operator+(const Vector3& q1, const Vector3& q2) {
	Vector3 q(q1.x + q2.x, q1.y + q2.y, q1.z + q2.z);
	return q;
}
    Quaternion q_initial ( 1, 0, 0, 0 );//初始时刻的姿态四元数
    Quaternion g_world ( 0, 0, 0, Gg );//世界坐标系下的重力加速度_四元数形式
/*IMU解算模块*/////////////////////////
//class ImuCalculate_Qua
//{
//public:


   //获取四元数的共轭
   Quaternion getConjugate(const Quaternion& q) {
      Quaternion qConj(q.w, -q.x, -q.y, -q.z);
      return qConj;
   }
      
   /*计算从q1的坐标系变换到q2的坐标系的四元数*/
   //q1当前，q2目标
   Quaternion RotateQua(Quaternion  q1 , Quaternion  q2)
   {
      Quaternion q_1To2 = q2 *getConjugate(q1);
      return q_1To2;
   } 

   	/*计算本时刻加速度在大地坐标系下的值V2，根据姿态信息旋转*/
	Vector3 RotateAcceV2(Quaternion qua, Quaternion acce)
	{
		//旋转向量
		Quaternion qvq = qua * (acce * getConjugate(qua));
		Vector3 a_world(qvq.x, qvq.y, qvq.z);
		return a_world;
	}

	// /*筛选加速度增量，在世界坐标系下*/
	// void FilterAcceInWorld(Quaternion qua, Vector3 &a_local)
	// {
	// 	//将加速度增量旋转至世界坐标系下
	// 	Quaternion a_local_(0, a_local.x, a_local.y, a_local.z);
	// 	Quaternion q_toIni = RotateQua(qua, q_initial);
	// 	Vector3 a_world = RotateAcceV2(q_toIni, a_local_);
		
	// 	//将加速度增量小于区间范围的值滤除
	// 	if (a_world.x > -correct_ax && a_world.x < correct_ax) a_local.x = 0;
	// 	if (a_world.y > -correct_ay && a_world.y < correct_ay) a_local.y = 0;
	// 	if (a_world.z > -correct_az && a_world.z < correct_az) a_local.z = 0;

	// 	///测试
	// 	//std::cout << a_world.x << "  " << a_world.y << "  " << a_world.z << std::endl;
	// }

//};


//ImuCalculate_Qua ImuCalculate_Qua;










void INS_Init()
{
   INS = Get_IMU_Data(&IMU931);
}



void INS_region_cal()
{

	
	/*update data*/
	rotate[0] = 1 - 2 * INS->q2 *INS->q2 - 2 * INS->q3 *INS->q3;
	rotate[1] = 2 * (INS->q1 * INS->q2 - INS->q0 * INS->q3);
	rotate[2] = 2 * (INS->q1 * INS->q3 + INS->q0 * INS->q2);       // R13


	rotate[3] = 2 * (INS->q1 * INS->q2 + INS->q0 * INS->q3);       // R21
	rotate[4] = 1 - 2 * INS->q1 * INS->q1 - 2 * INS->q3 * INS->q3; // R22
	rotate[5] = 2 * (INS->q2 * INS->q3 - INS->q0 * INS->q1);       // R23

	rotate[6] = 2 * (INS->q1 * INS->q3 - INS->q0 * INS->q2);       // R31
	rotate[7] = 2 * (INS->q2 * INS->q3 + INS->q0 * INS->q1);       // R32
	rotate[8] = 1 - 2 * INS->q1 * INS->q1 - 2 * INS->q2 * INS->q2; // R33
	
	acc[0] = INS->ACC_x;
	acc[1] = INS->ACC_y;
	acc[2] = INS->ACC_z;
	
	/*update Matrix*/
	Matrixf<3,1> Gg_acc_Matrix(Gg_acc);
	Matrixf<3,3> rotate_Matrix(rotate);
	Matrixf<3,1> acc_local_Matrix(acc);
	Matrixf<3,1> acc_after_rotate_Matrix;
	Matrixf<3,1> acc_after_minus_Gg;
	
	
	/*1、overall_axis + minus Gg_acc*/
	acc_after_rotate_Matrix = vector3f::cross(rotate_Matrix,acc_local_Matrix);
	acc_after_minus_Gg = acc_after_rotate_Matrix - Gg_acc_Matrix;
	acc_after_minus_Gg.getData_to_vector(INS_acc);
	
	if(INS_acc.x<0.05&&INS_acc.x>-0.05 )
	{
		INS_acc.x =0;
		INS_last_v.x =0;
	}
	if(INS_acc.y<0.05&&INS_acc.y>-0.05)
	{
		INS_last_v.y =0;
		INS_acc.y =0;
	}

	/*Twice Intergral*/
	INS_v_sum = INS_last_v + INS_acc * INS_T;
	INS_location = INS_last_location + INS_v_sum * INS_T + INS_acc * INS_T * INS_T * 0.5;//v_local * 2 * halfT + a_local * 0.5 * (2*halfT) * (2*halfT);
	INS_last_v = INS_v_sum;
	INS_last_location = INS_location;
	
	out[0] = INS_location.x;
	out[1] = INS_location.y;
	out[2] = INS_location.z;

}









