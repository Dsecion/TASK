
#include <ucos_ii.h>
#include <os.h>
#include "stm32f4xx.h"                  // Device header
#include "GY86.h"
#include <math.h>
#include "ATKBLE01.h"
#define beta_max 5.0f
#define beta_min 0.1f
#define zeta 0.05f
#define t  0.01f

volatile float q[4] = {1,0,0,0,};
volatile float roll = 0 ;
volatile float pitch=0;
volatile float yaw=0;

// 角速度 (rad/s)
volatile float gyro_roll = 0;
volatile float gyro_pitch = 0;
volatile float gyro_yaw = 0;

const float mag_kx = 0.9917f, mag_ky = 1.0563f, mag_kz = 0.9665f;
const float mag_bx = 246.0f, mag_by = -31.5f, mag_bz =  -177.5f;

void Getdata(void){

    static float wBias_x = 0.0f, wBias_y = 0.0f, wBias_z = 0.0f;
	static float beta = 0.0f;
	float Mx , My , Mz ;	// 归一化后的磁力计数据
	float AX, AY, AZ;	// 归一化后的加速度计数据
	float GX, GY, GZ;	// 陀螺仪数据
	// float ge[3]= {0,0,1};
  	float gb[3];	// 机体坐标系下的重力加速度方向向量
  	static float be[3];	// 地球坐标系下的磁场方向向量
	be[0] = 1.0f;
	be[2] = 0.0f;
  	float bb[3];	// 机体坐标系下的磁场方向向量
	float et[4]={0,0,0,0};	// 误差四元数
	MPU6050_AccDataTypeDef A1;
	MPU6050_GyroDataTypeDef G1;
	HMC5883L_DataTypeDef M1;
    MPU6050_GetAccData(&A1);
	MPU6050_GetGyroData(&G1);
	HMC5883L_GetData(&M1);
//	BLE_Printf("%d,%d,%d\r\n", M1.Mag_X,M1.Mag_Y,M1.Mag_Z);
	// 归一化磁力计数据，M /= ||M||

	 Mx = (M1.Mag_X - mag_bx) * mag_kx;
   My = (M1.Mag_Y - mag_by) * mag_ky;
   Mz = (M1.Mag_Z - mag_bz) * mag_kz;

	// normalise mag data
	float m_norm = sqrt(Mx * Mx + My * My + Mz * Mz);
	 if(m_norm > 0.001f) {
	    Mx /= m_norm;
	    My /= m_norm;
	    Mz /= m_norm;
	 }

	// 归一化加速度计数据，A /= ||A||
	 float a_norm = sqrt(A1.Acc_X*A1.Acc_X+A1.Acc_Y*A1.Acc_Y+A1.Acc_Z*A1.Acc_Z);
	AX = A1.Acc_X / a_norm;
	AY = A1.Acc_Y / a_norm;
	AZ = A1.Acc_Z / a_norm;

	// 陀螺仪数据处理，W /= LSB
	GX = (G1.Gyro_X/65.5)*(3.14159265359f/180.0f);		
	GY = (G1.Gyro_Y/65.5)*(3.14159265359f/180.0f);	
	GZ = (G1.Gyro_Z/65.5)*(3.14159265359f/180.0f);
	 
	 	// 保存角速度数据供PID使用
	gyro_roll = GX;
	gyro_pitch = GY;
	gyro_yaw = GZ;


	// 机体系下重力加速度方向向量（旋转矩阵*[0,0,1]T）
	gb[0]= 2*(q[1]*q[3]-q[0]*q[2]);
    gb[1]= 2*(q[3]*q[2]+q[0]*q[1]);
    gb[2]= q[0]*q[0]+q[3]*q[3]-q[1]*q[1]-q[2]*q[2];

	// 地球坐标系下磁场方向向量（旋转矩阵*M）
	be[0] = (1-2*(q[2]*q[2]+q[3]*q[3])) * Mx
		  + 2*(q[1]*q[2]-q[0]*q[3])     * My
		  + 2*(q[1]*q[3]+q[0]*q[2])     * Mz;

    be[1] = 2*(q[1]*q[2]+q[0]*q[3])     * Mx
		  + (1-2*(q[1]*q[1]+q[3]*q[3])) * My
		  + 2*(q[2]*q[3]-q[0]*q[1])     * Mz;
	
    be[2] = 2*(q[1]*q[3]-q[0]*q[2])     * Mx
		  + 2*(q[2]*q[3]+q[0]*q[1])     * My
		  + (1-2*(q[2]*q[2]+q[1]*q[1])) * Mz;

	// 处理be[1]方向的分量问题
	be[0] = sqrt(be[0]*be[0]+be[1]*be[1]);
	//be[1] = 0;
	
	be[2] = be[2];

	// 将处理后的be转回机体坐标系下，得到机体坐标系下的磁场方向向量
	bb[0] = be[0] * (1-2*q[3]*q[3]-2*q[2]*q[2])
		  + be[2] * 2*(q[1]*q[3]-q[0]*q[2]);
    bb[1] = be[0] * 2*(q[1]*q[2]-q[0]*q[3])
		  + be[2] * 2*(q[3]*q[2]+q[0]*q[1]);
    bb[2] = be[0] * 2*(q[3]*q[1]+q[0]*q[2])
		  + be[2] * (1-2*q[1]*q[1]-2*q[2]*q[2]);

	// 计算▽E
	float gx_err = gb[0] - AX;
	float gy_err = gb[1] - AY;
	float gz_err = gb[2] - AZ;
	float mx_err = bb[0] - Mx;
	float my_err = bb[1] - My;
	float mz_err = bb[2] - Mz;

	float be0_q2 = be[0] * q[2];
	float be0_q3 = be[0] * q[3];
	float be0_q1 = be[0] * q[1];
	float be0_q0 = be[0] * q[0];
	float be2_q0 = be[2] * q[0];
	float be2_q1 = be[2] * q[1];
	float be2_q2 = be[2] * q[2];
	float be2_q3 = be[2] * q[3];

	et[0] = 
		- 2.0f * q[2] * gx_err
		+ 2.0f * q[1] * gy_err
		- 2.0f * be2_q2 * mx_err
		+ (-2.0f * be0_q3 + 2.0f * be2_q1) * my_err
		+ 2.0f * be0_q2 * mz_err;

	et[1] =
		+ 2.0f * q[3] * gx_err
		+ 2.0f * q[0] * gy_err
		- 4.0f * q[1] * gz_err
		+ 2.0f * be2_q3 * mx_err
		+ (2.0f * be0_q2 + 2.0f * be2_q0) * my_err
		+ (2.0f * be0_q3 - 4.0f * be2_q1) * mz_err;

	et[2] =
		- 2.0f * q[0] * gx_err
		+ 2.0f * q[3] * gy_err
		- 4.0f * q[2] * gz_err
		+ (-4.0f * be0_q2 - 2.0f * be2_q0) * mx_err
		+ ( 2.0f * be0_q1 + 2.0f * be2_q3) * my_err
		+ ( 2.0f * be0_q0 - 4.0f * be2_q2) * mz_err;

	et[3] =
		+ 2.0f * q[1] * gx_err
		+ 2.0f * q[2] * gy_err
		+ (-4.0f * be0_q3 + 2.0f * be2_q1) * mx_err
		+ (-2.0f * be0_q0 + 2.0f * be2_q2) * my_err
		+ 2.0f * be0_q1 * mz_err;
	
	// ▽E /= ||▽E||
	float em = sqrt(et[0]*et[0]+et[1]*et[1]+et[2]*et[2]+et[3]*et[3]);
	
	beta = (em / 0.5f) * beta_max;
    if(beta > beta_max) beta = beta_max;
    if(beta < beta_min) beta = beta_min;
	
	if(em > 0.0f) {
		et[0] /= em;
		et[1] /= em;		
		et[2] /= em;		
		et[3] /= em;
	}
	
	// compute the dynamic bias of gyro
    wBias_x += 2 * (-q[1] * et[0] + q[0] * et[1] + q[3] * et[2] - q[2] * et[3]) * t;
    wBias_y += 2 * (-q[2] * et[0] - q[3] * et[1] + q[0] * et[2] + q[1] * et[3]) * t;
    wBias_z += 2 * (-q[3] * et[0] + q[2] * et[1] - q[1] * et[2] + q[0] * et[3]) * t;

    GX -= zeta * wBias_x;
    GY -= zeta * wBias_y;
    GZ -= zeta * wBias_z;

	// 四元数微分计算，q̇ = 0.5 * q ⊗ ω的实现
	float qwt = -0.5f*( q[1]*GX+q[2]*GY+q[3]*GZ);
	float qxt =  0.5f*( q[0]*GX-q[3]*GY+q[2]*GZ);
	float qyt =  0.5f*( q[3]*GX+q[0]*GY-q[1]*GZ);
	float qzt =  0.5f*(-q[2]*GX+q[1]*GY+q[0]*GZ);
	
	// 更新四元数
	q[0] += (qwt - beta*et[0])*t;
	q[1] += (qxt - beta*et[1])*t;
	q[2] += (qyt - beta*et[2])*t;
	q[3] += (qzt - beta*et[3])*t;

	// 四元数归一化
	float q_norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	if(q_norm > 0.0f) {
		q[0] /= q_norm;
		q[1] /= q_norm;
		q[2] /= q_norm;
		q[3] /= q_norm;
		
	roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
  pitch = asin(2.0f * (q[0] * q[2] - q[3] * q[1]));
  yaw = atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));

		

		
	}
}
