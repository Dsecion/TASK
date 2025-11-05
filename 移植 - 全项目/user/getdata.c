
#include <ucos_ii.h>
#include <os.h>
#include "stm32f4xx.h"                  // Device header
#include "GY86.h"
#include <math.h>

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


//const float mag_kx = 0.98320f, mag_ky = 1.05344f, mag_kz = 0.96968;
//const float mag_bx = 34.0f, mag_by = -51.0f, mag_bz = -105.0f;
const float mag_kx = 1.0f, mag_ky = 1.0f, mag_kz = 1.0f;
const float mag_bx = 0.0f, mag_by = 0.0f, mag_bz = 0.0f;

void Getdata(void){
	int16_t Mx1, My1, Mz1;	// 磁力计原始数据
 	int16_t  AX1, AY1, AZ1, GX1, GY1, GZ1;	// 加速度计和陀螺仪原始数据
    static float wBias_x = 0.0f, wBias_y = 0.0f, wBias_z = 0.0f;
	static float beta = 0.0f;
	float Mx = 0, My = 0, Mz = 0;	// 归一化后的磁力计数据
	float AX, AY, AZ;	// 归一化后的加速度计数据
	float GX, GY, GZ;	// 陀螺仪数据
	// float ge[3]= {0,0,1};
  	float gb[3];	// 机体坐标系下的重力加速度方向向量
  	float be[3];	// 地球坐标系下的磁场方向向量
  	float bb[3];	// 机体坐标系下的磁场方向向量
	float et[4];	// 误差四元数

	GY86_GetData(&Mx1, &My1, &Mz1, &AX1, &AY1, &AZ1, &GX1, &GY1, &GZ1);

  
	// 归一化磁力计数据，M /= ||M||
	 float use_mag = 1.0f;
	 if(use_mag > 0.0f) {
	     Mx = (Mx1 - mag_bx) * mag_kx;
       My = (My1 - mag_by) * mag_ky;
       Mz = (Mz1 - mag_bz) * mag_kz;
	}
	// normalise mag data
	float m_norm = sqrt(Mx * Mx + My * My + Mz * Mz);
	 if(m_norm > 0.001f) {
	    Mx /= m_norm;
	    My /= m_norm;
	    Mz /= m_norm;
	 }

	// 归一化加速度计数据，A /= ||A||
	AX = AX1 / sqrt(AX1*AX1+AY1*AY1+AZ1*AZ1);
	AY = AY1 / sqrt(AX1*AX1+AY1*AY1+AZ1*AZ1);
	AZ = AZ1 / sqrt(AX1*AX1+AY1*AY1+AZ1*AZ1);

	// 陀螺仪数据处理，W /= LSB
	GX = (GX1/65.5)*(3.1415926/180);		
	GY = (GY1/65.5)*(3.1415926/180);	
	GZ = (GZ1/65.5)*(3.1415926/180);
	
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
	be[1] = 0;
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
