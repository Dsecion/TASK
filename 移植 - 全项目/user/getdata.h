#ifndef  __GETDATA_H_
#define  __GETDATA_H_

#include "stm32f4xx.h"                  // Device header

// 四元数结构体
typedef struct {
    int16_t w;  // 标量部分
    int16_t x;  // 向量部分 i
    int16_t y;  // 向量部分 j
    int16_t z;  // 向量部分 k
} Quaternion;

extern volatile float q[4]; 
extern volatile float roll;     // 横滚角 (rad)
extern volatile float pitch;    // 俯仰角 (rad)
extern volatile float yaw;      // 偏航角 (rad)

// 角速度 (rad/s)
extern volatile float gyro_roll;
extern volatile float gyro_pitch;
extern volatile float gyro_yaw;

void Getdata(void);

#endif
