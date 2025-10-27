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

extern float q[4];
extern float t;
extern float Beta;
extern float roll;
extern float pitch;
extern float yaw;
void MakeM(void);
void Getdata(void);

// 四元数转欧拉角（弧度），输入 q = {w, x, y, z}
void QuaternionToEuler(const float q_in[4], float roll, float pitch, float yaw);

// 使用全局四元数 q 计算欧拉角（弧度）
void GetEulerAngles(float roll, float pitch, float yaw);

#endif
