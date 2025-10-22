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

void MakeM(void);
void Getdata(void);

#endif
