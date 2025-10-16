#ifndef  __GETDATA_H_
#define  __GETDATA_H_

// 四元数结构体
typedef struct {
    float w;  // 标量部分
    float x;  // 向量部分 i
    float y;  // 向量部分 j
    float z;  // 向量部分 k
} Quaternion;

// 欧拉角结构体
typedef struct {
    float roll;   // 横滚角 (度)
    float pitch;  // 俯仰角 (度)
    float yaw;    // 偏航角 (度)
} EulerAngles;

extern float q[4];
extern float t;
extern float Beta;

void MakeM(void);
void Getdata(void);

#endif
