#ifndef  __GETDATA_H_
#define  __GETDATA_H_

// 四元数结构体
typedef struct {
    float w;  // 标量部分
    float x;  // 向量部分 i
    float y;  // 向量部分 j
    float z;  // 向量部分 k
} Quaternion;

extern float q[4];
extern float t;
extern float Beta;

void MakeM(void);
void Getdata(void);

#endif
