#ifndef __GY86_H_
#define __GY86_H_
#include "stm32f4xx.h"                  // Device header

#define MPU6050_ADDRESS 0xD0
#define MPU6050_SMPLRT_DIV 0X19 //预分频器

#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0X1B
#define MPU6050_ACCEL_CONFIG 0X1C

#define MPU6050_ACCEL_XOUT_H 0X3B
#define MPU6050_ACCEL_XOUT_L 0X3C
#define MPU6050_ACCEL_YOUT_H 0X3D
#define MPU6050_ACCEL_YOUT_L 0X3E
#define MPU6050_ACCEL_ZOUT_H 0X3F
#define MPU6050_ACCEL_ZOUT_L 0X40
#define MPU6050_TEMP_OUT_H 0X41
#define MPU6050_TEMP_OUT_L 0X42
#define MPU6050_GYRO_XOUT_H 0X43
#define MPU6050_GYRO_XOUT_L 0X44
#define MPU6050_GYRO_YOUT_H 0X45
#define MPU6050_GYRO_YOUT_L 0X46
#define MPU6050_GYRO_ZOUT_H 0X47
#define MPU6050_GYRO_ZOUT_L 0X48

#define MPU6050_INT_PIN_CFG 0X37
#define MPU6050_USER_CTRL 0x6A 

#define MPU6050_PWR_MGMT_1 0X6B 
#define MPU6050_PWR_MGMT_2 0X6C

#define MPU6050_WHO_AM_I 0X75

#define HMC5883L_ADDRESS 0x1E
#define HMC5883L_WRITE_ADDRESS 0x3C
#define HMC5883L_READ_ADDRESS 0x3D

#define HMC5883L_CONFIGA 0x00
#define HMC5883L_CONFIGB 0x01
#define HMC5883L_MODE 0x02

#define HMC5883L_STATUS 0x09

#define HMC5883L_X_M 0x03
#define HMC5883L_X_L 0x04
#define HMC5883L_Y_M 0x07
#define HMC5883L_Y_L 0x08
#define HMC5883L_Z_M 0x05
#define HMC5883L_Z_L 0x06
#define HMC5883L_REG_IDENT_A          (0x0A)
#define HMC5883L_REG_IDENT_B          (0x0B)
#define HMC5883L_REG_IDENT_C          (0x0C)

void GY86_WriteRegister(uint8_t SlaveAddress, uint8_t RegisterAddress, uint8_t val);

uint8_t GY86_ReadRegister(uint8_t SlaveAddress, uint8_t RegisterAddress);
void MPU6050_Init(void);
void HMC5883L_Init(void);
void GY86_Init(void);
void GY86_GetData(int16_t *x, int16_t *y, int16_t *z, int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                              int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);


#endif
