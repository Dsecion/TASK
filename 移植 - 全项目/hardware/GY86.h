#ifndef __GY86_H__
#define __GY86_H__

#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "Delay.h"



//MPU6050
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_TEMP_OUT_L 0x42
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48
#define MPU6050_USER_CTRL 0x6A
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C
#define MPU6050_WHO_AM_I 0x75

//HMC5883L
#define HMC5883L_CONFIGA 0x00
#define HMC5883L_CONFIGB 0x01
#define HMC5883L_MODE 0x02
#define HMC5883L_XOUT_H 0x03
#define HMC5883L_XOUT_L 0x04
#define HMC5883L_YOUT_H 0x07
#define HMC5883L_YOUT_L 0x08
#define HMC5883L_ZOUT_H 0x05
#define HMC5883L_ZOUT_L 0x06
#define HMC5883L_SR 0x09
#define HMC5883L_IRA 0x0A
#define HMC5883L_IRB 0x0B
#define HMC5883L_IRC 0x0C


typedef struct
{
	int16_t Acc_X;
	int16_t Acc_Y;
	int16_t Acc_Z;
}MPU6050_AccDataTypeDef;

typedef struct
{
	int16_t Gyro_X;
	int16_t Gyro_Y;
	int16_t Gyro_Z;
}MPU6050_GyroDataTypeDef;

typedef struct
{
	int16_t Mag_X;
	int16_t Mag_Y;
	int16_t Mag_Z;
}HMC5883L_DataTypeDef;

typedef struct
{
	int16_t reserve;
	int16_t C[6];
	int16_t crc;
}MS5611_Prom_DataTypeDef;

typedef struct
{
	int32_t TEMP;
	int32_t P;
}MS5611_ResultTypeDef;

void GY86_Init(void);
void SCL_W(uint8_t BitValue);
void SDA_W(uint8_t BitValue);
BitAction SDA_R(void);
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t Byte);
uint8_t MyI2C_ReceiveByte(void);
void MyI2C_SendAck(uint8_t AckBit);
uint8_t MyI2C_ReceiveAck(void);
void MyI2C_Init(void);

void GY86_WriteReg(uint8_t SADDR, uint8_t RegAddr, uint8_t Data);
uint8_t GY86_ReadReg(uint8_t SADDR, uint8_t RegAddr);

void MPU6050_Init(void);
void MPU6050_GetAccData(MPU6050_AccDataTypeDef* DataStruct);
void MPU6050_GetGyroData(MPU6050_GyroDataTypeDef* DataStruct);

void HMC5883L_Init(void);
void HMC5883L_GetData(HMC5883L_DataTypeDef* DataStruct);

void GY86_MS5611_SendCMD(uint8_t CMD);
void MS5611_Reset(void);
void MS5611_ReadProm(MS5611_Prom_DataTypeDef* DataStruct);
uint32_t MS5611_ReadTempPress(uint8_t OSR);
void MS5611_Calculate(MS5611_Prom_DataTypeDef* Prom, uint32_t Dtemp, uint32_t Dp, MS5611_ResultTypeDef* DataStruct);

#endif
