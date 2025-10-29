#include "stm32f4xx.h"                  // Device header
#include "MyIIC.h"
#include "GY86.h"


void GY86_WriteRegister(uint8_t SlaveAddress, uint8_t RegisterAddress, uint8_t val){
	
	
	MyIIC_Start();
	MyIIC_SendByte(SlaveAddress);
	MyIIC_ReceiveACK();
	MyIIC_SendByte(	RegisterAddress);
	MyIIC_ReceiveACK();
	MyIIC_SendByte(val);
	MyIIC_ReceiveACK();
	MyIIC_Stop();
	
	
}

uint8_t GY86_ReadRegister(uint8_t SlaveAddress, uint8_t RegisterAddress){
	uint8_t val;
	
	MyIIC_Start();
  MyIIC_SendByte(SlaveAddress);
	MyIIC_ReceiveACK();
	MyIIC_SendByte(	RegisterAddress);
	MyIIC_ReceiveACK();
	
	MyIIC_Start();
	MyIIC_SendByte(SlaveAddress);
	MyIIC_ReceiveACK();
	val = MyIIC_ReceiveByte();
	MyIIC_SendACK(1);
	MyIIC_Stop();
	
	return val;
}

void MPU6050_Init(void){
	MyIIC_Init();
	GY86_WriteRegister(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x01); //内置陀螺仪时钟
	GY86_WriteRegister(MPU6050_ADDRESS, MPU6050_PWR_MGMT_2, 0x00); 
	
  GY86_WriteRegister(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV,0x09);
	
	GY86_WriteRegister(MPU6050_ADDRESS, MPU6050_CONFIG, 0x06);
  GY86_WriteRegister(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0x18);//选择满量程
	GY86_WriteRegister(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0x18);
	
	//GY86_WriteRegister(MPU6050_ADDRESS, MPU6050_INT_PIN_CFG, 0x80);//旁路使能
	//GY86_WriteRegister(MPU6050_ADDRESS, MPU6050_USER_CTRL, 0x02);//开启主模式
	
}

void HMC5883L_Init(void){
	
	GY86_WriteRegister(MPU6050_ADDRESS, MPU6050_INT_PIN_CFG, 0x02);//旁路使能
	GY86_WriteRegister(MPU6050_ADDRESS, MPU6050_USER_CTRL, 0x00);//开启主模式
	GY86_WriteRegister(HMC5883L_WRITE_ADDRESS, HMC5883L_CONFIGA, 0x50);
	GY86_WriteRegister(HMC5883L_WRITE_ADDRESS, HMC5883L_CONFIGB, 0x20);
										
	GY86_WriteRegister(HMC5883L_WRITE_ADDRESS, HMC5883L_MODE, 0x00);


	

} 



void GY86_GetData(int16_t *x, int16_t *y, int16_t *z, int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                              int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)

{

	*x = (GY86_ReadRegister(HMC5883L_READ_ADDRESS, HMC5883L_X_M)<<8) |	GY86_ReadRegister(HMC5883L_READ_ADDRESS, HMC5883L_X_L);
	
  *y = (GY86_ReadRegister(HMC5883L_READ_ADDRESS, HMC5883L_Y_M)<<8) |	GY86_ReadRegister(HMC5883L_READ_ADDRESS, HMC5883L_Y_L);
	*z = (GY86_ReadRegister(HMC5883L_READ_ADDRESS, HMC5883L_Z_M)<<8) |	GY86_ReadRegister(HMC5883L_READ_ADDRESS, HMC5883L_Z_L);
	
	uint8_t DataH, DataL;
	DataH = GY86_ReadRegister(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H);
	DataL = GY86_ReadRegister(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_L);
	*AccX = (DataH << 8) | DataL;

	DataH = GY86_ReadRegister(MPU6050_ADDRESS,MPU6050_ACCEL_YOUT_H);
	DataL = GY86_ReadRegister(MPU6050_ADDRESS,MPU6050_ACCEL_YOUT_L);
	*AccY = (DataH << 8) | DataL;

	DataH = GY86_ReadRegister(MPU6050_ADDRESS, MPU6050_ACCEL_ZOUT_H);
	DataL = GY86_ReadRegister(MPU6050_ADDRESS, MPU6050_ACCEL_ZOUT_L);
	*AccZ = (DataH << 8) | DataL;
                      
	
	DataH = GY86_ReadRegister(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H);
	DataL = GY86_ReadRegister(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_L);
	*GyroX = (DataH << 8) | DataL;

	DataH = GY86_ReadRegister(MPU6050_ADDRESS, MPU6050_GYRO_YOUT_H);
	DataL = GY86_ReadRegister(MPU6050_ADDRESS, MPU6050_GYRO_YOUT_L);
	*GyroY = (DataH << 8) | DataL;

	DataH = GY86_ReadRegister(MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_H);
	DataL = GY86_ReadRegister(MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_L);
	*GyroZ = (DataH << 8) | DataL;


	
}			
