
#include <ucos_ii.h>
#include <os.h>
#include "stm32f4xx.h"                  // Device header
#include "GY86.h"
#include <math.h>


void MakeM(void){
	
}
void Getdata(void){
	uint16_t Mx1,My1,Mz1;
  int16_t  AX1, AY1, AZ1, GX1, GY1, GZ1;
  
  float Mx,My,Mz;
  float AX, AY, AZ, GX, GY, GZ;
  float q[4]= {1,0,0,0};
//  float ge[3]= {0,0,1};
  float gb[3];
  float be[3]=  {1,0,0};
  float bb[3];
  float et[4];
	float t = 0.01;
	float Beta = sqrt(3/4)*0.005*sqrt(50);
	while(1){
		GY86_GetData(&Mx1, &My1, &Mz1, &AX1, &AY1, &AZ1, &GX1, &GY1, &GZ1);
		Mx /= sqrt(Mx1*Mx1+My1*My1+Mz1*Mz1);
		My /= sqrt(Mx1*Mx1+My1*My1+Mz1*Mz1);
		Mz /= sqrt(Mx1*Mx1+My1*My1+Mz1*Mz1);
		AX /= sqrt(AX1*AX1+AY1*AY1+AZ1*AZ1);
		AY /= sqrt(AX1*AX1+AY1*AY1+AZ1*AZ1);
		AZ /= sqrt(AX1*AX1+AY1*AY1+AZ1*AZ1);
		GX = GX1*3.1415926/16.4*180;		
		GY = GY1*3.1415926/16.4*180;	
		GZ = GZ1*3.1415926/16.4*180;	
		gb[0]= 2*(q[1]*q[3]-q[0]*q[2]);
    gb[1]= 2*(q[3]*q[2]+q[0]*q[1]);
    gb[2]= q[0]*q[0]+q[3]*q[3]-q[1]*q[1]-q[2]*q[2];
		be[0] = sqrt(Mx*Mx+My*My);
		be[1]=0;
		be[2]=Mz;
		bb[0]= be[0]*(q[0]*q[0]+q[1]*q[1]-q[3]*q[3]-q[2]*q[2])+be[2]*2*(q[1]*q[2]-q[0]*q[2]);
    bb[1]= be[0]*2*(q[1]*q[2]-q[0]*q[3])+be[2]*2*(q[3]*q[2]+q[0]*q[1]);
    bb[2]= be[0]*2*(q[3]*q[1]+q[0]*q[2])+be[2]*(q[0]*q[0]+q[3]*q[3]-q[1]*q[1]-q[2]*q[2]);
		et[0]=(-2)*q[2]*(gb[0]-AX)+2*q[1]*(gb[1]-AY)+(-2)*be[2]*q[2]*(bb[0]-Mx)+((-2)*be[0]*q[3]+2*be[2]*q[1])*(bb[1]-My)+(2*be[0]*q[2])*(bb[2]-Mz);
		et[1]=2*q[3]*(gb[0]-AX)+2*q[0]*(gb[1]-AY)+(-4)*q[1]*(gb[2]-AZ)+(2)*be[2]*q[3]*(bb[0]-Mx)+((2)*be[0]*q[2]+2*be[2]*q[0])*(bb[1]-My)+((2)*be[0]*q[3]-4*be[2]*q[1])*(bb[2]-Mz);
		et[2]=-2*q[0]*(gb[0]-AX)+2*q[3]*(gb[1]-AY)+(-4)*q[2]*(gb[2]-AZ)+((-4)*be[0]*q[2]-2*be[2]*q[0])*(bb[0]-Mx)+((2)*be[0]*q[1]+2*be[2]*q[3])*(bb[1]-My)+((2)*be[0]*q[0]-4*be[2]*q[2])*(bb[2]-Mz);
		et[3]=(2)*q[1]*(gb[0]-AX)+2*q[2]*(gb[1]-AY)+((-4)*be[0]*q[3]+2*be[2]*q[1])*(bb[0]-Mx)+((-2)*be[0]*q[0]+2*be[2]*q[2])*(bb[1]-My)+(2*be[0]*q[1])*(bb[2]-Mz);
		float em = sqrt(et[0]*et[0]+et[1]*et[1]+et[2]*et[2]+et[3]*et[3]);
		
		et[0] /= em;
		et[1] /= em;		
		et[2] /= em;		
		et[3] /= em;
		
		float qwt = -0.5f*(q[1]*GX+q[2]*GY+q[3]*GZ);
		float qxt = 0.5f*(q[0]*GX-q[3]*GY+q[2]*GZ);
		float qyt = 0.5f*(q[3]*GX+q[0]*GY-q[1]*GZ);
		float qzt = 0.5f*(-q[2]*GX+q[1]*GY+q[0]*GZ);
		
		q[0] += (qwt- Beta*et[0])*t;
		q[1] += (qxt - Beta*et[1])*t;
		q[2] += (qyt - Beta*et[2])*t;
		q[3] += (qzt - Beta*et[3])*t;
		
		
		
		
			
		
		
	}
}
