#ifndef __L3GD20_H
#define __L3GD20_H

#include "stm32f4xx.h"
#include "main.h"


typedef struct
{
	int16_t Accel_X_RAW;
	int16_t Accel_Y_RAW;
	int16_t Accel_Z_RAW;
	double Ax;
	double Ay;
	double Az;
}L3GD20_t;

typedef struct
{
	int16_t Gyro_X_RAW;
	int16_t Gyro_Y_RAW;
	int16_t Gyro_Z_RAW;
	double Gx;
	double Gy;
	double Gz;

	double KalmanAngleX;
	double KalmanAngleY;

}LMS303DLHC_t;

typedef struct
{
	double Q_angle;
	double Q_bias;
	double R_measure;
	double angle;
	double bias;
	double P[2][2];
}Kalman_t;

double Kalman(double newAngle, double newRate, double dt);
void L3GD20_Init(void);
void L3GD20_Read(void);
void LMS303DLHC_Init(void);
void LMS303DLHC_Read(void);
#endif


