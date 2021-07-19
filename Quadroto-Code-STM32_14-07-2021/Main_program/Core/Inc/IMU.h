///*******************************************//
/// get data form IMU LMS303DLHC, L3GD20 /////
/// DATE: 5/3/2021
/// author: DUC
/////
/////
#ifndef __IMU_H
#define __IMU_H

#ifndef PARAKALMAN
#define PARAKALMAN
#define Q_ANGLE 0.0001f;// Process noise variance for the accelerometer
#define Q_BIAS 0.02865f; // Process noise variance for the gyro bias
#define R_MEASURE 0.0004f
#define DT 0.025f
#endif 
#define RAD_TO_DO 57.2958f
#define DO_TO_RAD 0.01745f 

extern int the_first;
//#define TT 0.05f
typedef struct axis
	{
		double ak2;
		double ak1;
		double a;
		double gocreal;
		double gocacc;
		double gocacc1;
		double gocrate; 
		double Q_angle;// Process noise variance for the accelerometer
		double Q_bias; // Process noise variance for the gyro bias
		double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise
		double angle; // The angle calculated by the Kalman filter - part of the 2x1 state matrix
		double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix
		double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate    
		double P[2][2]; // Error covariance matrix - This is a 2x2 matrix
		double K[2]; // Kalman gain - This is a 2x1 matrix   | 1|
		double y; // Angle difference - 1x1 matrix           | 2|
		double S; // Estimate error
		
		double goc_gryo;
		double goc_acel;
		
		}axis;
	
void LMS303DLHC_Init(void);
void L3GD20_Init(void);
void LMS303DLHC_getdata(axis* x, axis* y,axis* z);
void L3GD20_getdata(axis* x, axis* y,axis* z);
void Kalman(axis* qq, float dt);
void init_kalman(axis* xx);
void myFilter( axis* qq, float dt);
		void yaw_cal(axis *x, axis *y, axis *z);
#endif		