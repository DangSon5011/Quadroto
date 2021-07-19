///*******************************************//
/// get data form IMU LMS303DLHC, L3GD20 /////
/// DATE: 5/3/2021
/// author: DUC
/////
/////

#include "IMU.h"
#include "spi.h"
#include "i2c.h"
#include <math.h>
#include "main.h"

double TT =0.1;
int the_first = 0;
void LMS303DLHC_Init(void)	
	{ 
		
		uint8_t buffer_tx[2];
		for(int i=0;i<200;i++)
	{
		if(HAL_I2C_IsDeviceReady(&hi2c1,0x32,2,10)== HAL_OK)
			{
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
				break;
			}
	}
	//cau hinh che do lam viec
	buffer_tx[0]=0x20;
	buffer_tx[1]=0x57;//toc do  lay mau 100hz
  HAL_I2C_Master_Transmit(&hi2c1,0x32,buffer_tx,2,100);
	HAL_Delay(10);
	buffer_tx[0]=0x23;
	buffer_tx[1]=0x80;// enable BDU// data LSB @ lower address//Full-scale  2g
  HAL_I2C_Master_Transmit(&hi2c1,0x32,buffer_tx,2,100);
	HAL_Delay(10);
	buffer_tx[0]=0x00;
	buffer_tx[1]=0x10;
	HAL_Delay(10);
	HAL_I2C_Master_Transmit(&hi2c1,0x3C,buffer_tx,2,100);
	buffer_tx[0]=0x01;
	buffer_tx[1]=0x20;
	HAL_I2C_Master_Transmit(&hi2c1,0x3C,buffer_tx,2,100);
	buffer_tx[0]=0x02;
	buffer_tx[1]=0x00;
	HAL_I2C_Master_Transmit(&hi2c1,0x3C,buffer_tx,2,100);
	
	}
void L3GD20_Init(void)
	{  
		uint8_t buffer_tx[2];//dem truyen
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
		buffer_tx[0]=0x20;///
		buffer_tx[1]=0x0f;// che do binh thuong, ODA=95hz, loc thong thap 25hz
		HAL_SPI_Transmit(&hspi1,buffer_tx,2,100);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
		HAL_Delay(10);
		///
		buffer_tx[0]=0x23;
		buffer_tx[1]=0x90;// enable BDU// full scale 500dsp//   Data LSb @ lower address
		HAL_SPI_Transmit(&hspi1,buffer_tx,2,100);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
		HAL_Delay(10);
	//loc
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
		buffer_tx[0]=0x24;
		buffer_tx[1]=0x12;// , loc thong cao, thong thap lan 2
		HAL_SPI_Transmit(&hspi1,buffer_tx,2,100);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
		HAL_Delay(10);
	
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
		buffer_tx[0]=0x21;
		buffer_tx[1]=0x14;// Reference signal for filtering//, loc thong cao 0.45 Hz
		HAL_SPI_Transmit(&hspi1,buffer_tx,2,100);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	}

	
	void LMS303DLHC_getdata(axis* x, axis* y,axis* z)
	{  
		 
		uint8_t buffer_tx[2];
		uint8_t buffer_rx[6];
		int16_t giatocx,giatocy,giatocz;
		
		buffer_tx[0]=0xA8;
		float ax,ay,az;
		
		HAL_I2C_Master_Transmit(&hi2c1,0x32,buffer_tx,1,100);
		HAL_I2C_Master_Receive(&hi2c1,0x32,buffer_rx,6,100);
		giatocx=(int16_t)((uint16_t)buffer_rx[1]<<8)|(uint16_t)buffer_rx[0];
		giatocy=(int16_t)((uint16_t)buffer_rx[3]<<8)|(uint16_t)buffer_rx[2];
		giatocz=(int16_t)((uint16_t)buffer_rx[5]<<8)|(uint16_t)buffer_rx[4];
		
		ax=(float)giatocx/16384;
		ay =((float)giatocy)/16384;
		az=((float)giatocz)/16384;

		x->goc_acel = (float)atan(ay/sqrt((ax*ax)+(az*az))) * RAD_TO_DO;
		y->goc_acel = (float)(atan(-ax/sqrt(ay*ay+az*az))) * RAD_TO_DO;
		
		
	}
	
void L3GD20_getdata(axis* x, axis* y,axis* z)
	{ 
		// gocrate[3] la bien toan cuc can khai bao truoc khi su dung
		uint8_t buffer1_tx[2];//dem truyen
		uint8_t buffer1_rx[6];//dem nhan
		int16_t gocxx,gocyy,goczz;//rate goc
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
		buffer1_tx[0]=0x28|0xc0;// 0xC0 yeu cau doc nhieu byte  
		HAL_SPI_Transmit(&hspi1,buffer1_tx,1,40);// gui dia chi thanh gi chua rate goc
		HAL_SPI_Receive(&hspi1,buffer1_rx,6,40);// doc rate goc
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
		gocxx=(int16_t)(((uint16_t)buffer1_rx[1]<<8)|(uint16_t)buffer1_rx[0]);
		x->gocrate= -((double)gocxx/32767)*500;//toc do goc
		gocyy=(int16_t)(((uint16_t)buffer1_rx[3]<<8)|(uint16_t)buffer1_rx[2]);
		y->gocrate= -((double)gocyy/32767)*500;// toc do goc truc y
		goczz=(int16_t)(((uint16_t)buffer1_rx[5]<<8)|(uint16_t)buffer1_rx[4]);
		z->gocrate= -((double)goczz/32767)*500;//toc do goc truc z
		
		x->goc_gryo = 0.025*x->gocrate;
		y->goc_gryo = 0.025*y->gocrate;
		z->goc_gryo = 0.025*z->gocrate;	
}

void Kalman(axis* qq, float dt)
	{
		
//				qq->rate = qq->gocrate - qq->bias;
       // qq->goc_gryo = dt * qq->rate;
				qq->gocreal = 0.98*(qq->gocreal + qq->goc_gryo) + 0.02*qq->goc_acel;
		
//		    qq->rate = qq->gocrate - qq->bias;
//        qq->angle += dt * qq->rate;
//        // Update estimation error covariance - Project the error covariance ahead
//        /* Step 2 */
//        qq->P[0][0] += dt * (dt*qq->P[1][1] - qq->P[0][1] - qq->P[1][0] + qq->Q_angle);
//        qq->P[0][1] -= dt * qq->P[1][1];
//        qq->P[1][0] -= dt * qq->P[1][1];
//        qq->P[1][1] += qq->Q_bias * dt;
//        
//        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
//        // Calculate Kalman gain - Compute the Kalman gain
//        /* Step 4 */
//        qq->S = qq->P[0][0] + qq->R_measure;   //(Pk-) +R
//        /* Step 5 */
//        qq->K[0] = qq->P[0][0] / qq->S;        // K[0] = Kk = (Pk-)/ [(Pk-) +R]
//        qq->K[1] = qq->P[1][0] / qq->S;
//        
//        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
//        /* Step 3 */
//        qq->y = qq->gocacc*RAD_TO_DO - qq->angle;      // Zk - (Xk-)  = y
//        /* Step 6 */
//        qq->angle += qq->K[0] * qq->y;         // Xk = (Xk-) + Kk * [Zk - (Xk-)]
//        qq->bias += qq->K[1] * qq->y;
//        
//        // Calculate estimation error covariance - Update the error covariance
//        /* Step 7 */
//				    double P00_temp = qq->P[0][0];
//						double P01_temp = qq->P[0][1];
//					qq->P[0][0] -= qq->K[0] * P00_temp;
//					qq->P[0][1] -= qq->K[0] * P01_temp;
//					qq->P[1][0] -= qq->K[1] * P00_temp;
//					qq->P[1][1] -= qq->K[1] * P01_temp;
//					qq->gocreal=qq->angle*DO_TO_RAD;				 
	}
	
void init_kalman(axis* xx)
{
	
	xx->ak1=xx->gocacc;
	xx->ak2=xx->ak1;
	xx->Q_angle=Q_ANGLE;
	xx->Q_bias= Q_BIAS;
	xx->R_measure =R_MEASURE;
	xx->angle= xx->gocacc*RAD_TO_DO;
}		
		
		void yaw_cal(axis *x, axis *y, axis *z)
		{
			uint8_t buffer_tx[2];
			uint8_t buffer_rx[6];
			int16_t tu_x,tu_y,tu_z;
			float xM,yM,zM ;
			float xh, yh;
			buffer_tx[0]=0x03;
			HAL_I2C_Master_Transmit(&hi2c1,0x3C,buffer_tx,1,100);
			HAL_I2C_Master_Receive(&hi2c1,0x3C,buffer_rx,6,100);
			tu_x=(int16_t)(((uint16_t)buffer_rx[0]<<8)|(uint16_t)buffer_rx[1]);
			tu_z=(int16_t)(((uint16_t)buffer_rx[2]<<8)|(uint16_t)buffer_rx[3]);
			tu_y=(int16_t)(((uint16_t)buffer_rx[4]<<8)|(uint16_t)buffer_rx[5]);
			xM= (float)tu_x;
			yM= (float)tu_y;
			zM= (float)tu_z;
			
			xh= xM*cos(y->gocreal*DO_TO_RAD)+zM*sin(y->gocreal*DO_TO_RAD);
			yh=xM*sin(x->gocreal*DO_TO_RAD)*sin(y->gocreal*DO_TO_RAD)+yM*cos(x->gocreal*DO_TO_RAD)-zM*sin(x->gocreal*DO_TO_RAD)*cos(y->gocreal*DO_TO_RAD
			 );
		
		if((xh>0)&&(yh>=0))
			{	
				z->gocreal=atan(yh/xh) * RAD_TO_DO;
			}
	  else if (xh<0)
				{
					z->gocreal=(3.1416+atan(yh/xh)) *RAD_TO_DO;
					}
		else if((xh>0)&&(yh<=0))
			{
				z->gocreal= (3.1416*2+atan(yh/xh)) * RAD_TO_DO;
				}
		else if((xh==0)&&(yh<0))
			{
				z->gocreal= 3.1416/2 * RAD_TO_DO;
				}
		else if((xh==0)&&(yh>0))
			{
				z->gocreal= 1.5*3.1416* RAD_TO_DO;
			}	
		}
			
			
