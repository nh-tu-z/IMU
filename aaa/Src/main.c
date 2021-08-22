/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#ifdef __GNUC__
 /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
   PUTCHAR_PROTOTYPE 
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART */
   HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);
   return ch;
}

unsigned short lowBit3_r26 = 0x07, allBit7_r25 = 0x07, bit43_r27 = 0x00, bit43_r28 = 0x00;
// addition
int enable_calib_mag = 0;
int enable_calib_acc = 0;
int enable_calib_gyro = 0;
int enable_para = 0;
int enable_calib = 0;
int t = 1000;
float AxPre = 0, AyPre = 0, AzPre = 0;
char bAx, bAy, bAz;
float mxPre = 0, myPre = 0, mzPre = 0;
char bmx, bmy, bmz;
float gxPre = 0, gyPre = 0, gzPre = 0;
char bgx, bgy, bgz;
//float deviGyro[3];

//
char Rx_data[5];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		// interrupt fnc
		
			/*if(Rx_data[1] == '1')
			{
				switch (Rx_data[2])
				{
					case '0': 
						bit43_r28 = 0x00;
						break;
					case '1':
						bit43_r28 = 0x08;
						break;
					case '2':
						bit43_r28 = 0x10;
						break;
					case '3':
						bit43_r28 = 0x18;
						break;
					default:
						break;
			}
				printf("a");
				//printf("Received value of Accel = %d\n", bit43_r28);
				HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_14);}
			if (Rx_data[1] == '2')
			{
				switch (Rx_data[2])
				{
					case '0': 
						bit43_r27 = 0x00;
						break;
					case '1':
						bit43_r27 = 0x08;
						break;
					case '2':
						bit43_r27 = 0x10;
						break;
					case '3':
						bit43_r27 = 0x18;
						break;
					default:
						break;
			}
			printf("g");
			//printf("Received value of Gyro = %d\n",bit43_r27);
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_15);}*/
		if(Rx_data[0] == 'M') // calib mag
		{
			enable_calib_mag = 1;
			printf("M\n");
			//HAL_Delay(1000);
		}
		if(Rx_data[0] == 'A') //calib Accel
		{
			enable_calib_acc = 1;
			printf("A\n");
		}
		if(Rx_data[0] == 'D') //calib gyro
		{
			enable_calib_gyro = 1;
			printf("D\n");
		}
		if(Rx_data[0] == 'S') //start parameter
		{
			enable_para = 1;
			printf("S\n");
		}
		if(Rx_data[0] == 'T') //start parameter
		{
			enable_para = 0;
			enable_calib = 0;
			printf("T\n");
		}
		if(Rx_data[0] == '1') //start parameter
		{
			t = 100;
			printf("1\n");
		}
		if(Rx_data[0] == '2') //start parameter
		{
			t = 1000;
			printf("1\n");
		}
		if(Rx_data[0] == 'C') //start parameter
		{
			enable_calib = 1;
			printf("C\n");
		}
		HAL_UART_Receive_IT(&huart2,(uint8_t*)Rx_data,1);
	}
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

const float pi=3.14159265358979323846;
const int T_sample= 2;
const float s_i =0.1567;
const float c_i =0.9876;

//MPU6050
#define MPU6050_ADDR (0xD0 )
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define DLPF_CFG     0x1A

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz;
void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, DLPF_CFG, 1, &Data, 1, 1000);
		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}
	}
void MPU6050_Read_Accel (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	Ax = Accel_X_RAW/16384.0;
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;
}
void MPU6050_Read_Gyro (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

  Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;}


	//QMC5883L
#define QMC5883l_ADDRESS (0x0D <<1)
#define QMC5883l_CONTROL_REGISTER_1 0x09
#define QMC5883l_CONTROL_REGISTER_2 0x0A
#define QMC5883l_DATAOUT_X_LSB_REGISTER 0x00
#define QMC5883l_SET_RESET_PERIOD_REGISTER 0x0B
#define QMC5883l__STATUS_REGISTER 0x06
//

//
void QMC5883l_Init (void)
{
	uint8_t Data;
	//
	Data = 0x1D;
	HAL_I2C_Mem_Write(&hi2c1, QMC5883l_ADDRESS, QMC5883l_CONTROL_REGISTER_1, 1, &Data, 1, 1000);
	//
	Data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, QMC5883l_ADDRESS, QMC5883l_CONTROL_REGISTER_2, 1, &Data, 1, 1000);
	//
	Data = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, QMC5883l_ADDRESS, QMC5883l_SET_RESET_PERIOD_REGISTER, 1, &Data, 1, 1000);
	
	
}
//
int16_t X_gauss = 0;
int16_t Y_gauss = 0;
int16_t Z_gauss = 0;
//
float m_x, m_y, m_z;
//
void Read_QMC5883l (void)
{
	uint8_t gData[6];

	
HAL_I2C_Mem_Read(&hi2c1, QMC5883l_ADDRESS, QMC5883l__STATUS_REGISTER, 1, gData, 6, 1000);
	gData[0] = gData[0] & 0x01;
	if (gData[0] == 1)  
		{
		HAL_I2C_Mem_Read(&hi2c1, QMC5883l_ADDRESS, QMC5883l_DATAOUT_X_LSB_REGISTER, 1, gData, 6, 1000);
		X_gauss = (int16_t)( gData[0] | gData[1]<<8 );
		Y_gauss = (int16_t)( gData[2] | gData[3]<<8 );
		Z_gauss = (int16_t)( gData[4] | gData[5]<<8 );
		m_x = -Y_gauss/3000.0;
		m_y = X_gauss/3000.0;
		m_z = Z_gauss/3000.0;
		/*m_x = X_gauss/3000.0;
		m_y = Y_gauss/3000.0;
	  m_z = Z_gauss/3000.0;*/
			}
	}

float mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz;
float ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
float Mang_Ax[10];
float Mang_Ay[10];
float Mang_Az[10];
float Mang_Gx[10];
float Mang_Gy[10];
float Mang_Gz[10];	
float Mang_mx[10];
float Mang_my[10];
float Mang_mz[10];	
//


int i_a = 0;
int i_m = 0;
int i_g = 0;

void output_m()
{
		if(i_m < 100)
		{
			Mang_mx[i_m] = m_x;
			Mang_my[i_m] = m_y;
			Mang_mz[i_m] = m_z;
			printf("g%fx%fy%fz\n",Mang_mx[i_m],Mang_my[i_m],Mang_mz[i_m]);
			i_m++;
		}
		else
		{
			i_m = 0;
			enable_calib_mag = 0;
		}
}	

/*void output_A ()
	{
		if(i < 10)
		{
			Mang_Ax[i] = Ax;
			Mang_Ay[i] = Ay;
			Mang_Az[i] = Az;
			Mang_Gx[i] = Gx;
			Mang_Gy[i] = Gy;
			Mang_Gz[i] = Gz;
			Mang_mx[i] = m_x;
			Mang_my[i] = m_y;
			Mang_mz[i] = m_z;
			*/
			/*
			mean_ax= mean_ax+Mang_Ax[i];
			mean_ay= mean_ay+Mang_Ay[i];
			mean_az= mean_az+Mang_Az[i];
			mean_gx= mean_gx+Mang_Gx[i];
			mean_gy= mean_gy+Mang_Gy[i];
			mean_gz= mean_gz+Mang_Gz[i];*/
	/*		i++;
		}
		else 
		{
			i = 0;
			mean_ax=mean_ax/10;
			mean_ay=mean_ay/10;
			mean_az=mean_az/10;
			mean_gx=mean_gx/10;
			mean_gy=mean_gy/10;
			mean_gz=mean_gz/10;	*/
		/*	 enable_acc = 0;
		}	
	}*/
void outputA()
{
	if(i_a < 100)
	{
		Mang_Ax[i_a] = Ax;
		Mang_Ay[i_a] = Ay;
		Mang_Az[i_a] = Az;
		printf("s%fx%fy%fz\n",Mang_Ax[i_a],Mang_Ay[i_a],Mang_Az[i_a]);
		i_a++;
	}
	else
	{
		i_a = 0;
		enable_calib_acc = 0;
	}
}
void outputG()
{
	if(i_g < 100)
	{
		Mang_Gx[i_g] = Gx;
		Mang_Gy[i_g] = Gy;
		Mang_Gz[i_g] = Gz;
		/*deviGyro[0] += Gx;
		deviGyro[1] += Gy;
		deviGyro[2] += Gz;*/
		printf("d%fx%fy%fz\n",Mang_Gx[i_g],Mang_Gy[i_g],Mang_Gz[i_g]); //8 + 4*8 + 8 + 4*8 + 8 + 4*8 + 8 +8
		i_g++;                                                         // d   %f   x    %f   y    %f   z  /n  
		
	}
	else
	{
		/*deviGyro[0] /= i_g;
		deviGyro[1] /= i_g;
		deviGyro[2] /= i_g;*/
		i_g = 0;
		enable_calib_gyro = 0;
	}
} 
 
 
float x[13][1];
float roll, pitch, yaw;
	
float roll_s, pitch_s,yaw_s;
float q0,q1,q2,q3;
	float x_k_1[13][1]; // x(k+1)
void matrix_quanterion(void)
{
		roll = atan(Ax/(sqrt(Ax*Ax+Az*Az)))*180/pi; //lay 1 lan duy nhat luc calib
		pitch = atan((sqrt(Ax*Ax+Ay*Ay))/Az)*180/pi;
	  yaw = atan(Ax/(sqrt(Ay*Ay+Az*Az)))*180/pi;
	
	//sai doi tu do sang rad (xxxx)
	x[0][0] = cos((roll*pi)/360)*cos((pitch*pi)/360)*cos((yaw*pi)/360)+sin((roll*pi)/360)*sin((pitch*pi)/360)*sin((yaw*pi)/360);
	x[1][0] = sin((roll*pi)/360)*cos((pitch*pi)/360)*cos((yaw*pi)/360)-cos((roll*pi)/360)*sin((pitch*pi)/360)*sin((yaw*pi)/360);
	x[2][0] = cos((roll*pi)/360)*sin((pitch*pi)/360)*cos((yaw*pi)/360)+sin((roll*pi)/360)*cos((pitch*pi)/360)*sin((yaw*pi)/360);
	x[3][0] = cos((roll*pi)/360)*cos((pitch*pi)/360)*sin((yaw*pi)/360)-sin((roll*pi)/360)*sin((pitch*pi)/360)*cos((yaw*pi)/360);
	x[4][0] = Ax;
	x[5][0] = Ay;
	x[6][0] = Az;
	x[7][0] = m_x;
	x[8][0] = m_y;
	x[9][0] = m_z;
	x[10][0] = Gx*pi/180;
	x[11][0] = Gy*pi/180;
	x[12][0] = Gz*pi/180;
		
}

	//phuong trinh process model va cac ma tran dao ham

void process_model(void)
{
	x_k_1[0][0] = x[0][0] - (T_sample/2)*(x[1][0]*x[10][0]+x[2][0]*x[11][0]+x[3][0]*x[12][0]);
	x_k_1[1][0] = x[1][0] + (T_sample/2)*(x[0][0]*x[10][0]-x[3][0]*x[11][0]+x[2][0]*x[12][0]);
	x_k_1[2][0] = x[2][0] + (T_sample/2)*(x[3][0]*x[10][0]+x[0][0]*x[11][0]-x[1][0]*x[12][0]);
	x_k_1[3][0] = x[3][0] - (T_sample/2)*(x[2][0]*x[10][0]-x[1][0]*x[11][0]-x[0][0]*x[12][0]);
	x_k_1[4][0] = 0.98*x[4][0];
	x_k_1[5][0] = 0.98*x[5][0];
  x_k_1[6][0] = 0.98*x[6][0];
	x_k_1[7][0] = 0.98*x[7][0];
	x_k_1[8][0] = 0.98*x[8][0];
	x_k_1[9][0] = 0.98*x[9][0];
	x_k_1[10][0] = 0;
	x_k_1[11][0] = 0;
	x_k_1[12][0] = 0;
	
	  q0 =x[0][0]= x_k_1[0][0];	//q30 format to floating point
		q1 =x[1][0]= x_k_1[1][0] ;
		q2 =x[2][0]= x_k_1[2][0] ;
		q3=x[3][0] = x_k_1[3][0];
	  x[4][0] = x_k_1[4][0];
	  x[5][0] = x_k_1[5][0];
	  x[6][0] = x_k_1[6][0];
	  x[7][0] = x_k_1[7][0];
	  x[8][0] = x_k_1[8][0];
	  x[9][0] = x_k_1[9][0];
	  x[10][0] = x_k_1[10][0];
	  x[11][0] = x_k_1[11][0];
	  x[12][0] = x_k_1[12][0];
 //???
	pitch_s = asin(-2*q1*q3 + 2*q0*q2)*180/pi;	// pitch
	roll_s  = atan2(2*q2*q3 + 2*q0*q1, -q1*q1 - q2*q2 + q0*q0+ q3*q3)*180/pi;	// roll
	yaw_s = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)*180/pi;	//yaw
	
	roll = roll_s;
	pitch = pitch_s;
	yaw = yaw_s;

}


float F_k [13][13];
void matrix_F1_process_model (void)
	{
		F_k[0][0]=F_k[1][1]=F_k[2][2]=F_k[3][3]=1; //
		F_k[4][4]=F_k[5][5]=F_k[6][6]=F_k[7][7]=F_k[8][8]=F_k[9][9]=0.95; //
		F_k[0][1]=F_k[3][2]=-T_sample/2*x[10][0];//
		F_k[0][2]=F_k[1][3]=-T_sample/2*x[11][0];//
		F_k[0][3]=F_k[2][1]=-T_sample/2*x[12][0];//
	  F_k[0][4]=F_k[0][5]=F_k[0][6]=F_k[0][7]=F_k[0][8]=F_k[0][9]=0;
		F_k[1][4]=F_k[1][5]=F_k[1][6]=F_k[1][7]=F_k[1][8]=F_k[1][9]=0;
		F_k[2][4]=F_k[2][5]=F_k[2][6]=F_k[2][7]=F_k[2][8]=F_k[2][9]=0;
		F_k[3][4]=F_k[3][5]=F_k[3][6]=F_k[3][7]=F_k[3][8]=F_k[3][9]=0;//
		F_k[4][0]=F_k[4][1]=F_k[4][2]=F_k[4][3]=F_k[4][5]=F_k[4][6]=F_k[4][7]=F_k[4][8]=F_k[4][9]=F_k[4][10]=F_k[4][11]=F_k[4][12]=0;
		F_k[5][0]=F_k[5][1]=F_k[5][2]=F_k[5][3]=F_k[5][4]=F_k[5][6]=F_k[5][7]=F_k[5][8]=F_k[5][9]=F_k[5][10]=F_k[5][11]=F_k[5][12]=0;
		F_k[6][0]=F_k[6][1]=F_k[6][2]=F_k[6][3]=F_k[6][4]=F_k[6][5]=F_k[6][7]=F_k[6][8]=F_k[6][9]=F_k[6][10]=F_k[6][11]=F_k[6][12]=0;
		F_k[7][0]=F_k[7][1]=F_k[7][2]=F_k[7][3]=F_k[7][4]=F_k[7][5]=F_k[7][6]=F_k[7][8]=F_k[7][9]=F_k[7][10]=F_k[7][11]=F_k[7][12]=0;
		F_k[8][0]=F_k[8][1]=F_k[8][2]=F_k[8][3]=F_k[8][4]=F_k[8][5]=F_k[8][6]=F_k[8][7]=F_k[8][9]=F_k[8][10]=F_k[8][11]=F_k[8][12]=0;
		F_k[9][0]=F_k[9][1]=F_k[9][2]=F_k[9][3]=F_k[9][4]=F_k[9][5]=F_k[9][6]=F_k[9][7]=F_k[9][8]=F_k[9][10]=F_k[9][11]=F_k[9][12]=0;
		F_k[10][0]=F_k[10][1]=F_k[10][2]=F_k[10][3]=F_k[10][4]=F_k[10][5]=F_k[10][6]=F_k[10][7]=F_k[10][8]=F_k[10][9]=F_k[10][10]=F_k[10][11]=F_k[10][12]=0;//
		F_k[11][0]=F_k[11][1]=F_k[11][2]=F_k[11][3]=F_k[11][4]=F_k[11][5]=F_k[11][6]=F_k[11][7]=F_k[11][8]=F_k[11][9]=F_k[11][10]=F_k[11][11]=F_k[11][12]=0;
		F_k[12][0]=F_k[12][1]=F_k[12][2]=F_k[12][3]=F_k[12][4]=F_k[12][5]=F_k[12][6]=F_k[12][7]=F_k[12][8]=F_k[12][9]=F_k[12][10]=F_k[12][11]=F_k[12][12]=0;
		F_k[0][10]=F_k[2][12]=-(T_sample/2)*x[1][0];//
		F_k[0][11]=F_k[3][10]=-T_sample/2*x[2][0];//
		F_k[0][12]=F_k[1][11]=-T_sample/2*x[3][0];//
		F_k[1][0]=F_k[2][3]=T_sample/2*x[10][0];//
		F_k[1][10]=F_k[3][12]=F_k[2][11]=T_sample/2*x[0][0];//
		F_k[1][12]=T_sample/2*x[2][0];//
		F_k[2][10]=T_sample/2*x[3][0];//
		F_k[2][0]=F_k[3][1]=T_sample/2*x[11][0];//
		F_k[3][0]=F_k[1][2]=T_sample/2*x[12][0];
		
		}
	float L_k [13][9];
void matrix_Lk (void)
	{
		L_k[0][0]=L_k[0][1]=L_k[0][2]=L_k[0][3]=L_k[0][4]=L_k[0][5]=L_k[0][6]=L_k[0][7]=L_k[0][8]=0;
		L_k[1][0]=L_k[1][1]=L_k[1][2]=L_k[1][3]=L_k[1][4]=L_k[1][5]=L_k[1][6]=L_k[1][7]=L_k[1][8]=0;
	  L_k[2][0]=L_k[2][1]=L_k[2][2]=L_k[2][3]=L_k[2][4]=L_k[2][5]=L_k[2][6]=L_k[2][7]=L_k[2][8]=0;
		L_k[3][0]=L_k[3][1]=L_k[3][2]=L_k[3][3]=L_k[3][4]=L_k[3][5]=L_k[3][6]=L_k[3][7]=L_k[3][8]=0;
		L_k[4][1]=L_k[4][2]=L_k[4][3]=L_k[4][4]=L_k[4][5]=L_k[4][6]=L_k[4][7]=L_k[4][8]=0;
		L_k[5][0]=L_k[5][2]=L_k[5][3]=L_k[5][4]=L_k[5][5]=L_k[5][6]=L_k[5][7]=L_k[5][8]=0;
		L_k[6][0]=L_k[6][1]=L_k[6][3]=L_k[6][4]=L_k[6][5]=L_k[6][6]=L_k[6][7]=L_k[6][8]=0;
		L_k[7][0]=L_k[7][1]=L_k[7][2]=L_k[7][4]=L_k[7][5]=L_k[7][6]=L_k[7][7]=L_k[7][8]=0;
		L_k[8][0]=L_k[8][1]=L_k[8][2]=L_k[8][3]=L_k[8][5]=L_k[8][6]=L_k[8][7]=L_k[8][8]=0;
		L_k[9][0]=L_k[9][1]=L_k[9][2]=L_k[9][3]=L_k[9][4]=L_k[9][6]=L_k[9][7]=L_k[9][8]=0;
		L_k[10][0]=L_k[10][1]=L_k[10][2]=L_k[10][3]=L_k[10][4]=L_k[10][5]=L_k[10][7]=L_k[10][8]=0;
		L_k[10][0]=L_k[10][1]=L_k[10][2]=L_k[10][3]=L_k[10][4]=L_k[10][5]=L_k[10][7]=L_k[10][8]=0;
		L_k[11][0]=L_k[11][1]=L_k[11][2]=L_k[11][3]=L_k[11][4]=L_k[11][5]=L_k[10][6]=L_k[10][8]=0;
		L_k[12][0]=L_k[12][1]=L_k[12][2]=L_k[12][3]=L_k[12][4]=L_k[12][5]=L_k[12][7]=L_k[12][8]=0;
		L_k[4][0]=L_k[5][1]=L_k[6][2]=L_k[7][3]=L_k[8][4]=L_k[9][5]=L_k[10][6]=L_k[11][7]=L_k[12][8]=1;
	}
	//phuong trinh measurement model va ma tran dao ham
	float z_k [9][1];

	void measurement_model (void)
		{
			z_k[0][0]= x[10][0];
			z_k[1][0]= x[11][0];
			z_k[2][0]= x[12][0];
			z_k[3][0]= x[4][0]-2*(x[0][0]*x[2][0]-x[1][0]*x[3][0]);
			z_k[4][0]= x[5][0]+2*(x[0][0]*x[1][0]+x[2][0]*x[3][0]);
		  z_k[5][0]= x[6][0]+(x[0][0]*x[0][0]-x[1][0]*x[1][0]-x[2][0]*x[2][0]+x[3][0]*x[3][0]);
			z_k[6][0]= x[7][0]+ c_i*(x[1][0]*x[1][0]+x[0][0]*x[0][0]-x[2][0]*x[2][0]-x[3][0]*x[3][0])-2*s_i*(x[0][0]*x[2][0]-x[1][0]*x[3][0]);
			z_k[7][0]= x[8][0]+2*c_i*(x[1][0]*x[2][0]-x[0][0]*x[3][0])+2*s_i*(x[0][0]*x[1][0]+x[2][0]*x[3][0]);
			z_k[8][0]= x[9][0]+2*c_i*(x[0][0]*x[2][0]+x[1][0]*x[3][0])+s_i*(-x[1][0]*x[1][0]+x[0][0]*x[0][0]-x[2][0]*x[2][0]+x[3][0]*x[3][0]);
	}
		
	float H_k [9][13];
	void matrix_H_k(void)
	{
		H_k[0][0]=H_k[0][1]=H_k[0][2]=H_k[0][3]=H_k[0][4]=H_k[0][5]=H_k[0][6]=H_k[0][7]=H_k[0][8]=H_k[0][9]=H_k[0][11]=H_k[0][12]=0;
		H_k[1][0]=H_k[1][1]=H_k[1][2]=H_k[1][3]=H_k[1][4]=H_k[1][5]=H_k[1][6]=H_k[1][7]=H_k[1][8]=H_k[1][9]=H_k[1][10]=H_k[0][12]=0;
		H_k[2][0]=H_k[2][1]=H_k[2][2]=H_k[2][3]=H_k[2][4]=H_k[2][5]=H_k[2][6]=H_k[2][7]=H_k[2][8]=H_k[2][9]=H_k[2][10]=H_k[2][11]=0;
		H_k[3][5]=H_k[3][6]=H_k[3][7]=H_k[3][8]=H_k[3][9]=H_k[3][10]=H_k[3][11]=H_k[3][12]=0;
		H_k[4][4]=H_k[4][6]=H_k[4][7]=H_k[4][8]=H_k[4][9]=H_k[4][10]=H_k[4][11]=H_k[4][12]=0;
		H_k[5][4]=H_k[5][5]=H_k[5][7]=H_k[5][8]=H_k[5][9]=H_k[5][10]=H_k[5][11]=H_k[5][12]=0;
		H_k[6][4]=H_k[6][5]=H_k[6][6]=H_k[6][8]=H_k[6][9]=H_k[6][10]=H_k[6][11]=H_k[6][12]=0;
		H_k[7][4]=H_k[7][5]=H_k[7][6]=H_k[7][7]=H_k[7][9]=H_k[7][10]=H_k[7][11]=H_k[7][12]=0;
		H_k[8][4]=H_k[8][5]=H_k[8][6]=H_k[8][7]=H_k[8][8]=H_k[8][10]=H_k[8][11]=H_k[8][12]=0;
		H_k[0][10]=H_k[1][11]=H_k[2][12]=H_k[3][4]=H_k[4][5]=H_k[5][6]=H_k[6][7]=H_k[7][8]=H_k[8][9]=1;
		H_k[3][0]=H_k[5][2]=-2*x[2][0];
		H_k[4][0]=H_k[3][3]=2*x[1][0];
		H_k[5][0]=H_k[4][1]=2*x[0][0];
		H_k[3][1]=H_k[5][3]=H_k[4][2]=2*x[3][0];
	  H_k[3][2]=-2*x[0][0];
		H_k[5][1]=-2*x[1][0];
		H_k[4][3]=2*x[2][0];
		H_k[6][0]=H_k[8][2]=2*c_i*x[0][0] -2*s_i*x[2][0];
		H_k[7][0]=H_k[6][3]=-2*c_i*x[3][0] +2*s_i*x[1][0];
		H_k[8][0]=H_k[7][1]=2*c_i*x[2][0] +2*s_i*x[0][0];
    H_k[6][1]=H_k[7][2]=H_k[8][3]=2*c_i*x[1][0] +2*s_i*x[3][0];
		H_k[6][2]=-2*c_i*x[2][0] -2*s_i*x[0][0];
		H_k[7][3]=-2*c_i*x[0][0] +2*s_i*x[2][0];
		H_k[8][1]= 2*c_i*x[3][0] -2*s_i*x[1][0];
	}
	void XuatMaTran(float x[][1], int m, int n)
{
   for(int i = 0; i < m; i++)
   {
      for(int j = 0; j < n; j++)
         printf("%f\t"  , x[i][j]);
      printf("\n");
   }
}
void XuatMaTran1(float x[][13], int m, int n)
{
   for(int i = 0; i < m; i++)
   {
      for(int j = 0; j < n; j++)
         printf("%f\t", x[i][j]);
         printf("\n");
   }
}
void XuatMaTran2(float x[13][9], int m, int n)
{
   for(int i = 0; i < m; i++)
   {
      for(int j = 0; j < n; j++)
         printf("%f\t"  , x[i][j]);
		     printf("\t");
         printf("\n");
   }
}


void OUT_M(float x[10])
	{
		 for(int i = 0; i < 9; i++)
		{ printf("%f\t"  , x[10]);}
	}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
	
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	MX_DMA_Init(); 
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	MPU6050_Init();
	QMC5883l_Init();
  /* USER CODE END 2 */
		HAL_UART_Receive_IT(&huart2,(uint8_t*)Rx_data, 1);
		
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	/*for(int i = 0; i < 3; i++)
	{
		deviGyro[i] = 0;
	}*/
  while (1)
  {/* USER CODE END WHILE */
		if(enable_calib_mag == 1)
		{
			Read_QMC5883l();
			output_m();
			HAL_Delay(200);
		}
		if(enable_calib_gyro == 1)
		{
			MPU6050_Read_Gyro();
			outputG();
			HAL_Delay(20);
		}
		if(enable_calib_acc == 1)
		{
			MPU6050_Read_Accel();
			outputA();
			HAL_Delay(200);
		}
		if(enable_para == 1)
		{
			//MPU6050_Read_Gyro();
			Read_QMC5883l();
			MPU6050_Read_Accel();
			bAx = (fabs(Ax-AxPre)> 0.1)?'t':'f';
			bAy = (fabs(Ay-AyPre)> 0.1)?'t':'f';
			bAz = (fabs(Az-AzPre)> 0.1)?'t':'f';
			bmx = (fabs(m_x-mxPre)> 0.1)?'t':'f';
			bmy = (fabs(m_y-myPre)> 0.1)?'t':'f';
			bmz = (fabs(m_z-mzPre)> 0.1)?'t':'f';
			if((bAx == 't') | (bAy == 't') | (bAz == 't') | (bmx == 't') | (bmy == 't') | (bmz == 't'))
			{
				printf("a%fx%fy%fz\n",Ax,Ay,Az); 
				printf("m%fx%fy%fz\n",m_x,m_y,m_z); 
				AxPre = Ax;
				AyPre = Ay;
				AzPre = Az;
				mxPre = m_x;
				myPre = m_y;
				mzPre = m_z;
				HAL_Delay(100);
			}}
			if(enable_calib == 1)
		{
			MPU6050_Read_Gyro();
			Read_QMC5883l();
			MPU6050_Read_Accel();
			printf("r%fp%fy%fe\n",Gx,Gy,Gz); 
				printf("a%fx%fy%fz\n",Ax,Ay,Az); 
				printf("m%fx%fy%fz\n",m_x,m_y,m_z); 
			
			
			/*bmx = (fabs(m_x-mxPre)> 0.1)?'t':'f';
			bmx = (fabs(m_y-myPre)> 0.1)?'t':'f';
			bmx = (fabs(m_z-mzPre)> 0.1)?'t':'f';*/
			//printf("r%fp%fy%fe\n",Gx,Gy,Gz); 
			//printf("a%fx%fy%fz\n",Ax,Ay,Az); 
			//printf("m%fx%fy%fz\n",m_x,m_y,m_z); 
			//sprintf();
			//HAL_UART_Transmit_DMA(&huart2,);
			HAL_Delay(1000);
		}

			/*if (enable ==1)
			{
			  output_A();
			}*/
				/*matrix_quanterion();
				process_model();*/ 
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
		//printf("Starting!!!\n");
		
		//printf("Ax = %f\n",Ax);
		//printf("Ay = %f\n",Ay);
		//printf("Az = %f\n",Az);
		
	//			printf("Gx = %f\n",Gx);
	//			printf("Gy = %f\n",Gy);
	//			printf("Gz = %f\n",Gz);
    
		//printf("m_x = %f\n",m_x);
		//printf("m_y = %f\n",m_y);
		//printf("m_z = %f\n",m_z);
//				printf("mean_gx = %f\n",mean_gx);
	//			printf("mean_gy = %f\n",mean_gy);
//				printf("mean_gz =%f\n",mean_gz);
	//	
		//printf("r%fp%fy%fe",roll_s,pitch_s,yaw_s);
		
	//printf("pitch= %f\n",pitch_s);
	//printf("yaw= %f\n",yaw_s);
		//OUT_M(Mang_mx);
		//OUT_M(Mang_my);
		//OUT_M(Mang_mz);
		
		/*XuatMaTran(x, 13, 1);
		XuatMaTran(x_k_1, 13, 1);
		XuatMaTran3(F_k, 13, 13);
		XuatMaTran2(L_k, 13, 9);
		XuatMaTran(z_k,9,1);
		XuatMaTran1(H_k,9,13);*/
			
			
  
		
		
		
		
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */

  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */



/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
