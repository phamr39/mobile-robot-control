/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <math.h>
#include "stdio.h"
#include <stdlib.h>
//#include "ros.h"
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
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//FIFO Buffer
#define ARRAY_SIZE 11
#define BUFFER_SIZE ARRAY_SIZE-1

float gBuffer[ARRAY_SIZE]; //Buffer has a size of 99
uint8_t pBufferWrite = 1;
uint8_t pBufferRead = 0;

void FIFOBufferWrite(float pDataIn)
{
    if(pBufferWrite == pBufferRead)
    {
        // Buffer is full, do nothing or return error message.
    }
    else
    {
        // Write and increase pBufferWrite
        gBuffer[pBufferWrite] = pDataIn;
        if (pBufferWrite < BUFFER_SIZE)
        {
            pBufferWrite++;
        }
        else
        {
            pBufferWrite = 0;
        }
    }
}

float FIFOBufferRead(void)
{
    float dataOut = 0x00;
    uint8_t tempBufferRead = pBufferRead + 1;
    if(tempBufferRead == pBufferWrite)
    {
        // Buffer is empty, do nothing or return error message.
			//dataOut = 100000;
    }
    else
    {
        // Increase pBufferRead and read data from buffer
        if (pBufferRead < BUFFER_SIZE)
        {
            pBufferRead++;
        }
        else
        {
            pBufferRead = 0;
        }
        dataOut = gBuffer[pBufferRead];
    }
    return dataOut;
}
/* Communication define*/
// Start of text bit: "("
unsigned char stx = 40;
unsigned char *add_stx = &stx;
// End of text bit: ")"
unsigned char etx = 41;
unsigned char *add_etx = &etx;
// Start of Header bit: "<"
unsigned char sth = 60;
unsigned char *add_sth = &sth;
// End of Transmission bit: ">"
unsigned char eot = 62;
unsigned char *add_eot = &eot;

/*Convert function*/
/* 2 digits after comma*/
// Another variable
float data;
char sign;
int lenostr;
// This function help to encode data to sent through uart
void SendData(float r_data)
{
	unsigned char str_float[7];
	if (r_data < 0) {sign = 1;}
	if (r_data >= 0) {sign = 0;}
	data = fabs(r_data);
	// Get the length of data
	if (data < 10) 
	{
		lenostr = 5;
		if (sign == 1){str_float[0] = 45;} // - ascii code
		if (sign == 0){str_float[0] = 43;} // + ascii code
		str_float[1] = (int)data%10; // left comma digit
		str_float[2] = 46; // Comma ASCII code
		str_float[3] = ((data - str_float[1])*100)/10; // first digit after comma
		str_float[4] = (int)((data - str_float[1])*100)%10; // second digit after comma
		}
	if (data < 100 && data >=10) 
	{
		lenostr = 6;
		if (sign == 1){str_float[0] = 45;} // - ascii code
		if (sign == 0){str_float[0] = 43;} // + ascii code
		str_float[1] = data/10; // second digit before comma
		str_float[2] = (int)(data)%10; // first digit before comma
		str_float[3] = 46; // Comma ASCII code
		str_float[4] = ((data - str_float[1]*10 - str_float[2])*100)/10; // first digit after comma
		str_float[5] = (int)((data - str_float[1]*10 - str_float[2])*100)%10; // second digit after comma
	}
	if (data < 1000 && data >= 100) 
	{
		lenostr = 7;
		if (sign == 1){str_float[0] = 45;} // - ascii code
		if (sign == 0){str_float[0] = 43;} // + ascii code
		str_float[1] = data/100; // third digit before comma
		str_float[2] = ((data-str_float[1])/100)/10; // second digit before comma
		str_float[3] = ((int)(data-str_float[1])/100)%10; // first digit before comma
		str_float[4] = 46; // Comma ASCII code
		str_float[5] = ((data - str_float[1]*100 - str_float[2]*10 - str_float[3])*100)/10; // first digit after comma
		str_float[6] = (int)((data - str_float[1]*100 - str_float[2]*10 - str_float[3])*100)%10; // second digit after comma
	}

	// Start Communication
	// Send Start of text code
	//HAL_UART_Transmit(&huart2,add_sth,1,1000);
	HAL_UART_Transmit(&huart2,add_stx,1,1000);
	// Send data
	for (int i = 0; i<lenostr;i++)
	{
		if (str_float[i]!=43 && str_float[i]!=45 && str_float[i]!=46)
		{
			unsigned char temppp;
			temppp = str_float[i] + 48;
			unsigned char *add = &temppp;
			HAL_UART_Transmit(&huart2,add,1,1000);
			HAL_Delay(1);
		}
		else
		{
			unsigned char *add = &str_float[i];
			HAL_UART_Transmit(&huart2,add,1,1000);
			HAL_Delay(1);
		}
	}
	// Send End of text code
	HAL_UART_Transmit(&huart2,add_etx,1,1000);
	//HAL_UART_Transmit(&huart2,add_eot,1,1000);
}
#define NUM_DIGITS_AFTER_COMMA 2
// This function will return number of digits (include comma and sign)
void SendAGift(float data)
{	
	unsigned char str_float[10];
	// Get sign of data
	if (data < 0) {str_float[0] = 45;}
	if (data >= 0) {str_float[0] = 43;}
	data = fabs(data);
	// Get number of digits in data
	int left_comma_digits,num_of_digits;
	float tmp_dt;
	int count=1,n;
	tmp_dt = data;
	while(tmp_dt >=1)
	{
		tmp_dt = tmp_dt/10;
		left_comma_digits++;
	}
	num_of_digits = left_comma_digits + 2 + NUM_DIGITS_AFTER_COMMA; // 2 include sign and comma
	n = left_comma_digits;
	while(n > 0)
	{
		str_float[count] = data/(10^(n-1));
		data = data - ((int)(data/(10^(n-1))))*(10^(n-1));
		count++;
		n = n - 1;
	}
	str_float[left_comma_digits+1] = 46; // Comma ASCII code
	str_float[left_comma_digits+2] = (data*100)/10; // first digit after comma
	str_float[left_comma_digits+3] = (int)(data*100)%10; // second digit after comma
	
	HAL_UART_Transmit(&huart2,add_stx,1,1000);
	// Send data
	for (int i = 0; i<num_of_digits;i++)
	{
		if (str_float[i]!=43 && str_float[i]!=45 && str_float[i]!=46)
		{
			unsigned char temppp;
			temppp = str_float[i] + 48;
			unsigned char *add = &temppp;
			HAL_UART_Transmit(&huart2,add,1,1000);
			HAL_Delay(1);
		}
		else
		{
			unsigned char *add = &str_float[i];
			HAL_UART_Transmit(&huart2,add,1,1000);
			HAL_Delay(1);
		}
	}
	// Send End of text code
	HAL_UART_Transmit(&huart2,add_etx,1,1000);
}
// End of sent function
// Receive function
#define NUMBER_OF_DIGITS_RECEIVE 8 //2 digits after comma
float rec_num;
uint8_t WAIT_TIME = 10;
uint8_t ready_flag = 72;
uint8_t *add_ready = &ready_flag;
uint8_t done_flag = 80;
uint8_t *add_done = &done_flag;
uint8_t confirm_flag = 86;
uint8_t *add_confirm = &confirm_flag;
uint8_t end_transmission_flag = 83;
HAL_StatusTypeDef DoneStatus;
void ReceiveAGift()
{
	rec_num = 0;
	// Feedback to Jetson: Ready to receive the next
	HAL_UART_Transmit(&huart2,add_ready,1,100);
	HAL_Delay(10);
	uint8_t tmp_float[NUMBER_OF_DIGITS_RECEIVE];
	for (int k = -1; k<NUMBER_OF_DIGITS_RECEIVE;k++)
	{
		uint8_t tmp_num[1];
		HAL_UART_Receive(&huart2,tmp_num,1,100);
		if(k>=0)
		{
		tmp_float[k] = tmp_num[0];
		}
		//DoneStatus = HAL_UART_Transmit(&huart2,add_done,1,100);
		HAL_UART_Transmit(&huart2,add_done,1,100);
		
	}
	for (int i = 1; i < (NUMBER_OF_DIGITS_RECEIVE-2);i++)
	{
		rec_num += (tmp_float[i]-48)*pow(10,(5-i));
	}
	rec_num = rec_num + (tmp_float[NUMBER_OF_DIGITS_RECEIVE-2]-48)*0.1 + (tmp_float[NUMBER_OF_DIGITS_RECEIVE-1]-48)*0.01;
	if (tmp_float[0] == 45)
	{rec_num = -rec_num;}
	else
	{rec_num = rec_num;}
	//if(fabs(rec_num) < 1000)
	//{
	HAL_UART_RxCpltCallback(&huart2);
	FIFOBufferWrite(rec_num);
	//}
}
// End of receive function
int loop_times = 0;
uint32_t check;
uint8_t flaggg;
void Communication()
{
	//while(etf[0] != end_transmission_flag)
	for (int kj = 0;kj < WAIT_TIME;kj++)
	{
		ReceiveAGift();
		//HAL_UART_Receive(&huart2,etf,1,100);
		//HAL_Delay(100);
	}
	HAL_Delay(100);
	uint8_t eot[1];
	HAL_UART_Receive(&huart2,eot,1,100);


	check = (huart2.Instance->DR);
	//loop_times = (int)sizeof(gBuffer)/sizeof(gBuffer[0])/2;
	loop_times = 5;
	if (check == 48)
	{
		flaggg = 1;
	}
	else
	{
		flaggg = 0;
	}
	HAL_Delay(100);
}
// Start mortor control function
#define CW 0	// Back
#define CCW 1 // Go ahead
int pwm_0,pwm_1;
void DodoGo(uint8_t direct,int pwm_0,int pwm_1)
{
		if (direct == 0)
		{
			TIM1->CCR2 = pwm_0;
			TIM1->CCR1 = 0;
			TIM1->CCR3 = 0;
			TIM1->CCR4 = pwm_1;
		}
		if (direct == 1)
		{
			TIM1->CCR1 = pwm_0;
			TIM1->CCR2 = 0;
			TIM1->CCR4 = 0;
			TIM1->CCR3 = pwm_1;
		}
		if (direct == 2)
		{
			TIM1->CCR2 = 0;
			TIM1->CCR1 = 0;
			TIM1->CCR3 = 0;
			TIM1->CCR4 = 0;
		}
		
}
int sign_0,sign_1;
void MotorGo(int pwm_0, int pwm_1)
{
	if (pwm_0>0)
	{
		TIM1->CCR1 = pwm_0;
		TIM1->CCR2 = 0;
	}
	if (pwm_1>0)
	{
		TIM1->CCR4 = 0;
		TIM1->CCR3 = pwm_1;
	}
	if (pwm_0<0)
	{
		TIM1->CCR2 = -pwm_0;
		TIM1->CCR1 = 0;
	}
	if (pwm_1<0)
	{
		TIM1->CCR3 = 0;
		TIM1->CCR4 = -pwm_1;
	}
	if (pwm_0==0)
	{
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
	}
	if (pwm_1==0)
	{
		TIM1->CCR4 = 0;
		TIM1->CCR3 = 0;
	}
	
	
}
/*End of motor control functions
-
-
*/


/* Get data from MPU6050, filter with kalman and send them to Pi*/
/*
Angular Velocity Limit  |   Sensitivity
----------------------------------------
250�/s                  |    131
500�/s                  |    65.5 
1000�/s                 |    32.8 
2000�/s                 |    16.4

Acceleration Limit  |   Sensitivity
----------------------------------------
2g                  |    16,384
4g                  |    8,192  
8g                  |    4,096 
16g                 |    2,048 

	*/
float vel_x, vel_y, pos_x = 0, pos_y = 0, vel_z, pos_z, omega_z,ang_z;
/* Get position from encoder data*/
#define REDUCTION_RATIO 78
#define AUTO_RELOAD_COUNTER 65535
#define UNIT_OF_MEA 1 // Default is meter, new unit is m*UNIT_OF_MEA
int pre_count_0 = 0, pre_count_1 = 0;
float delta_count_0, delta_count_1;
float D_0,D_1,D_c,delta_phi;
float en_pos_x, en_pos_y,phi=0, pre_en_pos_x, pre_en_pos_y;
volatile int count_0,count_1;
//float test;
void Encoder_data_processing()
{
	const int N = 26; // Encoder resolution
	const float pi = 3.14159;
	const float R = 0.025*UNIT_OF_MEA;
	//const float L = 0.15;
	// Read both encoder data
	//count_0 = __HAL_TIM_GET_COUNTER(&htim4);
	//count_1 = __HAL_TIM_GET_COUNTER(&htim3);
	//count_1 = getEncoder_1();
	//count_0 = getEncoder_0();
	// This Variable below was invert due to the config is CCW - Go ahead, CW - Go back
	delta_count_0 = (count_0 - pre_count_0);
	delta_count_1 = (count_1 - pre_count_1);
	if (pre_count_0 > 10*count_0 && count_0 != 0)
	{
		delta_count_0 = (count_0 - pre_count_0 + AUTO_RELOAD_COUNTER);
	}
	if (pre_count_1 > 10*count_1 && count_1 != 0)
	{
		delta_count_1 = (count_1 - pre_count_1 + AUTO_RELOAD_COUNTER);
	}
	if (pre_count_0*10 < count_0 && pre_count_0 != 0)
	{
		delta_count_0 = (-(count_0 - pre_count_0 + AUTO_RELOAD_COUNTER));
	}
	if (pre_count_1*10 < count_1 && pre_count_1 != 0)
	{
		delta_count_1 = (-(count_1 - pre_count_1 + AUTO_RELOAD_COUNTER));
	}
	if (pre_count_1 == 0 && count_1 > 40000)
	{
		delta_count_1 = (count_1 - AUTO_RELOAD_COUNTER);
	}
	if (pre_count_0 == 0 && count_0 > 40000)
	{
		delta_count_0 = (count_0 - AUTO_RELOAD_COUNTER);
	}
	if (pre_count_0 > 40000 && count_0 == 0)
	{
		delta_count_0 = (- pre_count_0 + AUTO_RELOAD_COUNTER);
	}
	if (pre_count_1 > 40000 && count_1 == 0)
	{
		delta_count_1 = (- pre_count_1 + AUTO_RELOAD_COUNTER);
	}
	if (fabs(delta_count_0)>10000)
	{
		delta_count_0 = 2*(delta_count_0/fabs(delta_count_0));
	}
	if (fabs(delta_count_1)>10000)
	{
		delta_count_1 = 2*(delta_count_1/fabs(delta_count_1));
	}
	//phi += atan2((en_pos_y-pre_en_pos_y),(en_pos_x-pre_en_pos_x));
	D_0 = (2*pi*delta_count_0*R)/(N*REDUCTION_RATIO);
	D_1 = (2*pi*delta_count_1*R)/(N*REDUCTION_RATIO);
	D_c = (D_0+D_1)/2;
	phi+= (D_1-D_0)/2;
	//delta_phi = (D_1-D_0)/2;
	// Update postion 
	pre_count_0 = count_0;
	pre_count_1 = count_1;
	//pre_en_pos_x = en_pos_x;
	//pre_en_pos_y = en_pos_y;
	//if (fabs(D_c*cos(phi)) > 0.002)
	en_pos_x = en_pos_x + D_c*cos(phi);
	//if (fabs(D_c*sin(phi)) > 0.002)
	en_pos_y = en_pos_y + D_c*sin(phi);
	//if (D_c*cos(phi) > 0.002) en_pos_x = en_pos_x + D_c*cos(phi);
	//if (D_c*sin(phi) > 0.002) en_pos_y = en_pos_y + D_c*sin(phi);
	//test = D_c*cos(phi);
	//HAL_Delay(100);
}
/*Check goal function:
	This function is used to check the robot position which
	will be compared with goal point.
	If that is a acceptable position, this function will return GOAL_STATUS
	0: OK 
	1: Not OK*/
uint8_t GOAL_STATUS = 1;
float tmp_xgoal;
void CheckGoal(float pos_x, float pos_y, float x_goal, float y_goal)
{
	//tmp_xgoal = x_goal;
	//if((fabs(roundf(-pos_x*10)/10) != fabs(x_goal)) || (fabs(roundf(-pos_y*10)/10) != fabs(y_goal)))
	//if(((fabs(roundf(-pos_x*100)/100) != x_goal)) || ((fabs(roundf(-pos_y*100)/100)) != y_goal))
	if((fabs(roundf(-pos_x*100)/100) < fabs(x_goal)))
	{
		GOAL_STATUS = 0;
	}
	else
	{
		GOAL_STATUS = 1;
	}
	if(((fabs(roundf(x_goal*100)/100) == 0)) && ((fabs(roundf(y_goal*100)/100)) == 0)) GOAL_STATUS = 1;
}
/* PID Algorithm */
//float Kp=259169.1404, Ki=2805927.6279 , Kd=549.8702;
//float Kp=3.255, Ki=9.665 , Kd=0.216;
#define LEFT_WHEEL 0
#define RIGHT_WHEEL 1
float Kp, Ki, Kd;
float Kpl = 195.5258, Kil = 393.2754, Kdl = 21.87;
float Kpr = 234.1664, Kir = 475.3947, Kdr = 25.7305;
float P,I,D;
float output;
float pre_error;
float PID_Cal(float error, int whell)
{
	if (whell == 0)
	{
	Kp = Kpl; Ki = Kil; Kd = Kdl;
	}
	else
	{
	Kp = Kpr; Ki = Kir; Kd = Kdr;
	}
	P = Kp * error;
	I += Ki * error;
	D = Kd * (error - pre_error);
	output = P + I + D;
	pre_error = error;
	return output;
}
// Rotate function, + : turn left, -: turn right
#define ROT_SPEED 40000
int ROT_DONE; 
int rot_direction;
float pr_gphi,pr_phi;
void rotate(float g_phi)
{
	//float tmp = fabs(phi);
	//const int N = 26; // Encoder resolution
	const float pi = 3.14159;
	//const float R = 0.025;
	//const float L = 0.175;
	//if (g_phi <0) g_phi = fabs(g_phi) + pi;
	if(g_phi < 0) 
	{
		rot_direction = -1;
		//rot_direction = 1;
	}
	else
	{
		rot_direction = 1;
	}
	if(g_phi == 0)
	{
		ROT_DONE = 1;
	}
	else
	{
		ROT_DONE = 0;
	}
	//pr_phi = phi;
	//phi = phi - pr_gphi;
	//phi = 0;
	delta_phi = 0;
	D_1 = 0;
	D_0 = 0;
	while(ROT_DONE != 1)
	{
		if (fabs(delta_phi) < fabs(g_phi))
			{
				MotorGo(rot_direction*ROT_SPEED,-rot_direction*ROT_SPEED*1.174964);
				Encoder_data_processing();
				delta_phi += (D_1-D_0)/2;
				//HAL_Delay(100);
		}
		else 
		{
			MotorGo(0,0);
			ROT_DONE = 1;
			phi = g_phi;
			//pr_gphi = g_phi;
			if(phi >= pi)
			{
				phi = phi - 2*pi;
			}
			if(phi<-pi)
			{	
				phi = phi + 2*pi;
			}
		}
	}
}
#define SCALE_RATIO 0.1
#define TRANS_RATIO_R 1
#define TRANS_RATIO_L 1.01
// Config parameters for go_to_goal() function
float x=0, y=0;
float pre_phi = 0,phi_d;
float vel_left, vel_right; // Velocity of left and right wheels
float omega_l,omega_r;
int vel = 10000; // the velocity be set, mm/s
float error;
uint8_t goal_flag = 0;
float pr_xg=0,pr_yg=0;
void go_to_goal(float x_goal, float y_goal)
{
	//const int N = 26; // Encoder resolution
	//const float pi = 3.14159;
	const float R = 0.025*UNIT_OF_MEA;
	const float L = 0.175*UNIT_OF_MEA;
	//-----------------------------
	//rotate(1.57);
	//-----------------------------
	en_pos_x = 0;
	en_pos_y = 0;
	pos_x = 0;
	pos_y = 0;
	while (GOAL_STATUS == 0)
	{
	x = pos_x;
	y = pos_y;
	//phi = atan2((x_goal-x),(y_goal-y));
	//error = phi - pre_phi;
	phi_d = atan2((y_goal-y),(x_goal-x));
	error = phi_d-phi;
	CheckGoal(pos_x = pos_x,pos_y = pos_y,x_goal,y_goal);
	if (GOAL_STATUS == 0)
	{
		// Get vel_left
		omega_l = PID_Cal(error,LEFT_WHEEL);
		vel_left = SCALE_RATIO*TRANS_RATIO_L*(2 * vel - omega_l * L) / (2 * R);
		if (fabs(vel_left) > 65000) vel_left = 40000*TRANS_RATIO_L*(vel_left/(fabs(vel_left)));
		// Get vel_right 1.174964057476054
		omega_r = PID_Cal(error,RIGHT_WHEEL);
		vel_right = SCALE_RATIO*TRANS_RATIO_R*(2 * vel + omega_r * L) / (2 * R);
		if (fabs(vel_right) > 65000) vel_right = 40000*TRANS_RATIO_R*(vel_right/(fabs(vel_right)));
		MotorGo(fabs(vel_left),fabs(vel_right));
		}
	else 
	{
		MotorGo(0,0);
	}
	Encoder_data_processing();
		//pos_x = IMU_x;
		//pos_y = IMU_y;
	pos_x = en_pos_x;
	pos_y = en_pos_y;
		// Feedback position to Jetson Nano
		// Start transmission 
	HAL_UART_Transmit(&huart2,add_sth,1,1000);
	SendData(pos_x);
	SendData(pos_y);
	HAL_UART_Transmit(&huart2,add_eot,1,1000);
		// End transmission
		//PosCompare(phi);
		//pre_phi = phi;
}
}
/*END OF go_to_goal*/
// Orbit following function
uint8_t STEP_DONE = 0;
//float points_x[5] = {0,0.5,0.5,0,0};
//float points_y[5] = {0,0,0.5,0.5,0};
int count__;
void OrbitFollow()
{
	float x_goal;
	float y_goal;
	//float pre_gphi = 0;
	//float x_goal = 0.5;
	//float y_goal = 0.5;
	//float g_phi;
	//float way = roundf(sqrt(powf(x_goal,2)+powf(y_goal,2))*10)/10;
	//ROT_DONE = 0;
	//GOAL_STATUS = 0;
	if (count__ > loop_times)
	{
		MotorGo(0,0);
		//count__ = 0;
	}
	else
	{
		for(count__ = 0;count__ <= loop_times;count__++)
		{
			/*pos_x = 0;
			pos_y = 0;
			x=0;
			y=0;*/
			x_goal = FIFOBufferRead();
			y_goal = FIFOBufferRead();
			//x_goal = 0.5;
			//y_goal = 0.5;
			if (x_goal <= 1000)
			{
				//g_phi = atan2((y_goal-y),(x_goal-x));
				//g_phi = atan2(y_goal,x_goal);
				//pre_gphi = g_phi;
				if (GOAL_STATUS == 1) ROT_DONE = 0;
				while(ROT_DONE == 0)
				{
					rotate(y_goal);
				}
				if (ROT_DONE == 1) GOAL_STATUS = 0;
				x = 0;
				y = 0;
				phi = 0;
				while(GOAL_STATUS == 0)
				{
					go_to_goal(1*x_goal,0);
				}
			}
			else
			{
				MotorGo(0,0);
			}
			}
		}
	}

// End of Orbit following function
int ttt;
/* This function below is used for collect data to calculate transfer function 
* Usage: motor: 0 or 1, choose the motor
* With motor = 0, encoder pin match with TIM2_CH1 (PA0)
* With motor = 1, encoder pin match with TIM4_CH1 (PB6)*/
/*
#define PWM_STEP 5000
int test_pwm = 20000;
void MeasureData()
{
	HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start_IT(&htim4,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start_IT(&htim4,TIM_CHANNEL_2);
	int i;
	while(test_pwm < 65500)
	{
		//__HAL_TIM_SET_COUNTER(&htim4,0);
		//__HAL_TIM_SET_COUNTER(&htim3,0);
		htim3.Instance->CNT = 0;
		htim4.Instance->CNT = 0;
		for(i = 0; i < 50000; i++)
		{
			MotorGo(test_pwm,test_pwm);
		}
		HAL_Delay(956);
		MotorGo(0,0);
		//count_1 = __HAL_TIM_GET_COUNTER(&htim3);
		//count_0 = __HAL_TIM_GET_COUNTER(&htim4);
		count_1 = htim3.Instance->CNT;
		count_0 = htim4.Instance->CNT;
		delta_count_0 = (count_0 - pre_count_0);
		delta_count_1 = (count_1 - pre_count_1);
		// Update postion 
		pre_count_0 = count_0;
		pre_count_1 = count_1;
		//SendingData to nano
		//HAL_UART_Transmit(&huart2,add_sth,1,1000);
		//SendData(tmp_count);
		//SendData(tmp_pwm);
		//HAL_UART_Transmit(&huart2,add_eot,1,1000);
		//End of transmission
		test_pwm = test_pwm + PWM_STEP;
		HAL_Delay(3000);
		}
}
*/
// Test orbits
void heart_orbit()
{
	float points_x[13] = {0,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1};
	float points_y[13] = {0,-0.785,0,0.262,0.523,1.047,1.047,-1.57,0.785,1.047,0.262,0.523,0};
	int i_;
	for(i_ = 0; i_< 13;i_++)
	{
		FIFOBufferWrite(points_x[i_]);
		FIFOBufferWrite(points_y[i_]);
	}
	size_t l = sizeof(points_x)/sizeof(points_x[0]);
	loop_times = (int)l;
}
void square_orbit()
{
	float points_x[5] = {0,0.3,0.3,0.3,0.3};
	float points_y[5] = {0,0,1.57,1.57,1.57};
	int i_;
	for(i_ = 0; i_< 5;i_++)
	{
		FIFOBufferWrite(points_x[i_]);
		FIFOBufferWrite(points_y[i_]);
	}
	size_t l = sizeof(points_x)/sizeof(points_x[0]);
	loop_times = (int)l;
}
void go_to_points()
{
	float points_x[3] = {0.3,0.3,0.3};
	float points_y[3] = {0.8,-0.8,0};
	int i_;
	for(i_ = 0; i_< 3;i_++)
	{
		FIFOBufferWrite(points_x[i_]);
		FIFOBufferWrite(points_y[i_]);
	}
	size_t l = sizeof(points_x)/sizeof(points_x[0]);
	loop_times = (int)l;
}
void reset_buffer()
{
	pBufferWrite = 1;
	pBufferRead = 0;
	for (int i = 0;i<=10;i++)
	{
		FIFOBufferWrite(0);
		}
	pBufferWrite = 1;
	pBufferRead = 0;
	}
void getMap()
{
	GOAL_STATUS = 0;
	HAL_Delay(30000);
	rotate(3.1);
	HAL_Delay(2000);
	go_to_goal(3,0);
	ROT_DONE = 0;
	HAL_Delay(2000);
	rotate(3.1);
	MotorGo(0,0);
}
void gotochargestation()
{
	HAL_Delay(15000);
	// Go back, get out the station
	MotorGo(-44000,-40000);
	HAL_Delay(3000);
	MotorGo(0,0);
	HAL_Delay(1000);
	// Rotate
	rotate(3.14);
	HAL_Delay(1000);
	square_orbit();
	OrbitFollow();
	HAL_Delay(1000);
	rotate(-1.57);
	HAL_Delay(500);
	MotorGo(44000,40000);
	HAL_Delay(3000);
	MotorGo(0,0);
}
void getoutchargestation()
{
	HAL_Delay(45000);
	// Getback
	MotorGo(-44000,-40000);
	HAL_Delay(4000);
	MotorGo(0,0);
	rotate(1.6);
	if (ROT_DONE == 1)
	{
		GOAL_STATUS = 0;
		go_to_goal(2,0);
	}
	ROT_DONE = 0;
	rotate(3.1);
	if (ROT_DONE == 1)
		{
			GOAL_STATUS = 0;
			go_to_goal(2,0);
		}
	rotate(1.6);
}
/*
float delta_v_0,delta_v_1;
float v_0,v_1;
int __count;
//const float set_v = {50,90,130,110,70};
float set_v;
void vel_test()
{
	HAL_Delay(5000);
	HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start_IT(&htim4,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start_IT(&htim4,TIM_CHANNEL_2);
	for(__count = 0; __count <= 50; __count++)
	{
		if (__count <10) set_v = 50;
		if (__count >=10 && __count < 20) set_v = 90;
		if (__count >=20 && __count < 30) set_v = 130;
		if (__count >=30 && __count < 40) set_v = 110;
		if (__count >=40) set_v = 70;
		__HAL_TIM_SET_COUNTER(&htim4,0);
		__HAL_TIM_SET_COUNTER(&htim3,0);
		delta_v_0 = set_v - count_0;
		delta_v_1 = set_v - count_1;
		v_0 = PID_Cal(delta_v_0, LEFT_WHEEL)/100;
		v_1 = PID_Cal(delta_v_1, RIGHT_WHEEL)/100;
		MotorGo(v_0,v_1);
		HAL_Delay(95);
		//MotorGo(0,0);
		count_1 = __HAL_TIM_GET_COUNTER(&htim3);
		count_0 = __HAL_TIM_GET_COUNTER(&htim4);
	}
}
*/
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
	/*
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start(&htim3);
	*/
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  /*HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start_IT(&htim4,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start_IT(&htim4,TIM_CHANNEL_2);
  htim3.Instance->CNT = 0;
  htim4.Instance->CNT = 0;
	*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//HAL_Delay(15000);
	//go_to_points();
	//GOAL_STATUS = 0;
	//gotochargestation();
	//int hg;
	//reset_buffer();
  while (1)
  {
	  //HAL_Delay(10000);
	  //getoutchargestation();
	  //Communication();
	  //OrbitFollow();
	  //if(pBufferWrite == pBufferRead)
	  //{
		//  reset_buffer();
	  	//  }
	  //vel_test();
	  //MotorGo(15000,14000);
	  //count__ = 0;
	  //reset_buffer();
	  //HAL_Delay(15000);
	  //(0.5,0);
	  //rotate(1.57);
		MotorGo(45000,45000);
		//go_to_goal(1.5,1);
	  //MeasureData();
	  Encoder_data_processing();
    /* USER CODE END WHILE */

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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 7999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB14 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_14|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
