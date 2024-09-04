/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "math.h"
//#include "inv_mpu.h"
//#include "inv_mpu_dmp_motion_driver.h"
//#include "vofa.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum dir{Pos,Neg};
typedef struct{
    float Roll;//??????
    float Pitch;
    float Yaw;
 
    float Roll_a;//???????????
    float Pitch_a;
    float Roll_g;//???????????
    float Pitch_g;
 
    float lastRoll;//???????
    float lastPitch;
 
    int offset_gx;//??????
    int offset_gy;
    int offset_gz;
}IMU;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR1_A_H HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
#define MOTOR1_A_L HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	
#define MOTOR1_B_H HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
#define MOTOR1_B_L HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	
#define MOTOR1_C_H HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
#define MOTOR1_C_L HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	
#define MOTOR1_D_H HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
#define MOTOR1_D_L HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);


#define MOTOR2_A_H HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
#define MOTOR2_A_L HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	
#define MOTOR2_B_H HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
#define MOTOR2_B_L HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	
#define MOTOR2_C_H HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
#define MOTOR2_C_L HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	
#define MOTOR2_D_H HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
#define MOTOR2_D_L HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

#define OFFSET_COUNT 200
#define Ka 0.80  
#define dt 0.005 
#define PI 3.1415926
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int speed;
float gyro_x,gyro_y,gyro_z;
float data[3];
float accel_x,accel_y,accel_z;
IMU IMU_Data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void stepper1(uint8_t dir,int speed);
void stepper2(uint8_t dir,int speed);
void stepper_stop(int speed);
void IMU_offset(int OFFSET);
float FOCF(float acc_m,float gyro_m,float* last_angle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	MPU6050_Init(Sensor_I2C2_Serch());//
	
	IMU_offset(200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		MPU6050_Read_Gyro();	
		gyro_x=Mpu6050_Data.Gyro_X-IMU_Data.offset_gx;
		gyro_y=Mpu6050_Data.Gyro_Y-IMU_Data.offset_gy;
		gyro_z=Mpu6050_Data.Gyro_Z-IMU_Data.offset_gz;
		MPU6050_Read_Accel();
		accel_x=Mpu6050_Data.Accel_X;
		accel_y=Mpu6050_Data.Accel_Y;
		accel_z=Mpu6050_Data.Accel_Z;
		
		IMU_Data.Roll_a=atan2(accel_x,accel_z)/(PI/180);
		IMU_Data.Roll_g=-(gyro_y)/14.3;
		IMU_Data.Roll=FOCF(IMU_Data.Roll_a,IMU_Data.Roll_g,&IMU_Data.lastRoll);
		//vofaJustFloat(3,data);
		speed=50/IMU_Data.Roll;
		if(IMU_Data.Roll>1)
		{
			stepper1(Pos,1);
			stepper2(Neg,1);
		}
		else if(IMU_Data.Roll<-1)
		{
			stepper1(Neg,1);
			stepper2(Pos,1);
		}
		else
		{
			stepper_stop(1);
		}
		/*HAL_Delay(2000);
		stepper(Neg,1);
		HAL_Delay(2000);*/
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void stepper1(uint8_t dir,int speed)
{
	static uint8_t step=0;
	if(dir == Pos)
	{
		if(0==step)
		{
			step=8;
		}
		step--;
	}
	else
	{
		step++;
		if(8==step)
		{
			step=0;
		}
	}
	if(0==step)
	{
		MOTOR1_A_H;
		MOTOR1_B_L;
		MOTOR1_C_L;
		MOTOR1_D_L;
		HAL_Delay(speed);
	}
	else if(1==step)
	{
		MOTOR1_A_H;
		MOTOR1_B_H;
		MOTOR1_C_L;
		MOTOR1_D_L;
		HAL_Delay(speed);
	}
	else if(2==step)
	{
		MOTOR1_A_L;
		MOTOR1_B_H;
		MOTOR1_C_L;
		MOTOR1_D_L;
		HAL_Delay(speed);
	}
	else if(3==step)
	{
		MOTOR1_A_L;
		MOTOR1_B_H;
		MOTOR1_C_H;
		MOTOR1_D_L;
		HAL_Delay(speed);
	}
	else if(4==step)
	{
		MOTOR1_A_L;
		MOTOR1_B_L;
		MOTOR1_C_H;
		MOTOR1_D_L;
		HAL_Delay(speed);
	}
	else if(5==step)
	{
		MOTOR1_A_L;
		MOTOR1_B_L;
		MOTOR1_C_H;
		MOTOR1_D_H;
		HAL_Delay(speed);
	}
	else if(6==step)
	{
		MOTOR1_A_L;
		MOTOR1_B_L;
		MOTOR1_C_L;
		MOTOR1_D_H;
		HAL_Delay(speed);
	}
	else if(7==step)
	{
		MOTOR1_A_H;
		MOTOR1_B_L;
		MOTOR1_C_L;
		MOTOR1_D_H;
		HAL_Delay(speed);
	}
}

void stepper2(uint8_t dir,int speed)
{
	static uint8_t step=0;
	if(dir == Pos)
	{
		if(0==step)
		{
			step=8;
		}
		step--;
	}
	else
	{
		step++;
		if(8==step)
		{
			step=0;
		}
	}
	if(0==step)
	{
		MOTOR2_A_H;
		MOTOR2_B_L;
		MOTOR2_C_L;
		MOTOR2_D_L;
		HAL_Delay(speed);
	}
	else if(1==step)
	{
		MOTOR2_A_H;
		MOTOR2_B_H;
		MOTOR2_C_L;
		MOTOR2_D_L;
		HAL_Delay(speed);
	}
	else if(2==step)
	{
		MOTOR2_A_L;
		MOTOR2_B_H;
		MOTOR2_C_L;
		MOTOR2_D_L;
		HAL_Delay(speed);
	}
	else if(3==step)
	{
		MOTOR2_A_L;
		MOTOR2_B_H;
		MOTOR2_C_H;
		MOTOR2_D_L;
		HAL_Delay(speed);
	}
	else if(4==step)
	{
		MOTOR2_A_L;
		MOTOR2_B_L;
		MOTOR2_C_H;
		MOTOR2_D_L;
		HAL_Delay(speed);
	}
	else if(5==step)
	{
		MOTOR2_A_L;
		MOTOR2_B_L;
		MOTOR2_C_H;
		MOTOR2_D_H;
		HAL_Delay(speed);
	}
	else if(6==step)
	{
		MOTOR2_A_L;
		MOTOR2_B_L;
		MOTOR2_C_L;
		MOTOR2_D_H;
		HAL_Delay(speed);
	}
	else if(7==step)
	{
		MOTOR2_A_H;
		MOTOR2_B_L;
		MOTOR2_C_L;
		MOTOR2_D_H;
		HAL_Delay(speed);
	}
}

void stepper_stop(int speed)
{
	MOTOR1_A_L;
	MOTOR1_B_L;
	MOTOR1_C_L;
	MOTOR1_D_H;
	
	MOTOR2_A_L;
	MOTOR2_B_L;
	MOTOR2_C_L;
	MOTOR2_D_H;
	HAL_Delay(speed);
}


 
void IMU_offset(int OFFSET)
{
	int i;
   for(i=0;i<OFFSET;i++)
	{
     HAL_Delay(5);
		 MPU6050_Read_Gyro();	
	   gyro_x=Mpu6050_Data.Gyro_X;
	   gyro_y=Mpu6050_Data.Gyro_Y;
	   gyro_z=Mpu6050_Data.Gyro_Z;
	   MPU6050_Read_Accel();
	   accel_x=Mpu6050_Data.Accel_X;
	   accel_y=Mpu6050_Data.Accel_Y;
	   accel_z=Mpu6050_Data.Accel_Z;
     if(gyro_x==gyro_y)
			 {
         i--;
       }
     else
			 {
         IMU_Data.offset_gx+=gyro_x;
         IMU_Data.offset_gy+=gyro_y;
         IMU_Data.offset_gz+=gyro_z;
       }
  }
    IMU_Data.offset_gx/=OFFSET;
    IMU_Data.offset_gy/=OFFSET;
    IMU_Data.offset_gz/=OFFSET;
}


 
float FOCF(float acc_m,float gyro_m,float* last_angle)
{
  float temp_angle;
  temp_angle=Ka*acc_m+(1-Ka)*(*last_angle+gyro_m*dt);
  *last_angle=temp_angle;
  return temp_angle;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
