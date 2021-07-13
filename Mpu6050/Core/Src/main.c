/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "at24c256.h"  // bunu eklememiz lazim


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

/* USER CODE BEGIN PV */
HAL_StatusTypeDef ret;
//////////Mpu Variabels /////////

#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19


#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75


int16_t Accel_X_RAW=0;
int16_t Accel_Y_RAW=0;
int16_t Accel_Z_RAW=0;

int16_t Gyro_X_RAW=0;
int16_t Gyro_Y_RAW=0;
int16_t Gyro_Z_RAW=0;

float Ax,Ay,Az,Gx,Gy,Gz;

///////////////////MY variables////////////////////


uint8_t MPUtest=0;
uint8_t retval[100];
uint8_t retval1[100];

float value1=0;
float value2=0;
float value3=0;

uint8_t sayac=0;
uint8_t kontrol=0;

















/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void MPU6050_Init(void){
uint8_t check, Data;
// check device id
HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG,1, &check, 1, 1000);
if (check == 0x68){ // boyle bir cihaz varsa
 // power management regisgter 0x6b power up
Data = 0;
HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR,
PWR_MGMT_1_REG,1, &Data, 1, 1000);
//
Data = 0x07;
HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR,
SMPLRT_DIV_REG,1, &Data, 1, 1000);
// set acc. meter configuration in ACCEL_CONFIG reg.

Data = 0x00;
HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR,
ACCEL_CONFIG_REG,1, &Data, 1, 1000);
// Set Gyroscopic configuration in GYRO_CONFIG
//Register
// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ? 250 ?/s
Data = 0x00;
HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR,
GYRO_CONFIG_REG, 1, &Data, 1, 1000);
}
}
void MPU6050_Read_Accel (void)
{
uint8_t Rx_data[6];
// Read 6 BYTES of data starting from ACCEL_XOUT_H register
HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG,1, Rx_data, 6,1000);
Accel_X_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data [1]);
Accel_Y_RAW = (int16_t)(Rx_data[2] << 8 | Rx_data [3]);
Accel_Z_RAW = (int16_t)(Rx_data[4] << 8 | Rx_data [5]);
Ax = Accel_X_RAW/16384.0;
Ay = Accel_Y_RAW/16384.0;
Az = Accel_Z_RAW/16384.0;
}
void MPU6050_Read_Gyro (void)
{
uint8_t Rx_data[6];
// read 6 bytes of data starting from GYRO_XOUT_H reg.
HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG,1, Rx_data,6, 1000);
Gyro_X_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data [1]);
Gyro_Y_RAW = (int16_t)(Rx_data[2] << 8 | Rx_data [3]);
Gyro_Z_RAW = (int16_t)(Rx_data[4] << 8 | Rx_data [5]);

Gx = Gyro_X_RAW/131.0;
Gy = Gyro_Y_RAW/131.0;
Gz = Gyro_Z_RAW/131.0;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	MPU6050_Init();
/*

  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG,1, &MPUtest, 1, 1000);

  HAL_Delay (500);
  uint8_t send_eeprom[6] = {'1','3','8','3','F','S'};
  _writeEEPROMString(&hi2c1, 0xA0, 0,(uint8_t*)send_eeprom);

  _readEEPROMString(&hi2c1, 0xA0, 0,7,retval);
	
	*/


////rom temizle
//_eraseEEPROM(&hi2c1, 0xA0);

 
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		
		MPU6050_Read_Accel();
		MPU6050_Read_Gyro();
		value1 = Ax*100;
		value2= Ay*100;
		value3= Az*100;

		if(value1<0)
			value1=0-value1;
		
		if(value2<0)
			value2=0-value2;

		if(value1 > 60 || value2 > 60){
		  _writeEEPROM(&hi2c1, 0xA0, sayac,(uint8_t) value1);
			sayac++;
		  _writeEEPROM(&hi2c1, 0xA0, sayac,(uint8_t) value2);
			sayac++;
		  _writeEEPROM(&hi2c1, 0xA0, sayac,(uint8_t)value3);
			sayac++;
			
			HAL_Delay(1000);
			_readEEPROMString(&hi2c1, 0xA0, 0, 50,  retval1);
		}
		

		

		
		HAL_Delay(100);
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
