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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "math.h"
#include "micros.h" //https://github.com/leech001/micros
#include "i2c-lcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define map(x,in_min,in_max,out_min,out_max) ( (x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min )
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//serial komunikasi
uint8_t buff_s[200];
uint16_t ukuranstring;
char buffer[5];
//timer
unsigned long time_send = 0;
unsigned long startfreq = 0;
unsigned long currentMillisAC = 0;
unsigned long startMillisAC = 0;
unsigned long time_power = 0;
unsigned long time_energy = 0;
int ACfreqState = 0, cure = 0;
//sensor arus
float sensitivity;
//tegangan
float Vsensitivity;
float Volt_rms = 0, Voltoffset = 0;
//frekuesi
float frequensi = 0;
uint32_t absl = 2, factor_inner = 0;
float freqsample = 0, cycleNumber = 0;
int ACFrequencyWait = 0;
//power
float Power = 0, realPowerAC = 0, apparentPower = 0;
float powerFactorAC = 0;
uint32_t Power_count = 0;
//energymeter
float finalEnergyAC = 0;
float netEnergyAC = 0, importEnergyAC = 0, exportEnergyAC = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t micros();
void delay_us();
//kalibrasi sensor arus
int zero_acs = 0;
int zero_zmpt = 0;
float Arus_rms = 0;
int32_t Inow;
int32_t Vnow;
int acs712_calibrate(){
	uint16_t acc = 0;
	for (int i = 0; i < 10; i++) {
		HAL_ADC_Start(&hadc2);
		while (__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_EOC) != SET){;}
		HAL_ADC_Stop(&hadc2);
		acc += HAL_ADC_GetValue(&hadc2);

		ukuranstring = sprintf((char*)buff_s, "%d ADC \r\n", acc);
		HAL_UART_Transmit(&huart1, buff_s, ukuranstring, 100);
	}
	int zero = acc / 10;
	return zero;
}
int Zmpt_calibrate(){
	uint16_t vcc = 0;
	for (int i = 0; i < 10; i++) {
		HAL_ADC_Start(&hadc1);
		while (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC) != SET){;}
		HAL_ADC_Stop(&hadc1);
		vcc += HAL_ADC_GetValue(&hadc1);

		ukuranstring = sprintf((char*)buff_s, "%d ADC \r\n", vcc);
		HAL_UART_Transmit(&huart1, buff_s, ukuranstring, 100);
	}
	int zero = vcc / 10;
	return zero;
}
int factor_freq(){
	uint32_t fa = absl * 25;
	uint32_t absl2 = 0;
	for (int i = 0; i < fa; i++) {
		absl2 ++;
	}
	return absl2;
}
//get sensor acs712
float acs712_get(uint16_t frequency){
	uint32_t periode = 1000000 / frequency;
	uint32_t t_start = micros();
	int32_t Isum = 0, measurements_count = 0;
	while (micros() - t_start < periode) {
		HAL_ADC_Start(&hadc2);
		while (__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_EOC) != SET){;}
		Inow = HAL_ADC_GetValue(&hadc2) - zero_acs;
		HAL_ADC_Stop(&hadc2);
		Isum += Inow*Inow;
		measurements_count++;
	}
	sensitivity = 0.180; /*ACS712_05B:*/ //0.185
	//sensitivity = 0.100; /*ACS712_20A:*/
	//sensitivity = 0.066; // ACS712_30A:
	if (Isum < 0){
	   Arus_rms = (sqrt(Isum / measurements_count)/ 4095 * 3.30/ sensitivity)*-1;
	} else if (Isum >= 0){
	   Arus_rms = sqrt(Isum / measurements_count)/ 4095 * 3.30/ sensitivity;
	}
	return Arus_rms;
}
void Lcd_name(){
	lcd_clear();
	lcd_send_cmd(0x80|0x02);
	lcd_send_string("KWH METER SOLLAR");
	lcd_send_cmd(0x80|0x43);
	lcd_send_string("BI-DIRECTIONAL");
	lcd_send_cmd(0x80|0x16);
	lcd_send_string("NAMA: M.FARKHAN");
	lcd_send_cmd(0x80|0x57);
	lcd_send_string("NRP: 1303181015");
	HAL_Delay(3000);
	lcd_clear();
}
void set(){
	  lcd_send_cmd(0x80|0x09); lcd_send_string("|");
	  lcd_send_cmd(0x80|0x49); lcd_send_string("|");
	  lcd_send_cmd(0x80|0x1D); lcd_send_string("|");
	  lcd_send_cmd(0x80|0x5D); lcd_send_string("|");
	  lcd_send_cmd(0x80|0x0A); lcd_send_string("|");
	  lcd_send_cmd(0x80|0x4A); lcd_send_string("|");
	  lcd_send_cmd(0x80|0x1E); lcd_send_string("|");
	  lcd_send_cmd(0x80|0x5E); lcd_send_string("|");
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  DWT_Init();
  ukuranstring = sprintf((char*)buff_s, "Mulai Perhitungan Solar\r\n");
  HAL_UART_Transmit(&huart1, buff_s, ukuranstring, 10);
  zero_acs = acs712_calibrate();
  zero_zmpt = Zmpt_calibrate();
  factor_inner = factor_freq();
  ukuranstring = sprintf((char*)buff_s, "Zero ACS = %d | Zero ZMPT : %d\r\n", zero_acs, zero_zmpt);
  HAL_UART_Transmit(&huart1, buff_s, ukuranstring, 10);
  Lcd_name();
  time_power = HAL_GetTick();
  time_energy = HAL_GetTick();
  time_send = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  float Arus_acs = acs712_get(factor_inner);
	  uint32_t Vperiode = 1000000 / factor_inner; //f_factor
	  uint32_t Vt_start = micros();
	  int32_t Vsum = 0, Vmeasurements_count = 0;
	  while (micros() - Vt_start < Vperiode) {
		HAL_ADC_Start(&hadc1);
		while (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC) != SET){;}
		Vnow = HAL_ADC_GetValue(&hadc1) - zero_zmpt;
		HAL_ADC_Stop(&hadc1);
		Vsum += Vnow*Vnow;
		Vmeasurements_count++;
	  }
	  sensitivity = 0.00239;
	  float V_tick = Vsum / Vmeasurements_count;
	  float Vmean = sqrt(Vsum / Vmeasurements_count);
	  Volt_rms = Vmean/ 4095 * 3.30/ sensitivity;
	  frequensi = (V_tick /10000) + ((V_tick /10000)/6); //EoC setting DMA micros stm32f103c8t6
	  //power
	  if (HAL_GetTick() >= time_power + 1){
		  float Pow = Arus_acs * Volt_rms;
		  Power += Pow;
		  Power_count ++;
		  time_power = HAL_GetTick();
	  }
	  if (Power_count == 100){
		  realPowerAC = Power/100;
		  apparentPower = Arus_acs * Volt_rms;
		  powerFactorAC = cos(realPowerAC/apparentPower);
		  if(realPowerAC < 0){
		      powerFactorAC = -1* cos(realPowerAC / apparentPower);
		  }
		  if(powerFactorAC > 1 || powerFactorAC <-1){
			  powerFactorAC = 0;
		  }
		  Power_count = 0; Power = 0; cure = 1;
	  }
	  if ((cure == 1) && (HAL_GetTick() >= time_energy + 1000)){
		  finalEnergyAC = ((realPowerAC / 3600) * (5000/1000));
		  netEnergyAC += finalEnergyAC;
		  if( realPowerAC >= 0){
		      importEnergyAC = importEnergyAC + finalEnergyAC;
		  }
		  if(realPowerAC < 0){
		      exportEnergyAC = exportEnergyAC - finalEnergyAC;
		  }
		  time_energy = HAL_GetTick();
	  }
	  if ((cure == 0) && (Power_count <=100)){
		  ukuranstring = sprintf((char*)buff_s, "Calibration : %lu | Volt : %f\r\n", Power_count, Volt_rms);
		  HAL_UART_Transmit(&huart1, buff_s, ukuranstring, 10);
		  lcd_send_cmd(0x80|0x05); lcd_send_string("Calibration");
		  lcd_send_cmd(0x80|0x16); sprintf(buffer, "time: %lu", Power_count); lcd_send_string(buffer);
		  lcd_send_cmd(0x80|0x42); sprintf(buffer, "Vrms: %.2fV", Volt_rms); lcd_send_string(buffer);
		  if (Power_count >= 99){
			  lcd_clear(); set();
			  ukuranstring = sprintf((char*)buff_s, "Kalibrasi Berhasil\r\n");
			  HAL_UART_Transmit(&huart1, buff_s, ukuranstring, 10);
		  }
	  }
	  if ((cure == 1) && (HAL_GetTick() >= time_send + 500)){
		  lcd_send_cmd(0x80|0x00); sprintf(buffer, "V:%.1f V", Volt_rms); lcd_send_string(buffer);
		  lcd_send_cmd(0x80|0x40); sprintf(buffer, "I:%.3f A", Arus_acs); lcd_send_string(buffer);
		  lcd_send_cmd(0x80|0x0B); sprintf(buffer, "P:%.2f W", realPowerAC); lcd_send_string(buffer);
		  lcd_send_cmd(0x80|0x4B); sprintf(buffer, "S:%.2fVA", apparentPower); lcd_send_string(buffer);
		  lcd_send_cmd(0x80|0x14); sprintf(buffer, "F:%.1f Hz", frequensi); lcd_send_string(buffer);
		  lcd_send_cmd(0x80|0x54); sprintf(buffer, "net:%.1fWh", netEnergyAC); lcd_send_string(buffer);
		  lcd_send_cmd(0x80|0x1F); sprintf(buffer, "Ex:%.2fWh", exportEnergyAC); lcd_send_string(buffer);
		  lcd_send_cmd(0x80|0x5F); sprintf(buffer, "Im:%.2fWh", importEnergyAC); lcd_send_string(buffer);
		  ukuranstring =
				  sprintf((char*)buff_s, "Arus: %.3f | Volt: %.2f | Daya: %.2f | VA: %.2f "
						  	  	  	  	  "| PF: %.3f | netE: %.2f | import: %.2f "
						  	  	  	  	  "| export: %.2f | f: %.2f\r\n",
										  Arus_acs, Volt_rms, realPowerAC, apparentPower,
										  powerFactorAC, netEnergyAC, importEnergyAC,
										  exportEnergyAC, frequensi);
		  HAL_UART_Transmit(&huart1, buff_s, ukuranstring, 100);
		  time_send = HAL_GetTick();
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
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
