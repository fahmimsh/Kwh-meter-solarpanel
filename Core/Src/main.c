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
#include "math.h"
#include "i2c-lcd.h"
#include "micros.h" //https://github.com/leech001/micros
#include "stdio.h"
#include "keypad_4x4.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/*-----Var Serial UART-------*/
uint8_t buff_s[200];
uint16_t ukuranstring;
int decimalPrecision = 1; //pembulatan nilai desimal
/*------var LCD------*/
char key_val, buffer[5];
unsigned long startButtonMillis;
unsigned long currentButtonMillis;
unsigned long startMillisLCD;
unsigned long currentMillisLCD;
const unsigned long periodLCD = 1000;
int page = 1;
int z = 0;

/*-----Var Sample data Volt---*/
float voltageSampleRead = 0;
float voltageLastSample  = 0;
float voltageSampleSum   = 0;
float voltageSampleCount = 0;
float voltageMean;
float RMSVoltageMean;
//offset Voltage AC
int voltageOffsetRead = 0;
float voltageOffset1 = 0;

float voltageOffset2 = 0;
float voltageSampleSumOffset =0;
float offsetVoltageMean = 0;
float voltageOffsetLastSample = 0;
float voltageOffsetSampleCount = 0;

/*-----Var Sample data Arus---*/
float mVperAmpValue = 185 ;
float currentSampleRead = 0;
float currentLastSample  = 0;
float currentSampleSum   = 0;
float currentSampleCount = 0;
float currentMean ;
float RMSCurrentMean = 0;
float FinalRMSCurrent ;
float currentDisplay = 0;
// offset Arus AC
int currentOffsetRead = 0;
float currentOffset1 = 0;

float currentOffset2 = 0;
float currentSampleSumOffset = 0;
float offsetCurrentMean = 0;
float currentOffsetLastSample = 0;
float currentOffsetSampleCount = 0;

/*-----  3- AC Power Measurement-------*/
float sampleCurrent1 ;
float sampleCurrent2 ;
float sampleCurrent3 ;
float apparentPower;
float realPower = 0;
float powerSampleRead  = 0;
float powerLastSample   = 0;
float powerSampleCount  = 0;
float powerSampleSum    = 0;
float powerFactor = 0;
// Offset AC Power
int powerOffsetRead = 0;
float powerOffset = 0;
float powerOffsetLastSample = 0;
float powerOffsetSampleCount = 0;

/*-------4 - Daily Energy Measurement--*/
float dailyEnergy =   0;
float energyLastSample = 0;
float energySampleCount= 0;
float energySampleSum  = 0;
float finalEnergyValue = 0;
float accumulateEnergy = 0;
//Import & Export Energy
float importFinalEnergyValue = 0;
float exportFinalEnergyValue = 0;

/*-------var frequensi perhitungan-----*/
unsigned long startMicros;
unsigned long currentMicros;
int expectedFrequency = 47;

float frequencySampleCount = 0;
float frequency = 0;
float a;
float switch01 = 9;
float VanalogRead = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t micros();
void delay_us();
void cleardata(){
	lcd_clear();
	lcd_send_cmd(0x80|0x03);
	lcd_send_string("KWH METER SOLAR");
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
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();
  startMicros = micros();
  lcd_init();
  keypad_init();
  lcd_send_cmd(0x80|0x03);
  lcd_send_string("KWH METER SOLAR");

  startMillisLCD = HAL_GetTick();
  startButtonMillis = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  key_val = keypad_get_key_value();
	  currentButtonMillis = HAL_GetTick();
	  if (currentButtonMillis - startButtonMillis >= 300){
		  if (key_val == '6'){
			lcd_send_cmd(0x80|0x40);
			lcd_send_string("<SELECT>TO CALIBRATE");
		  }
		  if (key_val == '2'){
			  page = page - 1;
			  if (page <= 0){
				  page = 1;
			  }
			  startButtonMillis = HAL_GetTick();
		  }
		  if (key_val == '8'){
			  page = page - 1;
			  if (page > 3){
				  page = 3;
			  }
			  startButtonMillis = HAL_GetTick();
		  }
		  if (key_val == '4'){
				lcd_send_cmd(0x80|0x40);
				lcd_send_string("<SELECT>TO CALIBRATE");
		  }
		  if (key_val == '5'){
			  // untuk mengaktifkan offset
            currentOffsetRead = 1;
            voltageOffsetRead = 1;
            powerOffsetRead = 1;
			lcd_send_cmd(0x80|0x40);
			lcd_send_string("INISIALISASI>5 DETIK");
		  }
		  if (key_val == 'D'){
			  cleardata();
		  }
	  }


	  /*-------Perhitungan tegangan AC--------*/
	  if(HAL_GetTick() >= voltageLastSample + 1){
		  HAL_ADC_Start(&hadc1);
		  while (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC) != SET){;}
		  HAL_ADC_Stop(&hadc1);
	  	  float getvalue1 = HAL_ADC_GetValue(&hadc1);
		  float voltageanalogInput = map(getvalue1, 0, 4095, 0, 1024);
		  voltageSampleRead = 2 * (voltageanalogInput - 512) + voltageOffset1;
		  voltageSampleSumOffset = voltageSampleSumOffset + voltageSampleRead;

		  voltageSampleSum = voltageSampleSum + (voltageSampleRead * voltageSampleRead);
		  voltageSampleCount = voltageSampleCount + 1;
		  voltageLastSample = HAL_GetTick();
	  }
	  if (voltageSampleCount == 1000){
		  offsetVoltageMean = voltageSampleSumOffset/voltageSampleCount;

		  voltageMean = voltageSampleSum/voltageSampleCount;
		  RMSVoltageMean = sqrt(voltageMean)+ voltageOffset2;

		  ukuranstring = sprintf((char*)buff_s, "%.1f V  ",RMSVoltageMean);
		  HAL_UART_Transmit(&huart1, buff_s, ukuranstring, 100);

          voltageSampleSum =0;
          voltageSampleCount=0;
          voltageSampleSumOffset=0;
	  }


	  /*-------AC offset Voltage-------*/
	  if (voltageOffsetRead == 1){
		  voltageOffset1 = 0;
		  if(HAL_GetTick() >= voltageOffsetLastSample + 1){
			  voltageOffsetSampleCount = voltageOffsetSampleCount + 1;
			  voltageOffsetLastSample = HAL_GetTick();
		  }
		  if (voltageOffsetSampleCount == 2000){
              voltageOffset1 = -1*(offsetVoltageMean);   /*set nilai offset */
              voltageOffsetRead = 2;
              voltageOffsetSampleCount = 0;
		  }
	  }

	  if (voltageOffsetRead == 2){
		  voltageOffset2 = 0;
		  if(HAL_GetTick() >= voltageOffsetLastSample + 1){
              voltageOffsetSampleCount = voltageOffsetSampleCount + 1;
              voltageOffsetLastSample = HAL_GetTick();
		  }
		  if (voltageOffsetSampleCount == 2000){
              voltageOffset2 = - RMSVoltageMean; /* set nilai offset*/
              voltageOffsetRead = 0;
              voltageOffsetSampleCount = 0;
		  }
	  }


	  /*--------Perhitungan Arus AC--------*/
	  if (HAL_GetTick() >= currentLastSample + 1){
			HAL_ADC_Start(&hadc2);
			while (__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_EOC) != SET){;}
			HAL_ADC_Stop(&hadc2);
			float CurrentAnalogInput = HAL_ADC_GetValue(&hadc2);
			currentSampleRead = CurrentAnalogInput-512 + currentOffset1;

			currentSampleSumOffset = currentSampleSumOffset + currentSampleRead;

            currentSampleSum = currentSampleSum + (currentSampleRead * currentSampleRead) ;
            currentSampleCount = currentSampleCount + 1;
            currentLastSample = HAL_GetTick();
	  }

	  if (currentSampleCount == 1000){
          offsetCurrentMean = currentSampleSumOffset/currentSampleCount; /*rata rata nilai offset*/

          currentMean = currentSampleSum/currentSampleCount;
          RMSCurrentMean = sqrt(currentMean)+currentOffset2 ;
          FinalRMSCurrent = (((RMSCurrentMean /1024) *5000) /mVperAmpValue);

		  ukuranstring = sprintf((char*)buff_s, "%.1f A  ",FinalRMSCurrent);
		  HAL_UART_Transmit(&huart1, buff_s, ukuranstring, 100);

          currentSampleSum =0;
          currentSampleCount=0;
          currentSampleSumOffset=0;
	  }


	  /*-----AC Offset Arus-------*/
	  if (currentOffsetRead == 1){
		  currentOffset1 = 0;
		  if (HAL_GetTick() >= currentOffsetLastSample + 1){
              currentOffsetSampleCount = currentOffsetSampleCount + 1;
              currentOffsetLastSample = HAL_GetTick();
		  }
		  if (currentOffsetSampleCount == 2000){
              currentOffset1 = - offsetCurrentMean;
              currentOffsetRead = 2;
              currentOffsetSampleCount = 0;
		  }
	  }

	  if (currentOffsetRead == 2){
		  currentOffset2 = 0;
		  if (HAL_GetTick() >= currentOffsetLastSample + 1){
              currentOffsetSampleCount = currentOffsetSampleCount + 1;
              currentOffsetLastSample = HAL_GetTick();
		  }
          if(currentOffsetSampleCount == 2000)   /* setelah 2 detik, jalan program*/
            {
              currentOffset2 = - RMSCurrentMean;/* set nilai offset */
              currentOffsetRead = 0;
              currentOffsetSampleCount = 0;
            }
	  }


	  /*----AC POWER dengan penyearah---*/
	  if (HAL_GetTick() >= powerLastSample + 1){
			HAL_ADC_Start(&hadc2);
			while (__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_EOC) != SET){;}
			HAL_ADC_Stop(&hadc2);
	  	  	float getvalue2 = HAL_ADC_GetValue(&hadc2);
			float CurrentAnalogInput2 = map(getvalue1, 0, 4095, 0, 1024);
			sampleCurrent1 = CurrentAnalogInput2-512 + currentOffset1;
            sampleCurrent2 = (sampleCurrent1/1024)*5000;
            sampleCurrent3 = sampleCurrent2/mVperAmpValue;

  		  HAL_ADC_Start(&hadc1);
  		  while (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC) != SET){;}
  		  HAL_ADC_Stop(&hadc1);
  		  float voltageanalogInput2 = HAL_ADC_GetValue(&hadc1);
  		  voltageSampleRead = 2 * (voltageanalogInput2 - 512) + voltageOffset1;

          powerSampleRead = voltageSampleRead * sampleCurrent3; /*sample nilai daya*/
          powerSampleSum = powerSampleSum + powerSampleRead; /*perhitungan nilai sample daya*/
          powerSampleCount = powerSampleCount + 1;
          powerLastSample = HAL_GetTick();
	  }

	  if (powerSampleCount == 1000){
		  realPower = ((powerSampleSum/powerSampleCount)+ powerOffset) ;

		  ukuranstring = sprintf((char*)buff_s, "%.1f W  ",realPower);
		  HAL_UART_Transmit(&huart1, buff_s, ukuranstring, 100);

		  apparentPower = FinalRMSCurrent*RMSVoltageMean;

		  ukuranstring = sprintf((char*)buff_s, "%.1f VA  ",apparentPower);
		  HAL_UART_Transmit(&huart1, buff_s, ukuranstring, 100);

          powerFactor = realPower/apparentPower;
          if(powerFactor >1 || powerFactor<0) /* jika power Factor </> 0, maka powerfactor = 0 */
          {
            powerFactor = 0;
          }
          powerSampleSum =0;
          powerSampleCount=0;
	  }


	  /*------Offset AC Power-----*/
	  if (powerOffsetRead == 1){
          powerOffset = 0;
          if(HAL_GetTick()>= powerOffsetLastSample + 1)
            {
              powerOffsetSampleCount = powerOffsetSampleCount + 1;
              powerOffsetLastSample = HAL_GetTick();
            }
          if(powerOffsetSampleCount == 5000) /*setelah 5 detik, program jalan*/
            {
              powerOffset = -realPower;
              powerOffsetRead = 0;
              powerOffsetSampleCount = 0;

              exportFinalEnergyValue = 0;
              importFinalEnergyValue = 0;
              finalEnergyValue = 0;
            }
	  }


	  /*------Akumulasi & Pengukuran Energi Harian----*/
	  if (HAL_GetTick() >= energyLastSample + 1){
          energySampleCount = energySampleCount + 1;
          energyLastSample = HAL_GetTick();
	  }
	  if (energySampleCount == 1000){
          accumulateEnergy = realPower/3600;
          finalEnergyValue = finalEnergyValue + accumulateEnergy;

		  ukuranstring = sprintf((char*)buff_s, "%.1f kWh  ",finalEnergyValue/1000);
		  HAL_UART_Transmit(&huart1, buff_s, ukuranstring, 100);
		  /*Impor & Ekspor Akumulasi & Pengukuran Energi Harian*/
          if (accumulateEnergy >=0)                                                     /*Jika energinya positif, itu adalah energi import*/
            {
              importFinalEnergyValue = importFinalEnergyValue + accumulateEnergy;
            }

          if (accumulateEnergy <0)                                                      /* Jika energinya negatif, itu adalah energi ekspor*/
            {
              exportFinalEnergyValue = exportFinalEnergyValue - accumulateEnergy;
            }

      energySampleCount = 0 ;
	  }


	  if (z == 1){
          VanalogRead = voltageSampleRead;
          if(VanalogRead < 0 && switch01 == 9)
          {
            switch01 = 8;
          }

          if(VanalogRead >= 0 && switch01 ==8)
          {
            startMicros = micros();
            switch01 = 7;
          }

          if(VanalogRead < 0 && switch01 == 7)
          {
            switch01 = 6;
          }
          if(VanalogRead >=0 && switch01 == 6)
          {
             currentMicros = micros();
             frequencySampleCount = frequencySampleCount +1 ;
             switch01 = 7;
          }

          if(frequencySampleCount == expectedFrequency)
            {

                a = currentMicros-startMicros;
                frequency = 1/((a/1000000)/frequencySampleCount); //rata rata frequensi
                ukuranstring = sprintf((char*)buff_s, "%.1f Hz  ",frequency);
                HAL_UART_Transmit(&huart1, buff_s, ukuranstring, 100);
                frequencySampleCount = 0;
                switch01 = 9;
                z = 0;
            }
	  }


	  /*=======>tampilan LCD*/
	  currentMillisLCD = HAL_GetTick();
	  if (currentMillisLCD - startMillisLCD >= periodLCD && page ==1){
		  if(realPower >= 0){
			  currentDisplay = FinalRMSCurrent;
		  }
		  if(realPower < 0){
			  currentDisplay = -FinalRMSCurrent;
		  }
		  lcd_send_cmd(0x80|0x14);
		  sprintf(buffer, "%.2f A   ", currentDisplay);
		  lcd_send_string(buffer);
		  lcd_send_cmd(0x80|0x21);
		  sprintf(buffer, "%.1f V   ", RMSVoltageMean);
		  lcd_send_string(buffer);
		  lcd_send_cmd(0x80|0x54);
		  sprintf(buffer, "%.1f W   ", realPower);
		  lcd_send_string(buffer);
		  lcd_send_cmd(0x80|0x60);
		  sprintf(buffer, "%.1f VA  ", apparentPower);
		  lcd_send_string(buffer);
		  startMillisLCD = currentMillisLCD;
		  z = 1;
	  }

	  if (currentMillisLCD - startMillisLCD >= periodLCD && page == 2){
		  lcd_send_cmd(0x80|0x14);
		  lcd_send_string("PF=");
		  sprintf(buffer, "%.2f   ", powerFactor);
		  lcd_send_string(buffer);
		  lcd_send_cmd(0x80|0x21);
		  sprintf(buffer, "%.1f Hz   ", frequency);
		  lcd_send_string(buffer);
		  lcd_send_cmd(0x80|0x54);
		  lcd_send_string("Net=");
		  sprintf(buffer, "%.2f kWh", finalEnergyValue/1000);
		  lcd_send_string(buffer);
		  startMillisLCD = currentMillisLCD;
		  z = 1;
	  }
	  if (currentMillisLCD - startMillisLCD >= periodLCD && page == 3){
		  lcd_send_cmd(0x80|0x14);
		  lcd_send_string("impE=");
		  sprintf(buffer, "%.2f kWh", importFinalEnergyValue/1000);
		  lcd_send_string(buffer);
		  lcd_send_cmd(0x80|0x54);
		  lcd_send_string("expE=");
		  sprintf(buffer, "%.2f kWh", exportFinalEnergyValue/1000);
		  lcd_send_string(buffer);
		  startMillisLCD = currentMillisLCD;
		  z = 1;
	  }



/*
	HAL_ADC_Start(&hadc1);
	while (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC) != SET){;}
	voltageSampleRead = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	lcd_send_cmd(0x80|0x40);
	lcd_send_string("VOLT:");
	sprintf(buffer, "%.1f", voltageSampleRead);
	lcd_send_string(buffer);

	HAL_ADC_Start(&hadc2);
	while (__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_EOC) != SET){;}
	currentSampleRead = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	lcd_send_cmd(0x80|0x14);
	lcd_send_string("ARUS:");
	sprintf(buffer, "%.1f", currentSampleRead);
	lcd_send_string(buffer);
	delay_us(100000);

		key_val = keypad_get_key_value();
		lcd_send_cmd(0x80|0x54);
		lcd_send_string("KEYP:");
		lcd_send_cmd(0x80 | 0x59);
		lcd_send_data(key_val);
*/

//	   ukuranstring = sprintf((char*)buff_s, "Data salah\r\n");
//	   HAL_UART_Transmit(&huart1, buffer, ukuranstring, 100);
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
