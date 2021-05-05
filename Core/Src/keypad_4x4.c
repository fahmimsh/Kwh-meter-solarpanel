// This file modified by M. Maulana Muzhaffar Zuhri April 2020 for Proyek Akhir
// Program Study Teknik Elektro Industri
// mamuzu99@gmail.com

#include "stm32f1xx_hal.h"
#define __KEYPAD_4X4    					1             

// Deklarasikan Port sambungan Keypad 
#if __KEYPAD_4X4
// inisalisasi Keypad  
   
#define R1_PORT GPIOB
#define R1_PIN GPIO_PIN_15

#define R2_PORT GPIOB
#define R2_PIN GPIO_PIN_14

#define R3_PORT GPIOB
#define R3_PIN GPIO_PIN_13

#define R4_PORT GPIOB
#define R4_PIN GPIO_PIN_12

#define C1_PORT GPIOA
#define C1_PIN GPIO_PIN_6

#define C2_PORT GPIOA
#define C2_PIN GPIO_PIN_5

#define C3_PORT GPIOA
#define C3_PIN GPIO_PIN_4

#define C4_PORT GPIOA
#define C4_PIN GPIO_PIN_3
//


void keypad_init(void)
{		
	GPIO_InitTypeDef GPIO_InitStructure; 
	/********************** Init GPIO Keypad *************************/
 	// aktivasi Kolom bit
	GPIO_InitStructure.Pin = C1_PIN|C2_PIN|C3_PIN|C4_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// aktivasi Baris bit
	GPIO_InitStructure.Pin = R1_PIN|R2_PIN|R3_PIN|R4_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	HAL_GPIO_WritePin(GPIOB,R1_PIN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,R2_PIN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,R3_PIN,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,R4_PIN,GPIO_PIN_SET);
/********************** End GPIO LCD ********************/
}


char keypad_get_key_value(void)
{     int key;
			key=' ';
      HAL_GPIO_WritePin(GPIOB,R1_PIN,GPIO_PIN_RESET);
			HAL_Delay(10);
      if(HAL_GPIO_ReadPin(C1_PORT,C1_PIN)==0) key = '1'; 
      else if(HAL_GPIO_ReadPin(C2_PORT,C2_PIN)==0) key ='2';
      else if(HAL_GPIO_ReadPin(C3_PORT,C3_PIN)==0) key ='3';
      else if(HAL_GPIO_ReadPin(C4_PORT,C4_PIN)==0) key ='A';
      HAL_GPIO_WritePin(GPIOB,R1_PIN,GPIO_PIN_SET);
      
			HAL_GPIO_WritePin(GPIOB,R2_PIN,GPIO_PIN_RESET);
			HAL_Delay(10);
      if(HAL_GPIO_ReadPin(C1_PORT,C1_PIN)==0) key = '4'; 
      else if(HAL_GPIO_ReadPin(C2_PORT,C2_PIN)==0) key ='5';
      else if(HAL_GPIO_ReadPin(C3_PORT,C3_PIN)==0) key ='6';
      else if(HAL_GPIO_ReadPin(C4_PORT,C4_PIN)==0) key ='B';
      HAL_GPIO_WritePin(GPIOB,R2_PIN,GPIO_PIN_SET);
		
 			HAL_GPIO_WritePin(GPIOB,R3_PIN,GPIO_PIN_RESET);
			HAL_Delay(10);
      if(HAL_GPIO_ReadPin(C1_PORT,C1_PIN)==0) key = '7'; 
      else if(HAL_GPIO_ReadPin(C2_PORT,C2_PIN)==0) key ='8';
      else if(HAL_GPIO_ReadPin(C3_PORT,C3_PIN)==0) key ='9';
      else if(HAL_GPIO_ReadPin(C4_PORT,C4_PIN)==0) key ='C';
      HAL_GPIO_WritePin(GPIOB,R3_PIN,GPIO_PIN_SET);
	
 			HAL_GPIO_WritePin(GPIOB,R4_PIN,GPIO_PIN_RESET);
			HAL_Delay(10);
      if(HAL_GPIO_ReadPin(C4_PORT,C1_PIN)==0) key = '*'; 
      else if(HAL_GPIO_ReadPin(C2_PORT,C2_PIN)==0) key ='0';
      else if(HAL_GPIO_ReadPin(C3_PORT,C3_PIN)==0) key ='#';
      else if(HAL_GPIO_ReadPin(C4_PORT,C4_PIN)==0) key ='D';
      HAL_GPIO_WritePin(GPIOA,R4_PIN,GPIO_PIN_SET);
      
			return key;
}

#endif
