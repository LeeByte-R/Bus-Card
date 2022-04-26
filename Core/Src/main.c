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
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <stdlib.h>
#include "MFRC522.h"
#include "LCD1602I2C.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define STATION_SIZE 16
#define ROUTE_NAME_SIZE 15
#define NAME_SIZE       16
#define BLOCK_SIZE      16

// fare type
#define REGULAR 0x00
#define HALF    0x01
#define OTHER   0x02

// direction symbol in card
#define CLEAR     0x00
#define DEPARTURE '>'
#define RETURN    '<'

typedef enum{
  GETON,
  GETOFF,
  RWERROR,      //read/write PICC error
  INSUFFICIENT, //not enough money to get on
  ROUTEERROR,   //route of PICC does not conform to routeo f PCD
  WAITINITIAL,  //start initialize route
  WAITUPDATE,   //wait update route 
  STOPSERVICE,  //stop PCD
  ROUTESTATION
} PrintCode;

typedef struct{   // default store in sector1
  uint8_t name[NAME_SIZE];                  // block 0 [0:15]  card user name consists of 15 character at most
  int money;                                // block 1 [0:3]   remains in PICC
  int get_on_station;                       // block 1 [5:8]   get_on_station number if -1 then did not get on
  uint8_t type;                             // block 1 [4]     regular or half fare, 0x00 regular, 0x01 half
  uint8_t padding[7];                       // block 1 [9:15]  reserved
  uint8_t direction;                        // block 2 [0]     get on departure('>') or return('>'), get off = 0x00 
  uint8_t route_name[ROUTE_NAME_SIZE];      // block 2 [1:15]  route name consists of 14 character at most
} Passenger;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int *fare;              // fare[x][y] is station x -> station y fare
int *half_fare;         // half_fare[x][y] is station x -> station y half fare
int base_fare;
int base_half_fare;

uint8_t route_name[ROUTE_NAME_SIZE];    // route name consists of 9 character at most
uint8_t *route;         // each station on route uint8_t[16]
int station_n;          // how many stations on route

int now_station;        // bus now station
uint8_t direction;    // departure or return
StatusCode status;
int fare_money;         // passenger's fare

uint8_t empty_array[16] = {'\0'};
uint8_t s[50];
uint32_t n;

volatile bool update_flag;
volatile bool next_flag;
volatile bool reverse_flag;
volatile bool stop_flag;
volatile bool debounce_wait_flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void Delay_Us(uint16_t wait);
bool Read_Passenger(Passenger *pas, int sector);
bool Write_Passenger(Passenger *pas, int sector);
void Update_Route(void);
void Center_Align_String(char *str, int w); // center align str in w width, result is stored in global value s
void Print_Message(PrintCode code, Passenger *pas);
void Print_Passenger(Passenger *pas);
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  int store_sector = 1;
  int base_money; //how much get on money at least 
  Passenger passenger;
  
  LCD1602I2C_Init();
	LCD1602I2C_Clear();
  PCD_Init();
  PCD_DumpVersionToSerial();
  
  MIFARE_Key key;
  for(int i=0;i<16;i++){
    key.keyByte[i] = 0xFF;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Print_Message(WAITINITIAL, &passenger);
  Update_Route();   //initialize route data
  stop_flag = false;
  update_flag = false;
  next_flag = false;
  reverse_flag = false;
  
  while (1)
  {
    if(stop_flag){
      Print_Message(STOPSERVICE, &passenger);
      while(stop_flag){
        update_flag = false;
        next_flag = false;
        reverse_flag = false;
      }
      stop_flag = false;
    }
    
    if(update_flag){
      update_flag = false;
      Print_Message(WAITUPDATE, &passenger);
      Update_Route();
    }
    if(next_flag){
      next_flag = false;
      if(direction == DEPARTURE){
        if(now_station+1 < station_n)
          now_station ++;
      }else if(direction == RETURN){
        if(0 <= now_station-1)
          now_station --;
      }
    }
    if(reverse_flag){
      reverse_flag = false;
      if(direction == DEPARTURE){
        direction = RETURN;
        now_station = station_n - 1;
      }else if (direction == RETURN){
        direction = DEPARTURE;
        now_station = 0;
      }
    }
    
    if (PICC_IsNewCardPresent()&&PICC_ReadCardSerial()) {
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
      HAL_Delay(200);
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
      
      status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, store_sector * 4, &key, &uid); // authenticate sector 1
      if (status != STATUS_OK) {
        GetStatusCodeName(status, s, &n);
        printf("PCD_Authenticate() failed: %s\r\n", s);
      }
      
      if(Read_Passenger(&passenger, store_sector)){
        Print_Passenger(&passenger);
        //deal passenger information
        if(passenger.get_on_station < 0){
          //get_on_station no data, get on
          switch(passenger.type){
            case REGULAR:
              base_money = base_fare;
              break;
            case HALF:
              base_money = base_half_fare;
              break;
            case OTHER:
              break;
          }
          
          if(passenger.money - base_money >= 0){ //update get on station, direction, route name to PICC
            passenger.get_on_station = now_station;
            passenger.direction = direction;
            memcpy(passenger.route_name, route_name, ROUTE_NAME_SIZE);
            if(Write_Passenger(&passenger, store_sector)){
              Print_Message(GETON, &passenger);
            }else{
              Print_Message(RWERROR, &passenger);
            }
          }else{
              Print_Message(INSUFFICIENT, &passenger);
          }
        }else{
          //direction and route of PICC is equal to PCD
          if(strcmp((char*)passenger.route_name, (char*)route_name) == 0 && passenger.direction == direction){
            if((passenger.direction == DEPARTURE && passenger.get_on_station <= now_station) || 
               (passenger.direction == RETURN && now_station <= passenger.get_on_station)){ 
              //fare = fare[get on][get off]
              switch(passenger.type){
                case REGULAR:
                  fare_money = fare[passenger.get_on_station * station_n + now_station];
                  break;
                case HALF:
                  fare_money = half_fare[passenger.get_on_station * station_n + now_station];
                  break;
                case OTHER:
                  break;
              }
              
              passenger.money -= fare_money;
              passenger.get_on_station = -1;  // -1 is not get on
              passenger.direction = CLEAR;
              memcpy(passenger.route_name, empty_array, ROUTE_NAME_SIZE); //clear route name
              if(Write_Passenger(&passenger, store_sector)){
                Print_Message(GETOFF, &passenger);
              }else{
                Print_Message(RWERROR, &passenger);
              }
              }else{  //departure: get off < get on, return get off > get on, error
              Print_Message(ROUTEERROR, &passenger);
            }
          }else{  //route of PICC does not conform to PCD route
            Print_Message(ROUTEERROR, &passenger);
          }
        }
      }else{
        Print_Message(RWERROR, &passenger);
      }
      PICC_HaltA();
      PCD_StopCrypto1();
      HAL_Delay(1300);
    }

    Print_Message(ROUTESTATION, &passenger);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000E14;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 3;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MFRC522_SS_Pin|MFRC522_RST_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MFRC522_SS_Pin MFRC522_RST_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = MFRC522_SS_Pin|MFRC522_RST_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STOP_SERVICE_Pin REVERSE_ROUTE_Pin */
  GPIO_InitStruct.Pin = STOP_SERVICE_Pin|REVERSE_ROUTE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : NEXT_STATION_Pin UPDATE_ROUTE_Pin */
  GPIO_InitStruct.Pin = NEXT_STATION_Pin|UPDATE_ROUTE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
bool Read_Passenger(Passenger *pas, int sector){
  uint8_t size = 18;
  uint8_t buffer[size];
  uint8_t *p = (uint8_t *)pas;
  
  status = MIFARE_Read(sector * 4 + 0, buffer, &size);
  if (status != STATUS_OK) {
    GetStatusCodeName(status, s, &n);
    printf("read name failed: %s\r\n", s);
    return false;
  }
  memcpy(p, buffer, 16);
  
  status = MIFARE_Read(sector * 4 + 1, buffer, &size);
  if (status != STATUS_OK) {
    GetStatusCodeName(status, s, &n);
    printf("read card information failed: %s\r\n", s);
    return false;
  }
  memcpy(p+16, buffer, 16);
  
  status = MIFARE_Read(sector * 4 + 2, buffer, &size);
  if (status != STATUS_OK) {
    GetStatusCodeName(status, s, &n);
    printf("read route name failed: %s\r\n", s);
    return false;
  }
  memcpy(p+16, buffer, 16);
  
  return true;
}
bool Write_Passenger(Passenger *pas, int sector){
  uint8_t size = 16;
  uint8_t *p = (uint8_t *)pas;
  
  status = MIFARE_Write(sector * 4 + 0, p, size);
  if (status != STATUS_OK) {
    GetStatusCodeName(status, s, &n);
    printf("write name failed: %s\r\n", s);
    return false;
  }

  status = MIFARE_Write(sector * 4 + 1, p+16, size);
  if (status != STATUS_OK) {
    GetStatusCodeName(status, s, &n);
    printf("write card infromation failed: %s\r\n", s);
    return false;
  }

  status = MIFARE_Write(sector * 4 + 2, p+32, size);
  if (status != STATUS_OK) {
    GetStatusCodeName(status, s, &n);
    printf("write route name failed: %s\r\n", s);
    return false;
  }
  
  return true;
}

void Delay_Us(uint16_t wait){
  HAL_TIM_Base_Start(&htim7);
  __HAL_TIM_SET_COUNTER(&htim7, 0);  // set the counter value a 0
  while(__HAL_TIM_GET_COUNTER(&htim7) < wait);
  HAL_TIM_Base_Stop(&htim7);
}

void Center_Align_String(char *str, int w){
  sprintf((char*)s, "%*s%*s", 
          (w + strlen(str)) / 2, 
          str,
          w - ((w + strlen(str)) / 2),
          ""
         );
}

void Print_Message(PrintCode code, Passenger *pas){
  switch(code){
    case GETON:
      LCD1602I2C_Cursor(0, 0);
      Center_Align_String("Get On Station", 16);
      LCD1602I2C_SendString((char*)s);
      LCD1602I2C_Cursor(1, 0);
      sprintf((char*)s, "balance %-8d", pas->money);
      LCD1602I2C_SendString((char*)s);
      break;
    case GETOFF:
      LCD1602I2C_Cursor(0, 0);
      sprintf((char*)s, "Fare     Balance");
      LCD1602I2C_SendString((char*)s);
      LCD1602I2C_Cursor(1, 0);
      sprintf((char*)s, "%-8d%8d", fare_money, pas->money);
      LCD1602I2C_SendString((char*)s);
      break;
    case RWERROR:
      LCD1602I2C_Cursor(0, 0);
      Center_Align_String((char*)"Card Error", 16);
      LCD1602I2C_SendString((char*)s);    
      LCD1602I2C_Cursor(1, 0);
      Center_Align_String((char*)"Please Try Again", 16);
      LCD1602I2C_SendString((char*)s);
      break;
    case INSUFFICIENT:
      LCD1602I2C_Cursor(0, 0);
      sprintf((char*)s, "Not Enough Money");
      LCD1602I2C_SendString((char*)s);  
      LCD1602I2C_Cursor(1, 0);
      sprintf((char*)s, "balance %-8d", pas->money);
      LCD1602I2C_SendString((char*)s);
      break;
    case ROUTEERROR:
      LCD1602I2C_Cursor(0, 0);
      Center_Align_String("Error Route", 16);
      LCD1602I2C_SendString((char*)s);
      LCD1602I2C_Cursor(1, 0);
      Center_Align_String("Can't Get Off", 16);
      LCD1602I2C_SendString((char*)s);
      break;
    case WAITUPDATE:
      LCD1602I2C_Cursor(0, 0);
      Center_Align_String("Update Route", 16);
      LCD1602I2C_SendString((char*)s);
      LCD1602I2C_Cursor(1, 0);
      Center_Align_String("", 16);
      LCD1602I2C_SendString((char*)s);
      break;
    case STOPSERVICE:
      LCD1602I2C_Cursor(0, 0);
      Center_Align_String("Stop Service", 16);
      LCD1602I2C_SendString((char*)s);
      LCD1602I2C_Cursor(1, 0);
      Center_Align_String("", 16);
      LCD1602I2C_SendString((char*)s);
      break;
    case WAITINITIAL:
      LCD1602I2C_Cursor(0, 0);
      Center_Align_String("Please", 16);
      LCD1602I2C_SendString((char*)s);
      LCD1602I2C_Cursor(1, 0);
      Center_Align_String("Initialize Route", 16);
      LCD1602I2C_SendString((char*)s);
      break;
    case ROUTESTATION:
      LCD1602I2C_Cursor(0, 0);
      Center_Align_String((char*)route_name, ROUTE_NAME_SIZE);
      sprintf((char*)s, "%s%c", (char*)s, direction);
      LCD1602I2C_SendString((char*)s);
      LCD1602I2C_Cursor(1, 0);
      Center_Align_String((char*)&route[now_station*STATION_SIZE], STATION_SIZE);
      LCD1602I2C_SendString((char*)s);
  }

}

void Print_Passenger(Passenger *pas){
  printf("name:       %s\r\n",   pas->name);
  printf("money:      %d\r\n",   pas->money);
  printf("type:       %02X\r\n", pas->type);
  printf("get on:     %d\r\n",   pas->get_on_station);
  printf("direction:  %02X\r\n", pas->get_on_station);
  printf("route name: %s\r\n",   pas->route_name);      
}

void Update_Route(void){
  direction = DEPARTURE;
  now_station = 0;

  HAL_UART_Receive(&huart2, s, 4, HAL_MAX_DELAY); //receive int how many station on route to update
  station_n = (uint32_t)s[0] << 24 | (uint32_t)s[1] << 16 | (uint32_t)s[2] << 8 | (uint32_t)s[3]; // consist of 4 bytes 0~3, 0 bytes is MSB.
  HAL_UART_Transmit(&huart2, s, 4, HAL_MAX_DELAY); //send echo
  
  HAL_UART_Receive(&huart2, route_name, ROUTE_NAME_SIZE, HAL_MAX_DELAY);  //receive route name
  HAL_UART_Transmit(&huart2, route_name, ROUTE_NAME_SIZE, HAL_MAX_DELAY); //send echo
      
  //start receive new route
  free(route);
  route = (uint8_t*)malloc(station_n * STATION_SIZE * sizeof(uint8_t)); //reallocate route memory 
  for(int i=0; i<station_n; i++){
    HAL_UART_Receive(&huart2, &route[i*STATION_SIZE], STATION_SIZE, HAL_MAX_DELAY);  //receive station string
    HAL_UART_Transmit(&huart2, &route[i*STATION_SIZE], STATION_SIZE, HAL_MAX_DELAY); //send echo
  }
  //end receive new route
     
  //start receive new fare
  HAL_UART_Receive(&huart2, s, 4, HAL_MAX_DELAY); //receive base fare
  base_fare = (uint32_t)s[0] << 24 | (uint32_t)s[1] << 16 | (uint32_t)s[2] << 8 | (uint32_t)s[3]; // consist of 4 bytes 0~3, 0 bytes is MSB.
  HAL_UART_Transmit(&huart2, s, 4, HAL_MAX_DELAY); //send echo
    
  free(fare);
  fare = (int*)malloc(station_n * station_n * sizeof(int)); //reallocate fare memory 
  for(int i=0;i<station_n;i++){
    for(int j=0;j<station_n;j++){
      HAL_UART_Receive(&huart2, s, 4, HAL_MAX_DELAY);  //receive fare number(int type 4 bytes) to s buffer
      // decode buffer to int value, consist of 4 bytes 0~3, 0 bytes is MSB.
      fare[i*station_n+j] = (uint32_t)s[0] << 24 | (uint32_t)s[1] << 16 | (uint32_t)s[2] << 8 | (uint32_t)s[3];
      HAL_UART_Transmit(&huart2, s, 4, HAL_MAX_DELAY); //send echo
    }
  }
  //end receive new fare
      
  HAL_UART_Receive(&huart2, s, 4, HAL_MAX_DELAY); //receive base half fare
  base_half_fare = (uint32_t)s[0] << 24 | (uint32_t)s[1] << 16 | (uint32_t)s[2] << 8 | (uint32_t)s[3]; // consist of 4 bytes 0~3, 0 bytes is MSB.
  HAL_UART_Transmit(&huart2, s, 4, HAL_MAX_DELAY); //send echo
  //start receive new half fare
  free(half_fare);
  half_fare = (int*)malloc(station_n * station_n * sizeof(int)); //reallocate half fare memory 
  for(int i=0;i<station_n;i++){
    for(int j=0;j<station_n;j++){
      HAL_UART_Receive(&huart2, s, 4, HAL_MAX_DELAY);  //receive half fare number(int type 4 bytes) to s buffer
      // decode buffer to int value, consist of 4 bytes 0~3, 0 bytes is MSB.
      half_fare[i*station_n+j] = (uint32_t)s[0] << 24 | (uint32_t)s[1] << 16 | (uint32_t)s[2] << 8 | (uint32_t)s[3];
      HAL_UART_Transmit(&huart2, s, 4, HAL_MAX_DELAY); //send echo
    }
  }
  //end receive new fare
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){  // timer2 preemption priority 0
  if(htim == &htim2)
    debounce_wait_flag = false;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){  // button exti preemption priority 1, lower timer2 so that timer2 can preempt
  debounce_wait_flag = true;

  __HAL_TIM_SetCounter(&htim2, 0);
  HAL_TIM_Base_Start_IT(&htim2);
  while(debounce_wait_flag){} // wait 100 milliseconds
  HAL_TIM_Base_Stop_IT(&htim2);
    
  if(GPIO_Pin == UPDATE_ROUTE_Pin){
    if(HAL_GPIO_ReadPin(UPDATE_ROUTE_GPIO_Port, UPDATE_ROUTE_Pin) == GPIO_PIN_RESET)
      return;
    update_flag = true;
  }else if(GPIO_Pin == NEXT_STATION_Pin){
    if(HAL_GPIO_ReadPin(NEXT_STATION_GPIO_Port, NEXT_STATION_Pin) == GPIO_PIN_RESET)
      return;
    next_flag = true;
  }else if(GPIO_Pin == REVERSE_ROUTE_Pin){
    if(HAL_GPIO_ReadPin(REVERSE_ROUTE_GPIO_Port, REVERSE_ROUTE_Pin) == GPIO_PIN_RESET)
      return;
    reverse_flag = true;
  }else if(GPIO_Pin == STOP_SERVICE_Pin){
    if(HAL_GPIO_ReadPin(STOP_SERVICE_GPIO_Port, STOP_SERVICE_Pin) == GPIO_PIN_RESET)
      return;
    stop_flag = !stop_flag;
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
