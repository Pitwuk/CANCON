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
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "LCD1602.h"
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
CAN_HandleTypeDef hcan;

SDADC_HandleTypeDef hsdadc1;
SDADC_HandleTypeDef hsdadc2;
SDADC_HandleTypeDef hsdadc3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
//CAN VARIABLES
uint32_t* tx_mailbox; // Tx Mailbox for CAN messages
CAN_FilterTypeDef can_filter; // CAN filter structure
CAN_TxHeaderTypeDef headers_1, headers_2, headers_3; // CAN headers
uint8_t PSC=2; //Prescaler for CAN Baud

uint8_t can_id_1, can_id_2, can_id_3;//CAN IDs of analog devices
uint8_t id_arr[4];//array of can ids
uint32_t *CAN_IDs;//32 bit int of can ids
uint8_t analog_enable_arr[4];// array used to store if analog values are enabled
uint32_t *enable_word;//32 bit int of enable bools
char modified_can_id[3];//can id of the can signal being modified

//ANALOG VARIABLES
//booleans to track which devices are enabled
uint8_t analog_1_enabled=1;
uint8_t analog_2_enabled=1;
uint8_t analog_3_enabled=1;

//analog offsets
uint32_t analog_1_offset=0;
uint32_t analog_2_offset=0;
uint32_t analog_3_offset=0;

uint8_t store_offsets=0;//boolean used to store the offsets


int32_t raw_1, raw_2, raw_3; //raw analog data from the sensors
int32_t a_in_1, a_in_2, a_in_3; //adjusted analog data from the sensors
uint8_t a_data_1[2], a_data_2[2], a_data_3[2]; //analog data in a byte array
uint16_t data_arr_length = 2000;
uint16_t a_data_arr_1[2000], a_data_arr_2[2000], a_data_arr_3[2000]; // data array of values collected between sends
uint16_t a_1_index = 0; // index in data array
uint16_t a_2_index = 0; // index in data array
uint16_t a_3_index = 0; // index in data array
uint16_t for_index = 0; // index used in for loops
uint32_t max = 0; // max index in for loop


//DISPLAY VARIABLES
//menu arrays
char main_menu[4][17] = {"CAN Bus Config  ", "Analog Config   ", "Display Values  ", "Delay:          "};
char can_menu[6][17] = {"Back            ", "Baud rate:      ", "CAN ID 1:    001", "CAN ID 2:    002", "CAN ID 3:    003", "Reset CAN Config"};
char analog_menu[10][17] = {"Back            ", "Zero All        ", "Zero Analog 1   ", "Zero Analog 2    ", "Zero Analog 3    ", "Analog 1:     ON",
		"Analog 2:     ON", "Analog 3:     ON", "Reset Offsets   ", "Store Offsets   "};
//baud rates to cycle through
const char baud_rates[4][4] = {"125k", "250k", "500k", "  1M"};

//booleans to track current menu
uint8_t in_main_menu = 1;
uint8_t in_can_menu = 0;
uint8_t in_analog_menu = 0;

uint8_t display_scroll = 0;//int for scrolling through devices in display menu

//menu vars
const int main_menu_length=(sizeof(main_menu)/sizeof(main_menu[0]));//the number of strings in the main menu
const int can_menu_length=(sizeof(can_menu)/sizeof(can_menu[0]));//the number of strings in the can config menu
const int analog_menu_length=(sizeof(analog_menu)/sizeof(analog_menu[0]));//the number of strings in the analog config menu
const int baud_rates_length=(sizeof(baud_rates)/sizeof(baud_rates[0]));//the number of baud rates to select from
char temp[17]; //temporary array used for copying
char char_arr[16];//second array used for copying

int8_t menu_pos = 0; //main menu position
int8_t can_pos = 0; //can menu position
int8_t analog_pos = 0; //analog menu position
int8_t baud_pos = 3; //baud_rate array position


//MISC VARIABLES
uint8_t byte_arr[4];//used in wordToByte for storing the resulting bytes

int16_t us = 300;//delay between loops in microseconds
uint8_t num_delays = 0;//the number of loops in between samples

uint8_t change_value_bool = 0; //boolean for changing the value the selected item
uint16_t btn_counter=0;//button counter used to debounce buttons
uint16_t debounce_delay = 1000; // number of delays for debounce

uint8_t debug_val=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_SDADC1_Init(void);
static void MX_SDADC2_Init(void);
static void MX_SDADC3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void changeBaudRate(uint8_t direction);
void setBaudRate(void);
void changeCANID(uint8_t direction);
void setCANID(void);
void resetCAN(void);
void changeDelay(uint16_t direction);
void setDelay(void);
void zeroAnalog(uint8_t analog_id);
void resetOffsets(void);
void toggleAnalog(uint8_t analog_id);
void displayValues(void);
uint32_t bytesToWord(uint8_t* arr);
uint8_t* wordToBytes(uint32_t value);
void storeInFlash(void);
void calibrateSDADC(SDADC_HandleTypeDef* adc, uint32_t channel);

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
	//Get variables from Flash Memory

	CAN_IDs=(*(__IO uint32_t *) 0x0800F800);
	memcpy(id_arr, wordToBytes(CAN_IDs), 4);
	can_id_1=id_arr[0];
	can_id_2=id_arr[1];
	can_id_3=id_arr[2];
	//if duplicate IDs set IDs to 1, 2, and 3 respectively
	if(can_id_1==can_id_2 || can_id_1==can_id_3||can_id_2==can_id_3){
		can_id_1=1;
		can_id_2=2;
		can_id_3=3;
	}

	//get the baud rate
	baud_pos=(uint8_t)(*(__IO uint32_t *) 0x0800F804);
	//if invalid baud rate then set to 1M
	if(baud_pos<0||baud_pos>3)
		baud_pos=3;
	PSC=pow(2,4-baud_pos);//set prescaler

	//get the number of delays between samples
	num_delays=(uint8_t)(*(__IO uint32_t *) 0x0800F808);
	if(num_delays<(uint8_t)0)
		num_delays=0;

	// retrieve the analog enabled booleans
	enable_word=(*(__IO uint32_t *) 0x0800F80C);
	memcpy(analog_enable_arr, wordToBytes(enable_word), 4);
	//invert values so the default is on after erasing the memory
	analog_1_enabled=!analog_enable_arr[0];
	analog_2_enabled=!analog_enable_arr[1];
	analog_3_enabled=!analog_enable_arr[2];

	//get the offsets
	analog_1_offset=(*(__IO uint32_t *) 0x0800F810);
	analog_2_offset=(*(__IO uint32_t *) 0x0800F814);
	analog_3_offset=(*(__IO uint32_t *) 0x0800F818);


	// Set the headers for the first analog device
	headers_1.StdId = can_id_1; // set the CAN ID
	headers_1.IDE = CAN_ID_STD;
	headers_1.RTR = CAN_RTR_DATA;
	headers_1.DLC = sizeof(a_data_1); // set the size of the data
	headers_1.TransmitGlobalTime = DISABLE; // disable transmission of time

	// Set the headers for the second analog device
	headers_2.StdId = can_id_2; // set the CAN ID
	headers_2.IDE = CAN_ID_STD;
	headers_2.RTR = CAN_RTR_DATA;
	headers_2.DLC = sizeof(a_data_2); // set the size of the data
	headers_2.TransmitGlobalTime = DISABLE; // disable transmission of time

	// Set the headers for the third analog device
	headers_3.StdId = can_id_3; // set the CAN ID
	headers_3.IDE = CAN_ID_STD;
	headers_3.RTR = CAN_RTR_DATA;
	headers_3.DLC = sizeof(a_data_3); // set the size of the data
	headers_3.TransmitGlobalTime = DISABLE; // disable transmission of time


	//Save the stored values to the menus
	//set the baud rate
	memset(temp,0,17); // erase the temp array
	strncpy(temp, "Baud rate:  ",16); // copy the string to the temp array (limiting the length to 16 characters)
	strlcat(temp,baud_rates[baud_pos],17);
	strlcpy(can_menu[1],temp,17);

	//set can IDs
	memset(temp,0,17); // erase the temp array
	strncpy(temp, "CAN ID ",16); // copy the string to the temp array (limiting the length to 16 characters)
	strcat(temp, "1:    ");
	memset(char_arr, 0, 16);
	sprintf(char_arr, "%03d", can_id_1);
	strlcat(temp, char_arr,17);
	memcpy(can_menu[2],temp,17);
	//can id 2
	memset(temp,0,17); // erase the temp array
	strncpy(temp, "CAN ID ",16); // copy the string to the temp array (limiting the length to 16 characters)
	strcat(temp, "2:    ");
	memset(char_arr, 0, 16);
	sprintf(char_arr, "%03d", can_id_2);
	strlcat(temp, char_arr,17);
	memcpy(can_menu[3],temp,17);
	//can id 3
	memset(temp,0,17); // erase the temp array
	strncpy(temp, "CAN ID ",16); // copy the string to the temp array (limiting the length to 16 characters)
	strcat(temp, "3:    ");
	memset(char_arr, 0, 16);
	sprintf(char_arr, "%03d", can_id_3);
	strlcat(temp, char_arr,17);
	memcpy(can_menu[4],temp,17);

	//Delay
	memset(temp,0,17); // erase the temp array
	strncpy(temp, "Delay:   ",16); // copy the string to the temp array (limiting the length to 16 characters)
	memset(char_arr, 0, 16);
	sprintf(char_arr, "%05d", us*(num_delays+1));
	strcat(temp, char_arr);
	strlcat(temp,"us",17);
	memcpy(main_menu[3],temp,17);

	//Analog enabled bools
	if(!analog_1_enabled)
		memcpy(analog_menu[5],"Analog 1:    OFF",17);
	if(!analog_2_enabled)
		memcpy(analog_menu[6],"Analog 2:    OFF",17);
	if(!analog_3_enabled)
		memcpy(analog_menu[7],"Analog 3:    OFF",17);
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
  MX_CAN_Init();
  MX_SDADC1_Init();
  MX_SDADC2_Init();
  MX_SDADC3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);//start timer

	//initialize LCD
	lcd_init();
	lcd_put_cur(0,0);
	lcd_send_string("Starting Up...");
	HAL_Delay(1000);
	lcd_put_cur(0,0);
	lcd_send_string("Initializing CAN");
	HAL_Delay(1000);

	//setup CAN filter
	can_filter.FilterMaskIdHigh = 0x0000;
	can_filter.FilterMaskIdLow = 0x0000;
	can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	can_filter.FilterBank = 0;
	can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter.FilterActivation = CAN_FILTER_ENABLE;
	//check that CAN Filter was setup correctly
	if (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK) {
		Error_Handler();
	}

	//start the can protocol and verify that it has started
	if (HAL_CAN_Start(&hcan) != HAL_OK) {
		Error_Handler();
	}

	//display main menu
	lcd_put_cur(0,0);
	//the string is copied to temp to remove the null character at the end of the string in the menu
	lcd_send_string(strncpy(temp,main_menu[menu_pos],16));
	lcd_put_cur(1,0);
	lcd_send_string(strncpy(temp,main_menu[menu_pos+1],16));
	lcd_put_cur(0,0);

	int display_counter=0;//counter used in updating the display
	int16_t timer_compensation=0;//compensation for when the operations within the loop take too long

	//calibrate the SDADCs
	calibrateSDADC(&hsdadc1, SDADC_CHANNEL_1);
	calibrateSDADC(&hsdadc2, SDADC_CHANNEL_0);
	calibrateSDADC(&hsdadc3, SDADC_CHANNEL_4);
	HAL_Delay(300);

	//start ADC conversion interrupts
	if(analog_1_enabled)
		HAL_SDADC_Start_IT(&hsdadc1);
	if(analog_2_enabled)
		HAL_SDADC_Start_IT(&hsdadc2);
	if(analog_3_enabled)
		HAL_SDADC_Start_IT(&hsdadc3);

	// start can timer
	TIM3->ARR = ((num_delays+1)*us) - 1;
	HAL_TIM_Base_Start_IT(&htim3);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		//start ADC conversion interrupts
		if(analog_1_enabled)
			HAL_SDADC_Start_IT(&hsdadc1);
		if(analog_2_enabled)
			HAL_SDADC_Start_IT(&hsdadc2);
		if(analog_3_enabled)
			HAL_SDADC_Start_IT(&hsdadc3);


		//Delay between samples
		if ((int16_t)__HAL_TIM_GET_COUNTER(&htim2) >= us);
			__HAL_TIM_SET_COUNTER(&htim2,0); // reset timer
			//reset display counter (waits 200 delays between updates to the display in the display values menu)
			if(display_counter>200)
				display_counter=0;

			//Display analog values on display
			if(in_main_menu && menu_pos==2 && change_value_bool && display_counter==0)
				displayValues();

			//increment counters
			display_counter++;
			btn_counter++;


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDADC;
  PeriphClkInit.SdadcClockSelection = RCC_SDADCSYSCLK_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG1);
  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG2);
  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG3);
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
	//set the prescaler to the right value for the stored baud rate
	hcan.Init.Prescaler = PSC;
	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		Error_Handler();
	}

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SDADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDADC1_Init(void)
{

  /* USER CODE BEGIN SDADC1_Init 0 */

  /* USER CODE END SDADC1_Init 0 */

  SDADC_ConfParamTypeDef ConfParamStruct = {0};

  /* USER CODE BEGIN SDADC1_Init 1 */

  /* USER CODE END SDADC1_Init 1 */
  /** Configure the SDADC low power mode, fast conversion mode,
  slow clock mode and SDADC1 reference voltage
  */
  hsdadc1.Instance = SDADC1;
  hsdadc1.Init.IdleLowPowerMode = SDADC_LOWPOWER_NONE;
  hsdadc1.Init.FastConversionMode = SDADC_FAST_CONV_DISABLE;
  hsdadc1.Init.SlowClockMode = SDADC_SLOW_CLOCK_DISABLE;
  hsdadc1.Init.ReferenceVoltage = SDADC_VREF_EXT;
  if (HAL_SDADC_Init(&hsdadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Set parameters for SDADC configuration 0 Register
  */
  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_ZERO_REFERENCE;
  ConfParamStruct.Gain = SDADC_GAIN_1;
  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VSSA;
  ConfParamStruct.Offset = 0;
  if (HAL_SDADC_PrepareChannelConfig(&hsdadc1, SDADC_CONF_INDEX_0, &ConfParamStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDADC1_Init 2 */

  /* USER CODE END SDADC1_Init 2 */

}

/**
  * @brief SDADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDADC2_Init(void)
{

  /* USER CODE BEGIN SDADC2_Init 0 */

  /* USER CODE END SDADC2_Init 0 */

  SDADC_ConfParamTypeDef ConfParamStruct = {0};

  /* USER CODE BEGIN SDADC2_Init 1 */

  /* USER CODE END SDADC2_Init 1 */
  /** Configure the SDADC low power mode, fast conversion mode,
  slow clock mode and SDADC1 reference voltage
  */
  hsdadc2.Instance = SDADC2;
  hsdadc2.Init.IdleLowPowerMode = SDADC_LOWPOWER_NONE;
  hsdadc2.Init.FastConversionMode = SDADC_FAST_CONV_DISABLE;
  hsdadc2.Init.SlowClockMode = SDADC_SLOW_CLOCK_DISABLE;
  hsdadc2.Init.ReferenceVoltage = SDADC_VREF_EXT;
  if (HAL_SDADC_Init(&hsdadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Set parameters for SDADC configuration 0 Register
  */
  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_ZERO_REFERENCE;
  ConfParamStruct.Gain = SDADC_GAIN_1;
  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VDDA;
  ConfParamStruct.Offset = 0;
  if (HAL_SDADC_PrepareChannelConfig(&hsdadc2, SDADC_CONF_INDEX_0, &ConfParamStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDADC2_Init 2 */

  /* USER CODE END SDADC2_Init 2 */

}

/**
  * @brief SDADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDADC3_Init(void)
{

  /* USER CODE BEGIN SDADC3_Init 0 */

  /* USER CODE END SDADC3_Init 0 */

  SDADC_ConfParamTypeDef ConfParamStruct = {0};

  /* USER CODE BEGIN SDADC3_Init 1 */

  /* USER CODE END SDADC3_Init 1 */
  /** Configure the SDADC low power mode, fast conversion mode,
  slow clock mode and SDADC1 reference voltage
  */
  hsdadc3.Instance = SDADC3;
  hsdadc3.Init.IdleLowPowerMode = SDADC_LOWPOWER_NONE;
  hsdadc3.Init.FastConversionMode = SDADC_FAST_CONV_DISABLE;
  hsdadc3.Init.SlowClockMode = SDADC_SLOW_CLOCK_DISABLE;
  hsdadc3.Init.ReferenceVoltage = SDADC_VREF_EXT;
  if (HAL_SDADC_Init(&hsdadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Set parameters for SDADC configuration 0 Register
  */
  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_ZERO_REFERENCE;
  ConfParamStruct.Gain = SDADC_GAIN_1;
  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VSSA;
  ConfParamStruct.Offset = 0;
  if (HAL_SDADC_PrepareChannelConfig(&hsdadc3, SDADC_CONF_INDEX_0, &ConfParamStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDADC3_Init 2 */

  /* USER CODE END SDADC3_Init 2 */

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
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 300-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC9 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
//read adc values
void HAL_SDADC_ConvCpltCallback(SDADC_HandleTypeDef* hsdadc)
{
	//get analog 1 data
	if(hsdadc==&hsdadc1){
		//get raw data for analog 1
		raw_1 = (int16_t)HAL_SDADC_GetValue(&hsdadc1);
		raw_1+=32768;
		//convert and scale raw data
		a_in_1=(raw_1-analog_1_offset);
		if(a_in_1<0)
			a_in_1=0;
		a_data_arr_1[a_1_index%data_arr_length]=(uint16_t)(a_in_1*((65535)/(float)(65535-analog_1_offset)));//scale value
		a_1_index++;
	}


	//get analog 2 data
	if(hsdadc==&hsdadc2){
		//get raw data for analog 2
		raw_2 = (int16_t)HAL_SDADC_GetValue(&hsdadc2);
		raw_2+=32768;
		//convert and scale raw data
		a_in_2=(raw_2-analog_2_offset);
		if(a_in_2<0)
			a_in_2=0;
		a_data_arr_2[a_2_index%data_arr_length]=(uint16_t)(a_in_2*((65535)/(float)(65535-analog_2_offset)));//scale value
		a_2_index++;
	}

	//get analog 3 data
	if(hsdadc==&hsdadc3){
		//get raw data for analog 3
		raw_3 = (int16_t)HAL_SDADC_GetValue(&hsdadc3);
		raw_3+=32768;
		//convert and scale raw data
		a_in_3=(raw_3-analog_3_offset);
		if(a_in_3<0)
			a_in_3=0;
		a_data_arr_3[a_3_index%data_arr_length]=(uint16_t)(a_in_3*((65535)/(float)(65535-analog_3_offset)));//scale value
		a_3_index++;
	}
}
//sends data over can when timer 3 interrupts
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback
  if (htim == &htim3)
  {
    if(analog_1_enabled){
    	//average stored values
    	a_in_1=0;
    	max = (a_1_index<data_arr_length?a_1_index:data_arr_length);
    	if(max>0){
			for(for_index=0; for_index<max; for_index++)
				a_in_1+=a_data_arr_1[for_index];
			a_in_1/=max;
			a_1_index=0;
    	}

    	//put raw data into byte arrays
    	a_data_1[0]=a_in_1 & 0xff;
    	a_data_1[1]=(a_in_1 >> 8);
    	//transmit CAN data for analog 1
    	HAL_CAN_AddTxMessage(&hcan, &headers_1, a_data_1, *tx_mailbox);
    	while (HAL_CAN_IsTxMessagePending(&hcan, *tx_mailbox));//wait until data is sent for analog 1
    }
    if(analog_2_enabled){
    	a_in_2=0;
		max = (a_2_index<data_arr_length?a_2_index:data_arr_length);
		if(max>0){
			for(for_index=0; for_index<max; for_index++)
				a_in_2+=a_data_arr_2[for_index];
			a_in_2/=max;
			a_2_index=0;
		}
    	//put raw data into byte arrays
    	a_data_2[0]=a_in_2 & 0xff;
    	a_data_2[1]=(a_in_2 >> 8);
    	//transmit CAN data for analog 2
    	HAL_CAN_AddTxMessage(&hcan, &headers_2, a_data_2, *tx_mailbox);
    	while (HAL_CAN_IsTxMessagePending(&hcan, *tx_mailbox));//wait until data is sent for analog 2
    }
    if(analog_3_enabled){
    	a_in_3=0;
		max = (a_3_index<data_arr_length?a_3_index:data_arr_length);
		if(max>0){
			for(for_index=0; for_index<max; for_index++)
				a_in_3+=a_data_arr_3[for_index];
			a_in_3/=max;
			a_3_index=0;
		}
		//put raw data into byte arrays
		a_data_3[0]=a_in_3 & 0xff;
		a_data_3[1]=(a_in_3 >> 8);
		//transmit CAN data for analog 3
		HAL_CAN_AddTxMessage(&hcan, &headers_3, a_data_3, *tx_mailbox);
		while (HAL_CAN_IsTxMessagePending(&hcan, *tx_mailbox));//wait until data is sent for analog 3
    }
  }
}
//handles the button interupts
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	debug_val++;
	//if the up button is pressed
			if(GPIO_Pin==UP_BTN_Pin && btn_counter>debounce_delay){
				if(change_value_bool){//if changing a value
					//change the value of the selected item
					//main menu changes
					if(in_main_menu){
						if(menu_pos==2)
							display_scroll=(display_scroll==0)?2:(display_scroll-1);//scroll through devices in display values menu
						if(menu_pos == 3)
							changeDelay(1);//increment Delay
					}
					//can menu changes
					if(in_can_menu){
						if(can_pos==1)
							changeBaudRate(1);//increment baud rate
						else if(can_pos>=2&&can_pos<=4)
							changeCANID(1);//increment CAN ID
					}

				} else {
					//scroll menu up
					if(in_main_menu){
						menu_pos=(menu_pos==0)?main_menu_length-1:(menu_pos-1);//decrement menu position
						lcd_put_cur(0,0);
						lcd_send_string(strncpy(temp,main_menu[menu_pos],16));
						lcd_put_cur(1,0);
						lcd_send_string(strncpy(temp,main_menu[(menu_pos+1)%main_menu_length],16));
						lcd_put_cur(0,0);
					} else if(in_can_menu){
						can_pos=(can_pos==0)?can_menu_length-1:(can_pos-1);//decrement menu position
						lcd_put_cur(0,0);
						lcd_send_string(strncpy(temp,can_menu[can_pos],16));
						lcd_put_cur(1,0);
						lcd_send_string(strncpy(temp,can_menu[(can_pos+1)%can_menu_length],16));
						lcd_put_cur(0,0);
					} else if(in_analog_menu){
						analog_pos=(analog_pos==0)?analog_menu_length-1:(analog_pos-1);//decrement menu position
						lcd_put_cur(0,0);
						lcd_send_string(strncpy(temp,analog_menu[analog_pos],16));
						lcd_put_cur(1,0);
						lcd_send_string(strncpy(temp,analog_menu[(analog_pos+1)%analog_menu_length],16));
						lcd_put_cur(0,0);
					}
				}
				btn_counter=0; // reset btn counter
			}

			//if the down button is pressed
			else if(GPIO_Pin==DOWN_BTN_Pin && btn_counter>debounce_delay){
				if(change_value_bool){//if changing a value
					//change the value of the selected item
					//main menu changes
					if(in_main_menu){
						if(menu_pos==2)
							display_scroll=(display_scroll+1)%3;//scroll through devices in display values menu
						if(menu_pos == 3)
							changeDelay(-1);//decrement Delay
					}
					//can menu changes
					if(in_can_menu){
						if(can_pos==1)
							changeBaudRate(-1);//decrement baud rate
						else if(can_pos>=2&&can_pos<=4)
							changeCANID(-1);//decrement CAN ID
					}
				} else {
					//scroll menu down
					if(in_main_menu){
						menu_pos = (menu_pos+1)%main_menu_length;//increment menu position
						lcd_put_cur(0,0);
						lcd_send_string(strncpy(temp,main_menu[menu_pos],16));
						lcd_put_cur(1,0);
						lcd_send_string(strncpy(temp,main_menu[(menu_pos+1)%main_menu_length],16));
						lcd_put_cur(0,0);
					} else if(in_can_menu){
						can_pos=(can_pos+1)%can_menu_length;//increment menu position
						lcd_put_cur(0,0);
						lcd_send_string(strncpy(temp,can_menu[can_pos],16));
						lcd_put_cur(1,0);
						lcd_send_string(strncpy(temp,can_menu[(can_pos+1)%can_menu_length],16));
						lcd_put_cur(0,0);
					} else if(in_analog_menu){
						analog_pos=(analog_pos+1)%analog_menu_length;//increment menu position
						lcd_put_cur(0,0);
						lcd_send_string(strncpy(temp,analog_menu[analog_pos],16));
						lcd_put_cur(1,0);
						lcd_send_string(strncpy(temp,analog_menu[(analog_pos+1)%analog_menu_length],16));
						lcd_put_cur(0,0);
					}

				}
				btn_counter=0; // reset btn counter
			}

			//if the select button is pressed
			else if(GPIO_Pin==SEL_BTN_Pin && btn_counter>debounce_delay){
				//set changes and reinitialize can bus
				if(!change_value_bool){
					if(in_main_menu){
						if(menu_pos==0){
							//display can menu
							lcd_put_cur(0,0);
							lcd_send_string(strncpy(temp,can_menu[can_pos],16));
							lcd_put_cur(1,0);
							lcd_send_string(strncpy(temp,can_menu[(can_pos+1)%can_menu_length],16));
							lcd_put_cur(0,0);
							//update menu booleans
							in_can_menu=1;
							in_main_menu=0;
							change_value_bool=!change_value_bool;//invert change value bool
						} else if(menu_pos==1){
							//display can menu
							lcd_put_cur(0,0);
							lcd_send_string(strncpy(temp,analog_menu[analog_pos],16));
							lcd_put_cur(1,0);
							lcd_send_string(strncpy(temp,analog_menu[(analog_pos+1)%analog_menu_length],16));
							lcd_put_cur(0,0);
							//update menu booleans
							in_analog_menu=1;
							in_main_menu=0;
							change_value_bool=!change_value_bool;//invert change value bool
						} else if (menu_pos==2){
							display_scroll=0;// reset display value menu position
						} else if (menu_pos==3){
							//move cursor for delay change
							lcd_put_cur(0,13);
						}
					} else if((in_can_menu&&can_pos==0)||(in_analog_menu&&analog_pos==0)){// back buttons
						//display main menu
						lcd_put_cur(0,0);
						lcd_send_string(strncpy(temp,main_menu[menu_pos],16));
						lcd_put_cur(1,0);
						lcd_send_string(strncpy(temp,main_menu[(menu_pos+1)%main_menu_length],16));
						lcd_put_cur(0,0);
						//update menu booleans
						in_can_menu=0;
						in_analog_menu=0;
						in_main_menu=1;
						change_value_bool=!change_value_bool;//invert change value bool
					} else if (in_analog_menu){
						if(analog_pos>=1 && analog_pos<=4){
							zeroAnalog(analog_pos-1);// set the analog offsets to the current value
							change_value_bool=!change_value_bool;//invert change value bool
						}else if(analog_pos>=5 && analog_pos<=7){
							toggleAnalog(analog_pos-4);
							change_value_bool=!change_value_bool;//invert change value bool
						}else if(analog_pos==8){
							resetOffsets();// reset the analog offsets
							change_value_bool =!change_value_bool;//invert change value bool
						}else if(analog_pos==9){
							store_offsets=1;
							storeInFlash();// store the analog offsets
							change_value_bool=!change_value_bool;//invert change value bool
						}

					} else if (in_can_menu){
						if(can_pos==5){
							resetCAN();// reset the CAN options to default
							change_value_bool=!change_value_bool;//invert change value bool
						}
					} else{
						//move cursor to end
						lcd_put_cur(0,15);
					}

				}else{
					if(in_main_menu){
						if(menu_pos==2){ //display menu
							lcd_put_cur(0,0);
							lcd_send_string(strncpy(temp,main_menu[menu_pos],16));
							lcd_put_cur(1,0);
							lcd_send_string(strncpy(temp,main_menu[(menu_pos+1)%main_menu_length],16));
							lcd_put_cur(0,0);
						}
						else if(menu_pos==3)
							setDelay();//set Delay
					}
					else if(in_can_menu){
						if(can_pos==1)
							setBaudRate(); // set and store the selected baud rate
						else if(can_pos>=2 && can_pos<=4)
							setCANID(); // set and store the selected can id
					}

				}
				change_value_bool=!change_value_bool;//invert change value bool
				btn_counter=0; // reset btn counter
			}
}


//changes the baud rate of the CAN connection
void changeBaudRate(uint8_t direction){
	baud_pos = ((baud_pos+direction)<0)?baud_rates_length-1:(baud_pos+direction)%baud_rates_length;//move baud rate position in given direction

	//display the selected baud rate
	lcd_put_cur(0,12);
	lcd_send_string(baud_rates[baud_pos]);
	lcd_put_cur(0,15);
}

//set the baud rate and store it
void setBaudRate(void){
	//stop can bus
	HAL_CAN_Stop(&hcan);

	//set prescaler
	PSC=pow(2,4-baud_pos);
	hcan.Init.Prescaler = PSC;

	//reinitialize can
	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		Error_Handler();
	}

	//start the can protocol and verify that it has started
	if (HAL_CAN_Start(&hcan) != HAL_OK) {
		Error_Handler();
	}

	//store new baud in flash
	storeInFlash();

	//save the baud rate in menu
	memset(temp,0,17); // erase the temp array
	strncpy(temp, "Baud rate:  ",16);
	strlcat(temp,baud_rates[baud_pos],17);
	strlcpy(can_menu[1],temp,17);
	lcd_put_cur(0,0);
}

//changes the ID of the selected can device
void changeCANID(uint8_t direction){
	if(can_pos==2){//change can id 1
		can_id_1+=direction;
		while(can_id_1==can_id_2 || can_id_1==can_id_3)//avoid conflicting IDs
			can_id_1+=direction;
		sprintf(char_arr, "%03d", can_id_1);
		memcpy(modified_can_id, char_arr, 3);//store new can id in modified_can_id
	} else if(can_pos==3){//change can id 2
		can_id_2+=direction;
		while(can_id_2==can_id_1 || can_id_2==can_id_3)//avoid conflicting IDs
			can_id_2+=direction;
		sprintf(char_arr, "%03d", can_id_2);
		memcpy(modified_can_id, char_arr, 3);//store new can id in modified_can_id
	} else if(can_pos==4){// change can id 3
		can_id_3+=direction;
		while(can_id_3==can_id_1 || can_id_3==can_id_2)//avoid conflicting IDs
			can_id_3+=direction;
		sprintf(char_arr, "%03d", can_id_3);
		memcpy(modified_can_id, char_arr, 3);//store new can id in modified_can_id
	}
	//display the selected CAN ID
	lcd_put_cur(0,13);
	lcd_send_string("    ");
	lcd_put_cur(0,13);
	lcd_send_string(modified_can_id);
	lcd_put_cur(0,15);
}

//set the can IDs
void setCANID(void){
	headers_1.StdId = can_id_1; // set CAN ID 1
	headers_2.StdId = can_id_2; // set CAN ID 2
	headers_3.StdId = can_id_3; // set CAN ID 3

	//Store the CAN IDs
	id_arr[0]=can_id_1;
	id_arr[1]=can_id_2;
	id_arr[2]=can_id_3;
	id_arr[3]=0;

	storeInFlash();


	//save can IDs in menu
	//can id 1
	memset(temp,0,17); // erase the temp array
	strncpy(temp, "CAN ID ",16);
	strcat(temp, "1:    ");
	memset(char_arr, 0, 16);
	sprintf(char_arr, "%03d", can_id_1);
	strlcat(temp, char_arr,17);
	memcpy(can_menu[2],temp,17);
	//can id 2
	memset(temp,0,17); // erase the temp array
	strncpy(temp, "CAN ID ",16);
	strcat(temp, "2:    ");
	memset(char_arr, 0, 16);
	sprintf(char_arr, "%03d", can_id_2);
	strlcat(temp, char_arr,17);
	memcpy(can_menu[3],temp,17);
	//can id 3
	memset(temp,0,17); // erase the temp array
	strncpy(temp, "CAN ID ",16);
	strcat(temp, "3:    ");
	memset(char_arr, 0, 16);
	sprintf(char_arr, "%03d", can_id_3);
	strlcat(temp, char_arr,17);
	memcpy(can_menu[4],temp,17);

	lcd_put_cur(0,0);
}

//reset CAN Variables
void resetCAN(void){
	//reset IDs
	can_id_1=1;
	can_id_2=2;
	can_id_3=3;
	setCANID();

	//reset baud rate to 1M
	baud_pos=3;
	setBaudRate();
}

//change the number of delays between samples
void changeDelay(uint16_t direction){
	num_delays+=direction;//move num_delays in given direction

	//display new delay in microseconds
	lcd_put_cur(0,9);
	char temp_arr[5];
	sprintf(temp_arr, "%05d", us*(num_delays+1));
	lcd_send_string(temp_arr);
	lcd_put_cur(0,13);
}

//change the prescaler to achieve the desired baud rate and reinitialize the can bus
void setDelay(void){
	//sore the nmber of delays in flash
	storeInFlash();
	TIM3->ARR = ((num_delays+1)*us) - 1;//change CAN interrupt timer period

	//save the delay in microseconds in menu
	memset(temp,0,17); // erase the temp array
	strncpy(temp, "Delay:   ",16);
	memset(char_arr, 0, 16);
	sprintf(char_arr, "%05d", us*(num_delays+1));
	strcat(temp, char_arr);
	strlcat(temp,"us",17);
	memcpy(main_menu[3],temp,17);
	lcd_put_cur(0,0);
}

//sets an offset for the specified analog value
void zeroAnalog(uint8_t analog_id){
	if(analog_id==0){//zero all
		analog_1_offset = raw_1;
		analog_2_offset = raw_2;
		analog_3_offset = raw_3;
	}else if(analog_id==1)//zero 1
		analog_1_offset = raw_1;
	else if(analog_id==2)//zero 2
		analog_2_offset = raw_2;
	else if(analog_id==3)//zero 3
		analog_3_offset = raw_3;
}

//resets all analog offsets to 0 and stores it in flash
void resetOffsets(void){
	analog_1_offset=0;
	analog_2_offset=0;
	analog_3_offset=0;

	store_offsets=1;

	storeInFlash();
}

//turns the analog device on or off
void toggleAnalog(uint8_t analog_id){
	uint8_t enabled=0;//bool to indicate if the change enabled the device
	//update state and display it
	lcd_put_cur(0,13);
	if(analog_id==1){
		if(analog_1_enabled)
			lcd_send_string("OFF");
		else{
			lcd_send_string(" ON");
			enabled=1;}
		analog_1_enabled=!analog_1_enabled;
	}else if(analog_id==2){
		if(analog_2_enabled)
			lcd_send_string("OFF");
		else{
			lcd_send_string(" ON");
			enabled=1;}
		analog_2_enabled=!analog_2_enabled;
	}else if(analog_id==3){
		if(analog_3_enabled)
			lcd_send_string("OFF");
		else{
			lcd_send_string(" ON");enabled=1;}
		analog_3_enabled=!analog_3_enabled;
	}

	//store the state in the menu
	if(enabled){
		memset(temp,0,17); // erase the temp array
		strncpy(temp, "Analog ",16);
		memset(char_arr, 0, 16);
		sprintf(char_arr, "%01d", analog_id);
		strcat(temp, char_arr);
		strcat(temp,":     ");
		strlcat(temp, "ON",17);
		memcpy(analog_menu[analog_id+4],temp,17);
	}else{
		memset(temp,0,17); // erase the temp array
		strncpy(temp, "Analog ",16);
		memset(char_arr, 0, 16);
		sprintf(char_arr, "%01d", analog_id);
		strcat(temp, char_arr);
		strcat(temp,":    ");
		strlcat(temp, "OFF",17);
		memcpy(analog_menu[analog_id+4],temp,17);}

	//invert values so the default is on after erasing memory
	analog_enable_arr[0]=!analog_1_enabled;
	analog_enable_arr[1]=!analog_2_enabled;
	analog_enable_arr[2]=!analog_3_enabled;

	storeInFlash();

	//reset cursor on display
	lcd_put_cur(0,0);
}

//display the analog values on the display
void displayValues(void){
	lcd_put_cur(0,0);
	if(analog_1_enabled&&analog_2_enabled&&analog_3_enabled){//all 3 devices are enabled
		if(display_scroll==0){//position 1
			lcd_send_string("Analog 1:       ");
			lcd_put_cur(0,11);
			memset(char_arr, 0, 16);
			sprintf(char_arr, "%05d", a_in_1);
			lcd_send_string(char_arr);
			lcd_put_cur(1,0);
			lcd_send_string("Analog 2:       ");
			lcd_put_cur(1,11);
			memset(char_arr, 0, 16);
			sprintf(char_arr, "%05d", a_in_2);
			lcd_send_string(char_arr);
		}else if(display_scroll==1){//position 2
			lcd_send_string("Analog 2:       ");
			lcd_put_cur(0,11);
			memset(char_arr, 0, 16);
			sprintf(char_arr, "%05d", a_in_2);
			lcd_send_string(char_arr);
			lcd_put_cur(1,0);
			lcd_send_string("Analog 3:       ");
			lcd_put_cur(1,11);
			memset(char_arr, 0, 16);
			sprintf(char_arr, "%05d", a_in_3);
			lcd_send_string(char_arr);
		}else {//position 3
			lcd_send_string("Analog 3:       ");
			lcd_put_cur(0,11);
			memset(char_arr, 0, 16);
			sprintf(char_arr, "%05d", a_in_3);
			lcd_send_string(char_arr);
			lcd_put_cur(1,0);
			lcd_send_string("Analog 1:       ");
			lcd_put_cur(1,11);
			memset(char_arr, 0, 16);
			sprintf(char_arr, "%05d", a_in_1);
			lcd_send_string(char_arr);
		}

	}else{//less than 3 are enabled
		if(analog_1_enabled){//display device 1 on first line
			lcd_send_string("Analog 1:       ");
			lcd_put_cur(0,11);
			memset(char_arr, 0, 16);
			sprintf(char_arr, "%05d", a_in_1);
			lcd_send_string(char_arr);
		} else if(analog_2_enabled){//display device 2 on first line
			lcd_send_string("Analog 2:       ");
			lcd_put_cur(0,11);
			memset(char_arr, 0, 16);
			sprintf(char_arr, "%05d", a_in_2);
			lcd_send_string(char_arr);
		} else if(analog_3_enabled){//display device 3 on first line
			lcd_send_string("Analog 3:       ");
			lcd_put_cur(0,11);
			memset(char_arr, 0, 16);
			sprintf(char_arr, "%05d", a_in_3);
			lcd_send_string(char_arr);
		} else{//display no devices enabled
			lcd_send_string("No Devices      ");
			lcd_put_cur(1,0);
			lcd_send_string("Enabled         ");
			lcd_put_cur(1,15);
		}

		if(analog_1_enabled&&analog_2_enabled){//display device 2 on second line
			lcd_put_cur(1,0);
			lcd_send_string("Analog 2:       ");
			lcd_put_cur(1,11);
			memset(char_arr, 0, 16);
			sprintf(char_arr, "%05d", a_in_2);
			lcd_send_string(char_arr);
		} else if((analog_1_enabled&&analog_3_enabled)||(analog_2_enabled&&analog_3_enabled)){//display device 3 on second line
			lcd_put_cur(1,0);
			lcd_send_string("Analog 3:       ");
			lcd_put_cur(1,11);
			memset(char_arr, 0, 16);
			sprintf(char_arr, "%05d", a_in_3);
			lcd_send_string(char_arr);
		}
	}
}


//convert a 4 byte array to a word (unsigned long int)
uint32_t bytesToWord(uint8_t* arr) {
	uint32_t value = arr[0] & 0xFF;
	value |= (arr[1] << 8) & 0xFFFF;
	value |= (arr[2] << 16) & 0xFFFFFF;
	value |= (arr[3] << 24) & 0xFFFFFFFF;
	return value;
}

//convert a word (unsigned long int) to a 4 byte array
uint8_t* wordToBytes(uint32_t value){
	byte_arr[3] = (value >> 24) & 0xFF;
	byte_arr[2] = (value >> 16) & 0xFF;
	byte_arr[1] = (value >> 8) & 0xFF;
	byte_arr[0] = value & 0xFF;
	return byte_arr;
}

//calibrate the given sdadc for the given channel
void calibrateSDADC(SDADC_HandleTypeDef* adc, uint32_t channel){
	//Display Calibrating
	lcd_put_cur(0,0);
	lcd_send_string("Calibrating ADC ");
	lcd_put_cur(1,0);
	lcd_send_string("                ");

	//Run Calibration Sequence
	HAL_SDADC_AssociateChannelConfig(adc, channel, SDADC_CONF_INDEX_0);
	HAL_SDADC_ConfigChannel(adc, channel, SDADC_CONTINUOUS_CONV_OFF);
	HAL_SDADC_CalibrationStart(adc, SDADC_CALIBRATION_SEQ_1);
	HAL_SDADC_PollForCalibEvent(adc, HAL_MAX_DELAY);
	HAL_SDADC_ConfigChannel(adc, channel, SDADC_CONTINUOUS_CONV_ON);

	//Display Menu
	lcd_put_cur(0,0);
	lcd_send_string(strncpy(temp,main_menu[menu_pos],16));
	lcd_put_cur(1,0);
	lcd_send_string(strncpy(temp,main_menu[menu_pos+1],16));
	lcd_put_cur(0,0);
}

//store all saved values in flash memory
void storeInFlash(void){
	//Unlock the Flash Program Erase controller
	HAL_FLASH_Unlock();

	//Erase page
	FLASH_EraseInitTypeDef eraseParams;
	eraseParams.TypeErase = FLASH_TYPEERASE_PAGES;
	eraseParams.PageAddress = 0x0800F800;
	eraseParams.NbPages=1;
	uint32_t eraseError=0;
	HAL_FLASHEx_Erase(&eraseParams, &eraseError);
	FLASH_WaitForLastOperation(HAL_MAX_DELAY);


	//Store the CAN IDs
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800F800, bytesToWord(id_arr));//store can ids
	FLASH_WaitForLastOperation(HAL_MAX_DELAY);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800F804, (uint32_t)baud_pos); //store baud rate
	FLASH_WaitForLastOperation(HAL_MAX_DELAY);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800F808, (uint32_t)num_delays); // store number of delays
	FLASH_WaitForLastOperation(HAL_MAX_DELAY);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800F80C, bytesToWord(analog_enable_arr)); // store device enable bools
	FLASH_WaitForLastOperation(HAL_MAX_DELAY);
	if(store_offsets){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800F810, analog_1_offset); // store analog 1 offset
		FLASH_WaitForLastOperation(HAL_MAX_DELAY);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800F814, analog_2_offset); // store analog 2 offset
		FLASH_WaitForLastOperation(HAL_MAX_DELAY);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800F818, analog_3_offset); // store analog 3 offset
		FLASH_WaitForLastOperation(HAL_MAX_DELAY);
		store_offsets=0;
	}


	//Lock the Flash Program Erase controller
	HAL_FLASH_Lock();
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
