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
#include "LCD1602.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
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

/* USER CODE BEGIN PV */
uint32_t* tx_mailbox; //initialize Tx Mailbox
CAN_FilterTypeDef can_filter; //initialize CAN filter structure

CAN_TxHeaderTypeDef headers_1, headers_2, headers_3; // initialize CAN headers

// Flash addresses for CAN IDs
uint8_t can_id_1, can_id_2, can_id_3;//CAN IDs of analog signals
uint8_t id_arr[4];
uint32_t *CAN_IDs;
char modified_can_id[3];//can id of the can signal being modified

uint8_t PSC=2; //Prescaler for CAN Baud
uint16_t us = 300;//delay between samples

uint8_t device_id=0;//id of the device connected to the can module (0=EC-LV, 1=JB3)

//array of display rows to cycle through
char display_rows[10][17] = {"Baud rate:      ", "CAN ID 1:    001","CAN ID 2:    002","CAN ID 3:    003", "Calibrate ADC  1", "Calibrate ADC  2", "Calibrate ADC  3", "Display Values  ", "Device:         ", "Delay:          "};
//devices to cycle through
const char device_options[2][5] = {"EC-LV","  JB3"};
//baud rates to cycle through
const char baud_rates [4][4] = {"125k", "250k", "500k", "  1M"};


//menu vars
const int display_rows_length=(sizeof(display_rows)/sizeof(display_rows[0]));
const int device_options_length=(sizeof(device_options)/sizeof(device_options[0]));
const int baud_rates_length=(sizeof(baud_rates)/sizeof(baud_rates[0]));
char temp[17]; //temporary array used for copying
const char device_str[16]="Device:    ";
const char baud_str[16]="Baud rate:  ";
const char can_id_str[16]="CAN ID ";
const char delay_str[16]="Delay:   ";

int8_t menu_pos = 0; //display menu position
int8_t device_pos=0; //device menu position
int8_t baud_pos = 3; //baud_rate array position

char char_arr[16];//character array used for copying


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_SDADC1_Init(void);
static void MX_SDADC2_Init(void);
static void MX_SDADC3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void changeBaudRate(uint8_t direction);
void setBaudRate(void);
void changeDevice(uint8_t direction);
void setDeviceID(void);
void changeCANID(uint8_t direction);
void setCANID(void);
void changeDelay(uint16_t direction);
void setDelay(void);
char * intToString(uint8_t number, uint8_t digits);
float intToDeg(uint16_t num);
uint32_t bytesToWord(uint8_t* arr);
void wordToBytes(uint32_t value);
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
	uint8_t up_val = 0; // up button value
	uint8_t down_val = 0; // down button value
	uint8_t sel_val = 0; // select button value
	uint8_t change_value_bool = 0; //boolean for changing the value the selected item

	int32_t raw_1, raw_2, raw_3; //raw analog data from the sensors
	uint8_t a_data_1[2], a_data_2[2], a_data_3[2]; //analog data in a byte array

	// retrieve the stored CAN IDs
	CAN_IDs=(*(__IO uint32_t *) 0x0800F800);
	wordToBytes(CAN_IDs);

	can_id_1=(uint8_t)id_arr[0];
	can_id_2=id_arr[1];
	can_id_3=id_arr[2];
	device_id=id_arr[3];//get device id

	//if duplicate IDs set IDs to 1, 2, and 3
	if(can_id_1==can_id_2 || can_id_1==can_id_3||can_id_2==can_id_3){
		can_id_1=1;
		can_id_2=2;
		can_id_3=3;
	}
	//if invalid device id then set to EC-LV
	if(device_id!=1&&device_id!=0)
		device_id=0;
	device_pos=device_id;

	//get the baud rate
	baud_pos=(uint8_t)(*(__IO uint32_t *) 0x0800F804);
	//if invalid baud rate then set to 1M
	if(baud_pos<0||baud_pos>3)
		baud_pos=3;
	//PSC=pow(2,4-baud_pos);//set prescaler

	//get delay
	us=(uint16_t)(*(__IO uint32_t *) 0x0800F808);
	//if delay too large then make it reasonable
	if(us>1000)
		us=225*(4-baud_pos);


	// Set the headers for the first analog signal
	headers_1.StdId = can_id_1; // set the CAN ID
	headers_1.IDE = CAN_ID_STD;
	headers_1.RTR = CAN_RTR_DATA;
	headers_1.DLC = sizeof(a_data_1); // set the size of the data
	headers_1.TransmitGlobalTime = DISABLE; // disable transmission of time

	// Set the headers for the second analog signal
	headers_2.StdId = can_id_2; // set the CAN ID
	headers_2.IDE = CAN_ID_STD;
	headers_2.RTR = CAN_RTR_DATA;
	headers_2.DLC = sizeof(a_data_2); // set the size of the data
	headers_2.TransmitGlobalTime = DISABLE; // disable transmission of time

	// Set the headers for the third analog signal
	headers_3.StdId = can_id_3; // set the CAN ID
	headers_3.IDE = CAN_ID_STD;
	headers_3.RTR = CAN_RTR_DATA;
	headers_3.DLC = sizeof(a_data_3); // set the size of the data
	headers_3.TransmitGlobalTime = DISABLE; // disable transmission of time


	//add options to menu
	//set the baud rate
	memset(temp,0,17);
	strncpy(temp, baud_str,16);
	strlcat(temp,baud_rates[baud_pos],17);
	strlcpy(display_rows[0],temp,17);
	//set the device
	memset(temp,0,17);
	strncpy(temp, device_str,16);
	strlcat(temp, device_options[device_pos],17);
	strlcpy(display_rows[8],temp,17);
	//set can IDs

	memset(temp, 0, 17);
	strncpy(temp, can_id_str,16);
	strcat(temp, "1:    ");
	memset(char_arr, 0, 16);
	sprintf(char_arr, "%03d", can_id_1);
	strlcat(temp, char_arr,17);
	memcpy(display_rows[1],temp,17);
	//can id 2
	memset(temp, 0, 17);
	strncpy(temp, can_id_str,16);
	strcat(temp, "2:    ");
	memset(char_arr, 0, 16);
	sprintf(char_arr, "%03d", can_id_2);
	strlcat(temp, char_arr,17);
	memcpy(display_rows[2],temp,17);
	//can id 3
	memset(temp, 0, 17);
	strncpy(temp, can_id_str,16);
	strcat(temp, "3:    ");
	memset(char_arr, 0, 16);
	sprintf(char_arr, "%03d", can_id_3);
	strlcat(temp, char_arr,17);
	memcpy(display_rows[3],temp,17);

	//Delay
	memset(temp, 0, 17);
	strncpy(temp, delay_str,16);
	memset(char_arr, 0, 16);
	sprintf(char_arr, "%05d", us);
	strcat(temp, char_arr);
	strlcat(temp,"us",17);
	memcpy(display_rows[9],temp,17);
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
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim2);// start timer

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

	//display start menu
	lcd_put_cur(0,0);
	lcd_send_string(strncpy(temp,display_rows[menu_pos],16));
	lcd_put_cur(1,0);
	lcd_send_string(strncpy(temp,display_rows[menu_pos+1],16));
	lcd_put_cur(0,0);

	int counter=0;//counter used in updating the display
	int btn_counter=0;//button counter used to reduce double clicks


	calibrateSDADC(&hsdadc1, SDADC_CHANNEL_1);

	calibrateSDADC(&hsdadc2, SDADC_CHANNEL_0);

	calibrateSDADC(&hsdadc3, SDADC_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		__HAL_TIM_SET_COUNTER(&htim2,0); // reset timer
		if(counter>1000)
			counter=0;

		//get raw data for analog 1
		HAL_SDADC_Start(&hsdadc1);
		HAL_SDADC_PollForConversion(&hsdadc1, HAL_MAX_DELAY);
		raw_1 = HAL_SDADC_GetValue(&hsdadc1);

		//get raw data for analog 2
		HAL_SDADC_Start(&hsdadc2);
		HAL_SDADC_PollForConversion(&hsdadc2, HAL_MAX_DELAY);
		raw_2 = HAL_SDADC_GetValue(&hsdadc2);

		if(device_id==1){
			//get raw data for analog 3
			HAL_SDADC_Start(&hsdadc3);
			HAL_SDADC_PollForConversion(&hsdadc3, HAL_MAX_DELAY);
			raw_3 = HAL_SDADC_GetValue(&hsdadc3);
		}

		//conver raw data
		raw_1=(uint16_t)(raw_1+32768);
		raw_2=(uint16_t)(raw_2+32768);
		raw_3=(uint16_t)(raw_3+32768);

		//put raw data into byte arrays
		a_data_1[0]=raw_1 & 0xff;
		a_data_1[1]=(raw_1 >> 8);

		a_data_2[0]=raw_2 & 0xff;
		a_data_2[1]=(raw_2 >> 8);

		if(device_id==1){
			a_data_3[0]=raw_3 & 0xff;
			a_data_3[1]=(raw_3 >> 8);
		}

		//transmit CAN data for analog 1
		HAL_CAN_AddTxMessage(&hcan, &headers_1, a_data_1, *tx_mailbox);
		while (HAL_CAN_IsTxMessagePending(&hcan, *tx_mailbox));//wait until data is sent for analog 1
		//transmit CAN data for analog 2
		HAL_CAN_AddTxMessage(&hcan, &headers_2, a_data_2, *tx_mailbox);
		while (HAL_CAN_IsTxMessagePending(&hcan, *tx_mailbox));//wait until data is sent for analog 2
		if(device_id==1){
			//transmit CAN data for analog 3
			HAL_CAN_AddTxMessage(&hcan, &headers_3, a_data_3, *tx_mailbox);
			while (HAL_CAN_IsTxMessagePending(&hcan, *tx_mailbox));//wait until data is sent for analog 3
		}

		//if the up button is pressed
		if(up_val==0 && HAL_GPIO_ReadPin(UP_BTN_PORT, UP_BTN_PIN)){
			if(change_value_bool){
				//change the value of the selected item
				if(menu_pos==0)
					changeBaudRate(1);//increment baud rate
				else if(menu_pos < 4)
					changeCANID(1);//increment CAN ID
				else if(menu_pos == 8)
					changeDevice(1);//increment Device ID
				else if(menu_pos == 9)
					changeDelay(1);//increment Delay
			} else {
				//scroll menu up
				menu_pos=(menu_pos==0)?display_rows_length-1:(menu_pos-1);//decrement menu position
				lcd_put_cur(0,0);
				lcd_send_string(strncpy(temp,display_rows[menu_pos],16));
				lcd_put_cur(1,0);
				lcd_send_string(strncpy(temp,display_rows[(menu_pos+1)%display_rows_length],16));
				lcd_put_cur(0,0);
			}
			up_val = 1; // set button value to pressed
			btn_counter=0; // reset button counter
		} else if (up_val==1 && !HAL_GPIO_ReadPin(UP_BTN_PORT, UP_BTN_PIN)&&btn_counter>1000){
			up_val = 0; // reset up button value
		}

		//if the down button is pressed
		if(down_val==0 && HAL_GPIO_ReadPin(DOWN_BTN_PORT, DOWN_BTN_PIN)){
			if(change_value_bool){
				//change the value of the selected item
				if(menu_pos==0)
					changeBaudRate(-1);//decrement baud rate
				else if(menu_pos < 4)
					changeCANID(-1);//decrement CAN ID
				else if(menu_pos == 8)
					changeDevice(-1);//decrement Device ID
				else if(menu_pos == 9)
					changeDelay(-1);//decrement Delay
			} else {
				//scroll menu down
				menu_pos = (menu_pos+1)%display_rows_length;//increment menu position
				lcd_put_cur(0,0);
				lcd_send_string(strncpy(temp,display_rows[menu_pos],16));
				lcd_put_cur(1,0);
				lcd_send_string(strncpy(temp,display_rows[(menu_pos+1)%display_rows_length],16));
				lcd_put_cur(0,0);

			}
			down_val = 1; // set button value to pressed
			btn_counter=0;// reset button counter
		} else if (down_val==1 && !HAL_GPIO_ReadPin(DOWN_BTN_PORT, DOWN_BTN_PIN)&&btn_counter>1000){
			down_val = 0; // reset up button value
		}

		//if the select button is pressed
		if(sel_val==0 && HAL_GPIO_ReadPin(SEL_BTN_PORT, SEL_BTN_PIN)){
			//set changes and reinitialize can bus
			if(change_value_bool){
				if(menu_pos==0)
					setBaudRate();
				else if(menu_pos< 4)
					setCANID();
				else if(menu_pos==4)
					calibrateSDADC(&hsdadc1, SDADC_CHANNEL_1);
				else if(menu_pos==5)
					calibrateSDADC(&hsdadc2, SDADC_CHANNEL_0);
				else if(menu_pos==6)
					calibrateSDADC(&hsdadc3, SDADC_CHANNEL_4);
				else if(menu_pos==7){ //display menu
					lcd_put_cur(0,0);
					lcd_send_string(strncpy(temp,display_rows[menu_pos],16));
					lcd_put_cur(1,0);
					lcd_send_string(strncpy(temp,display_rows[(menu_pos+1)%display_rows_length],16));
					lcd_put_cur(0,0);
				}
				else if(menu_pos==8)
					setDeviceID();
				else if(menu_pos == 9)
					setDelay();//set Delay
			}else{
				lcd_put_cur(0,15);
			}
			change_value_bool=!change_value_bool;
			sel_val=1;//set select button to pressed
			btn_counter=0;// reset button counter
		} else if (sel_val==1 && !HAL_GPIO_ReadPin(SEL_BTN_PORT, SEL_BTN_PIN)&&btn_counter>1000){
			sel_val = 0; // reset select button value
		}

		//Display values on display
		if(menu_pos==7 && change_value_bool && counter==0){
			lcd_put_cur(0,0);
			lcd_send_string("Ang Pos:        ");
			lcd_put_cur(0,11);
			memset(char_arr, 0, 16);
			sprintf(char_arr, "%05d", raw_1);
			lcd_send_string(char_arr);
			lcd_put_cur(1,0);
			lcd_send_string("Ang Vel:        ");
			lcd_put_cur(1,11);
			memset(char_arr, 0, 16);
			sprintf(char_arr, "%05d", raw_2);
			lcd_send_string(char_arr);
		}


		//increment counters
		counter++;
		btn_counter++;

		while ((uint16_t)__HAL_TIM_GET_COUNTER(&htim2) < us);//Delay between samples
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
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
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
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
  hsdadc1.Init.ReferenceVoltage = SDADC_VREF_VDDA;
  if (HAL_SDADC_Init(&hsdadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Set parameters for SDADC configuration 0 Register
  */
  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_ZERO_REFERENCE;
  ConfParamStruct.Gain = SDADC_GAIN_1;
  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VDDA;
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
  hsdadc2.Init.ReferenceVoltage = SDADC_VREF_VDDA;
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
  hsdadc3.Init.ReferenceVoltage = SDADC_VREF_VDDA;
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
  htim2.Init.Period = 0xffff-1;
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
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC9 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
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

}

/* USER CODE BEGIN 4 */
//changes the baud rate of the can connection
void changeBaudRate(uint8_t direction){
	baud_pos = ((baud_pos+direction)<0)?baud_rates_length-1:(baud_pos+direction)%baud_rates_length;//move baud rate position in given direction
	lcd_put_cur(0,12);
	lcd_send_string(baud_rates[baud_pos]);
	lcd_put_cur(0,15);
}

//change the prescaler to achieve the desired baud rate and reinitialize the can bus
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

	//set the baud rate in menu
	char temp_arr[5];
	memset(temp,0,17);
	strncpy(temp, baud_str,16);
	strlcpy(temp_arr, baud_rates[baud_pos],5);
	strlcpy(display_rows[0],strcat(temp, temp_arr),17);
	lcd_put_cur(0,0);
}

//changes the device connected to the can module
void changeDevice(uint8_t direction){
	device_pos = !device_pos;//move baud rate position in given direction
	lcd_put_cur(0,11);
	lcd_send_string(device_options[device_pos]);
	lcd_put_cur(0,15);
}

//set the device ID
void setDeviceID(void){
	device_id=device_pos;

	//Store the id in flash
	id_arr[3]=device_id;

	storeInFlash();

	//save device to menu
	char temp_arr[5];
	memset(temp,0,17);
	strncpy(temp, device_str,16);
	strlcpy(display_rows[8],strncat(temp, strncpy(temp_arr, device_options[device_pos],5),16),17);
	lcd_put_cur(0,0);
}

//changes the ID of the selected can device
void changeCANID(uint8_t direction){
	if(menu_pos==1){
		can_id_1+=direction;
		while(can_id_1==can_id_2 || can_id_1==can_id_3)//avoid conflicting IDs
			can_id_1+=direction;
		sprintf(char_arr, "%03d", can_id_1);
		memcpy(modified_can_id, char_arr, 3);//store new can id in modified_can_id
	} else if(menu_pos==2){
		can_id_2+=direction;
		while(can_id_2==can_id_1 || can_id_2==can_id_3)//avoid conflicting IDs
			can_id_2+=direction;
		sprintf(char_arr, "%03d", can_id_2);
		memcpy(modified_can_id, char_arr, 3);//store new can id in modified_can_id
	} else if(menu_pos==3){
		can_id_3+=direction;
		while(can_id_3==can_id_1 || can_id_3==can_id_2)//avoid conflicting IDs
			can_id_3+=direction;
		sprintf(char_arr, "%03d", can_id_3);
		memcpy(modified_can_id, char_arr, 3);//store new can id in modified_can_id
	}
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

	storeInFlash();


	//save can IDs in menu
	memset(temp, 0, 17);
	strncpy(temp, can_id_str,16);
	strcat(temp, "1:    ");
	memset(char_arr, 0, 16);
	sprintf(char_arr, "%03d", can_id_1);
	strlcat(temp, char_arr,17);
	memcpy(display_rows[1],temp,17);
	//can id 2
	memset(temp, 0, 17);
	strncpy(temp, can_id_str,16);
	strcat(temp, "2:    ");
	memset(char_arr, 0, 16);
	sprintf(char_arr, "%03d", can_id_2);
	strlcat(temp, char_arr,17);
	memcpy(display_rows[2],temp,17);
	//can id 3
	memset(temp, 0, 17);
	strncpy(temp, can_id_str,16);
	strcat(temp, "3:    ");
	memset(char_arr, 0, 16);
	sprintf(char_arr, "%03d", can_id_3);
	strlcat(temp, char_arr,17);
	memcpy(display_rows[3],temp,17);

	lcd_put_cur(0,0);
}

//change delay between samples
void changeDelay(uint16_t direction){
	us+=direction*50;//move baud rate position in given direction
	//set max delay at 1ms
	if(us==65486)
		us=1000;
	else if(us>1000)
		us=0;

	//display new delay
	lcd_put_cur(0,9);
	char temp_arr[5];
	sprintf(temp_arr, "%05d", us);
	lcd_send_string(temp_arr);
	lcd_put_cur(0,13);
}

//change the prescaler to achieve the desired baud rate and reinitialize the can bus
void setDelay(void){
	//sore the delay in flash
	storeInFlash();

	//save the delay in menu
	memset(temp, 0, 17);
	strncpy(temp, delay_str,16);
	memset(char_arr, 0, 16);
	sprintf(char_arr, "%05d", us);
	strcat(temp, char_arr);
	strlcat(temp,"us",17);
	memcpy(display_rows[9],temp,17);
	lcd_put_cur(0,0);
}

//convert 16-bit int to degrees
float intToDeg(uint16_t num){
	return (num/65536)*360;
}
uint32_t bytesToWord(uint8_t* arr) {
	uint32_t value = arr[0] & 0xFF;
	value |= (arr[1] << 8) & 0xFFFF;
	value |= (arr[2] << 16) & 0xFFFFFF;
	value |= (arr[3] << 24) & 0xFFFFFFFF;
	return value;
}
void wordToBytes(uint32_t value){
	id_arr[3] = (value >> 24) & 0xFF;
	id_arr[2] = (value >> 16) & 0xFF;
	id_arr[1] = (value >> 8) & 0xFF;
	id_arr[0] = value & 0xFF;
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
	lcd_send_string(strncpy(temp,display_rows[menu_pos],16));
	lcd_put_cur(1,0);
	lcd_send_string(strncpy(temp,display_rows[menu_pos+1],16));
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
	//	FLASH_WaitForLastOperation(HAL_MAX_DELAY);


	//Store the CAN IDs
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800F800, bytesToWord(id_arr));
	FLASH_WaitForLastOperation(HAL_MAX_DELAY);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800F804, (uint32_t)baud_pos);
	FLASH_WaitForLastOperation(HAL_MAX_DELAY);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800F808, (uint32_t)us);
	FLASH_WaitForLastOperation(HAL_MAX_DELAY);

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
