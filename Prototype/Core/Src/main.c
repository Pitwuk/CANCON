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
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "LCD1602.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FLASH_STORAGE 0x080E0000
#define page_size 0x800

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

CAN_HandleTypeDef hcan1;

I2S_HandleTypeDef hi2s3;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
uint32_t* p_tx_mailbox; //initialize pointer to Tx Mailbox
CAN_FilterTypeDef can_filter; //initialize CAN filter structure

CAN_TxHeaderTypeDef headers_1, headers_2, headers_3; // initialize CAN headers
uint8_t id_arr[4];
uint32_t *CAN_IDs;
uint8_t PSC=2;
char modified_can_id[3];//can id of the can signal being modified

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

char temp[17];
const char device_str[16]="Device:    ";
const char baud_str[16]="Baud rate:  ";
const char can_id_str[16]="CAN ID ";
const char delay_str[16]="Delay:   ";



int menu_pos = 0; //display menu position
int device_pos=0; //device menu position
int baud_pos = 3; //baud_rate array position
// Flash addresses for CAN IDs
uint16_t VirtAddVarTab[3] = {0x5555, 0x6666, 0x7777};

uint8_t can_id_1; uint8_t can_id_2; uint8_t can_id_3;//CAN IDs of analog signals
char char_arr[16];//character array used in intToString
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2S3_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len){
	int i =0;
	for(i=0; i<len; i++)
		ITM_SendChar((*ptr++));
	return len;
}
float intToDeg(uint16_t num);
//convert an int to a char array
char* intToString(int number, uint8_t digits)
{
	sprintf(char_arr, "%d", number);
	return char_arr;
}
uint32_t getUInt32(uint8_t* arr) {
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
void storeInFlash(void){
	//Unlock the Flash Program Erase controller
	HAL_FLASH_Unlock();

	//Erase sector
	FLASH_Erase_Sector(FLASH_SECTOR_11, FLASH_VOLTAGE_RANGE_3);
	FLASH_WaitForLastOperation(HAL_MAX_DELAY);

	//Store the CAN IDs
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x080E0000, getUInt32(id_arr));
	FLASH_WaitForLastOperation(HAL_MAX_DELAY);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x080E0004, (uint32_t)baud_pos);
	FLASH_WaitForLastOperation(HAL_MAX_DELAY);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x080E0008, (uint32_t)us);
		FLASH_WaitForLastOperation(HAL_MAX_DELAY);

	//Lock the Flash Program Erase controller
	HAL_FLASH_Lock();
}
void changeBaudRate(uint8_t direction);
void setBaudRate(void);
void changeDevice(uint8_t direction);
void setDeviceID(void);
void changeCANID(uint8_t direction);
void setCANID(void);
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





	uint16_t raw_1, raw_2, raw_3; //raw analog data from the sensors
	uint8_t a_data_1[2], a_data_2[2], a_data_3[2]; //analog data in a byte array



	CAN_IDs=(*(__IO uint32_t *) 0x080E0000);
	wordToBytes(CAN_IDs);
	can_id_1=id_arr[0];
	can_id_2=id_arr[1];
	can_id_3=id_arr[2];
	device_id=id_arr[3];

	if(can_id_1==can_id_2||can_id_1==can_id_3||can_id_2==can_id_3){
		can_id_1=1;
		can_id_2=2;
		can_id_3=3;
	}
	if(device_id!=1&&device_id!=0){
		device_id=0;
	}
	device_pos=device_id;

	baud_pos=(uint8_t)(*(__IO uint32_t *) 0x080E0004);
	if(baud_pos<0||baud_pos>3)
		baud_pos=3;
	PSC=pow(2,4-baud_pos);

	us=(uint16_t)(*(__IO uint32_t *) 0x080E0008);
	if(us>1000)
		us=150*(4-baud_pos);







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
	MX_I2S3_Init();
	MX_USB_HOST_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_CAN1_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */


	HAL_TIM_Base_Start(&htim1);

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
	if (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK) {
		Error_Handler();
	}

	//start the can protocol and verify that it has started
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	char hee[16];

	//display start menu
	lcd_put_cur(0,0);
	lcd_send_string(strncpy(hee,display_rows[menu_pos],16));
	lcd_put_cur(1,0);
	lcd_send_string(strncpy(hee,display_rows[menu_pos+1],16));
	lcd_put_cur(0,0);

	int counter =0;
	int btn_counter=0;


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		__HAL_TIM_SET_COUNTER(&htim1,0);
		if(counter>1000)
			counter=0;




		//get raw data for analog 1
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		raw_1 = HAL_ADC_GetValue(&hadc1);

		//get raw data for analog 2
		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
		raw_2 = HAL_ADC_GetValue(&hadc2);


		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
		//		if(raw_1<3900)
		//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		//		if(raw_1<3000)
		//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		//		if(raw_1<2000)
		//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		//		if(raw_1<1000)
		//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0))
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1))
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2))
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);


		//put raw data into byte arrays
		a_data_1[0]=raw_1 & 0xff;
		a_data_1[1]=(raw_1 >> 8);

		a_data_2[0]=raw_2 & 0xff;
		a_data_2[1]=(raw_2 >> 8);



		HAL_CAN_AddTxMessage(&hcan1, &headers_1, a_data_1, p_tx_mailbox);//transmit CAN data for analog 1
		while (HAL_CAN_IsTxMessagePending(&hcan1,  p_tx_mailbox));//wait until data is sent for analog 1
		HAL_CAN_AddTxMessage(&hcan1, &headers_2, a_data_2, p_tx_mailbox);//transmit CAN data for analog 2
		while (HAL_CAN_IsTxMessagePending(&hcan1,  p_tx_mailbox));//wait until data is sent for analog 2

		//if the up button is pressed
		if(up_val==0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)&&btn_counter>1000){
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
				//						scroll menu up

				menu_pos=(menu_pos==0)?display_rows_length-1:(menu_pos-1);//decrement menu position
				lcd_put_cur(0,0);
				lcd_send_string(strncpy(hee,display_rows[menu_pos],16));
				lcd_put_cur(1,0);
				lcd_send_string(strncpy(hee,display_rows[(menu_pos+1)%display_rows_length],16));
				lcd_put_cur(0,0);

			}
			up_val = 1; // set button value to pressed
			btn_counter=0;
		} else if (up_val==1 && !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)&&btn_counter>100){
			up_val = 0; // reset up button value
		}

		//if the down button is pressed
		if(down_val==0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)&&btn_counter>1000){
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
				lcd_send_string(strncpy(hee,display_rows[menu_pos],16));
				lcd_put_cur(1,0);
				lcd_send_string(strncpy(hee,display_rows[(menu_pos+1)%display_rows_length],16));
				lcd_put_cur(0,0);

			}
			down_val = 1; // set button value to pressed
			btn_counter=0;
		} else if (down_val==1 && !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)&&btn_counter>100){
			down_val = 0; // reset up button value
		}

		//if the select button is pressed
		if(sel_val==0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)&&btn_counter>1000){
			//set changes and reinitialize can bus
			if(change_value_bool){
				if(menu_pos==0)
					setBaudRate();
				else if(menu_pos< 4)
					setCANID();
				//						else if(menu_pos==4)
				//							calibrateSDADC(&hsdadc1, SDADC_CHANNEL_1);
				//						else if(menu_pos==5)
				//							calibrateSDADC(&hsdadc2, SDADC_CHANNEL_0);
				//						else if(menu_pos==6)
				//							calibrateSDADC(&hsdadc3, SDADC_CHANNEL_4);
				else if(menu_pos==8)
					setDeviceID();
				else if(menu_pos == 9)
					setDelay();//set Delay
			}else{
				lcd_put_cur(0,15);
			}
			change_value_bool=!change_value_bool;
			sel_val=1;
			btn_counter=0;
		} else if (sel_val==1 && !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)&&btn_counter>100){

			sel_val = 0; // reset up button value
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

		counter++;
		btn_counter++;

		while ((uint16_t)__HAL_TIM_GET_COUNTER(&htim1) < us);
		/* USER CODE END WHILE */
		MX_USB_HOST_Process();

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
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
	PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
	PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{

	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler=PSC;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */

	/* USER CODE END CAN1_Init 2 */

}

/**
 * @brief I2S3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S3_Init(void)
{

	/* USER CODE BEGIN I2S3_Init 0 */

	/* USER CODE END I2S3_Init 0 */

	/* USER CODE BEGIN I2S3_Init 1 */

	/* USER CODE END I2S3_Init 1 */
	hi2s3.Instance = SPI3;
	hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
	hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
	hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
	hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
	hi2s3.Init.CPOL = I2S_CPOL_LOW;
	hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	if (HAL_I2S_Init(&hi2s3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2S3_Init 2 */

	/* USER CODE END I2S3_Init 2 */

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

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 168-1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 0xffff-1;
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
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

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
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
			|Audio_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : PE3 PE11 PE12 PE13 */
	GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PDM_OUT_Pin */
	GPIO_InitStruct.Pin = PDM_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
	GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : CLK_IN_Pin */
	GPIO_InitStruct.Pin = CLK_IN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PB11 PB12 PB13 PB14 */
	GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
	GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
			|Audio_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
	GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
	GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : MEMS_INT2_Pin */
	GPIO_InitStruct.Pin = MEMS_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void changeBaudRate(uint8_t direction){
	baud_pos = ((baud_pos+direction)<0)?baud_rates_length-1:(baud_pos+direction)%baud_rates_length;//move baud rate position in given direction
	lcd_put_cur(0,12);
	lcd_send_string(baud_rates[baud_pos]);
	lcd_put_cur(0,15);
}

//change the prescaler to achieve the desired baud rate and reinitialize the can bus
void setBaudRate(void){
	//stop can bus
	HAL_CAN_Stop(&hcan1);

	//set prescaler
	PSC=pow(2,4-baud_pos);
	hcan1.Init.Prescaler = PSC;

	//reinitialize
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}

	//start the can protocol and verify that it has started
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}

	storeInFlash();



	//set the baud rate in menu

	char hee[5];
	memset(temp,0,17);
	strncpy(temp, baud_str,16);
	strlcpy(hee, baud_rates[baud_pos],5);
	strlcpy(display_rows[0],strcat(temp, hee),17);
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

	//Store the id
	id_arr[3]=device_id;

	storeInFlash();

	//save device to menu
	char hee[5];
	memset(temp,0,17);
	strncpy(temp, device_str,16);
	strlcpy(display_rows[8],strncat(temp, strncpy(hee, device_options[device_pos],5),16),17);
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




	//set can IDs in menu
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

void changeDelay(uint16_t direction){
	us+=direction*50;//move baud rate position in given direction
	if(us==65486)
		us=1000;
	else if(us>1000)
		us=0;
	lcd_put_cur(0,9);
	char hee[5];
	sprintf(hee, "%05d", us);
	lcd_send_string(hee);
	lcd_put_cur(0,13);
}

//change the prescaler to achieve the desired baud rate and reinitialize the can bus
void setDelay(void){
	storeInFlash();

	//set the baud rate in menu
	memset(temp, 0, 17);
	strncpy(temp, delay_str,16);
	memset(char_arr, 0, 16);
	sprintf(char_arr, "%05d", us);
	strcat(temp, char_arr);
	strlcat(temp,"us",17);
	memcpy(display_rows[9],temp,17);
	lcd_put_cur(0,0);
}
//convert 12-bit int to degrees
float intToDeg(uint16_t num){
	return (num/4096)*360;
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
