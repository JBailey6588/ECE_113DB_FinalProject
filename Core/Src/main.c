/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define BUF_SIZE 8
#define SAMPLE_RATE 48000 //Hz

//used for delay effect
#define DELAY_TIME 0.2 //seconds
#define DELAY_LENGTH (SAMPLE_RATE * DELAY_TIME)


uint16_t rxBuf[BUF_SIZE];
uint16_t txBuf[BUF_SIZE];


int effect_state = 1; //this variable can be any value from 1 to 8
//each value corresponds to a different effect
// 1-tremolo
// 2-chorus
// 3-reverb
// 4-vibrato
// 5-flanger
// 6-delay
// 7-wah
// 8-distortion



// Tremolo effect
int tremoloDirection = 1; // 0 is up, 1 is down
float tremoloLowerThresh = 0.1;
float tremoloUpperThresh = 1.0;
float tremoloSpeed = 0.00005;
float tremoloMod = 1.0;

int tremolo(int sample) {
	// multiple sample by triangle wave from 0.5 to 1
	// 0 means go down, 1 means go up

	float fSample = (float)sample;
	if (tremoloDirection == 0){
		tremoloMod = tremoloMod + tremoloSpeed;
	}
	else{
		tremoloMod = tremoloMod - tremoloSpeed;
	}
	if (tremoloMod <= tremoloLowerThresh){
		tremoloDirection = 0;
	}
	else if (tremoloMod >= tremoloUpperThresh){
		tremoloDirection = 1;
	}
	fSample = tremoloMod*fSample;
	return (int)fSample;
}





// Chorus effect
int chorusCurrentVal_1 = 0;
float chorusDelay_1 = 0.050*SAMPLE_RATE;
//float chorusCircleBuff_1[(int)chorusDelay_1];
float chorusCircleBuff_1[1200];
int chorusCurrentVal_2 = 0;
float chorusDelay_2 = 0.025*SAMPLE_RATE;
//float chorusCircleBuff_2[(int)chorusDelay_2];
float chorusCircleBuff_2[480];
float chorusGain = 1.0;
int chorus(int sample) {
	float fSample = (float)sample;
	float newest = fSample;
	chorusCircleBuff_1[chorusCurrentVal_1] = newest;
	chorusCircleBuff_2[chorusCurrentVal_2] = newest;
	chorusCurrentVal_1++;
	chorusCurrentVal_2++;
	//if (currentVal_1 == chorusDelay_1 - 1)
if (chorusCurrentVal_1 == (1200 - 1))
	{
	chorusCurrentVal_1 = 0;
	}
	if (chorusCurrentVal_2 == (480 - 1))
	{
		chorusCurrentVal_2 = 0;
	}

	float chorusReturnVal_1 = chorusCircleBuff_1[chorusCurrentVal_1];
	float chorusReturnVal_2 = chorusCircleBuff_2[chorusCurrentVal_2];
	fSample = chorusReturnVal_1*chorusGain + chorusReturnVal_2*chorusGain  + newest;
	return (int)fSample;
}





// Reverb effect
int reverbCurrentVal_1 = 0;
int reverbCurrentVal_2 = 0;
int reverbCurrentVal_3 = 0;
int reverbCurrentVal_4 = 0;

int reverbLength_1 = 901*4;
int reverbLength_2 = 778*4;
int reverbLength_3 = 1011*4;
int reverbLength_4 = 1123*4;

float reverbCircBuff_1[901*4];
float reverbCircBuff_2[778*4];
float reverbCircBuff_3[1011*4];
float reverbCircBuff_4[1123*4];

float reverbGain_1 = 0.805;
float reverbGain_2 = 0.827;
float reverbGain_3 = 0.783;
float reverbGain_4 = 0.764;

int reverbCurrentVal_5 = 0;
int reverbCurrentVal_6 = 0;
int reverbCurrentVal_7 = 0;

int reverbLength_5 = 125*4;
int reverbLength_6 = 42*4;
int reverbLength_7 = 12*4;

float reverbCircBuff_5[125*4];
float reverbCircBuff_6[42*4];
float reverbCircBuff_7[12*4];

float reverbGain_5 = 0.7;
float reverbGain_6 = 0.7;
float reverbGain_7 = 0.7;

float reverbMix = 0.3;

int reverb(int sample){
	float fSample = (float)sample;
	float fSample_1 = (float)sample;

	float returnVal_1 = reverbCircBuff_1[reverbCurrentVal_1];
	float returnVal_2 = reverbCircBuff_2[reverbCurrentVal_2];
	float returnVal_3 = reverbCircBuff_3[reverbCurrentVal_3];
	float returnVal_4 = reverbCircBuff_4[reverbCurrentVal_4];

	float newest_1 = returnVal_1*reverbGain_1 + fSample;
	float newest_2 = returnVal_2*reverbGain_2 + fSample;
	float newest_3 = returnVal_3*reverbGain_3 + fSample;
	float newest_4 = returnVal_4*reverbGain_4 + fSample;

	reverbCircBuff_1[reverbCurrentVal_1] = newest_1;
	reverbCircBuff_2[reverbCurrentVal_2] = newest_2;
	reverbCircBuff_3[reverbCurrentVal_3] = newest_3;
	reverbCircBuff_4[reverbCurrentVal_4] = newest_4;

	reverbCurrentVal_1++;
	reverbCurrentVal_2++;
	reverbCurrentVal_3++;
	reverbCurrentVal_4++;

	if (reverbCurrentVal_1 == reverbLength_1 - 1){
		reverbCurrentVal_1 = 0;
	}
	if (reverbCurrentVal_2 == reverbLength_2 - 1){
		reverbCurrentVal_2 = 0;
	}
	if (reverbCurrentVal_3 == reverbLength_3 - 1){
		reverbCurrentVal_3 = 0;
	}
	if (reverbCurrentVal_4 == reverbLength_4 - 1){
		reverbCurrentVal_4 = 0;
	}

	fSample = (returnVal_1 + returnVal_2 + returnVal_3 + returnVal_4)/4;

	float returnVal_5 = reverbCircBuff_5[reverbCurrentVal_5];
	returnVal_5 = fSample - returnVal_5*reverbGain_5;
	float newest_5 = fSample + returnVal_5*reverbGain_5;
	reverbCircBuff_5[reverbCurrentVal_5] = newest_5;
	reverbCurrentVal_5++;
	if (reverbCurrentVal_5 == reverbLength_5 - 1){
		reverbCurrentVal_5 = 0;
	}
	fSample = returnVal_5;
	float returnVal_6 = reverbCircBuff_6[reverbCurrentVal_6];
	returnVal_6 = fSample - returnVal_6*reverbGain_6;
	float newest_6 = fSample + returnVal_6*reverbGain_6;
	reverbCircBuff_6[reverbCurrentVal_6] = newest_6;
	reverbCurrentVal_6++;
	if (reverbCurrentVal_6 == reverbLength_6 - 1){
		reverbCurrentVal_6 = 0;
	}
	fSample = returnVal_6;
	float returnVal_7 = reverbCircBuff_7[reverbCurrentVal_7];
	returnVal_7 = fSample - returnVal_7*reverbGain_7;
	float newest_7 = fSample + returnVal_7*reverbGain_7;
	reverbCircBuff_7[reverbCurrentVal_7] = newest_7;
	reverbCurrentVal_7++;
	if (reverbCurrentVal_7 == reverbLength_7 - 1){
		reverbCurrentVal_7 = 0;
	}
	fSample = returnVal_7*reverbMix + (1 - reverbMix)*fSample_1;

	return (int)fSample;
}





// Vibrato effect
int vibratoCurrentVal = 0;
//int vibratoBufferLength = 0.01*SAMPLE_RATE; // equals 480. 10 ms buffer
int vibratoBufferLength = 480*2;
int vibratoModLength = 240*2;
// circle buff length is BUFFER + WIDTH + WIDTH
//float vibratoCircleBuff[vibratoBufferLength];
//int vibratoBufferLengthFull = vibratoBufferLength + vibratoModLength + vibratoModLength;
int vibratoBufferLengthFull = 480*2 + 240*2 + 240*2;
//float vibratoCircleBuff[vibratoBufferLengthFull];
float vibratoCircleBuff[(480+240+240)*2];
//float vibratoSinCal;
float vibratoFreq = 5.0/48000.0; // frequency of 5 Hz

int vibrato(int sample){
//	sample = sample/4;
	float fSample = (float)sample;
	float newest = fSample;
	vibratoCircleBuff[vibratoCurrentVal] = newest;
	vibratoCurrentVal++;

	if (vibratoCurrentVal == (vibratoBufferLengthFull-1)){
		vibratoCurrentVal = 0;
	}
	float vibratoCurrentValF = (float)vibratoCurrentVal;
	float vibratoSinCalc = (float)sin(vibratoFreq*2*M_PI*vibratoCurrentValF);
//	float vibratoSinCalc = 0.001;
//	float vibratoCalc = (float)vibratoBufferLength + (float)vibratoModLength + (float)(vibratoModLength)*vibratoSinCalc - 1.0f;
	float vibratoCalc = (float)vibratoBufferLength + (float)(vibratoModLength)*vibratoSinCalc - 1.0f;
//	float vibratoCalc = 958.0;
	float vibratoCalcRemainder = vibratoCalc - floor(vibratoCalc);
	int vibratoCalcInt = (int)floor(vibratoCalc);
//	int vibratoCalcInt = 20;
	int vibratoReturnInt;
	if (vibratoCalcInt % 2 == 0){
		vibratoReturnInt = (vibratoCurrentVal + vibratoCalcInt) % (vibratoBufferLengthFull-1);
	}
	else{
		vibratoReturnInt = (vibratoCurrentVal + vibratoCalcInt - 1) % (vibratoBufferLengthFull-1);
	}
	int vibratoReturnInt2 = vibratoReturnInt - 2;
	if (vibratoReturnInt2 < 0){
		vibratoReturnInt2 = vibratoReturnInt2 + vibratoBufferLengthFull;
	}
	float vibratoReturnVal = vibratoCalcRemainder*vibratoCircleBuff[vibratoReturnInt] + (1- vibratoCalcRemainder)*vibratoCircleBuff[vibratoReturnInt2];
	//float vibratoReturnVal = vibratoCircleBuff[vibratoCurrentVal];
	return (int)vibratoReturnVal;
}



//Wah effect
float wahDamping = 0.05;
int wahDirection = 0; // 0 is up, 1 is down
float wahLowerThresh = 500.0;
float wahUpperThresh = 3000.0;
float wahSpeed = 48000/SAMPLE_RATE;
float wahMod = 500.0;
float wahHighpass = 0;
float wahLowpass = 0;
float wahBandpass = 0;

int wah(int sample)
{
	float fSample = (float)sample;

	if (wahDirection == 0){
		wahMod = wahMod + wahSpeed;
	}
	else{
		wahMod = wahMod - wahSpeed;
	}
	if (wahMod <= wahLowerThresh){
		wahDirection = 0;
	}
	else if (wahMod >= wahUpperThresh){
		wahDirection = 1;
	}

	float wahNewest = (float)(2*sin((M_PI*wahMod)/48000));

	wahHighpass = (float)(fSample - wahLowpass - (2*wahDamping*wahBandpass));
	wahBandpass = (float)((wahNewest*wahHighpass) + wahBandpass);
	wahLowpass = (float)((wahNewest*wahBandpass) + wahLowpass);

	return (int)wahBandpass;
}






// Delay effect
int delayCurrentVal = 0;
float delayBuff[(int)DELAY_LENGTH];
float delayGain = 0.8;
int delay(int sample)
{
	float fSample = (float)sample;
	float delayReturnVal = delayBuff[delayCurrentVal];
	float delayNewest = delayReturnVal*delayGain + fSample;
	delayBuff[delayCurrentVal] = delayNewest;
	delayCurrentVal++;
	if (delayCurrentVal == (int)DELAY_LENGTH - 1)
	{
		delayCurrentVal = 0;
	}
	fSample = delayReturnVal;
	return (int)fSample;
}





//Distortion effect
int distortion(int sample) // fuzz
{
	float fSample = (float)sample;
	//float returnVal = (fSample/abs(fSample)) * (1 - exp((pow(fSample, 2)) / abs(fSample)));

	float distortionReturnVal = (fSample/abs(fSample)) * (1 - (float)exp(0.7*pow(fSample, 2) / abs(fSample)));

	distortionReturnVal = distortionReturnVal + fSample;
	return (int)distortionReturnVal;
}




// Flanger effect
int flangerCurrentVal = 0;
float flangerRate = 1.0; //1 Hz
int flangerBufferLength = 0.003*SAMPLE_RATE;
float flangerGain = 0.7;
float flangerBuff[8640];

int flanger(int sample)
{
	float fSample = (float)sample;
	float flangerNewest;
	float flangerSinMod = abs(sin(2*M_PI*flangerCurrentVal*(flangerRate/SAMPLE_RATE)));
	//currentVal++;

	if (flangerCurrentVal <= flangerBufferLength)
	{
		flangerNewest = fSample;
	}

	else
	{
		int flangerDelay = ceil((flangerSinMod * flangerBufferLength));
		flangerNewest = (float)((flangerGain * fSample) + (flangerGain * flangerBuff[flangerCurrentVal - flangerDelay]));
	}

	flangerBuff[flangerCurrentVal] = flangerNewest;
	float flangerReturnVal = flangerBuff[flangerCurrentVal];
	flangerCurrentVal++;

	if (flangerCurrentVal == 8640)
	{
		flangerCurrentVal = 0;
	}

	return (int)flangerReturnVal;
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
  MX_USART3_UART_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_I2SEx_TransmitReceive_DMA (&hi2s2, txBuf, rxBuf, 4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_FULLDUPLEX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.FirstBit = I2S_FIRSTBIT_MSB;
  hi2s2.Init.WSInversion = I2S_WS_INVERSION_DISABLE;
  hi2s2.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
  hi2s2.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 11999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BT_2_Pin BT_1_Pin */
  GPIO_InitStruct.Pin = BT_2_Pin|BT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s){

	int lSample = (int) (rxBuf[0]<<16)|rxBuf[4];
	int rSample = (int) (rxBuf[1]<<16)|rxBuf[2];


	if (effect_state == 1)
	{
		rSample = tremolo(rSample);
	}
	else if (effect_state == 2)
	{
		rSample = chorus(rSample);
	}

	else if (effect_state == 3)
	{
		rSample = reverb(rSample);
	}

	else if (effect_state == 4)
	{
		rSample = vibrato(rSample);
	}

	else if (effect_state == 8)
	{
		rSample = distortion(rSample);
	}

	else if (effect_state == 6)
	{
		rSample = delay(rSample);
	}

	else if (effect_state == 7)
	{
		rSample = wah(rSample);
	}

	else if (effect_state == 5)
	{
		rSample = flanger(rSample);
	}


	txBuf[0] = 0;
	txBuf[4] = 0;
	txBuf[1] = (rSample>>16)&0xFFFF;
	txBuf[2] = rSample&0xFFFF;
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s){

	int lSample = (int) (rxBuf[4]<<16)|rxBuf[7];
	int rSample = (int) (rxBuf[5]<<16)|rxBuf[6];


	if (effect_state == 1)
	{
		rSample = tremolo(rSample);
	}
	else if (effect_state == 2)
	{
		rSample = chorus(rSample);
	}

	else if (effect_state == 3)
	{
		rSample = reverb(rSample);
	}

	else if (effect_state == 4)
	{
		rSample = vibrato(rSample);
	}

	else if (effect_state == 8)
	{
		rSample = distortion(rSample);
	}

	else if (effect_state == 6)
	{
		rSample = delay(rSample);
	}

	else if (effect_state == 7)
	{
		rSample = wah(rSample);
	}

	else if (effect_state == 5)
	{
		rSample = flanger(rSample);
	}



	txBuf[5] = (rSample>>16)&0xFFFF;
	txBuf[6] = rSample&0xFFFF;
	txBuf[0] = 0;
	txBuf[4] = 0;
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

