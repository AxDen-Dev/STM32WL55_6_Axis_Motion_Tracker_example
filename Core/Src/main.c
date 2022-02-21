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
#include "cmsis_os.h"
#include "app_subghz_phy.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "radio_pin_manager.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */

typedef enum {

	RADIO_TX_START = 0, RADIO_RX_START, RADIO_SLEEP

} radio_state_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LSM6DSL_ADDRESS 0xD4
#define SI7051_ADDRESS 0x80

#define RADIO_TX_TIMEOUT_COUNT 1000
#define RADIO_RX_TIMEOUT_COUNT 1000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

SUBGHZ_HandleTypeDef hsubghz;

/* Definitions for radioTask */
osThreadId_t radioTaskHandle;
uint32_t defaultTaskBuffer[128];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t radioTask_attributes = { .name = "radioTask", .stack_mem =
		&defaultTaskBuffer[0], .stack_size = sizeof(defaultTaskBuffer),
		.cb_mem = &defaultTaskControlBlock, .cb_size =
				sizeof(defaultTaskControlBlock), .priority =
				(osPriority_t) osPriorityNormal, };
/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
uint32_t mainTaskBuffer[128];
osStaticThreadDef_t mainTaskControlBlock;
const osThreadAttr_t mainTask_attributes = { .name = "mainTask", .stack_mem =
		&mainTaskBuffer[0], .stack_size = sizeof(mainTaskBuffer), .cb_mem =
		&mainTaskControlBlock, .cb_size = sizeof(mainTaskControlBlock),
		.priority = (osPriority_t) osPriorityLow, };
/* Definitions for radioBinarySem */
osSemaphoreId_t radioBinarySemHandle;
osStaticSemaphoreDef_t radioBinarySemControlBlock;
const osSemaphoreAttr_t radioBinarySem_attributes = { .name = "radioBinarySem",
		.cb_mem = &radioBinarySemControlBlock, .cb_size =
				sizeof(radioBinarySemControlBlock), };
/* USER CODE BEGIN PV */

static volatile uint8_t lsm6dsl_gyro_start_state = 0x00;
static int16_t ax = 0;
static int16_t ay = 0;
static int16_t az = 0;
static int16_t gx = 0;
static int16_t gy = 0;
static int16_t gz = 0;

static uint8_t si7051_conversion_state = 0x00;
static int16_t temperature = 0;

static volatile radio_state_t radio_state = RADIO_SLEEP;

static uint8_t radio_tx_buffer[64] = { 0x00 };

static int16_t radio_rx_rssi = 0;
static int8_t radio_rx_snr = 0;
static uint8_t radio_rx_buffer_size = 0;
static uint8_t radio_rx_buffer[128] = { 0x00 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
void StartRadioTask(void *argument);
void StartMainTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_SUBGHZ_Init();
	MX_ADC_Init();
	MX_I2C1_Init();
	MX_LPUART1_UART_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of radioBinarySem */
	radioBinarySemHandle = osSemaphoreNew(1, 1, &radioBinarySem_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of radioTask */
	radioTaskHandle = osThreadNew(StartRadioTask, NULL, &radioTask_attributes);

	/* creation of mainTask */
	mainTaskHandle = osThreadNew(StartMainTask, NULL, &mainTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEDiv = RCC_HSE_DIV1;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3 | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC;
	hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.NbrOfConversion = 1;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
	hadc.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
	hadc.Init.OversamplingMode = DISABLE;
	hadc.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00300F38;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief LPUART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPUART1_UART_Init(void) {

	/* USER CODE BEGIN LPUART1_Init 0 */

	/* USER CODE END LPUART1_Init 0 */

	/* USER CODE BEGIN LPUART1_Init 1 */

	/* USER CODE END LPUART1_Init 1 */
	hlpuart1.Instance = LPUART1;
	hlpuart1.Init.BaudRate = 115200;
	hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
	hlpuart1.Init.StopBits = UART_STOPBITS_1;
	hlpuart1.Init.Parity = UART_PARITY_NONE;
	hlpuart1.Init.Mode = UART_MODE_TX_RX;
	hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
	if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN LPUART1_Init 2 */

	/* USER CODE END LPUART1_Init 2 */

}

/**
 * @brief SUBGHZ Initialization Function
 * @param None
 * @retval None
 */
void MX_SUBGHZ_Init(void) {

	/* USER CODE BEGIN SUBGHZ_Init 0 */

	/* USER CODE END SUBGHZ_Init 0 */

	/* USER CODE BEGIN SUBGHZ_Init 1 */

	/* USER CODE END SUBGHZ_Init 1 */
	hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
	if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SUBGHZ_Init 2 */

	/* USER CODE END SUBGHZ_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, BAT_EN_GPIO_Pin | RF_SWITCH_CTRL_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, RF_SWITCH_VDD_Pin | LED1_GPIO_Pin | LED0_GPIO_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : BAT_EN_GPIO_Pin RF_SWITCH_CTRL_Pin */
	GPIO_InitStruct.Pin = BAT_EN_GPIO_Pin | RF_SWITCH_CTRL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : HAL_INPUT_GPIO_Pin */
	GPIO_InitStruct.Pin = HAL_INPUT_GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(HAL_INPUT_GPIO_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RF_SWITCH_VDD_Pin LED1_GPIO_Pin LED0_GPIO_Pin */
	GPIO_InitStruct.Pin = RF_SWITCH_VDD_Pin | LED1_GPIO_Pin | LED0_GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

static void sensor_error() {

	for (uint8_t i = 0; i < 4; i++) {

		HAL_GPIO_TogglePin(LED0_GPIO_GPIO_Port, LED0_GPIO_Pin);
		HAL_GPIO_TogglePin(LED1_GPIO_GPIO_Port, LED1_GPIO_Pin);

		osDelay(1000);

	}

	HAL_GPIO_WritePin(LED0_GPIO_GPIO_Port, LED0_GPIO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_GPIO_GPIO_Port, LED1_GPIO_Pin, GPIO_PIN_RESET);

}

static uint8_t i2c_bus_scan() {

	uint8_t count = 0;
	HAL_StatusTypeDef result;

	for (uint8_t i = 1; i < 128; i++) {

		result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t) (i << 1), 2, 2);

		if (result == HAL_OK) {

			count++;

		}

	}

	return count;

}

static uint8_t init_lsm6dslm() {

	uint8_t i2c_tx_buffer[2];

	i2c_tx_buffer[0] = 0x01;

	HAL_StatusTypeDef i2c_state = HAL_I2C_Master_Transmit(&hi2c1,
	LSM6DSL_ADDRESS, i2c_tx_buffer, 1, 100);

	if (i2c_state != HAL_OK) {

		return 0x00;

	}

//SW Reset
	i2c_tx_buffer[0] = 0x12;
	i2c_tx_buffer[1] = 0x01;

	i2c_state = HAL_I2C_Master_Transmit(&hi2c1, LSM6DSL_ADDRESS, i2c_tx_buffer,
			2, 100);

	if (i2c_state != HAL_OK) {

		return 0x00;

	}

	//Sleep 500ms
	osDelay(500);

	//CTRLS3_C BDU Enable & IF_INS Enable Setting
	i2c_tx_buffer[0] = 0x12;
	i2c_tx_buffer[1] = 0x44;

	i2c_state = HAL_I2C_Master_Transmit(&hi2c1, LSM6DSL_ADDRESS, i2c_tx_buffer,
			2, 100);

	if (i2c_state != HAL_OK) {

		return 0x00;

	}

	//FIFO_CTRL5 FIFO Disable Setting
	i2c_tx_buffer[0] = 0x0A;
	i2c_tx_buffer[1] = 0x00;

	i2c_state = HAL_I2C_Master_Transmit(&hi2c1, LSM6DSL_ADDRESS, i2c_tx_buffer,
			2, 100);

	if (i2c_state != HAL_OK) {

		return 0x00;

	}

	//CTRL_6 Disable XL_HM_MODE Setting
	i2c_tx_buffer[0] = 0x15;
	i2c_tx_buffer[1] = 0b00010000;

	i2c_state = HAL_I2C_Master_Transmit(&hi2c1, LSM6DSL_ADDRESS, i2c_tx_buffer,
			2, 100);

	if (i2c_state != HAL_OK) {

		return 0x00;

	}

	//CTRL8_XL
	i2c_tx_buffer[0] = 0x17;
	i2c_tx_buffer[1] = 0x00;

	i2c_state = HAL_I2C_Master_Transmit(&hi2c1, LSM6DSL_ADDRESS, i2c_tx_buffer,
			2, 100);

	if (i2c_state != HAL_OK) {

		return 0x00;

	}

	//CTRL4_C
	i2c_tx_buffer[0] = 0x13;
	i2c_tx_buffer[1] = 0x00;

	i2c_state = HAL_I2C_Master_Transmit(&hi2c1, LSM6DSL_ADDRESS, i2c_tx_buffer,
			2, 100);

	if (i2c_state != HAL_OK) {

		return 0x00;

	}

	//CTRL1_XL Accel ODR Setting 12.5Hz & 2G
	i2c_tx_buffer[0] = 0x10;
	i2c_tx_buffer[1] = 0b00010000;

	i2c_state = HAL_I2C_Master_Transmit(&hi2c1, LSM6DSL_ADDRESS, i2c_tx_buffer,
			2, 100);

	if (i2c_state != HAL_OK) {

		return 0x00;

	}

	//CTRL7_G Gyro HM_MODE Disable Setting
	i2c_tx_buffer[0] = 0x16;
	i2c_tx_buffer[1] = 0b10000000;

	i2c_state = HAL_I2C_Master_Transmit(&hi2c1, LSM6DSL_ADDRESS, i2c_tx_buffer,
			2, 100);

	if (i2c_state != HAL_OK) {

		return 0x00;

	}
	//CTRL2_G Gyro ODR Setting 12.5Hz * 250dps
	i2c_tx_buffer[0] = 0x11;
	i2c_tx_buffer[1] = 0x00; // 0x10

	i2c_state = HAL_I2C_Master_Transmit(&hi2c1, LSM6DSL_ADDRESS, i2c_tx_buffer,
			2, 100);

	if (i2c_state != HAL_OK) {

		return 0x00;

	}

	//Sensor Init Wait Time
	osDelay(200);

	return 0x01;

}

static uint8_t get_lsm6dslm(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx,
		int16_t *gy, int16_t *gz) {

	uint8_t i2c_tx_buffer = 0x00;
	uint8_t i2c_rx_buffer[12] = { 0x00 };

	if (lsm6dsl_gyro_start_state) {

		i2c_tx_buffer = 0x22;

	} else {

		i2c_tx_buffer = 0x28;

	}

	HAL_StatusTypeDef i2c_state = HAL_I2C_Master_Transmit(&hi2c1,
	LSM6DSL_ADDRESS, &i2c_tx_buffer, 1, 100);

	if (i2c_state != HAL_OK) {

		return 0x00;

	}

	if (lsm6dsl_gyro_start_state) {

		i2c_state = HAL_I2C_Master_Receive(&hi2c1, LSM6DSL_ADDRESS,
				i2c_rx_buffer, 12, 100);

		int16_t value = i2c_rx_buffer[0] | (i2c_rx_buffer[1] << 8);
		*gx = value;

		value = i2c_rx_buffer[2] | (i2c_rx_buffer[3] << 8);
		*gy = value;

		value = i2c_rx_buffer[4] | (i2c_rx_buffer[5] << 8);
		*gz = value;

		value = i2c_rx_buffer[6] | (i2c_rx_buffer[7] << 8);
		*ax = value;

		value = i2c_rx_buffer[8] | (i2c_rx_buffer[9] << 8);
		*ay = value;

		value = i2c_rx_buffer[10] | (i2c_rx_buffer[11] << 8);
		*az = value;

	} else {

		i2c_state = HAL_I2C_Master_Receive(&hi2c1, LSM6DSL_ADDRESS,
				i2c_rx_buffer, 6, 100);

		int16_t value = i2c_rx_buffer[0] | (i2c_rx_buffer[1] << 8);
		*ax = value;

		value = i2c_rx_buffer[2] | (i2c_rx_buffer[3] << 8);
		*ay = value;

		value = i2c_rx_buffer[4] | (i2c_rx_buffer[5] << 8);
		*az = value;

	}

	if (i2c_state != HAL_OK) {

		return 0x00;

	}

	return 0x01;

}

static uint8_t si7051_init() {

	uint8_t i2c_tx_buffer[2];

	i2c_tx_buffer[0] = 0xFE;

	HAL_StatusTypeDef i2c_state = HAL_I2C_Master_Transmit(&hi2c1,
	SI7051_ADDRESS, i2c_tx_buffer, 1, 100);

	if (i2c_state != HAL_OK) {

		return 0x00;

	}

	//Sensor Init Wait Time
	osDelay(200);

	return 0x01;

}

static uint8_t set_conversion_si7051() {

	uint8_t i2c_tx_buffer = 0x00;

	i2c_tx_buffer = 0xF3;

	HAL_StatusTypeDef i2c_state = HAL_I2C_Master_Transmit(&hi2c1,
	SI7051_ADDRESS, &i2c_tx_buffer, 1, 100);

	if (i2c_state != HAL_OK) {

		return 0x00;

	}

	return 0x01;

}

static uint8_t get_si7051(int16_t *temperature) {

	uint8_t i2c_rx_buffer[2] = { 0x00 };

	HAL_StatusTypeDef i2c_state = HAL_I2C_Master_Receive(&hi2c1, SI7051_ADDRESS,
			i2c_rx_buffer, 2, 100);

	if (i2c_state != HAL_OK) {

		return 0x00;

	}

	uint16_t value = (i2c_rx_buffer[0] << 8) | i2c_rx_buffer[1];

	float temperature_si7051 = ((((float) value * 175.72) / 65536) - 46.85);
	*temperature = (int16_t) (temperature_si7051 * 10);

	return 0x01;

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartRadioTask */
/**
 * @brief  Function implementing the radioTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartRadioTask */
void StartRadioTask(void *argument) {
	/* init code for SubGHz_Phy */
	MX_SubGHz_Phy_Init();
	/* USER CODE BEGIN 5 */

	radio_state = RADIO_SLEEP;

	MX_SubGhz_Phy_Radio_Sleep();

	/* Infinite loop */
	for (;;) {

		osSemaphoreAcquire(radioBinarySemHandle, osWaitForever);

		radio_state = RADIO_TX_START;
		MX_SubGhz_Phy_SendPacket(radio_tx_buffer, sizeof(radio_tx_buffer));

		osSemaphoreAcquire(radioBinarySemHandle, RADIO_TX_TIMEOUT_COUNT);

		if (MX_SubGhz_Phy_Get_SendPacket_State() == 0x01) {

			radio_state = RADIO_RX_START;
			MX_SubGhz_Phy_RecvicePacket();

			osSemaphoreAcquire(radioBinarySemHandle, RADIO_RX_TIMEOUT_COUNT);

			if (MX_SubGhz_Phy_Get_RecvicePacket_State() == 0x01) {

				MX_SubGhz_Phy_Get_RecvicePacket(&radio_rx_rssi, &radio_rx_snr,
						radio_rx_buffer, &radio_rx_buffer_size);

			}

		}

		radio_state = RADIO_SLEEP;

		MX_SubGhz_Phy_Radio_Sleep();

	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMainTask */
/**
 * @brief Function implementing the mainTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument) {
	/* USER CODE BEGIN StartMainTask */
	/* Infinite loop */
	uint32_t timer_count = 0;

	HAL_GPIO_WritePin(RF_SWITCH_VDD_GPIO_Port, RF_SWITCH_VDD_Pin,
			GPIO_PIN_RESET);
	set_radio_pin_manager_rf_switch_vdd_pin(RF_SWITCH_VDD_GPIO_Port,
	RF_SWITCH_VDD_Pin);

	HAL_GPIO_WritePin(RF_SWITCH_CTRL_GPIO_Port, RF_SWITCH_CTRL_Pin,
			GPIO_PIN_RESET);
	set_radio_pin_manager_rf_switch_pin(RF_SWITCH_CTRL_GPIO_Port,
	RF_SWITCH_CTRL_Pin);

	HAL_GPIO_WritePin(LED0_GPIO_GPIO_Port, LED0_GPIO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_GPIO_GPIO_Port, LED1_GPIO_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(BAT_EN_GPIO_GPIO_Port, BAT_EN_GPIO_Pin, GPIO_PIN_RESET);

	osDelay(100);

	i2c_bus_scan();

	if (!init_lsm6dslm()) {

		sensor_error();

	}

	if (!si7051_init()) {

		sensor_error();

	}

	HAL_GPIO_WritePin(LED0_GPIO_GPIO_Port, LED0_GPIO_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED1_GPIO_GPIO_Port, LED1_GPIO_Pin, GPIO_PIN_SET);

	/* Infinite loop */
	for (;;) {

		get_lsm6dslm(&ax, &ay, &az, &gx, &gy, &gz);

		timer_count++;

		//10 minute
		if (timer_count >= 6000) {

			if (si7051_conversion_state == 0x00) {

				set_conversion_si7051();
				si7051_conversion_state = 0x01;

			} else if (si7051_conversion_state == 0x01) {

				get_si7051(&temperature);
				si7051_conversion_state = 0x00;

				timer_count = 0;

				osSemaphoreRelease(radioBinarySemHandle);

			}

		}

		osDelay(100);

	}
	/* USER CODE END StartMainTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM17 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM17) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

