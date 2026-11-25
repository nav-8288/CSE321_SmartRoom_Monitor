/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include "bme280.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "piezo_buzzer.h"
#include "pir.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*The operating states for the Smart Room Monitor */
typedef enum
{
  STATE_NORMAL = 0, /*The room conditions are acceptable */
  STATE_WARNING, /*The temperature or air quality reading is elevated */
  STATE_ALERT, /*A reading exceeds the alerts threshold */
  STATE_SLEEP /*No recent motion, normal conditions */
} SystemState_t;

/*Holds the sensor readings and system status */
typedef struct
{
  int32_t  temperature;  /*Temperature in 0.01 C units */
  uint32_t humidity;     /*Humidity in 1/1024 %RH units */
  uint16_t lux;          /*Light level in lux (illuminance) */
  uint16_t airRaw;       /* MQ-135 ADC val (0 - 4095)*/
  uint8_t  bmeOk;        /*1 if BME280 reading worked, else 0 */
  uint8_t  lightOk;      /*1 if BH1750 reading worked, else 0 */
  uint8_t  motion;       /*1 if motion detected, else 0 */
  uint8_t  state;        /*Current system state (Normal, Warning, Alert, Sleep) */
} SensorData_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*Thresholds for Warning and Alert states */

#define TEMP_ALERT_CENTI   2800   /* Alert above 28 C */
#define AIR_ALERT_RAW      2000   /* MQ-135 alert threshold (ADC)*/
#define TEMP_WARN_CENTI    2600   /* Warning above 26 C */
#define AIR_WARN_RAW       1500   /* MQ-135 Warning threshold (ADC)*/

/*Other system settings */

#define SLEEP_TIMEOUT_MS   (5 * 60 * 1000) /*System goes into Sleep mode after 5 minutes without motion detected */

#define BUTTON_DEBOUNCE_MS 200 /*Ignore bounce for 200ms */
#define NUM_SCREENS        2 /*Num of OLED screens */

#define LED_ON   GPIO_PIN_RESET
#define LED_OFF  GPIO_PIN_SET

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for BlinkTask */
osThreadId_t BlinkTaskHandle;
const osThreadAttr_t BlinkTask_attributes = {
  .name = "BlinkTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AlertTask */
osThreadId_t AlertTaskHandle;
const osThreadAttr_t AlertTask_attributes = {
  .name = "AlertTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for MotionTask */
osThreadId_t MotionTaskHandle;
const osThreadAttr_t MotionTask_attributes = {
  .name = "MotionTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* USER CODE BEGIN PV */

/*FreeRTOS objects used by the tasks */
osMutexId_t i2cMutex; /*prevents multiple tasks from using I2C at once */
osMessageQueueId_t sensorQueue; /*Sends sensor readings to DisplayTask*/
osSemaphoreId_t alertSem; /*signals AlertTask to sound the buzzer */
osSemaphoreId_t motionSem; /*signals MotionTask when pir detects motion */

/*Vars shared between tasks and interrupts  */
volatile uint32_t lastMotionTick = 0; /*Time of last detected motion  */
volatile SystemState_t sysState = STATE_NORMAL; /* Current system state */
volatile uint8_t screenIndex = 0; /*Selected OLED screen */
volatile uint8_t alarmSilenced = 0; /*1 if the alarm is silenced, else 0 if active */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC_Init(void);
void StartBlinkTask(void *argument);
void StartSensorTask(void *argument);
void StartAlertTask(void *argument);
void StartDisplayTask(void *argument);
void StartMotionTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*sends printf output to serial monitor through UART */
int __io_putchar(int ch){

  uint8_t character = (uint8_t)ch; /*convert the char into 8-bit data */
  HAL_UART_Transmit(&huart2, &character, 1, HAL_MAX_DELAY); /*send one byte through UART2 */

  return ch;
}

/*BME280 I2C address */
static uint8_t bme_addr = BME280_I2C_ADDR_PRIM;   /* 0x76 */

/*function reads data from BME280 through I2C */
BME280_INTF_RET_TYPE bme_i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr){

  uint8_t addr = *(uint8_t *)intf_ptr; /*Get sensors address by casting as 8-bit integer */

  /*Reads len bytes from BME280 register and stores them in data*/
  /*I2C1 peripheral,shift sensor address left 1 bit, read from reg, reg address is 8 bits, data stores bytes received, len is num of bytes to read, max wait time 100ms */

  if (HAL_I2C_Mem_Read(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, (uint16_t)len, 100) == HAL_OK){
    return BME280_INTF_RET_SUCCESS; /*return that the read worked! */
  }

  return -1; /*otherwise return the read failed */
}

/*function writes data to the BME280 through I2C */
BME280_INTF_RET_TYPE bme_i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr){

  uint8_t addr = *(uint8_t *)intf_ptr; /*Get sensors address by casting as 8-bit integer */

  /*Writes len bytes from data to BME280 register through I2C */
  /*I2C1 peripheral, shift sensor address left 1 bit, write to reg, reg address is 8 bits, data contains bytes sent to sensor, len is num of bytes to send, max wait time 100ms  */
  if (HAL_I2C_Mem_Write(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, (uint16_t)len, 100) == HAL_OK){

    return BME280_INTF_RET_SUCCESS; /*return the write was successful */
  }
  return -1; /*otherwise return that the write failed */
}

/*Delay function used by BME280 driver */
void bme_delay_us(uint32_t period, void *intf_ptr){
  /*convert microseconds to RTOS ticks (1ms per tick) */
  osDelay((period / 1000) + 1);   // us -> ms, round up
}

/* Attempts to recover the I2C bus if a sensor get stuck */
static void i2c_bus_recover(void){

  GPIO_InitTypeDef g = {0}; /*Creates a GPIO settings var, init to 0*/

  /*stop I2C1 before temporarily controlling pins */
  HAL_I2C_DeInit(&hi2c1);

  /*Set up PB8 (SCL) and PB9 (SDA) as regular GPIO pins */
  g.Pin = GPIO_PIN_8 | GPIO_PIN_9; /*select both pins */
  g.Mode = GPIO_MODE_OUTPUT_OD; /*Open drain output: pull Low or release */
  g.Pull = GPIO_NOPULL; /*No internal pull-up or pull-down */
  g.Speed = GPIO_SPEED_FREQ_HIGH; /*set the GPIO output speed */
  HAL_GPIO_Init(GPIOB, &g); /*Apply these settings to GPIO port B */

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); /*release the SDA line */ 

  /*send 9 clock pulses to SCL to try freeing the stuck sensor */
  for (int i = 0; i < 9; i++){

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); /*Pull SCL low */
    for (volatile int d = 0; d < 100; d++){ /*short delay */
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); /*Release SCL high */
    for (volatile int d = 0; d < 100; d++){ /*short delay */
    }
  }

  /*Attempt an I2C STOP condition */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); /*Pull SDA low */

  for (volatile int d = 0; d < 100; d++){ /*short delay */

  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); /*Release SDA high */

  MX_I2C1_Init();   // back to I2C mode
}




/* Funct Initializes the BME280 and starts continuous (normal mode) measurements */
static int8_t bme_setup(struct bme280_dev *dev){
  struct bme280_settings settings; /*stores the sensor settings */

  int8_t result = bme280_init(dev); /*Initialize the BME280 sensor*/

  if (result != BME280_OK){ /*If initialization failed, return error */
  
    return result;
  } 

  bme280_get_sensor_settings(&settings, dev); /*Get current sensor settings */

  /*Set oversampling to 1x for each measurement */
  /*oversampling - tell the sensor to take one measurement per enabled parameter during each cycle */
  settings.osr_t = BME280_OVERSAMPLING_1X;/*Temperature */
  settings.osr_h = BME280_OVERSAMPLING_1X; /*Humidity */
  settings.osr_p = BME280_OVERSAMPLING_1X; /*Pressure */

  settings.filter = BME280_FILTER_COEFF_OFF; /*Disable sensor filter */
  settings.standby_time = BME280_STANDBY_TIME_1000_MS; /*1000ms standby time */

  /*Apply the sensor settings */
  result = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, dev);
  
  /*If the settings failed, return an error */
  if (result != BME280_OK){
    return result;
  } 
  /*Otherwise start the sensor in NORMAL mode */
  return bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, dev);
}


/*Stores the system state names for display and debugging */
static const char *const stateName[] = {
  "NORMAL", "WARNING", "ALERT", "SLEEP" 
  };

/* Function turns each color of the RGB LED on (1) or off (0) */
static void set_led(uint8_t red, uint8_t green, uint8_t blue){

  /*control red LED */
  if(red == 1){
    HAL_GPIO_WritePin(Led_R_GPIO_Port, Led_R_Pin, LED_ON);
  }else{
    HAL_GPIO_WritePin(Led_R_GPIO_Port, Led_R_Pin, LED_OFF);
  }
  /*control green LED */
  if(green == 1){
    HAL_GPIO_WritePin(Led_G_GPIO_Port, Led_G_Pin, LED_ON);
  }else{
    HAL_GPIO_WritePin(Led_G_GPIO_Port, Led_G_Pin, LED_OFF);
  }
  /*control blue LED */
  if(blue == 1){
    HAL_GPIO_WritePin(Led_B_GPIO_Port, Led_B_Pin, LED_ON);
  }else{
    HAL_GPIO_WritePin(Led_B_GPIO_Port, Led_B_Pin, LED_OFF);
  }
}
  


/* Sets the LED and fan based on the current system state */
static void apply_outputs(SystemState_t s){
  /*check the system state and set the LED color */
  switch (s){

    case STATE_NORMAL:
      set_led(0, 1, 0);  /*Green LED */
      break;   

    case STATE_WARNING: 
      set_led(1, 1, 0);  /*Yellow LED */
      break;   

    case STATE_ALERT:
       set_led(1, 0, 0);  /*Red LED */
       break;   

    default:            
      set_led(0, 0, 0); /*Turn LED off in sleep mode*/ 
      break;   
  }
  /*Turn the fan on if the system is in ALERT */
  if(s == STATE_ALERT){
    HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET); /*Fan ON */
  }else{

    HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET); /*Fan OFF */

  }

}

/*Determines the next system state using temperature and air quality */
static SystemState_t next_state(SystemState_t currentState, const SensorData_t *readings){
  /*Check if the temp or air quality exceeds alert threshold */
  uint8_t alert = (readings->bmeOk && readings->temperature > TEMP_ALERT_CENTI) || readings->airRaw > AIR_ALERT_RAW;

  /*Check if temp or air quality exceeds the warning threshold */
  uint8_t warn  = (readings->bmeOk && readings->temperature > TEMP_WARN_CENTI) || (readings->airRaw > AIR_WARN_RAW);

  /*Enter ALERT if either alert threshold is exceeded */
  if (alert){
    return STATE_ALERT;
  } 

  /*Stay in ALERT until both readings fall below warning thresholds*/
  if (currentState == STATE_ALERT && warn){
    return STATE_ALERT;
  } 
  /*Enter WARNING if either warning threshold is exceeded */
  if (warn){
    return STATE_WARNING;
  } 
  /*Otherwise return to NORMAL */
  return STATE_NORMAL;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */

  i2cMutex = osMutexNew(NULL);

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  alertSem = osSemaphoreNew(1, 0, NULL);
  motionSem = osSemaphoreNew(1, 0, NULL);

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  sensorQueue = osMessageQueueNew(4, sizeof(SensorData_t), NULL);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of BlinkTask */
  BlinkTaskHandle = osThreadNew(StartBlinkTask, NULL, &BlinkTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of AlertTask */
  AlertTaskHandle = osThreadNew(StartAlertTask, NULL, &AlertTask_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL, &DisplayTask_attributes);

  /* creation of MotionTask */
  MotionTaskHandle = osThreadNew(StartMotionTask, NULL, &MotionTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  if (SensorTaskHandle == NULL || DisplayTaskHandle == NULL ||
      AlertTaskHandle == NULL || MotionTaskHandle == NULL || BlinkTaskHandle == NULL){
    printf("Task creation FAILED\r\n");
  }

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
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
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 115200;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Led_B_Pin|FAN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_R_GPIO_Port, Led_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_G_GPIO_Port, Led_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PIR_Pin */
  GPIO_InitStruct.Pin = PIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Led_B_Pin FAN_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Led_B_Pin|FAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Led_R_Pin */
  GPIO_InitStruct.Pin = Led_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Led_G_Pin */
  GPIO_InitStruct.Pin = Led_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_G_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */






/*Handles interrupts from the PIR sensor and push buttons */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

  if (GPIO_Pin == PIR_Pin){  /*Check if the interrupt came from the PIR sensor*/


    /* Notify MotionTask that motion was detected */
    if (motionSem != NULL){
      osSemaphoreRelease(motionSem);
    }
    /*Check if either button was pressed */
  }else if (GPIO_Pin == BUTTON_Pin || GPIO_Pin == B1_Pin){

    static uint32_t lastPress = 0; /*Time of the last accepted press */
    uint32_t currentTick = HAL_GetTick(); /*Current time in millisec */
    
    /*Ignore button presses within 200ms of last press */
    if (currentTick - lastPress < BUTTON_DEBOUNCE_MS){
      return;
    }

    /*Save time of this button press */
    lastPress = currentTick;

    lastMotionTick = osKernelGetTickCount(); /*Button press also resets inactivity timer */


    /*Silence alarm if system is in ALERT */
    if (sysState == STATE_ALERT){
      alarmSilenced = 1; /* silence the buzzer */
     }else{
      screenIndex = (screenIndex + 1) % NUM_SCREENS;   /* otherwise switch to next screen */
    }
  }
}
/* USER CODE END 4 */



/* USER CODE BEGIN Header_StartBlinkTask */
/**
  * @brief  Function implementing the BlinkTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBlinkTask */
void StartBlinkTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    /*Blink the LED every 500ms */
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */


void StartSensorTask(void *argument){
  /* USER CODE BEGIN StartSensorTask */

  /*Vars used for sensor readings and setup */
  struct bme280_dev dev; /*BME280 device info */
  struct bme280_data data; /*Stores BME280 measurements  */
  SensorData_t reading = {0}; /*Stores all sensor readings  */
  int8_t rslt; /*result of BME280 operations */

  uint8_t cmd = 0x10; /* BH1750 continuous high-resolution mode */
  uint8_t buf[2]; /*Stores 2 bytes from BH1750 */
  uint8_t mode; /*Stores the BME280 operating mode*/ 

  uint32_t nextWake; /*Time for the next sensor reading */


  /*setup I2C communication for the BME280 */
  dev.intf = BME280_I2C_INTF; /*Use I2C*/
  dev.intf_ptr = &bme_addr; /*BME280 sensor address */

  /*set the functions used by the BME280 driver */
  dev.read = bme_i2c_read; /*read sensor data */

  dev.write = bme_i2c_write; /*Write sensor settings */
  dev.delay_us = bme_delay_us; /*Handle sensor delays */

  /*Lock I2C while setting up the sensors  */
  osMutexAcquire(i2cMutex, osWaitForever);
  
  rslt = bme_setup(&dev); /*Init the BME280 sensor */

  
  HAL_I2C_Master_Transmit(&hi2c1, 0x23 << 1, &cmd, 1, 100);/*Start BH1750 in continous high res mode*/
  /*Send the BH1750 mode command through I2C1 (100ms timeout) */

  osMutexRelease(i2cMutex); /*Release i2c so other tasks can use it */

  printf("\r\nSensorTask started, bme280 setup: %d\r\n", rslt); /*Print the BME280 setup result to the serial monitor */
  osDelay(180); /*Give the sensors time to start taking measurements */


  apply_outputs(sysState); /*set the initial LED and fan outputs */

  /*store the starting time for periodic sensor readings  */
  nextWake = osKernelGetTickCount();

  /*Continously read and monitor the sensors */
  for(;;)
  {
    osMutexAcquire(i2cMutex, osWaitForever); /*Lock I2C while accessing the sensors */


    /*Check if BME280 is NOT responding or is NOT in normal mode */
    if (bme280_get_sensor_mode(&mode, &dev) != BME280_OK || mode != BME280_POWERMODE_NORMAL){

      rslt = bme_setup(&dev); /*Try to init BME280 again */

      if (rslt != BME280_OK){ /*Check if the BME280 setup failed */

        /*Try to recover the I2C bus if communication failed*/
        i2c_bus_recover();

        /*Retry BME280 setup after I2C recovery */
        rslt = bme_setup(&dev);
      }
      
      /*Send the BH1750 mode command again  */
      HAL_I2C_Master_Transmit(&hi2c1, 0x23 << 1, &cmd, 1, 100);

      /*Print the latest BME280 setup result */
      printf("BME280 setup attempt result: %d\r\n", rslt);

      /*Skip BME280 measurements during this cycle */
      reading.bmeOk = 0;       
    }else{
      /*check whether the BME280 reading succeeded */
      if(bme280_get_sensor_data(BME280_ALL, &data, &dev) == BME280_OK){

        reading.bmeOk = 1; /*read successful */

      }else{

        reading.bmeOk = 0; /*read failed */
      }
    }

    /*Read 2 bytes from the BH1750 sensor through I2C */
    if(HAL_I2C_Master_Receive(&hi2c1, 0x23 << 1, buf, 2, 100) == HAL_OK){
      reading.lightOk = 1; /*Light sensor read successful */
    }else{
      reading.lightOk = 0; /*light sensor read failed */
    }

    /*Release I2C so other tasks can use it */
    osMutexRelease(i2cMutex);

    /*Read the MQ-135 using the ADC (not using I2C) */
    HAL_ADC_Start(&hadc);

    /*Wait up to 10ms for the ADC conversion */
    if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK){
      /*store the ADC reading */
      reading.airRaw = (uint16_t)HAL_ADC_GetValue(&hadc);
   }else{
      reading.airRaw = 0; /*ADC conversion failed */
   }
    HAL_ADC_Stop(&hadc); /*Stop ADC conversion */

    /*Store temperature and humidity if BME280 read succeeded */
    if(reading.bmeOk){
      reading.temperature = data.temperature;

      reading.humidity = data.humidity;
    
    }else{
      /*clear readings if BME280 reading failed */
      reading.temperature = 0;

      reading.humidity = 0;
    }

   
    if (reading.lightOk){/*Convert the BH1750 reading into lux */

      /*combine the two received bytes and convert to lux*/
      reading.lux =  (uint16_t)(((buf[0] << 8) | buf[1]) * 10 / 12);
    }else{
      reading.lux = 0; /*otherwise if read failed, set light reading to 0 */
    }

    reading.motion = readPIR(); /*Read the data from PIR motion sensor (0 = NOT DETECTED / 1 = DETECTED */

    /*Determine the next system state from the sensor readings */
    SystemState_t next = next_state(sysState, &reading);
    
    /*Enter sleep mode if conditions are normal and no activity is detected */
    if (next == STATE_NORMAL && (osKernelGetTickCount() - lastMotionTick) > SLEEP_TIMEOUT_MS){
      next = STATE_SLEEP;
    }

    /*if the system state has changed */
    if (next != sysState){
      /*print the. previous and new state to the serial monitor */
      printf("State: %s -> %s\r\n", stateName[sysState], stateName[next]);

      sysState = next; /*update the current system state */

      /*update the LED and fan  */
      apply_outputs(sysState); 
    }

    if (sysState != STATE_ALERT){
      alarmSilenced = 0;   /* re-arm the alarm once the alert clears */
    }

    /* Send the sensor readings and current state to DisplayTask */
    reading.state = sysState;
    osMessageQueuePut(sensorQueue, &reading, 0, 0);

    /*Notify AlertTask if the system is in ALERT and alarm is not silenced*/
    if (sysState == STATE_ALERT && !alarmSilenced)
    {
      osSemaphoreRelease(alertSem);
    }

    /*Schedule the next sensor reading after 1000 RTOS ticks */
    nextWake += 1000;
    osDelayUntil(nextWake);
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartAlertTask */
/**
* @brief Function implementing the AlertTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAlertTask */

void StartAlertTask(void *argument)
{
  /* USER CODE BEGIN StartAlertTask */
  /* Infinite loop to wait for an alert */
  for(;;){
    /*wait until SensorTask signals an alert through the semaphore */
    osSemaphoreAcquire(alertSem, osWaitForever);

    /* Sound the buzzer 3 times when an alert is received*/
    for (int i = 0; i < 3; i++){
      piezo_active(); /*turn buzzer ON */
      osDelay(150); /*keep on for 150ms */
      piezo_inactive(); /*turn buzzer off */

      osDelay(150); /*wait 150ms before next beep */

    }
  }
  /* USER CODE END StartAlertTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */

  /*vars used for sensor readings and OLED display */
  SensorData_t r; /*stores sensor readings from SensorTask */
  char line[32]; /*Store text to displaay on the OLED */
  uint8_t displayOn = 1; /*1 = display on, 0 = display off */

  /*Lock I2C while initializing the OLED */
  osMutexAcquire(i2cMutex, osWaitForever);

  ssd1306_Init(); /*Init OLED display */
  ssd1306_Fill(Black); /*clear display */
  ssd1306_SetCursor(0, 0); /*set cursor to top left corner */

/*write proj name on OLED*/
  ssd1306_WriteString("Smart Room", Font_11x18, White); 

  ssd1306_UpdateScreen(); /*send buffer to the OLED */

  osMutexRelease(i2cMutex);/*Release I2C for other tasks */

  /* Infinite loop; update the OLED when new sensor data arrives */
  for(;;)
  {
    /* Wait until sensorTask sends a new sensor reading */
    if (osMessageQueueGet(sensorQueue, &r, NULL, osWaitForever) != osOK){
      continue; /*skip cycle if receiving data failed */
    }

    /*check if the system has entereed sleep mode */
    if (r.state == STATE_SLEEP){
      if (displayOn){
        osMutexAcquire(i2cMutex, osWaitForever); /*Lock i2c*/
        ssd1306_SetDisplayOn(0); /*turn OLED off*/
        osMutexRelease(i2cMutex); /*Release I2C*/

        displayOn = 0; /*display is now off*/
      }
      continue; /*skip updating the oled while sleeping */
    }
    if (!displayOn){
      osMutexAcquire(i2cMutex, osWaitForever); /*lock i2c*/
      ssd1306_SetDisplayOn(1); /*Turn OLED on */
      osMutexRelease(i2cMutex);/*Release I2C*/
      displayOn = 1; /*display is now on */
    }

    /*Convert temp and humidity for printing */
    int32_t  tFrac = r.temperature % 100; /*temp decimal part */
    uint32_t h = r.humidity * 100 / 1024; /*convert humidity to percentage x100 */

    if (tFrac < 0){
      tFrac = -tFrac; /*Make temp decimal part positive */
    }


    if (r.bmeOk){ /*Print the BME280 Temp and humidity readings */
      printf("[%s] Temp: %ld.%02ld C  Hum: %lu.%02lu %%  ", stateName[r.state], r.temperature / 100, tFrac, h / 100, h % 100);
      }else{
        /*print an error if the BME280 reading failed */
    	printf("[%s] BME280 read failed  ", stateName[r.state]);
      }

      /*Print lght, air quality, and PIR readings */
    	printf("Light: %u lux  Air: %u  PIR: %u\r\n", r.lux, r.airRaw, r.motion);

        /*Clear the OLED before drawing new info */
        ssd1306_Fill(Black);

        ssd1306_SetCursor(0, 0); /*move the cursor top left corner */

        /*choose what to dislpay at top of OLED */
        if (screenIndex == 0){

          if(r.motion){ /*show whether the PIR detected motion */
            ssd1306_WriteString("Motion!", Font_11x18, White);
          }else{
            ssd1306_WriteString("No Motion", Font_11x18, White);
          }

        }else{
                
          /* show proj name on status screen */
          ssd1306_WriteString("Smart Room", Font_11x18, White);
        }

        /*check which OLED screen is selected  */
    if (screenIndex == 0){
      /* Screen 1: display sensor readings */
      if (r.bmeOk){

        /*Format and display temperature */
        snprintf(line, sizeof(line), "Temp: %ld.%02ld C", r.temperature / 100, tFrac);
        ssd1306_SetCursor(0, 22);
        ssd1306_WriteString(line, Font_7x10, White);
        
        /*Format and display humidity */
        snprintf(line, sizeof(line), "Hum:  %lu.%02lu %%", h / 100, h % 100);
        ssd1306_SetCursor(0, 32);
        ssd1306_WriteString(line, Font_7x10, White);
      }
      /*Format and display light level */
      snprintf(line, sizeof(line), "Light: %u lux", r.lux);
      ssd1306_SetCursor(0, 42);
      ssd1306_WriteString(line, Font_7x10, White);

      /*Format and display MQ-135 raw ADC reading*/
      snprintf(line, sizeof(line), "Air:   %u", r.airRaw);
      ssd1306_SetCursor(0, 52);
      ssd1306_WriteString(line, Font_7x10, White);

    }else{
    
      /* Screen 2: system status */

      /*Display current system state */
      snprintf(line, sizeof(line), "State: %s", stateName[r.state]);
      ssd1306_SetCursor(0, 22);
      ssd1306_WriteString(line, Font_7x10, White);
      
      /*Display whether the fan should be on or off*/
      if(r.state == STATE_ALERT){
        snprintf(line, sizeof(line), "Fan: ON");
      }else{
        snprintf(line, sizeof(line), "Fan: OFF");

      }
      ssd1306_SetCursor(0, 32);
      ssd1306_WriteString(line, Font_7x10, White);
      
      /*Display whether the alarm has been silenced */
      if(alarmSilenced){
        snprintf(line, sizeof(line), "Alarm: Silenced");
      }else{
        snprintf(line, sizeof(line), "Alarm: armed");
      }
      ssd1306_SetCursor(0, 42);
      ssd1306_WriteString(line, Font_7x10, White);

      /*Display time since the last detected activity */
      snprintf(line, sizeof(line), "Motion: %lus ago", (osKernelGetTickCount() - lastMotionTick) / 1000);
      ssd1306_SetCursor(0, 52);
      ssd1306_WriteString(line, Font_7x10, White);
    }

    /*Lock i2c while sending updated display to the OLED */
    osMutexAcquire(i2cMutex, osWaitForever);
    ssd1306_UpdateScreen(); /*update the OLED */
    osMutexRelease(i2cMutex); /*release i2c for other tasks  */
  }
  /* USER CODE END StartDisplayTask */
}


/* USER CODE BEGIN Header_StartMotionTask */
/**
* @brief Function implementing the MotionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotionTask */


void StartMotionTask(void *argument)
{
  /* USER CODE BEGIN StartMotionTask */
  uint32_t count = 0; /*num of motion events detected */
  uint32_t start; /*time when motion started */

  for(;;) /*forever loop waits for motion events */
  {
    /*Wait until PIR interrupt signals the motion semaphore */
    osSemaphoreAcquire(motionSem, osWaitForever); 

    count++; /*increase motion event counter*/
 
    /*record when motion started */
    start = osKernelGetTickCount();
    lastMotionTick = start;

    /*print motion even num to serial monitor */
    printf("Motion started (#%lu)\r\n", count);

    while (readPIR() == 1){ /*keep checking whole pir detects motion */

      lastMotionTick = osKernelGetTickCount(); /*update last motion time*/
      osDelay(100); /*check again after 100 rtos ticks*/
    } 

    /*print how long motion was detected  */
    printf("Motion ended after %lu ms\r\n", osKernelGetTickCount() - start);

    /*clear any extra motion semaphore signal */
    osSemaphoreAcquire(motionSem, 0);
  }
  /* USER CODE END StartMotionTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
