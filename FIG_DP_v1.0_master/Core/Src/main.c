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

#include "st2_driver/st2_driver.h"  // 2phase
#include "st5_driver/st5_driver.h"  // 5phase

#include "HX711/hx711.h" //HX711

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM11_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//For HX711
int read;
int read_aa;
int ave_read;
int ave_read_aa;

// IN
#define SENSOR_INPUT_NUM 5
#define INPUT_NUM 5


GPIO_TypeDef* sensorInputPort[] = {GPIOD, GPIOD, GPIOD, GPIOD, GPIOD};
uint16_t sensorInputPin[] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4};

GPIO_TypeDef* inputPort[] = {GPIOB, GPIOB, GPIOB, GPIOB};
uint16_t inputPin[] = {GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6};

GPIO_TypeDef* outputPort[] = {GPIOC};
uint16_t outputPin[] = {GPIO_PIN_11};


GPIO_PinState inputValue_1[5];
GPIO_PinState inputValue_2[4];
GPIO_PinState outputValue[1];

/////////////////////////////////////////////////////// 5Phase////////////////// 

#define _5_PHASE_MOTOR_FREQUENCY 20000 // 20kHz

float st5_pwmDuty[10];

/////////////////////////////////////////////////////// 5Phase-----------------

/////////////////////////////////////////////////////// 2Phase////////////////// 

#define _2_PHASE_MOTOR_FREQUENCY 20000 // 20kHz

float st2_pwmDuty[10];

/////////////////////////////////////////////////////// 2Phase-----------------

/////////////////////////////////////////////////////// step motor general /////

#define ENABLE_GATE     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define DISABLE_GATE    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)


// 20kHz callback interrupt with timer11
uint16_t cnt_for_tim11_debugging;     // Should be smaller than 9000
uint16_t max_cnt_for_tim11_debugging;
uint8_t cnt;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){


  if(htim == &htim11){
    
    ST2_Loop();
    ST5_Loop();

////////////////////////////////////////////5phase    
    
    ST5_getPWM(st5_pwmDuty);

    // A
    if(st5_pwmDuty[0] > 0 && st5_pwmDuty[5] == 0){
      // High or Hi-Z...? (H1 L0)
      TIM3->CCR3 = (uint16_t)(st5_pwmDuty[0] * TIM3->ARR);      // VOHGA
      TIM3->CCR2 = (uint16_t)(1 * TIM3->ARR);      // VOLA
    }
    else if(st5_pwmDuty[0] == 0 && st5_pwmDuty[5] > 0){
      // Low (H0 L1)
      TIM3->CCR3 = (uint16_t)(st5_pwmDuty[0] * TIM3->ARR);      // VOHGA
      TIM3->CCR2 = (uint16_t)(st5_pwmDuty[5] * TIM3->ARR);      // VOLA
    }
    else if(st5_pwmDuty[0] == 0 && st5_pwmDuty[5] == 0){
      // Hi-Z (H0 L0)
      TIM3->CCR3 = (uint16_t)(st5_pwmDuty[0] * TIM3->ARR);      // VOHGA
      TIM3->CCR2 = (uint16_t)(st5_pwmDuty[5] * TIM3->ARR);      // VOLA
    }

    // B
    if(st5_pwmDuty[1] > 0 && st5_pwmDuty[6] == 0){
      // High or Hi-Z...? (H1 L0)
      TIM3->CCR1 = (uint16_t)(st5_pwmDuty[1] * TIM3->ARR);      // VOHGB
      TIM2->CCR1 = (uint16_t)(1 * TIM2->ARR);      // VOLB
    }
    else if(st5_pwmDuty[1] == 0 && st5_pwmDuty[6] > 0){
      // Low (H0 L1)
      TIM3->CCR1 = (uint16_t)(st5_pwmDuty[1] * TIM3->ARR);      // VOHGB
      TIM2->CCR1 = (uint16_t)(st5_pwmDuty[6] * TIM2->ARR);      // VOLB
    }
    else if(st5_pwmDuty[1] == 0 && st5_pwmDuty[6] == 0){
      // Hi-Z (H0 L0)
      TIM3->CCR1 = (uint16_t)(st5_pwmDuty[1] * TIM3->ARR);      // VOHGB
      TIM2->CCR1 = (uint16_t)(st5_pwmDuty[6] * TIM2->ARR);      // VOLB
    }

    // C
    if(st5_pwmDuty[2] > 0 && st5_pwmDuty[7] == 0){
      // High or Hi-Z...? (H1 L0)
      TIM2->CCR3 = (uint16_t)(st5_pwmDuty[2] * TIM2->ARR);      // VOHGC
      TIM1->CCR4 = (uint16_t)(1 * TIM1->ARR);      // VOLC
    }
    else if(st5_pwmDuty[2] == 0 && st5_pwmDuty[7] > 0){
      // Low (H0 L1)
      TIM2->CCR3 = (uint16_t)(st5_pwmDuty[2] * TIM2->ARR);      // VOHGC
      TIM1->CCR4 = (uint16_t)(st5_pwmDuty[7] * TIM1->ARR);      // VOLC
    }
    else if(st5_pwmDuty[2] == 0 && st5_pwmDuty[7] == 0){
      // Hi-Z (H0 L0)
      TIM2->CCR3 = (uint16_t)(st5_pwmDuty[2] * TIM2->ARR);      // VOHGC
      TIM1->CCR4 = (uint16_t)(st5_pwmDuty[7] * TIM1->ARR);      // VOLC
    }

    // D
    if(st5_pwmDuty[3] > 0 && st5_pwmDuty[8] == 0){
      // High or Hi-Z...? (H1 L0)
      TIM1->CCR3 = (uint16_t)(st5_pwmDuty[3] * TIM1->ARR);      // VOHGD
      TIM1->CCR2 = (uint16_t)(1 * TIM1->ARR);      // VOLD
    }
    else if(st5_pwmDuty[3] == 0.0 && st5_pwmDuty[8] > 0.0){
      // Low (H0 L1)
      TIM1->CCR3 = (uint16_t)(st5_pwmDuty[3] * TIM1->ARR);      // VOHGD
      TIM1->CCR2 = (uint16_t)(st5_pwmDuty[8] * TIM1->ARR);      // VOLD
    }
    else if(st5_pwmDuty[3] == 0.0 && st5_pwmDuty[8] == 0.0){
      // Hi-Z (H0 L0)
      TIM1->CCR3 = (uint16_t)(st5_pwmDuty[3] * TIM1->ARR);      // VOHGD
      TIM1->CCR2 = (uint16_t)(st5_pwmDuty[8] * TIM1->ARR);      // VOLD
    }

    // E
    if(st5_pwmDuty[4] > 0 && st5_pwmDuty[9] == 0){
      // High or Hi-Z...? (H1 L0)
      TIM1->CCR1 = (uint16_t)(st5_pwmDuty[4] * TIM1->ARR);      // VOHGE
      TIM3->CCR4 = (uint16_t)(1 * TIM3->ARR);      // VOLE
    }
    else if(st5_pwmDuty[4] == 0 && st5_pwmDuty[9] > 0){
      // Low (H0 L1)
      TIM1->CCR1 = (uint16_t)(st5_pwmDuty[4] * TIM1->ARR);      // VOHGE
      TIM3->CCR4 = (uint16_t)(st5_pwmDuty[9] * TIM3->ARR);      // VOLE
    }
    else if(st5_pwmDuty[4] == 0 && st5_pwmDuty[9] == 0){
      // Hi-Z (H0 L0)
      TIM1->CCR1 = (uint16_t)(st5_pwmDuty[4] * TIM1->ARR);      // VOHGE
      TIM3->CCR4 = (uint16_t)(st5_pwmDuty[9] * TIM3->ARR);      // VOLE
    }
    
////////////////////////////////////////////2phase
    ST2_getPWM(st2_pwmDuty);

    // A
    if(st2_pwmDuty[0] > 0 && st2_pwmDuty[4] == 0){
      // High or Hi-Z...? (H1 L0)
      TIM8->CCR2 = (uint16_t)(st2_pwmDuty[0] * TIM8->ARR);      // VOHGA
      TIM8->CCR1 = (uint16_t)(1 * TIM8->ARR);      // VOLA
    }
    else if(st2_pwmDuty[0] == 0 && st2_pwmDuty[4] > 0){
      // Low (H0 L1)
      TIM8->CCR2 = (uint16_t)(st2_pwmDuty[0] * TIM8->ARR);      // VOHGA
      TIM8->CCR1 = (uint16_t)(st2_pwmDuty[4] * TIM8->ARR);      // VOLA
    }
    else if(st2_pwmDuty[0] == 0 && st2_pwmDuty[4] == 0){
      // Hi-Z (H0 L0)
      TIM8->CCR2 = (uint16_t)(st2_pwmDuty[0] * TIM8->ARR);      // VOHGA
      TIM8->CCR1 = (uint16_t)(st2_pwmDuty[4] * TIM8->ARR);      // VOLA
    }
    
    // C
    if(st2_pwmDuty[1] > 0 && st2_pwmDuty[5] == 0){
      // High or Hi-Z...? (H1 L0)
      TIM4->CCR2 = (uint16_t)(st2_pwmDuty[1] * TIM4->ARR);      // VOHGC
      TIM4->CCR1 = (uint16_t)(1 * TIM4->ARR);      // VOLC
    }
    else if(st2_pwmDuty[2] == 0 && st2_pwmDuty[5] > 0){
      // Low (H0 L1)
      TIM4->CCR2 = (uint16_t)(st2_pwmDuty[1] * TIM4->ARR);      // VOHGC
      TIM4->CCR1 = (uint16_t)(st2_pwmDuty[5] * TIM4->ARR);      // VOLC
    }
    else if(st2_pwmDuty[1] == 0 && st2_pwmDuty[5] == 0){
      // Hi-Z (H0 L0)
      TIM4->CCR2 = (uint16_t)(st2_pwmDuty[1] * TIM4->ARR);      // VOHGC
      TIM4->CCR1 = (uint16_t)(st2_pwmDuty[5] * TIM4->ARR);      // VOLC
    }
    
    // B
    if(st2_pwmDuty[2] > 0 && st2_pwmDuty[6] == 0){
      // High or Hi-Z...? (H1 L0)
      TIM4->CCR4 = (uint16_t)(st2_pwmDuty[2] * TIM4->ARR);      // VOHGB
      TIM4->CCR3 = (uint16_t)(1 * TIM4->ARR);      // VOLB
    }
    else if(st2_pwmDuty[2] == 0 && st2_pwmDuty[6] > 0){
      // Low (H0 L1)
      TIM4->CCR4 = (uint16_t)(st2_pwmDuty[2] * TIM4->ARR);      // VOHGB
      TIM4->CCR3 = (uint16_t)(st2_pwmDuty[6] * TIM4->ARR);      // VOLB
    }
    else if(st2_pwmDuty[2] == 0 && st2_pwmDuty[6] == 0){
      // Hi-Z (H0 L0)
      TIM4->CCR4 = (uint16_t)(st2_pwmDuty[2] * TIM4->ARR);      // VOHGB
      TIM4->CCR3 = (uint16_t)(st2_pwmDuty[6] * TIM4->ARR);      // VOLB
    }

    // D
    if(st2_pwmDuty[3] > 0 && st2_pwmDuty[7] == 0){
      // High or Hi-Z...? (H1 L0)
      TIM8->CCR4 = (uint16_t)(st2_pwmDuty[3] * TIM8->ARR);      // VOHGD
      TIM8->CCR3 = (uint16_t)(1 * TIM8->ARR);      // VOLD
    }
    else if(st2_pwmDuty[3] == 0.0 && st2_pwmDuty[7] > 0.0){
      // Low (H0 L1)
      TIM8->CCR4 = (uint16_t)(st2_pwmDuty[3] * TIM8->ARR);      // VOHGD
      TIM8->CCR3 = (uint16_t)(st2_pwmDuty[7] * TIM8->ARR);      // VOLD
    }
    else if(st2_pwmDuty[3] == 0.0 && st2_pwmDuty[7] == 0.0){
      // Hi-Z (H0 L0)
      TIM8->CCR4 = (uint16_t)(st2_pwmDuty[3] * TIM8->ARR);      // VOHGD
      TIM8->CCR3 = (uint16_t)(st2_pwmDuty[7] * TIM8->ARR);      // VOLD
    }

    
    cnt_for_tim11_debugging = TIM11->CNT;
    if(max_cnt_for_tim11_debugging < cnt_for_tim11_debugging){
      max_cnt_for_tim11_debugging = cnt_for_tim11_debugging;
    }
  }
}  

/////////////////////////////////////////////////////// step motor general -----

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM11_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //FOR HX711
  HX711 sensor;
  sensor.PD_SCK_PinType=GPIOB;
  sensor.PD_SCK_PinNumber=GPIO_PIN_14;
  sensor.DOUT_PinType=GPIOB;
  sensor.DOUT_PinNumber=GPIO_PIN_15;
  sensor.mode=0;

  HX711_Tare(&sensor, 15);
  
/////////////////////////////////////////////////////// step motor general -----
  
  DISABLE_GATE;
  
  
  ////////////////////5phase
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);     // ST5 VOHGA
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);     // ST5 VOHGB
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);    // ST5 VOHGC
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);     // ST5 VOHGD
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);     // ST5 VOHGE

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);     // ST5 VOLA
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);    // ST5 VOLB
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);     // ST5 VOLC
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);     // ST5 VOLD
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);     // ST5 VOLE

  ST5_Init(_5_PHASE_MOTOR_FREQUENCY);
  
  ////////////////////2phase
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);     // ST2 VOHGA
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);     // ST2 VOHGC
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);     // ST2 VOHGB
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);     // ST2 VOHGD

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);     // ST2 VOLA
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);     // ST2 VOLC
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);     // ST2 VOLB
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);     // ST2 VOLD

  ST2_Init(_2_PHASE_MOTOR_FREQUENCY);
  
  ENABLE_GATE;
  
  HAL_TIM_Base_Start_IT(&htim11); // 20kHz interrupt with timer11
  
/////////////////////////////////////////////////////// step motor general -----  
  

  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
   
    uint8_t i_io;
    
     // sensor input
      for(i_io=0; i_io<5; ++i_io){
        inputValue_1[i_io] = HAL_GPIO_ReadPin(sensorInputPort[i_io], sensorInputPin[i_io]);
        HAL_Delay(10);
      }
//      // input
      for(i_io=0; i_io<4; ++i_io){
        inputValue_2[i_io] = HAL_GPIO_ReadPin(inputPort[i_io], inputPin[i_io]);
        HAL_Delay(10);
      }
          

        // output ON & OFF
      if(outputValue[0] == 1){
        HAL_GPIO_WritePin(outputPort[0], outputPin[0], GPIO_PIN_SET);
        HAL_Delay(200);
      }
      if(outputValue[0] == 0){
        HAL_GPIO_WritePin(outputPort[0], outputPin[0], GPIO_PIN_RESET);
        HAL_Delay(200);
      }


      
    //HX711
    read = HX711_GetValue(&sensor);
    read_aa = (float)read/394.22;
    ave_read = HX711_GetAvgValue(&sensor, 15);
    ave_read_aa = (float)ave_read/394.22;
      

    
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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8999;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4499;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4499;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 8999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 8999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Hx711_GPIO_Port, Hx711_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port, Motor_Enable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Hx711_Pin */
  GPIO_InitStruct.Pin = Hx711_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Hx711_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB15 PB3 PB4 PB5
                           PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Motor_Enable_Pin */
  GPIO_InitStruct.Pin = Motor_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Motor_Enable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
