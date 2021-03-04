/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
	* @author         : Chan Tai Wing Vincent
	* @remark         : Kindest people in the world
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
#include <string.h>
#include <stdio.h>
#include "stdbool.h"
#include "mpu6050.h"
#include "pca9685_servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MIN(x,y) ((x<y)?x:y)
#define MAX(x,y) (x>y ?x:y)
#define BUFFER_SIZE 15
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
//@Timer Variables
volatile int TIMER_COUNTER = 0;

//@GPU Message Variables
unsigned char UART_RX_BUFFER[BUFFER_SIZE] = {0};
unsigned char UART_TX_BUFFER[BUFFER_SIZE] = {0};

volatile int MSG_COUNTER = 0;
int LANE_DISTANCE = 0;
int TARGET_X = 0;
int TARGET_Y = 0;

volatile bool SAMPLE_FLAG = false;
enum TRAFFIC_TYPE{STOP,LEFT,RIGHT,UNKOWN};
enum TRAFFIC_TYPE TRAFFIC_STATUS = UNKOWN;
//@GPU Message Variables


//@Fuzzy Logic Stabilizer Varibles
MPU6050_t MPU6050;                           //Create MPU6050 Object

volatile bool MPU_Sampling = false;          //flag to control sampling of MPU6050
bool average_filter = false;                 //flag to control average filtering
bool plot_curve = true;                      //flag to enable/disable curve plotting

int counter = 0;                             //sample taking counter
const int num_samples = 200;                 //number of samples for taking average

double Kalman_X_buffer = 0;                  //Storing num_samples of Kalman_X to perform averaging
double Kalman_Y_buffer = 0;                  //Storing num_samples of Kalman_Y to perform averaging

double offset_x;                             //noise remoiving from Kalman_X
double offset_y;                             //noise removing from Kalman_Y

double X_output;                             //Resultant X
double Y_output;                             //Resultant Y

enum servo_motor_type{raw_servo=0,pitch_servo=2,steering_servo=3};       //Servo motor index related to PCA9685

const int raw_init_angle = 91;                          //Initial raw angle for stabilizer   (make X_output = 0)
const int pitch_init_angle = 98;                        //Initial pitch angle for stabilizer( make Y_output = 0)

const int pitch_max = 15;                               //Mechanical limitation (please confirm with ME designer)
const int pitch_min = -15;                              //Mechanical limitation (please confirm with ME designer)

const int raw_max = 15;                                 //Mechanical limitation (please confirm with ME designer)
const int raw_min = -15;                                //Mechanical limitation (please confirm with ME designer)

int adjust_servo_pitch = 0;		                          //Fuzzy Logic Output of pitch
int servo_control_pitch = pitch_init_angle;             //Operation angle of servo motor (pitch)
int servo_control_pitch_old = 0;                        //Previous operation angle of servo motor (pitch)

int adjust_servo_raw = 0;                               //Fuzzy Logic Output of raw
int servo_control_raw = raw_init_angle;                 //Operation angle of servo motor (raw)
int servo_control_raw_old = 0;                          //Previous operation angle of servo motor (raw)

/*
When pitch_init_angle + pitch_max => KalmanX = 15
When pitch_init_angle + pitch_min => KalmanX = -15
When raw_init_angle   + raw_max   => KalmanY = 15
When raw_init_angle   + raw_min   => KalmanY = -15
*/

//@Fuzzy Logic Stabilizer Varibles

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_IWDG_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void gpu_msg_analyzer(void);

int calculate_MOTOR_PWM(unsigned char percentage);                              //Robot PWM Calculation
void robot_forward(int speed);                                                  //Robot Control -> Forward
void robot_stop(void);                                                          //Robot Control -> Stop
void robot_adjust(int motor1_speed, int motor2_speed);                          //Robot Control -> Adjustable

float trimf(float measurement, float start, float peak, float end);             //Fuzzy Logic: triangular membership function
float Rmf(float measurement, float top, float bottom);                          //Fuzzy Logic: R membership function
float Lmf(float measurement, float bottom, float top);                          //Fuzzy Logic: L membership function
float trapmf(float measurement, float start,float top_1,float top_2,float end); //Fuzzy Logic: trapezodial membership function

int raw_FLC(double error);                                                      //raw fuzzy logic controller
int pitch_FLC(double error);                                                    //pitch fuzzy logic controller
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,0xFFFF);
	return ch;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_IWDG_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  printf("[SYSTEM] Humane Help START...\r\n");                                     //Print startup function
	printf("[SYSTEM] Connecting to MPU6050...\r\n");        
	
	HAL_UART_Receive_DMA(&huart3,UART_RX_BUFFER,BUFFER_SIZE);                       //Receive Jetson AI Message
  HAL_TIM_Base_Start_IT(&htim2);                                                  //Start Timer interrupt [100Hz]
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);                                        //PWM for Control Motor1
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);                                        //PWM for Control Motor2

	PCA9685_Go();                 //PCA9685 Initialization
	SetPWMFreq(50);               //Set Servo PWM Frequency
	setServo(0,calculate_PWM(0)); //Set Initial Angle
	HAL_Delay(200);               //Delay for SERVO response
	
	//Connect MPU6050
  //while (MPU6050_Init(&hi2c1) == 1);
	printf("[SYSTEM] MPU6050 Connected...\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//if(MPU_Sampling) //Enable MPU6050
		if(false)          //Disble MPU6050
		{
			//Read MPU6050 value and processed by Kalman Filter
      MPU6050_Read_All(&hi2c1,&MPU6050);
			//Average Filter
			if(!average_filter)
			{
				//When average filer is not executed
				Kalman_Y_buffer += MPU6050.KalmanAngleY;
				Kalman_X_buffer += MPU6050.KalmanAngleX;
				if(!plot_curve)
				{
				  printf("[LOG] Counter:%d | Kalman Filter: Enabled | Average Filter: Pending\r\n",counter);
				  printf("[LOG] X: %.2f degree | Y: %.2f degree\r\n",MPU6050.KalmanAngleX,MPU6050.KalmanAngleY);
				  printf("[SYSTEM]Please do not move your gyro sensor!\r\n");
				}else{
					printf("$%d %d;\r\n",(int)MPU6050.KalmanAngleX,(int)MPU6050.KalmanAngleY);
				}
			}else{
				//When average filter is ready
				X_output = MPU6050.KalmanAngleX - offset_x;
				Y_output = MPU6050.KalmanAngleY - offset_y;
				if(!plot_curve)
				{
				  printf("[LOG] Smoothed X: %.2f | Smoothed Y: %.2f\r\n",X_output,Y_output);
				}else{
					printf("$%d %d;\r\n",(int)X_output,(int)Y_output);
				}
				
				//Fuzzy Logic Balancing
				adjust_servo_pitch = pitch_FLC(X_output);	
				servo_control_pitch += adjust_servo_pitch;
				
				adjust_servo_raw = raw_FLC(Y_output);
				servo_control_raw += adjust_servo_raw;
				
				//Protection (Pitch servo)
				if(servo_control_pitch>=(pitch_init_angle+pitch_max))
				{
					servo_control_pitch = pitch_init_angle+pitch_max;
				}
				if(servo_control_pitch<=(pitch_init_angle+pitch_min))
				{
					servo_control_pitch = pitch_init_angle+pitch_min;
				}
				//Protection (Raw servo)
				if(servo_control_raw>=(raw_init_angle+raw_max))
				{
					servo_control_raw = raw_init_angle+raw_max;
				}
				if(servo_control_raw<=(raw_init_angle+raw_min))
				{
					servo_control_raw = raw_init_angle+raw_min;
				}				
				//Adjustment
				if(!plot_curve)	printf("servo_control[pitch]:%d   servo_control[raw]:%d\r\n",servo_control_pitch,servo_control_raw);
				setServo(pitch_servo,calculate_PWM(servo_control_pitch));
				setServo(raw_servo,calculate_PWM(servo_control_raw));
        HAL_Delay(200);
				servo_control_pitch_old = servo_control_pitch;
				servo_control_raw_old = servo_control_raw;
			}
			
			//Find average offset
			if((counter==num_samples)&&(!average_filter))
			{
				//These statements will only be executed once after taking num_samples, then disable.
				average_filter = true;
				offset_x = Kalman_X_buffer/num_samples;
				offset_y = Kalman_Y_buffer/num_samples;
			}
			printf("\r\n");
			counter++;
		}
		
		//Robot Control//
		robot_forward(30);
		
		//Robot Control//
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_IWDG_Refresh(&hiwdg); //Limit: 3.2 seconds
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
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
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  htim1.Init.Prescaler = 20;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3600;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1024;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 709;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
	
  hdma_usart3_rx.Instance = DMA1_Channel3;
  hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart3_rx.Init.PeriphDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart3_rx.Init.Mode = DMA_NORMAL;
  hdma_usart3_rx.Init.Priority = DMA_PRIORITY_HIGH;
  HAL_DMA_Init(&hdma_usart3_rx);

  __HAL_LINKDMA(&huart3,hdmatx,hdma_usart3_rx);	
  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
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
  HAL_GPIO_WritePin(GPIOC, MOTOR1_IN1_Pin|MOTOR1_IN2_Pin|MOTOR2_IN1_Pin|MOTOR2_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BLUE_BUTTON_Pin */
  GPIO_InitStruct.Pin = BLUE_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLUE_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR1_IN1_Pin MOTOR1_IN2_Pin MOTOR2_IN1_Pin MOTOR2_IN2_Pin */
  GPIO_InitStruct.Pin = MOTOR1_IN1_Pin|MOTOR1_IN2_Pin|MOTOR2_IN1_Pin|MOTOR2_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GREEN_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == BLUE_BUTTON_Pin)
	{
		HAL_GPIO_WritePin(MOTOR1_IN1_GPIO_Port,MOTOR1_IN1_Pin,GPIO_PIN_RESET);    //MOTOR1 I/O
		HAL_GPIO_WritePin(MOTOR1_IN2_GPIO_Port,MOTOR1_IN2_Pin,GPIO_PIN_RESET);    //MOTOR1 I/O
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,calculate_MOTOR_PWM(0));       //MOTOR1 PWM
		HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_Port,MOTOR2_IN1_Pin,GPIO_PIN_RESET);    //MOTOR2 I/O
		HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_Port,MOTOR2_IN2_Pin,GPIO_PIN_RESET);    //MOTOR2 I/O
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,calculate_MOTOR_PWM(0));       //MOTOR2 PWM
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart==&huart3)
	{
		if(SAMPLE_FLAG == true)
		{
		  memcpy(UART_TX_BUFFER,UART_RX_BUFFER,BUFFER_SIZE);
			printf("MSG_INDEX:%d",MSG_COUNTER);
			printf("  Sampling Intervals:0.1s(10Hz)");
		  printf(" UART3-DMA-RX=");
		  printf("%s \r\n",UART_TX_BUFFER);
      gpu_msg_analyzer();
		  HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin);
			SAMPLE_FLAG = false;
			MSG_COUNTER++;
		}
	  HAL_UART_Receive_DMA(&huart3,UART_RX_BUFFER,BUFFER_SIZE);
	}
} 

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //Timer Interrupt
{
	if(htim==&htim2)
	{
		TIMER_COUNTER++;           //Add up itself every 0.01s
		if((TIMER_COUNTER%10)==0)  //0.01x10 = 0.1s
		{
			SAMPLE_FLAG = true;
		}
		if((TIMER_COUNTER%5)==0)  //0.01x5 = 0.05s
		{
			MPU_Sampling = !MPU_Sampling;
		}		
	}
}

int calculate_MOTOR_PWM(unsigned char percentage)
{
	return (int)((percentage*3600)/100);
}

void gpu_msg_analyzer()
{
	printf("<---------------GPU MSG Breakdown--------------->\r\n");
	printf("MSG Header:%c\r\n",UART_TX_BUFFER[0]);  
	int DIGIT_LANE_DISTANCE[3];	
	DIGIT_LANE_DISTANCE[0] = (UART_TX_BUFFER[1]-'0')*100;
	DIGIT_LANE_DISTANCE[1] = (UART_TX_BUFFER[2]-'0')*10;
	DIGIT_LANE_DISTANCE[2] = (UART_TX_BUFFER[3]-'0')*1;
	LANE_DISTANCE = DIGIT_LANE_DISTANCE[0]+DIGIT_LANE_DISTANCE[1]+DIGIT_LANE_DISTANCE[2];
	int DIGIT_TARGET_X[3];
	DIGIT_TARGET_X[0] = (UART_TX_BUFFER[5]-'0')*100;
	DIGIT_TARGET_X[1] = (UART_TX_BUFFER[6]-'0')*10;
	DIGIT_TARGET_X[2] = (UART_TX_BUFFER[7]-'0')*1;	
  TARGET_X = DIGIT_TARGET_X[0]+DIGIT_TARGET_X[1]+DIGIT_TARGET_X[2];
	int DIGIT_TARGET_Y[3];
	DIGIT_TARGET_Y[0] = (UART_TX_BUFFER[9]-'0')*100;
	DIGIT_TARGET_Y[1] = (UART_TX_BUFFER[10]-'0')*10;
	DIGIT_TARGET_Y[2] = (UART_TX_BUFFER[11]-'0')*1;	
  TARGET_X = DIGIT_TARGET_X[0]+DIGIT_TARGET_X[1]+DIGIT_TARGET_X[2];	
	TARGET_Y = DIGIT_TARGET_Y[0]+DIGIT_TARGET_Y[1]+DIGIT_TARGET_Y[2];	
	printf("Lane Distance(LHS):%d\r\n",LANE_DISTANCE);
	printf("Target Object Coordinages:X=%d|Y=%d\r\n",TARGET_X,TARGET_Y);
	switch(UART_TX_BUFFER[12])
	{
		case 'S': TRAFFIC_STATUS = STOP;printf("Traffic Status = STOP\r\n");break;
		case 'L': TRAFFIC_STATUS = LEFT;printf("Traffic Status = LEFT\r\n");break;
		case 'R': TRAFFIC_STATUS = RIGHT;printf("Traffic Status = RIGHT\r\n");break;
		case 'X': TRAFFIC_STATUS = UNKOWN;printf("Traffic Status = UNKOWN\r\n");break;
	}	
  printf("<----------------------DONE---------------------->\r\n");
	printf("\r\n");
}

void robot_forward(int speed)
{
	HAL_GPIO_WritePin(MOTOR1_IN1_GPIO_Port,MOTOR1_IN1_Pin,GPIO_PIN_SET);         //MOTOR1 I/O
	HAL_GPIO_WritePin(MOTOR1_IN2_GPIO_Port,MOTOR1_IN2_Pin,GPIO_PIN_RESET);       //MOTOR1 I/O
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,calculate_MOTOR_PWM(speed));      //MOTOR1 PWM
	
	HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_Port,MOTOR2_IN1_Pin,GPIO_PIN_SET);         //MOTOR2 I/O
	HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_Port,MOTOR2_IN2_Pin,GPIO_PIN_RESET);       //MOTOR2 I/O
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,calculate_MOTOR_PWM(speed));      //MOTOR2 PWM	
}

void robot_stop(void)
{
	HAL_GPIO_WritePin(MOTOR1_IN1_GPIO_Port,MOTOR1_IN1_Pin,GPIO_PIN_RESET);     //MOTOR1 I/O
	HAL_GPIO_WritePin(MOTOR1_IN2_GPIO_Port,MOTOR1_IN2_Pin,GPIO_PIN_RESET);     //MOTOR1 I/O
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,calculate_MOTOR_PWM(0));        //MOTOR1 PWM
	
	HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_Port,MOTOR2_IN1_Pin,GPIO_PIN_RESET);     //MOTOR2 I/O
	HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_Port,MOTOR2_IN2_Pin,GPIO_PIN_RESET);     //MOTOR2 I/O
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,calculate_MOTOR_PWM(0));        //MOTOR2 PWM	
}

void robot_adjust(int motor1_speed, int motor2_speed)
{
	HAL_GPIO_WritePin(MOTOR1_IN1_GPIO_Port,MOTOR1_IN1_Pin,GPIO_PIN_SET);              //MOTOR1 I/O
	HAL_GPIO_WritePin(MOTOR1_IN2_GPIO_Port,MOTOR1_IN2_Pin,GPIO_PIN_RESET);            //MOTOR1 I/O
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,calculate_MOTOR_PWM(motor1_speed));    //MOTOR1 PWM
	
	HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_Port,MOTOR2_IN1_Pin,GPIO_PIN_SET);              //MOTOR2 I/O
	HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_Port,MOTOR2_IN2_Pin,GPIO_PIN_RESET);            //MOTOR2 I/O
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,calculate_MOTOR_PWM(motor2_speed));    //MOTOR2 PWM		
}

float trimf(float measurement, float start, float peak, float end)
{
	float fx = 0.0;
	//Triangular Membership Function
	//Remember: end > peak > start (these values are indicating x axis)
	if(measurement <= start) fx=0;
	if((measurement > start)&&(measurement <= peak))
	{
		fx = (measurement-start)/(peak-start);
	}
	if((measurement < end)&&(measurement > peak))
	{
		fx = (end-measurement)/(end-peak);
	}
	if(measurement >= end) fx = 0;
	return fx;	
}

float Rmf(float measurement, float top, float bottom)
{
	float fx = 0.0;
	//Special Trapezodial function: R-function
	//Remember: bottom > top (these values are indicating x axis)
	if(measurement>bottom) fx = 0;
	if(measurement<top) fx = 1;
	if((measurement<=bottom)&&(measurement>=top))
	{
		fx = (bottom-measurement)/(bottom-top);
	}
	return fx;
}

float Lmf(float measurement, float bottom, float top)
{
	float fx = 0.0;
	//Special Trapezodial function: L-function
	//Remember: top > bottom (these values are indicating x axis)
	if(measurement<bottom) fx = 0;
	if(measurement>top) fx = 0;
	if((measurement>=bottom)&&(measurement<=top))
	{
		fx = (measurement-bottom)/(top-bottom);
	}
	return fx;
}

float trapmf(float measurement, float start,float top_1,float top_2,float end)
{
  float fx = 0.0;
	//Trapezodial function
	//Remember: start < top_1 < top_2 <end
	if((measurement < start)||(measurement > end)) fx = 0;
	if((measurement >= start)&&(measurement <= top_1))
	{
		fx = (measurement-start)/(top_1-start);
	}
	if((measurement>=top_1)&&(measurement<=top_2))
	{
		fx = 1;
	}
	if((measurement>=top_2)&&(measurement<=end))
	{
		fx = (end-measurement)/(end-top_2);
	}
	return fx;
}
int pitch_FLC(double error)
{
	double servo_adjust = 0;
	//This controller should try to keep Y_output = 0 (balance)
	
	//1.Fuzzification
	
	//define pitch input membership functions and levels
	//1. LN  -> Largely Negative         (the object is sliding down rapidly)
	//2. N   -> Negative                 (the object is still stationary)
	//3. Z   -> Zero                     (the object is stationary)
	//4. P   -> Positive                 (the object is sliding down slowly)
  //5. LP  -> Largely Positive         (the object is sliding down rapidly)
	
  //2.Inferencing

  double w1 = MAX(Rmf(error,-15,-4),trimf(error,-10,-6,0));        //Inference LN & N ->(OR)
	double w2 = MAX(trimf(error,-10,-6,0),trapmf(error,-6,-2,2,6));  //Inference N & Z  ->(OR)
	double w3 = MAX(trapmf(error,-6,-2,2,6),trimf(error,0,6,10));    //Inference Z & P  ->(OR)
	double w4 = MAX(trimf(error,0,6,10),Lmf(error,4,15));            //Inference P & LP ->(OR)
	
	double i = MAX(w1,w2); //Negative OR Zero    ->(OR)
	double j = MAX(w3,w4); //Positive OR Zero    ->(OR)

	printf("i:%.2f j:%.2f\r\n",i,j);
	//3.Defuzzification: Weighted Average Method
	
	//define servo adjustment membership functions and levels
	//1. XC  -> Clockwise                     
	//2. CC  -> Counterclockwise 
  servo_adjust = (i*pitch_max+j*pitch_min);    //Find out ratio of clockwise and and counterclocwise
	if(!plot_curve)	printf("Servo_adjust: %.2f\r\n",servo_adjust);

	return servo_adjust;
}

int raw_FLC(double error)
{
	double servo_adjust = 0;
	//This controller should try to keep Y_output = 0 (balance)
	
	//1.Fuzzification
	
	//define pitch input membership functions and levels
	//1. LN  -> Largely Negative         (the object is sliding down rapidly)
	//2. N   -> Negative                 (the object is still stationary)
	//3. Z   -> Zero                     (the object is stationary)
	//4. P   -> Positive                 (the object is sliding down slowly)
  //5. LP  -> Largely Positive         (the object is sliding down rapidly)
	
  //2.Inferencing

  double w1 = MAX(Rmf(error,-15,-4),trimf(error,-10,-6,0));        //Inference LN & N ->(OR)
	double w2 = MAX(trimf(error,-10,-6,0),trapmf(error,-6,-2,2,6));  //Inference N & Z  ->(OR)
	double w3 = MAX(trapmf(error,-6,-2,2,6),trimf(error,0,6,10));    //Inference Z & P  ->(OR)
	double w4 = MAX(trimf(error,0,6,10),Lmf(error,4,15));            //Inference P & LP ->(OR)
	
	double i = MAX(w1,w2); //Negative OR Zero    ->(OR)
	double j = MAX(w3,w4); //Positive OR Zero    ->(OR)

	printf("i:%.2f j:%.2f\r\n",i,j);
	//3.Defuzzification: Weighted Average Method
	
	//define servo adjustment membership functions and levels
	//1. XC  -> Clockwise                     
	//2. CC  -> Counterclockwise 
  servo_adjust = (i*pitch_max+j*pitch_min);    //Find out ratio of clockwise and and counterclocwise
	if(!plot_curve)	printf("Servo_adjust: %.2f\r\n",servo_adjust);

	return servo_adjust;
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
  printf("[SYSTEM] Error! Please contact Vincent\r\n");
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
