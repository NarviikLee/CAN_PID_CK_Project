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
#include "can.h"
#include "iwdg.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "PID_M.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union {
    uint8_t bytes[8];
    struct {
        int32_t target_velocity; // 4바이트: 목표 속도
        int16_t current_torque;  // 2바이트: 현재 토크
        uint8_t status;          // 1바이트: 상태
        uint8_t mode;            // 1바이트: 동작 모드
    } data;
} MotorPacket;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
//uint8_t TxData[8];
//volatile uint8_t RxData[8];
uint32_t TxMailbox;

MotorPacket txPacket;
volatile MotorPacket rxPacket;

uint32_t last_tx_tick = 0;
volatile uint8_t wakeup_flag = 0;

PID_Controller motorPID;

// 2. 모터 제어 변수 설정
float target_rpm = 100.0f;   // 목표 회전수 (RPM)
float current_rpm = 0.0f;    // 현재 측정 회전수
float control_output = 0.0f; // PID 계산 결과 (PWM Duty)

// 3. 모터 설정 (N20 모터에 맞춰 파라미터 설정)
const float PPR = 11.0f;       // 한 바퀴당 펄스
const float GEAR_RATIO = 50.0f; // 기어비 (예: 1:50)
const float SAMPLING_TIME = 0.01f; // 10ms (TIM3 주기)

volatile uint8_t button_wake = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Update_Motor_Drive(float output);
void Set_RTC_Alarm_Seconds(uint8_t seconds);
float Get_Motor_RPM(void);
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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
//  MX_IWDG_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  memset(&TxHeader, 0, sizeof(TxHeader));
  PID_Init(&motorPID, 20.0f, 10.0f, 0.0f, SAMPLING_TIME, -4199.0f, 4199.0f);

    // 2. 타이머 및 인터럽트 시작
  //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  HAL_TIM_Base_Start_IT(&htim3);      // 10ms 제어 주기 타이머 시작
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL); // 엔코더 카운터 시작
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);       // PWM 출력 시작
  printf("System Ready: PID Motor Control\r\n");

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
      printf("🔥 범인 검거: IWDG (독립형 와치독) 리셋!\r\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
      printf("🔥 범인 검거: WWDG (윈도우 와치독) 리셋!\r\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
      printf("🔥 범인 검거: POR/PDR (전원 불안정) 리셋!\r\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
      printf("🔥 범인 검거: 소프트웨어 리셋!\r\n");
  }
    // 확인 후 리셋 플래그 깨끗하게 지우기
    __HAL_RCC_CLEAR_RESET_FLAGS();

  TxHeader.StdId = 0x123;
  TxHeader.ExtId = 0x01;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;

  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 1;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

  // 🚨 필터 ID를 0으로 몽땅 비우면 모든 신호를 다 받습니다.
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;

  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
      Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
      Error_Handler();
  }
  HAL_Delay(10);
  if (HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_LISTENING) {
      if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txPacket.bytes, &TxMailbox) != HAL_OK) {
          // 에러 발생 시 처리 코드
          uint32_t err = HAL_CAN_GetError(&hcan1);
          printf("CAN Send Error: %lu\r\n", err);
      }
  }
    // 5. 초기 송신 테스트 (AddTxMessage)

//  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_WAKEUP);
  txPacket.data.target_velocity = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      // =================================================================
      // STEP 1: 기상 직후 시스템 대기 시작
      // (아두이노의 첫 번째 '더미 프레임'을 맞고 여기서부터 깨어납니다!)
      // =================================================================
      printf("\r\n🌞 Woke up from SLEEP Mode! Waiting for Arduino Command...\r\n");

      // =================================================================
      // STEP 2: 아두이노가 보내는 '진짜 명령(Rx)' 수신 대기
      // =================================================================
      uint32_t timeout = HAL_GetTick();
      // 아두이노가 깨우고 나서 진짜 명령을 보낼 때까지 최대 1초 정도 기다려줌
      while (wakeup_flag == 0) {
          if (HAL_GetTick() - timeout > 1000) {
              printf("⚠️ Command Receive Timeout! Going back to sleep...\r\n");
              break;
          }
      }

      // =================================================================
      // STEP 3: 명령 수신 확인 후 모터 구동
      // =================================================================
      if (wakeup_flag == 1) {
          printf("📥 CAN Command Received from Arduino! Target RPM: %ld\r\n", rxPacket.data.target_velocity);
          wakeup_flag = 0; // 플래그 초기화

          target_rpm = (float)rxPacket.data.target_velocity;

          printf("🔄 PID Auto Driving & Monitoring (3 Seconds)...\r\n");

          for (int i = 0; i < 300; i++) {
              if (i % 50 == 0) {
                  printf("   [MONITOR] Current RPM: %.2f\r\n", current_rpm);
              }
              HAL_Delay(10);
          }
      }
      // =================================================================
      // STEP 4: 구동 종료 후 안전 정지 및 SLEEP 모드 진입 준비
      // =================================================================
      printf("⏸️ Mission Complete. Stopping Motor and Preparing for Sleep...\r\n");
      HAL_TIM_Base_Stop_IT(&htim3);
      Update_Motor_Drive(0.0f);
      target_rpm = 0.0f;

      // =================================================================
      // STEP 5: SLEEP 모드 (아두이노가 깨울 때까지 대기 💤)
      // =================================================================
      printf("⏸️ I'll Going to Sleep Mode...\r\n");

      // [중요] 마지막 로그가 UART를 통해 완전히 나갈 때까지 대기
      while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET);

      // 🚨 [핵심] SLEEP 모드에서는 Systick을 끄면 아예 안 일어날 수 있으므로 유지합니다.
      // HAL_SuspendTick(); // 주석 처리 또는 삭제

      // CPU를 SLEEP 모드로 진입 (클럭 유지, CPU만 정지)
      printf("⏸️ Going to Sleep Mode...\r\n");
      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

      // =================================================================
      // STEP 6: 깨어난 직후 시스템 복구
      // =================================================================
      // 🚨 [핵심 변경] SLEEP 모드에서는 클럭이 죽지 않으므로 SystemClock_Config()를 호출하면 안 됩니다! (호출 시 충돌 발생)
      // SystemClock_Config(); <- 삭제됨
      // HAL_ResumeTick();     <- Suspend를 안 했으므로 삭제됨

      // 클럭이 유지되었으므로 바로 로그 출력이 가능합니다.
      printf("⏸️ I'll Wake UP...\r\n");

      // CAN 컨트롤러는 이미 살아있으므로 WakeUp 함수 생략 가능
      // HAL_CAN_WakeUp(&hcan1); <- 삭제됨

      HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
      HAL_TIM_Base_Start_IT(&htim3);

      printf("✅ System Restored! Ready for next command.\r\n");
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
}

/* USER CODE BEGIN 4 */

static uint16_t stall_counter = 0;
// 모터 차단 상태를 기억하는 변수 (1이면 차단됨)
static uint8_t motor_fault = 0;

int _write(int file, char *ptr, int len)
{
    // MX_USART2_UART_Init()에서 설정된 UART2를 통해 printf를 출력합니다.
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, (uint8_t *)rxPacket.bytes) == HAL_OK)
        {
            wakeup_flag = 1; // [핵심] 메시지 수신 플래그를 1로 세팅!
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // 보드 LED 토글
        }
    }
}

void Set_RTC_Alarm_Seconds(uint8_t seconds)
{
    RTC_AlarmTypeDef sAlarm = {0};
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0}; // 날짜 구조체 추가

    // 현재 시간을 먼저 읽어옵니다 (현재 시간을 기준으로 알람을 맞춰야 하니까요)
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); // 이 코드가 빠지면 시간 갱신이 안 될 수 있습니다.

    sAlarm.AlarmTime.Hours = sTime.Hours;
    sAlarm.AlarmTime.Minutes = sTime.Minutes;

    sAlarm.AlarmTime.Seconds = (sTime.Seconds + seconds) % 60; // 현재 초에 + n초를 더함
    sAlarm.AlarmTime.SubSeconds = 0;

    // 시, 분은 무시하고 '초'만 일치하면 알람 발생
    sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES;
    sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;

    sAlarm.Alarm = RTC_ALARM_A;

    if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) // 10ms 타이머 인터럽트
    {
        current_rpm = Get_Motor_RPM(); // 1. 현재 속도 측정

        // 2. PID 계산 (목표 속도와 현재 속도를 비교하여 PWM 값 뽑아냄)
        control_output = PID_Compute(&motorPID, target_rpm, current_rpm);

        // 3. 모터에 PWM 인가
        Update_Motor_Drive(control_output);
    }
}

// 모터 제어 신호 업데이트 (방향 및 속도)
void Update_Motor_Drive(float output)
{
    if (motor_fault == 1) {
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_DIR2_GPIO_Port, MOTOR_DIR2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        return; // 작동 중지
    }

    uint32_t pwm_value = (uint32_t)(fabs(output));

    // =================================================================
    // 🚨 [안전] 스톨(Stall) 감지 방어 로직
    // 조건: PWM 출력이 80%(약 3300) 이상으로 빵빵한데, 현재 RPM이 5 미만일 때
    // =================================================================
    if (pwm_value > 3300 && fabs(current_rpm) < 5.0f) {
        stall_counter++; // 10ms마다 카운터 증가

        // 0.5초(10ms * 50번) 이상 계속 막혀있다면 -> 모터 타기 직전!
        if (stall_counter > 50) {
            motor_fault = 1; // 모터 고장 플래그 발동 (더 이상 구동 불가)
            printf("🚨 [FATAL ERROR] Motor Stall Detected! System Shutdown.\r\n");

            // 전력 즉각 차단
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_DIR2_GPIO_Port, MOTOR_DIR2_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            return;
        }
    } else {
        // 모터가 정상적으로 돌고 있거나, 대기 중일 때는 카운터 초기화
        stall_counter = 0;
    }

    // =================================================================
    // 2. 모터 구동 로직 (기존 코드와 동일)
    // =================================================================
    if (output >= 0) {
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_DIR2_GPIO_Port, MOTOR_DIR2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);
    } else {
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_DIR2_GPIO_Port, MOTOR_DIR2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);
    }
}

float Get_Motor_RPM(void)
{
    // 1. 하드웨어 타이머(TIM8)에서 엔코더 펄스 값 읽어오기
    int16_t raw_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim8);

    // 2. 다음 주기를 위해 카운터 0으로 초기화
    __HAL_TIM_SET_COUNTER(&htim8, 0);

    // 3. RPM 계산 공식 적용
    // RPM = (측정 펄스 / 1바퀴당 총 펄스) * (1초/샘플링시간) * 60초
    float rpm = ((float)raw_cnt / (PPR * GEAR_RATIO * 4.0f)) * (1.0f / SAMPLING_TIME) * 60.0f;

    return rpm;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == B1_Pin) // B1_Pin이 PC13으로 설정되어 있으므로 이를 확인
    {
    	button_wake = 1;
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
      //HAL_UART_Transmit(&huart2, (uint8_t *)"🚨 Error Handler Trap!\r\n", 26, 100);
      HAL_Delay(1000); // 1초마다 출력 (원래 Systick이 죽어서 Delay가 안 먹힐 수 있으나, 일단 시도)
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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
