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
        int32_t target_velocity; // 4바이?��: 목표 ?��?��
        int16_t current_torque;  // 2바이?��: ?��?�� ?��?��
        uint8_t status;          // 1바이?��: ?��?��
        uint8_t mode;            // 1바이?��: ?��?�� 모드
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

// 2. ?��?�� �????�� �????��
float target_rpm = 100.0f;   // 목표 ?��?�� (RPM)
float current_rpm = 0.0f;    // ?��?�� 측정 ?��?��
float control_output = 0.0f; // PID 계산 결과 (PWM Duty)

// 3. 모터 ?��?�� (N20 모터?�� 맞춰 ?��?�� ?��?��)
const float PPR = 11.0f;       // ?�� 바�?�당 ?��?��
const float GEAR_RATIO = 50.0f; // 기어�??? (?��: 1:50)
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
//	HAL_DBGMCU_EnableDBGSleepMode();
//	HAL_DBGMCU_EnableDBGStopMode();
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
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  memset(&TxHeader, 0, sizeof(TxHeader));
//  PID_Init(&motorPID, 1.0f, 0.5f, 0.01f, SAMPLING_TIME, -4199.0f, 4199.0f);
  PID_Init(&motorPID, 20.0f, 10.0f, 0.0f, SAMPLING_TIME, -4199.0f, 4199.0f);

    // 2. ???���??? ?���??? ?��?��
  HAL_TIM_Base_Start_IT(&htim3);      // 10ms ?��?�� 주기 ???���??? (?��?��?��?��)
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL); // ?��코더 카운?�� ?��?��
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);       // PWM 출력 ?��?��
  printf("System Ready: PID Motor Control\r\n");

  TxHeader.StdId = 0x123;
  TxHeader.ExtId = 0x01;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;
  CAN_FilterTypeDef sFilterConfig;

  // 2. ?��?�� ?���???????? ?��?��
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14; // CAN2�???????? ?��?�� ?��?�� 뱅크 ?��?��

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
      Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
      Error_Handler();
  }
  HAL_Delay(10);
  if (HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_LISTENING) {
      if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txPacket.bytes, &TxMailbox) != HAL_OK) {
          // ?��?�� 발생 ?�� ?��?�� 코드 ?��?��
          uint32_t err = HAL_CAN_GetError(&hcan1);
          printf("CAN Send Error: %lu\r\n", err);
      }
  }
	// 5. ?��?�� ?��?�� 초기?�� (AddTxMessage ?��?��)


  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  txPacket.data.target_velocity = 0;
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
      printf("!!! System Reset by Watchdog !!!\r\n");
      __HAL_RCC_CLEAR_RESET_FLAGS(); // ?��?���?????? 초기?��
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // =================================================================
	// STEP 1: 기상 직후 시스템 복구 및 와치독 피딩
	// =================================================================
	HAL_IWDG_Refresh(&hiwdg);
	printf("\r\n🌞 Woke up from STOP Mode! Restoring system...\r\n");

	// =================================================================
	// STEP 2: CAN 통신을 통한 구동 명령 전달 (Loopback 테스트)
	// =================================================================
	txPacket.data.target_velocity = 100; // 50% 출력 명령 데이터 세팅
	txPacket.data.mode = 1;               // 구동 모드 활성화

	printf("📡 Sending CAN Command...\r\n");
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txPacket.bytes, &TxMailbox);

	// 인터럽트로 수신(wakeup_flag == 1)될 때까지 아주 잠시 대기
	uint32_t timeout = HAL_GetTick();
	while (wakeup_flag == 0) {
		if (HAL_GetTick() - timeout > 100) {
			printf("⚠️ CAN Receive Timeout!\r\n");
			break;
		}
	}
	if (wakeup_flag == 1) {
		printf("📥 CAN Command Received! Target RPM: %ld\r\n", rxPacket.data.target_velocity);
		wakeup_flag = 0;

		// 💡 [핵심] 모터를 직접 돌리지 않고, 전역 변수(목표 속도)만 갱신!
		// 그러면 TIM3 인터럽트가 알아서 이 목표값을 보고 PID 제어를 시작합니다.
		target_rpm = (float)rxPacket.data.target_velocity;

		printf("🔄 PID Auto Driving & Monitoring (3 Seconds)...\r\n");

		for (int i = 0; i < 300; i++) {
			// Update_Motor_Drive(...); // 삭제! (TIM3가 함)
			// current_rpm = Get_Motor_RPM(); // 삭제! (TIM3가 함)

			if (i % 50 == 0) {
				printf("   [MONITOR] Current RPM: %.2f\r\n", current_rpm); // 읽기만 함!
			}

			HAL_Delay(10);
			HAL_IWDG_Refresh(&hiwdg);
		}
	}

	// =================================================================
	// STEP 4: 구동 종료 후 안전 정지 및 STOP 모드 진입 준비
	// =================================================================
	printf("⏸️ Mission Complete. Stopping Motor and Preparing for Sleep...\r\n");

	// 💡 멈출 때는 사공(TIM3)을 잠시 퇴근시키고 전원을 완전히 차단하는 것이 안전합니다.
	HAL_TIM_Base_Stop_IT(&htim3);
	Update_Motor_Drive(0.0f);
	target_rpm = 0.0f;

	// UART 전송 대기 및 기타 수면 준비 (기존 동일)
	while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET);
	Set_RTC_Alarm_Seconds(3);
	HAL_IWDG_Refresh(&hiwdg);
	HAL_SuspendTick();

	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);

	// =================================================================
	// STEP 6: 기상 직후 시스템 복구
	// =================================================================
	SystemClock_Config();
	HAL_ResumeTick();

	// 💡 잠에서 깨어났으니 사공(TIM3 PID 제어기) 다시 출근!
	HAL_TIM_Base_Start_IT(&htim3);
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
// 안전 차단 상태를 기억하는 변수 (1이면 차단됨)
static uint8_t motor_fault = 0;

int _write(int file, char *ptr, int len)
{
    // MX_USART2_UART_Init()?���??????? ?��?��?�� UART2�??????? ?��?��?��?���??????? �????????��
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
    	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, (uint8_t *)rxPacket.bytes) == HAL_OK)
		{
			wakeup_flag = 1; // [?��?��] 메시�???????�??????? ?���??????? ?��?��그�?? ?��?��?��?��!
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // ?��?�� ?��?��?��
		}
    }
}
void Set_RTC_Alarm_Seconds(uint8_t seconds)
{
    RTC_AlarmTypeDef sAlarm = {0};
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0}; // ?���????? �??????�� 추�?
    // ?��?�� ?��간을 먼�? ?��?��?��?��?�� (?��?�� ?���????? 기�??���????? ?��?��?�� 맞춰?�� ?��?��까요)
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); // ?�� 코드�????? 빠�?�????? ?��?�� ?��?��?�� ?�� ?�� ?�� ?��?��

    sAlarm.AlarmTime.Hours = sTime.Hours;
    sAlarm.AlarmTime.Minutes = sTime.Minutes;

    sAlarm.AlarmTime.Seconds = (sTime.Seconds + seconds) % 60; // ?��?�� ?���????? + n�????? ?��
    sAlarm.AlarmTime.SubSeconds = 0;

//    sAlarm.AlarmMask = RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES; // ?��, 분�? 무시?���????? '�?????'�????? ?��치하�????? ?��?�� 발생
    sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES;
    sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;

//    sAlarm.AlarmDateWeekDay = 1;
    sAlarm.Alarm = RTC_ALARM_A;

    if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
    {
        Error_Handler();
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) // 10ms ?��?�� ???���???
    {
        // A. ?��코더 �??? ?���??? �??? RPM �????��
        // int16_t�??? 캐스?��?��?�� ?��?��?�� ?�� 마이?��?�� 값을 ?��??�??? ?��?��?��?��?��.
//        int16_t raw_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim8);
//        __HAL_TIM_SET_COUNTER(&htim8, 0); // ?��?�� 계산?�� ?��?�� 카운?�� 리셋
//
//        // RPM 계산 공식: (?��?��?�� / ?��바�?�당총펄?��) * (1�???/?��?��링주�???) * 60�???
//        current_rpm = ((float)raw_cnt / (PPR * GEAR_RATIO * 4.0f)) * (1.0f / SAMPLING_TIME) * 60.0f;
//
//        // B. PID 계산 ?��?��
//        control_output = PID_Compute(&motorPID, target_rpm, current_rpm);
//
//        // C. 모터 출력 ?��?��
//        Update_Motor_Drive(control_output);
    	current_rpm = Get_Motor_RPM(); // 1. 현재 속도 읽기

		// 2. PID 계산 (타겟 속도와 현재 속도를 비교해서 PWM 값 뽑아냄)
		control_output = PID_Compute(&motorPID, target_rpm, current_rpm);

		// 3. 모터에 PWM 쏘기
		Update_Motor_Drive(control_output);
    }
}

// 모터 ?��?��?���??? ?��?�� ?��?�� (?��?��)
void Update_Motor_Drive(float output)
{
//	uint32_t pwm_value = (uint32_t)(fabs(output)); // ?��??�? �??��
//
//	if (output >= 0) {
//		// ?��?��?�� (AIN1 = HIGH, AIN2 = LOW)
//		HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);    // PB0
//		HAL_GPIO_WritePin(MOTOR_DIR2_GPIO_Port, MOTOR_DIR2_Pin, GPIO_PIN_RESET); // PB1 (방금 추�??�� ??)
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);
//	} else {
//		// ?��?��?�� (AIN1 = LOW, AIN2 = HIGH)
//		HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(MOTOR_DIR2_GPIO_Port, MOTOR_DIR2_Pin, GPIO_PIN_SET);
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);
//	}
	if (motor_fault == 1) {
		HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_DIR2_GPIO_Port, MOTOR_DIR2_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		return; // 함수 탈출
	}

	uint32_t pwm_value = (uint32_t)(fabs(output));

	// =================================================================
	// 🚨 [핵심] 스톨(Stall) 감지 방어 로직
	// 조건: PWM 출력은 80%(약 3300) 이상으로 빵빵한데, 실제 RPM은 5 미만일 때
	// =================================================================
	if (pwm_value > 3300 && fabs(current_rpm) < 5.0f) {
		stall_counter++; // 10ms마다 카운트 증가

		// 0.5초(10ms * 50번) 동안 계속 막혀있다면? -> 터지기 직전!
		if (stall_counter > 50) {
			motor_fault = 1; // 에러 플래그 발동 (퓨즈 끊어짐)
			printf("🚨 [FATAL ERROR] Motor Stall Detected! System Shutdown.\r\n");

			// 전원 즉각 차단
			HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR_DIR2_GPIO_Port, MOTOR_DIR2_Pin, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			return;
		}
	} else {
		// 모터가 정상적으로 돌고 있거나, 전기를 적게 주고 있다면 카운터 초기화
		stall_counter = 0;
	}

	// =================================================================
	// 2. 정상 구동 로직 (기존 코드와 동일)
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
    // 1. ?��?��?��?�� ???���??(TIM8)?��?�� ?��?��?�� ?��?�� �?? ?���??
    int16_t raw_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim8);

    // 2. ?��?�� 주기�?? ?��?�� 카운?�� 0?���?? 초기?��
    __HAL_TIM_SET_COUNTER(&htim8, 0);

    // 3. RPM 계산 공식 ?��?��
    // RPM = (측정 ?��?�� / 1?��?��?�� �?? ?��?��) * (1�??/?��?��링시�??) * 60�??
    float rpm = ((float)raw_cnt / (PPR * GEAR_RATIO * 4.0f)) * (1.0f / SAMPLING_TIME) * 60.0f;

    return rpm;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == B1_Pin) // B1_Pin이 PC13으로 설정되어 있다고 가정
    {
        // STOP 모드에서 버튼을 누르면 이 함수가 가장 먼저 실행됩니다.
        // 클럭이 아직 완벽히 복구되기 전이므로 복잡한 작업(printf 등)은 피하고 간단한 플래그만 세우는 것이 좋습니다.

        // 예: 버튼으로 깨어났다는 것을 표시하는 변수
        // (미리 맨 위에 volatile uint8_t button_wake = 0; 선언 필요)
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
