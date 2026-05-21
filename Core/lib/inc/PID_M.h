/*
 * PID_M.h
 *
 *  Created on: 2026. 5. 11.
 *      Author: ihg78
 */

#ifndef LIB_INC_PID_M_H_
#define LIB_INC_PID_M_H_

#include "main.h" // HAL 라이브러리 사용을 위해 포함

typedef struct {
    /* 제어 파라미터 (Gain) */
    float Kp;
    float Ki;
    float Kd;

    /* 오차 및 누적 값 */
    float prevError;
    float integral;

    /* 출력 제한 (Saturation) */
    float outMin;
    float outMax;

    /* 적분 폭주 방지 (Anti-Windup) */
    float integralLimit;

    /* 샘플링 주기 (초 단위, 예: 0.01f = 10ms) */
    float dt;
} PID_Controller;

/* 함수 프로토타입 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float dt, float outMin, float outMax);
float PID_Compute(PID_Controller *pid, float target, float current);
void PID_Reset(PID_Controller *pid);

#endif /* LIB_INC_PID_M_H_ */
