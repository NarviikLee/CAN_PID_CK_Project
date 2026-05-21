/*
 * PID_M.c
 *
 *  Created on: 2026. 5. 11.
 *      Author: ihg78
 */


#include "PID_M.h"

/**
  * @brief PID 제어기 초기화
  */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float dt, float outMin, float outMax) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->dt = dt;
    pid->outMin = outMin;
    pid->outMax = outMax;
    pid->integralLimit = outMax * 0.5f; // 일반적으로 출력의 50~80%로 설정

    PID_Reset(pid);
}

/**
  * @brief PID 계산 수행
  * 이산형 PID 수식: U[n] = Kp*E[n] + Ki*sum(E)*dt + Kd*(E[n]-E[n-1])/dt
  */
float PID_Compute(PID_Controller *pid, float target, float current) {
    float error = target - current;

    // P 항
    float pTerm = pid->Kp * error;

    // I 항 (누적 오차)
    pid->integral += error * pid->dt;

    // Anti-Windup: 적분 값이 너무 커지지 않게 제한
    if (pid->integral > pid->integralLimit) pid->integral = pid->integralLimit;
    else if (pid->integral < -pid->integralLimit) pid->integral = -pid->integralLimit;

    float iTerm = pid->Ki * pid->integral;

    // D 항 (오차 변화율)
    float dTerm = pid->Kd * (error - pid->prevError) / pid->dt;

    // 최종 출력
    float output = pTerm + iTerm + dTerm;

    // 출력 제한 (Saturation)
    if (output > pid->outMax) output = pid->outMax;
    else if (output < pid->outMin) output = pid->outMin;

    // 다음 계산을 위해 현재 오차 저장
    pid->prevError = error;

    return output;
}

/**
  * @brief 누적값 초기화 (모터 정지 시 등 사용)
  */
void PID_Reset(PID_Controller *pid) {
    pid->prevError = 0.0f;
    pid->integral = 0.0f;
}
