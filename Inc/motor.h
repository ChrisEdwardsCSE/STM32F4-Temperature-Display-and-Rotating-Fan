/*
 * motor.h
 *
 *  Created on: Aug 11, 2024
 *      Author: Christopher Edwards
 */
#include <stdint.h>
#include "stm32f4xx_hal.h"

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#define FAN_LEVEL_1 25
#define FAN_LEVEL_2 50
#define FAN_LEVEL_3 100

#define PID_MAX_OUTPUT 980 // Maximum Compare value of PWM

/**
 * Encoder feedback struct
 */
typedef struct
{
	int16_t velocity;
	int32_t position;
	uint32_t prev_count;
} encoder_instance;

/**
 * PID Instance struct
 */
typedef struct
{
	float p_gain;
	float i_gain;
	float d_gain;
	int16_t sample_freq;
	int16_t prev_error;
	int32_t error_sum;
	int16_t output;
} pid_struct;

/**
 * Takes feedback from encoder and computes data of motor
 *
 * @param encoder - The encoder instance
 * @param htim - The encoder timer
 */
void update_encoder(encoder_instance *encoder, TIM_HandleTypeDef *htim);

/**
 * Motor PID Algorithm. Gives output as PWM signal for motor
 *
 * @param pid_handler - PID Controller handler
 * @param error - the error difference between the reference speed and the encoder feedback speed
 */
void pid_fan(pid_struct *pid_handler, int16_t error);

#endif /* INC_MOTOR_H_ */
