/*
 * motor.c
 *
 *  Created on: Aug 11, 2024
 *      Author: Christopher Edwards
 */
#include "motor.h"

/**
 * Takes feedback from encoder and computes data of motor
 *
 * @param encoder - The encoder instance
 * @param htim - The encoder timer
 */
void update_encoder(encoder_instance *encoder, TIM_HandleTypeDef *htim)
{
	uint32_t temp_count = htim->Instance->CNT;
	static uint8_t first_time = 0;

	// initialize velocity = 0
	if (!first_time)
	{
		encoder->velocity = 0;
		first_time = 1;
	}
	else
	{
		if (temp_count == encoder->prev_count)
		{
			encoder->velocity = 0;
		}
		else if (temp_count > encoder->prev_count)
		{
			// if counting down && temp_count greater than the previous, then overflow, there was overflow
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			{
				encoder->velocity = -(encoder->velocity) - (__HAL_TIM_GET_AUTORELOAD(htim) - temp_count);
			}
			// otherwise velocity just difference between count and prev_count
			else
			{
				encoder->velocity = temp_count - encoder->prev_count;
			}
		}
		// else if prevcount > count
		else
		{
			// overflow
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			{
				encoder->velocity = temp_count - encoder->prev_count;
			}
			else
			{
				encoder->velocity = temp_count + (htim->Instance->ARR - encoder->prev_count);
			}
		}
	}
	encoder->position += encoder->velocity;
	encoder->prev_count = temp_count;
}

/**
 * Motor PID Algorithm. Gives output as PWM signal for motor
 *
 * @param pid_handler - PID Controller handler
 * @param error - the error difference between the reference speed and the encoder feedback speed
 */
void pid_fan(pid_struct *pid_handler, int16_t error)
{
	pid_handler->error_sum += error;
	pid_handler->output = ( pid_handler->p_gain * error ) +
						( (pid_handler->i_gain * pid_handler->error_sum) / pid_handler->sample_freq ) +
						( pid_handler->d_gain * ((error - pid_handler->prev_error) * pid_handler->sample_freq) );
	pid_handler->output *= 8.91;

	pid_handler->prev_error = error;
	if (pid_handler->output >= PID_MAX_OUTPUT)
	{
		pid_handler->output = PID_MAX_OUTPUT;
	}
	if (pid_handler->output <= -PID_MAX_OUTPUT)
	{
		pid_handler->output = -PID_MAX_OUTPUT;
	}
	pid_handler->prev_error = error;
}
