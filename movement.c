/*
 * movement.c
 *
 *  Created on: Jan 25, 2026
 *      Author: GC
 */

#include <stdint.h>
#include "main.h"
#include "movement.h"

LineMode line_mode = WHITE_LINE_MODE;
int16_t kp = 70;
int16_t ki = 0;
int16_t kd = 0;
int16_t BASE_SPEED = 140;
int16_t global_threshold = 500;  // Default threshold for all sensors
float threshold[5] = {500,500,500,500,500};		// threshold to distinguish between black and white
int16_t sharp_left_0 = -100;
int16_t sharp_left_1 = 150;
int16_t sharp_right_0 = 150;
int16_t sharp_right_1 = -100;
uint16_t raw_adc[NUM_SENSORS];
uint16_t normalized_sensors[NUM_SENSORS];
// Default sensor min and max values
uint16_t sensor_min[NUM_SENSORS] = {1500,1500,1500,1500,1500};
uint16_t sensor_max[NUM_SENSORS] = {4090,4090,4090,4090,4090};

void set_line_mode_black(void) {
	line_mode = BLACK_LINE_MODE;
}

void set_line_mode_white(void) {
	line_mode = WHITE_LINE_MODE;
}

void lineSensorNormalization(void) {
    for(int i = 0; i < NUM_SENSORS; i++) {
        uint16_t raw = raw_adc[i];

        // Clamping to min or max range if sensor readings exceeds
        if(raw < sensor_min[i]) raw = sensor_min[i];
        if(raw > sensor_max[i]) raw = sensor_max[i];

        // BLACK LINE = LOW VALUE (0)
        // WHITE SURFACE = HIGH VALUE (1000)
        uint32_t range = sensor_max[i] - sensor_min[i];
        if(range == 0) range = 1;  // To avoid division by zero error

        uint16_t normalized = ((raw - sensor_min[i]) * 1000) / range;
        // INVERT if in black line mode (line is dark, background is light)
        if (line_mode == BLACK_LINE_MODE) {
            normalized_sensors[i] = 1000 - normalized;
        } else {
            // White line mode (line is light, background is dark)
            normalized_sensors[i] = normalized;
        }
    }
}

int16_t* PID(void) {
	// base speed (0 - 255)
    static int16_t motor_speeds[2];		// motor_speeds[0] => left & motor_speeds[1] => right
    float active_threshold = (line_mode == BLACK_LINE_MODE) ? 50 : 500; // Can use different thresholds
    // sharp left turn
    if (normalized_sensors[0] > active_threshold && normalized_sensors[4] < active_threshold) {
    	motor_speeds[0] = sharp_left_0;
    	motor_speeds[1] = sharp_left_1;
        return motor_speeds;
    }

    // sharp right turn
    else if (normalized_sensors[4] > active_threshold && normalized_sensors[0] < active_threshold) {
    	motor_speeds[0] = sharp_right_0;
    	motor_speeds[1] = sharp_right_1;
        return motor_speeds;
    }

    // PID
    else if (normalized_sensors[2] > active_threshold) {
        float position;
        float sensor_sum = normalized_sensors[0] + normalized_sensors[1] + normalized_sensors[2] + normalized_sensors[3] + normalized_sensors[4];

        if(sensor_sum != 0) {
            position = ((normalized_sensors[0] * (-2)) +
                        (normalized_sensors[1] * (-1)) +
                        (normalized_sensors[2] * (0))  +
                        (normalized_sensors[3] * (1))  +
                        (normalized_sensors[4] * (2))   ) / sensor_sum;		// Weighted average
        }

        else {
            position = 0;		// For divide by zero error
        }

		static int16_t last_error = 0;
		static int16_t integral = 0;

		integral = 0;
		float error = position;
		int16_t derivative = error - last_error;
		last_error = error;

		// kp , ki , kd tuning parameters
		float correction = (kp * error + ki * integral + kd * derivative);
		// Divided by 10 to reduce kp, ki, kd values while tuning


		motor_speeds[0] = BASE_SPEED + correction;  // Left motor speed
		motor_speeds[1] = BASE_SPEED - correction;  // Right motor speed

		// Clamping motor speeds to PWM range if it exceeds
		if(motor_speeds[0] > 255) motor_speeds[0] = 255;
		if(motor_speeds[0] < -255) motor_speeds[0] = -255;
		if(motor_speeds[1] > 255) motor_speeds[1] = 255;
		if(motor_speeds[1] < -255) motor_speeds[1] = -255;

		return motor_speeds;
    }
    return motor_speeds;		// Default condition : If line is lost it will perform the previous action
}

void lineSensorCalibration(void)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
    HAL_Delay(3000);

    for(int i = 0; i < NUM_SENSORS; i++) {
        sensor_min[i] = raw_adc[i];
    }

    //  Signal Next Step (White Calibration)
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
    HAL_Delay(3000);

    for(int i = 0; i < NUM_SENSORS; i++) {
        sensor_max[i] = raw_adc[i];
    }

    //  Signal Ready (Get in Position)
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
    HAL_Delay(7000);
}

void set_motor_speeds(int16_t right, int16_t left)
{
	#define PWM_MAX 255
    uint16_t dutyL = (left >= 0) ? left : -left;
    if (dutyL > PWM_MAX) dutyL = PWM_MAX;

    if (left >= 0)
    {
        // A forward: MA-1 = PWM, MA-2 = 0
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, dutyL);   // PA0
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);      // PA1
    }
    else
    {
        // A reverse: MA-1 = 0, MA-2 = PWM
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, dutyL);
    }

    uint16_t dutyR = (right >= 0) ? right : -right;
    if (dutyR > PWM_MAX) dutyR = PWM_MAX;

    if (right >= 0)
    {
        // B forward: MB-1 = PWM, MB-2 = 0
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, dutyR);   // PB10
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);      // PB11
    }
    else
    {
        // B reverse: MB-1 = 0, MB-2 = PWM
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, dutyR);
    }
}

uint8_t is_all_sensors_white(void)
{
	if(normalized_sensors[0]>900 && normalized_sensors[1]>900
			&& normalized_sensors[2]>900 && normalized_sensors[3]>900
			&& normalized_sensors[4]>900) {
		return 1;
	}
	return 0;
}

uint8_t is_all_sensors_black(void)
{
	if(normalized_sensors[0]>500
			&& normalized_sensors[4]>500) {
		return 1;
	}
	return 0;
}

uint8_t is_that_black_node(void)
{
	if((normalized_sensors[1] + normalized_sensors[2] + normalized_sensors[3])>1500) {
		return 1;
	}
	return 0;
}
