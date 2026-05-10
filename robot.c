/*
 * robot.c
 *
 *  Created on: Jan 25, 2026
 *      Author: GC
 */


#include "robot.h"
#include "main.h"
#include "movement.h"
#include "Color.h"

static uint8_t calibration_done = 0;
static uint8_t turn_done = 0;
static uint8_t pick_done = 0;
static uint8_t place_done = 0;


uint8_t is_button_pressed(void)
{
	return button_trigger;
}

void clear_button_pressed(void)
{
	button_trigger = 0;
}

void robot_stop(void)
{
    set_motor_speeds(0, 0);
}

void robot_turn_right(void)
{
	set_motor_speeds(130, -90);
	HAL_Delay(300);
}

void robot_turn_left(void)
{
	set_motor_speeds(-90, 130);
	HAL_Delay(400);
}

void robot_calibrate(void)
{
	lineSensorCalibration();
    calibration_done = 1;
}

uint8_t robot_calibration_done(void)
{
	return calibration_done;
}

void robot_follow_white_line(void)
{
    lineSensorNormalization();
    int16_t* speeds = PID();
    set_motor_speeds(speeds[0], speeds[1]);
}

void robot_follow_black_line(void)
{
    lineSensorNormalization();
    int16_t* speeds = PID();
    set_motor_speeds(speeds[0], speeds[1]);
}

uint8_t robot_box_detected(void)
{
    return box_detected;
}

void robot_clear_box_mask(void)
{
	box_mask = 0;
}

void robot_clear_box_detected(void)
{
	box_detected = 0;
}

void robot_move_forward_slightly(void)
{
	set_motor_speeds(125, 125);
}

void robot_move_forward(void)
{
	set_motor_speeds(125, 125);
	HAL_Delay(200);
}
void robot_turn(void)
{
	set_motor_speeds(-140, 140);
	HAL_Delay(1100);
	turn_done = 1;
}

uint8_t robot_turn_done(void)
{
	return turn_done;
}

void robot_pick_box(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, SET);
    pick_done = 1;
}

uint8_t robot_pick_done(void)
{
    return pick_done;
}


void robot_place_box(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET);
    place_done = 1;
}

uint8_t robot_place_done(void)
{
    return place_done;
}

uint8_t robot_detect_color(void)
{
	return Color_Detect();
}

uint8_t robot_black_node_detected(void)
{
	if(is_that_black_node()) {
		//black_node_counter++;
		return 1;
	}
	return 0;
}

void robot_transition_line(void)
{
	line_mode = BLACK_LINE_MODE;
}

void robot_retransition_line(void)
{
	line_mode = WHITE_LINE_MODE;
}

void robot_return(void)
{
	set_motor_speeds(-140, 140);
	HAL_Delay(1200);

}

uint8_t robot_return_done(void)
{
    return 1;
}

void robot_blink(void)
{
	HAL_GPIO_WritePin(COLOR_LED_R_PORT, COLOR_LED_R_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COLOR_LED_G_PORT, COLOR_LED_G_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COLOR_LED_B_PORT, COLOR_LED_B_PIN, GPIO_PIN_SET);

	HAL_Delay(500);

	HAL_GPIO_WritePin(COLOR_LED_R_PORT, COLOR_LED_R_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COLOR_LED_G_PORT, COLOR_LED_G_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COLOR_LED_B_PORT, COLOR_LED_B_PIN, GPIO_PIN_RESET);

	HAL_Delay(500);

}

