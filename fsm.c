/*
 * fsm.c
 *
 *  Created on: Jan 25, 2026
 *      Author: GC
 */

#include "fsm.h"
#include "movement.h"
#include "robot.h"

#include "main.h"

static RobotState present_state;
static uint8_t entry;
static uint8_t black_node_counter = 0;

static BoxColor detected_color;
static uint8_t current_pick_index = 1;

const BoxColor pickup_order[3] = {
    COLOR_RED,
    COLOR_BLUE,
    COLOR_GREEN
};

static uint32_t white_transition_counter = 0;
static uint32_t black_transition_counter = 0;

#define WHITE_TRANSITION_THRESHOLD 1000
#define BLACK_TRANSITION_THRESHOLD 1


void FSM_Init(void)
{
    present_state = STATE_INIT;
    entry = 1;
}

void FSM_Update(void)
{
    switch (present_state)
    {
        case STATE_INIT:
            if (entry) {
                robot_stop();
                entry = 0;
            }
            if(is_button_pressed()){
				present_state = STATE_CALIBRATION;
				clear_button_pressed();
				entry = 1;
            }
            break;

        case STATE_CALIBRATION:
            robot_calibrate();

            if (robot_calibration_done()) {
                present_state = STATE_START;
                entry = 1;
            }
            break;

        case STATE_START:
            present_state = STATE_FOLLOW_WHITE;
            entry = 1;
            break;

        case STATE_FOLLOW_WHITE:
            robot_follow_white_line();

            if (robot_box_detected()) {
            	robot_stop();
                present_state = STATE_BOX_DETECTION;
                entry = 1;
            }

            if (is_all_sensors_white()) {
                white_transition_counter++;
                if (white_transition_counter >= WHITE_TRANSITION_THRESHOLD) {
                    present_state = STATE_TRANSITION;
                    entry = 1;
                    white_transition_counter = 0;
                }
            }
            else {
                white_transition_counter = 0;
            }
            break;

        case STATE_BOX_DETECTION:
            //robot_stop();

            present_state = STATE_BOX_COLOR;

            break;

        case STATE_BOX_COLOR:
            if (entry) {
                detected_color = robot_detect_color();
                entry = 0;
            }

            if (detected_color == pickup_order[current_pick_index]) {
                present_state = STATE_PICKING;
                entry = 1;
            }
            else{
                present_state = STATE_SKIP_BOX;
                entry = 1;
            }
            break;

        case STATE_SKIP_BOX:
            robot_move_forward_slightly();
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET);

            if (!robot_box_detected())
            {
                present_state = STATE_FOLLOW_WHITE;
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, RESET);
            }

            robot_clear_box_mask();
            robot_clear_box_detected();
            break;

        case STATE_PICKING:
            if (entry) {
                robot_pick_box();
                robot_move_forward_slightly();
                HAL_Delay(100);
                entry = 0;
            }

            if (robot_pick_done()) {
            	robot_clear_box_detected();
                present_state = STATE_TURNING;
                entry = 1;
            }
            break;

        case STATE_TURNING:
            robot_turn();

            if (robot_turn_done()) {
                present_state = STATE_FOLLOW_WHITE;
                entry = 1;
            }
            break;

        case STATE_TRANSITION:
            robot_transition_line();
            present_state = STATE_FOLLOW_BLACK;
            entry = 1;
            break;

        case STATE_FOLLOW_BLACK:
            robot_follow_black_line();

            if (robot_black_node_detected()) {
                present_state = STATE_BLACK_NODE_DECIDE;
                black_node_counter++;
                entry = 1;
            }
            break;

        case STATE_BLACK_NODE_DECIDE:
            if (entry) {
                if (black_node_counter == 1 && pickup_order[current_pick_index] == COLOR_RED) {
                    robot_turn_right();   // RED
                    present_state = STATE_BRANCH_FOLLOW;
                    black_node_counter = 0;
                    entry = 1;
                }
                else if (black_node_counter == 2 && pickup_order[current_pick_index] == COLOR_BLUE) {
                    robot_turn_left();    // BLUE
                    present_state = STATE_BRANCH_FOLLOW;
                    black_node_counter = 0;
                    entry = 1;
                }
                else if (black_node_counter == 3 && pickup_order[current_pick_index] == COLOR_GREEN) {
                    robot_turn_right();   // GREEN
                    present_state = STATE_BRANCH_FOLLOW;
                    black_node_counter = 0;
                    entry = 1;
                }
                else {
                	robot_move_forward();
                    present_state = STATE_FOLLOW_BLACK;
                    //black_node_counter = 0;
                    entry = 1;
                }

                //entry = 0;
            }

//            if (1/*robot_turn_done()*/) {
//                present_state = STATE_FOLLOW_BLACK;
//                //black_node_counter = 0;
//                entry = 1;
//            }
            break;

        case STATE_BRANCH_FOLLOW:
            robot_follow_black_line();

            if (robot_black_node_detected()) {
                present_state = STATE_PLACING;
                black_node_counter = 0;
                entry = 1;
            }
            break;

        case STATE_PLACING:
            if (entry) {
            	robot_stop();
                robot_place_box();
                entry = 0;
            }

            if (robot_place_done()) {
                current_pick_index++;

                if (current_pick_index >= 3) {
                    present_state = STATE_END;
                }
                else {
                    present_state = STATE_RETURNING;
                }
            }
            break;

        case STATE_RETURNING:
            robot_return();
            black_node_counter = 0;

            if (robot_return_done()) {
                present_state = STATE_BRANCH_REFOLLOW;
                entry = 1;
            }
            break;

        case STATE_BRANCH_REFOLLOW:
            robot_follow_black_line();

            if (robot_black_node_detected()) {
                present_state = STATE_BLACK_NODE_REDECIDE;
                black_node_counter++;
                entry = 1;
            }
            break;

        case STATE_BLACK_NODE_REDECIDE:
            if (entry) {
            	if (black_node_counter > 1) {
            		robot_move_forward();
            	}
            	else if (pickup_order[current_pick_index-1] == COLOR_RED) {
                    robot_turn_left();   // RED
                }
                else if (pickup_order[current_pick_index-1] == COLOR_BLUE) {
                    robot_turn_right();    // BLUE
                }
                else if (pickup_order[current_pick_index-1] == COLOR_GREEN) {
                    robot_turn_left();   // GREEN
                }
                entry = 0;
            }

            if (1/*robot_turn_done()*/) {
                present_state = STATE_REFOLLOW_BLACK;
                entry = 1;
            }
            break;

        case STATE_REFOLLOW_BLACK:
            robot_follow_black_line();

            if (robot_black_node_detected()) {
            	black_node_counter++;
                present_state = STATE_BLACK_NODE_REDECIDE;
                entry = 1;
            }

            if (is_all_sensors_black()) {
                black_transition_counter++;
                if (black_transition_counter >= BLACK_TRANSITION_THRESHOLD) {
                    present_state = STATE_RETRANSITION;
                    black_node_counter = 0;
                    entry = 1;
                    black_transition_counter = 0;
                }
            }
            else {
                black_transition_counter = 0;
            }
            break;

        case STATE_RETRANSITION:
            robot_retransition_line();
            present_state = STATE_FOLLOW_WHITE;
            robot_clear_box_mask();
            robot_clear_box_detected();
            black_node_counter = 0;
            entry = 1;
            break;

        case STATE_END:
        default:
            robot_stop();
            while(1){
            	robot_blink();
            }
            break;
    }
}


