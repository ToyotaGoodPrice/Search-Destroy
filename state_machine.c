#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <motors.h>
#include <chprintf.h>
#include <leds.h>
#include <pi_regulator.h>
#include <process_image.h>

#define ROTATION_SPEED		150

enum state {IDLE, SEARCHING, MOVE_TO_OBJECT, PUSH_OBJECT};
//semaphore
static BSEMAPHORE_DECL(state_changed, TRUE);
static enum state system_state = SEARCHING;
static thread_t *search_ctrl;

void start_search_control();

static THD_WORKING_AREA(waSearchControl, 64);
static THD_FUNCTION(SearchControl, arg) {
	systime_t time;
	int8_t counter = 0;
	chRegSetThreadName(__FUNCTION__);

	while(!chThdShouldTerminateX()) {
		time = chVTGetSystemTime();
		counter++;
		if (counter == 10) {
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			system_state = MOVE_TO_OBJECT;
			chBSemSignal(&state_changed);
		} else if (!has_found_line()){
			counter = 0;
		}
		//20Hz
		chThdSleepUntilWindowed(time, time + MS2ST(20));
	}
	chThdExit(0);
}

static THD_WORKING_AREA(waStateMachine, 128);
static THD_FUNCTION(StateMachine, arg) {
	chRegSetThreadName(__FUNCTION__);
	while(1) {
		switch(system_state) {
		case SEARCHING:
			left_motor_set_speed(ROTATION_SPEED);
			right_motor_set_speed(-ROTATION_SPEED);
			start_search_control();
			break;
		case MOVE_TO_OBJECT:
			stop_search_control();
			start_pi_move2obj();
			set_led(LED1, 1);
			break;
		case PUSH_OBJECT:
			break;
		default:
			break;
		}
		chBSemWait(&state_changed);
	}
}

void start_search_control() {
	search_ctrl = chThdCreateStatic(waSearchControl, sizeof(waSearchControl), NORMALPRIO, SearchControl, NULL);
}

void stop_search_control() {
	chThdTerminate(search_ctrl);
}

void start_state_machine() {
	chThdCreateStatic(waStateMachine, sizeof(waStateMachine), NORMALPRIO+1, StateMachine, NULL);
}
