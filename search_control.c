#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include <motors.h>
#include <move2obj_controller.h>
#include <push_controller.h>
#include <process_image.h>

#define SEARCH_COUNTER	10

static thread_t *search_ctrl;

void start_search_control(void);
void stop_search_control(void);

static THD_WORKING_AREA(waSearchControl, 64);
static THD_FUNCTION(SearchControl, arg) {
	systime_t time;
	int8_t counter = 0;
	chRegSetThreadName(__FUNCTION__);

	while(!chThdShouldTerminateX()) {
		time = chVTGetSystemTime();
		//un compteur permet de limiter les fausses détections d'objets
		counter++;
		if (counter == SEARCH_COUNTER) {
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			request_state_change();
		} else if (!has_found_line()){
			counter = 0;
		}
		//20Hz
		chThdSleepUntilWindowed(time, time + MS2ST(20));
	}
	chThdExit(0);
}

//Public functions

void start_search_control() {
	search_ctrl = chThdCreateStatic(waSearchControl, sizeof(waSearchControl), NORMALPRIO, SearchControl, NULL);
}

void stop_search_control() {
	chThdTerminate(search_ctrl);
}
