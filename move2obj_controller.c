#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <search_control.h>
#include <motors.h>
#include <move2obj_controller.h>
#include <process_image.h>

#define SPEED 					450
#define ERROR_THRESHOLD			10.0f
#define KP						0.8f
#define KI 						0.0f
#define MAX_SUM_ERROR 			30.0f
#define OBJ_LOST_COUNTER		10

static thread_t *move_thread;

int16_t pi_regulator(float dist_to_center, float goal){

	float error = 0;
	float speed_corr = 0;

	static float sum_error = 0;

	error = dist_to_center - goal;

	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed_corr = KP * error + KI * sum_error;

    return (int16_t)speed_corr;
}

static THD_WORKING_AREA(waMove2Obj, 256);
static THD_FUNCTION(Move2Obj, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed_correction = 0;
    uint8_t counter = 0;

    while(!chThdShouldTerminateX()){
        time = chVTGetSystemTime();
        counter++;
        if (counter == OBJ_LOST_COUNTER) {
        	request_state_change();
    		right_motor_set_speed(0);
    		left_motor_set_speed(0);
        	chThdExit(0);
       	} else if (has_found_line()){
   			counter = 0; //reset counter
     	}
        speed_correction =  pi_regulator(get_line_position() - IMAGE_BUFFER_SIZE/2, 0);

		right_motor_set_speed(SPEED - speed_correction);
		left_motor_set_speed(SPEED + speed_correction);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
    chThdExit(0);
}

void start_pi_move2obj(void){
	move_thread = chThdCreateStatic(waMove2Obj, sizeof(waMove2Obj), NORMALPRIO, Move2Obj, NULL);
}

void stop_pi_move2obj(void) {
	chThdTerminate(move_thread);
}
