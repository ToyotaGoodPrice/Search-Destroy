#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <sensors/proximity.h>
#include <leds.h>

#define SPEED 				350
#define ERROR_THRESHOLD			10.0f	//[cm] because of the noise of the camera
#define KP						0.2f
#define KI 						0.0f
#define KD 						0.0f
#define MAX_SUM_ERROR 			30.0f
#define OBJECT_IS_CLOSE 	(left_IR > 0 || right_IR > 0)
#define OBJECT_IS_FAR 		(left_IR == 0 && right_IR == 0)


enum state {FAR, CLOSE, STOP};

static thread_t *push_thread;

//simple PI regulator implementation
int16_t pi_regulator(float dist_to_center, float goal){

	float error = 0;
	float speed_corr = 0;

	static float sum_error = 0;
	static float previous_err = 0;

	error = dist_to_center - goal;

	if (error < 0){
		set_led(LED3, 1);
		set_led(LED7, 0);
	} else {
		set_led(LED3, 0);
		set_led(LED7, 1);
	}

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed_corr = KP * error + KI * sum_error + KD * (error-previous_err);

	previous_err = error;

    return (int16_t)speed_corr;
}

static THD_WORKING_AREA(waPushController, 256);
static THD_FUNCTION(PushController, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    int16_t direction = 0;
    systime_t time;

    int16_t speed_correction = 0;
    int32_t left_IR = 0;
    int32_t right_IR = 0;
    enum state current_state = FAR;
    uint8_t counter = 0;

    while(!chThdShouldTerminateX()){
    	time = chVTGetSystemTime();
    	left_IR = get_calibrated_prox(7);
    	right_IR = get_calibrated_prox(0);

    	switch(current_state) {
    	case FAR: //avance tout droit en attendant d'être en contact direct avec la cible
    	    right_motor_set_speed(SPEED);
    	    left_motor_set_speed(SPEED);
    		if (OBJECT_IS_CLOSE) {
    			current_state = CLOSE;
    		}
    		break;
    	case CLOSE: //utilise une PID pour toujours rester en face de la cible
    		if (OBJECT_IS_FAR) {
    			if (counter == 20) {
    				current_state = STOP;
    				break;
    			}
    			counter++;
    		} else {
    			counter = 0;
    		}
    		direction = left_IR - right_IR;
    		speed_correction =  pi_regulator(direction, 0);

    		//applies the rotation from the PI regulator
    		right_motor_set_speed(SPEED + speed_correction);
    		left_motor_set_speed(SPEED - speed_correction);
    		break;
    	case STOP:
    		right_motor_set_speed(0);
    		left_motor_set_speed(0);
    	}

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
    chThdExit(0);
}

void start_push_controller(void){
	push_thread = chThdCreateStatic(waPushController, sizeof(waPushController), NORMALPRIO, PushController, NULL);
}

void stop_push_controller(void) {
	chThdTerminate(push_thread);
}
