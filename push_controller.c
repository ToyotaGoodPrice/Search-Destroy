#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <push_controller.h>
#include <process_image.h>
#include <sensors/proximity.h>
#include <leds.h>
#include <search_control.h>

#define SPEED 				450
#define ERROR_THRESHOLD		10.0f
#define KP					0.2f
#define MAX_SUM_ERROR 		30.0f
#define OBJECT_IS_CLOSE 	(left_IR > 0 || right_IR > 0)
#define OBJECT_IS_FAR 		(left_IR == 0 && right_IR == 0)
#define STOP_COUNTER		10


enum state {FAR, CLOSE, STOP};

static thread_t *push_thread;

//simple P regulator implementation
int16_t p_regulator(float dist_to_center, float goal){

	float error = 0;
	float speed_corr = 0;

	error = dist_to_center - goal;

	if (error < 0){
		set_led(LED3, 1);
		set_led(LED7, 0);
	} else {
		set_led(LED3, 0);
		set_led(LED7, 1);
	}

	//disables the PI regulator if the error is to small
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	speed_corr = KP * error;

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
    	case CLOSE: //utilise un PID pour toujours rester en face de la cible en la poussant
    		if (OBJECT_IS_FAR) {
    			if (counter == STOP_COUNTER) {
    				current_state = STOP;
    				break;
    			}
    			counter++;
    		} else {
    			counter = 0;
    		}
    		direction = left_IR - right_IR;
    		speed_correction =  p_regulator(direction, 0);

    		//applies the rotation from the PI regulator
    		right_motor_set_speed(SPEED + speed_correction);
    		left_motor_set_speed(SPEED - speed_correction);
    		break;
    	case STOP: //lorsque les capteurs IR ne détectent plus l'objet
    		right_motor_set_speed(0);
    		left_motor_set_speed(0);
    		request_state_change();
    		chThdExit(0);
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
