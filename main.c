#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include <leds.h>
#include <motors.h>
#include <camera/po8030.h>
#include <sensors/proximity.h>
#include <move2obj_controller.h>
#include <process_image.h>
#include <search_control.h>
#include <push_controller.h>

#define ROTATION_SPEED		150
#define TEMPS_CALIBRATION	2000

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

enum state {IDLE, SEARCHING, MOVE_TO_OBJECT, PUSH_OBJECT};
static BSEMAPHORE_DECL(state_changed, TRUE);

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();

	proximity_start();
	//Démarre le processus de calibration et laisse un délais pour le calibrer
	calibrate_ir();
    chThdSleepMilliseconds(TEMPS_CALIBRATION);

	enum state system_state = SEARCHING;

    /* Infinite loop. */
    while (1) {
		switch(system_state) {
		case SEARCHING:
			process_image_start();
			left_motor_set_speed(ROTATION_SPEED);
			right_motor_set_speed(-ROTATION_SPEED);
			start_search_control();
			chBSemWait(&state_changed);
			system_state = MOVE_TO_OBJECT;
			stop_search_control();
			break;
		case MOVE_TO_OBJECT:
			start_pi_move2obj();
			chBSemWait(&state_changed);
			system_state = PUSH_OBJECT;
			stop_pi_move2obj();
			process_image_stop();
			break;
		case PUSH_OBJECT:
			start_push_controller();
			chBSemWait(&state_changed);
			system_state = SEARCHING;
			break;
		default:
			break;
		}
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

//public functions

void request_state_change(void) {
	chBSemSignal(&state_changed);
}
