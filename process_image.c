#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <leds.h>
#include <process_image.h>
#include <motors.h>

#define RED								0xF800
#define GREEN							0x07E0
#define BLUE							0x001F
#define RED_THRESHOLD					0x4000
#define GREEN_THRESHOLD					0x0120

#define PIXEL_LINE						160
#define BEGIN							0
#define END								1
#define DELTA							10
#define CLAMP_BEGIN(begin)				((begin > 0) ? begin : 0)
#define CLAMP_END(end)					((end < IMAGE_BUFFER_SIZE) ? end : IMAGE_BUFFER_SIZE - 1)

static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static uint16_t interval[2] = {0, 0};
static uint8_t line_found = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);
static thread_t *cptr_img, *prcs_img;

uint8_t is_red_enough(uint16_t* buffer, uint16_t begin, uint16_t end) {
	uint16_t i = begin;
	uint32_t mean = 0;

	while (i <= end) {
		mean += ((uint16_t)buffer[i] & RED);
		i++;
	}
	mean /= (end - begin + 1);
	return (mean > GREEN_THRESHOLD);
}


uint8_t is_not_green(uint16_t* buffer, uint16_t begin, uint16_t end) {
	uint16_t i = begin;
	uint32_t mean = 0;

	while (i <= end) {
		mean += ((uint16_t)buffer[i] & GREEN);
		i++;
	}
	mean /= (end - begin + 1);
	return (mean < GREEN_THRESHOLD);
}

//reprit en grand ligne du TP4
int8_t find_line(uint16_t* buffer, uint16_t start, uint16_t finish, uint16_t color) {

	uint8_t wrong_line = 1, line_not_found = 0, stop = 0;
	uint16_t i = start, begin = 0, end = 0;
	uint32_t mean = 0;

	while (i <= finish) {
		mean += (buffer[i] & color);
		i++;
	}
	mean /= finish - start + 1;
	i = start;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i <= (finish - WIDTH_SLOPE))
		{
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(((uint16_t)buffer[i] & color) > mean && ((uint16_t)buffer[i+WIDTH_SLOPE] & color) < mean)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i <= (finish - WIDTH_SLOPE) && begin)
		{
		    stop = 0;

		    while(stop == 0 && i <= finish)
		    {
		        if(((uint16_t)buffer[i] & color) > mean && ((uint16_t)buffer[i-WIDTH_SLOPE] & color) < mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}while(wrong_line);

	if (line_not_found == 0) {
		interval[BEGIN] = begin;
		interval[END] = end;
		return 1;
	} else {
		interval[BEGIN] = 0;
		interval[END] = 0;
		return 0;
	}
}

void find_object(uint16_t *buffer) {
	int16_t begin_red = 0, end_red = 0;

	while(1) {
		//we check for a potential object using the blue channel
		if (find_line(buffer, end_red, IMAGE_BUFFER_SIZE - 1 , BLUE)) {
			begin_red = interval[BEGIN];
			end_red = interval[END];
			if (is_red_enough(buffer, begin_red, end_red) && is_not_green(buffer, begin_red, end_red)) {
				set_led(LED5, 1);
				line_position = (begin_red + end_red)/2;
				line_found = 1;
				break;
			} else {
				set_led(LED5, 0);
			}
		}
		else
		{
			set_led(LED5, 0);
			line_found = 0;
			break;
		}
	}
}


//reprit du TP4
static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
	po8030_advanced_config(FORMAT_RGB565, 0, PIXEL_LINE, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(!chThdShouldTerminateX()){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
    chThdExit(0);
}

static THD_WORKING_AREA(waProcessImage, 2048);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
	uint8_t *img_buff_ptr;
	uint16_t image[IMAGE_BUFFER_SIZE] = {0};

    while(!chThdShouldTerminateX()){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);

		img_buff_ptr = dcmi_get_last_image_ptr();
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts all pixel information
			image[i/2] = img_buff_ptr[i]<<8 | img_buff_ptr[i+1];
		}

		find_object(image);
    }
    chThdExit(0);
}

uint16_t get_line_position(void){
	return line_position;
}

uint8_t has_found_line(void) {
	return line_found;
}

void process_image_start(void){
	prcs_img = chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	cptr_img = chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

void process_image_stop(void) {
	chThdTerminate(prcs_img);
	chThdTerminate(cptr_img);
}
