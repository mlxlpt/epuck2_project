#include "ch.h"
#include "hal.h"

#include <camera/po8030.h>

#include <main.h>
#include <process_image.h>

#define CAPTURE_LINE_NB			200
#define NB_CAPTURED_LINES		2
#define WIDTH_SLOPE				6
#define MIN_OBJ_WIDTH			70 //40 previously but not good because noise/distance

static uint16_t ball_position = IMAGE_BUFFER_SIZE/2;	//middle
static bool seenLast = false;

/*
 *  Updates ball_position (center point) extracted from the image buffer given
 *  Updates seenLast (boolean), tell if the last extraction was successful or not.
 */
void extract_ball_pos(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0;
	uint8_t stop = 0, wrong_ball = 0, ball_not_found = 0;
	uint32_t mean = 0;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++)
		mean += buffer[i];
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_ball = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        ball_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    ball_not_found = 1;
		}

		//if a ball too small has been detected, continues the search
		if(!ball_not_found && (end-begin) < MIN_OBJ_WIDTH)
		{
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_ball = 1;
		}
	}while(wrong_ball);

	if(ball_not_found)
		seenLast = false;
	else
	{
		ball_position = (begin + end)/2;
		seenLast = true;
	}
}

/* THREAD CaptureProcessImg */
static THD_WORKING_AREA(waCaptureProcessImg, 1024);
static THD_FUNCTION(CaptureProcessImg, arg){

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of specified line
	po8030_advanced_config(FORMAT_RGB565, 0, CAPTURE_LINE_NB, IMAGE_BUFFER_SIZE, NB_CAPTURED_LINES, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1)
    {
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		img_buff_ptr = dcmi_get_last_image_ptr();

		//If the state is matching, analyze the image to find ball position
		if(getState() == SEARCH_BALL || getState() == BALL_LOCKED)
		{
			//Extracts only the red pixels
			for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2)
				image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;

			//search for a discontinuity in the image and gets its position
			extract_ball_pos(image);
		}
    }
}

void capture_process_img_start(void){
	chThdCreateStatic(waCaptureProcessImg, sizeof(waCaptureProcessImg), NORMALPRIO, CaptureProcessImg, NULL);
}

uint16_t getBallPos(){
	return ball_position;
}

bool ballSeenLast() {
	return seenLast;
}
