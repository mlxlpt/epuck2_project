#include <stdio.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"

#include <motors.h>
#include <camera/po8030.h>
#include <audio/microphone.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <leds.h>

#include <main.h>
#include <process_image.h>
#include <audio_processing.h>
#include <eputt_regulator.h>

#define SLEEP_MAIN_MS		200
#define NBCYCLES_MAIN_LEDS	7 //Needs to follow this rule 4*n-1, otherwise some lED will stay lit

enum {LED_OFF = 0, LED_ON, LED_TOGGLE};

static enum eputtState curr_state = STARTUP; //holds the state of the e-putt

int main(void){

	//System inits
    halInit();
    chSysInit();
    mpu_init();

    //Hardware inits
    dcmi_start();
	po8030_start();
	motors_init();

	//Thread starts
	mic_start(&processAudioData);
	capture_process_img_start();
	regulator_start();
	VL53L0X_start();

	//Startup state leds
	stateLed_update();

	bool led_state = false;

	/* Infinite loop. */
    while (1)
    {
    		if(getState() > LEDSTATE || led_state)
    			led_state = led_handler();

        chThdSleepMilliseconds(SLEEP_MAIN_MS);
    }
}

/* led_handler()
 * This function handles all the simple leds animation
 * Timing & length depends solely on SLEEP_MAIN_MS and led_cnt
 * Animations complexity are limited by NBCYCLES_MAIN_LEDS
 */
bool led_handler(){
	//This static variable could be avoided but it's already implemented if one's choose to add
	//a reset frequency in the E-Putt remote code.
	static enum eputtState stateLed = STARTUP;
	static uint8_t led_cnt=0, curr_led=0;

    if(led_cnt==0)
    		stateLed = getState();

	switch(stateLed)
	{
		case LED_MV_SB:
			set_led(curr_led, LED_TOGGLE);
			curr_led++;
			if(curr_led>=NUM_LED)
				curr_led=0;
			break;
		case LED_BALL_NF:
			set_front_led(LED_TOGGLE);
			break;
		case LED_SUCCESS:
			set_body_led(LED_TOGGLE);
			break;
		default: //in case something goes wrong. Should never happen.
			set_led(LED5, LED_ON);
	}
    if(led_cnt >= NBCYCLES_MAIN_LEDS)
    {
    		switchState(true);
    		led_cnt = 0;
    		return false;
    }
    else
    {
    		led_cnt++;
    		return true;
    }
}

/* switchState(boolean - success of current operation)
 * This is the State machine of the whole system.
 * The rest of the program doesn't know how the system work. Functions & threads just checks for
 * specific states and tell back the state machine if their task was successful or not.
 */
void switchState(bool opSuccess){
	switch(getState())
	{
		case STARTUP:
			setState(MANUAL_MOVE);
			break;
		case MANUAL_MOVE:
			setState(LED_MV_SB);
			break;
		case LED_MV_SB:
			setState(SEARCH_BALL);
			break;
		case SEARCH_BALL:
			if(opSuccess)
				setState(BALL_LOCKED);
			else
				setState(LED_BALL_NF);
			break;
		case BALL_LOCKED:
			if (opSuccess)
				setState(CHARGE_BALL);
			else
				setState(LED_BALL_NF);
			break;
		case CHARGE_BALL:
			if (opSuccess)
				setState(LED_SUCCESS);
			else
				setState(LED_BALL_NF);
			break;
		case LED_BALL_NF:
			setState(MANUAL_MOVE);
			break;
		case LED_SUCCESS: //succesful shot: go in reset state as it may be the end of the game
			setState(STARTUP);
			break;
		default: //If something goes wrong, if ever
			setState(STARTUP);
			break;
	}
	stateLed_update();
}

/* stateLed_update()
 * This function switch on/off some leds that are used all the time in specific states
 */
void stateLed_update(){

	switch(getState())
	{
		case STARTUP:
			set_led(LED3,LED_ON);
			set_led(LED7,LED_ON);
			break;
		case MANUAL_MOVE:
			set_led(LED3,LED_OFF);
			set_led(LED7,LED_OFF);
			set_body_led(LED_ON);
			break;
		case SEARCH_BALL:
			set_body_led(LED_OFF);
			set_led(LED1, LED_ON);
			break;
		case BALL_LOCKED:
			set_led(LED1, LED_OFF);
			break;
		case LED_BALL_NF:
			set_led(LED1, LED_OFF);
			break;
		default:
			break;
	}
}

void setState(enum eputtState new_eputtState){
	if(new_eputtState==STARTUP)
	{
		set_led(LED3, LED_ON);
		set_led(LED7, LED_ON);
	}
	curr_state = new_eputtState;
}

enum eputtState getState(){
	return curr_state;
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void){
    chSysHalt("Stack smashing detected");
}
