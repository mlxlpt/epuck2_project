#include "ch.h"
#include "hal.h"

#include <main.h>
#include <eputt_regulator.h>
#include <motors.h>
#include <process_image.h>
#include <sensors/VL53L0X/VL53L0X.h>

//Temporal paremeters during research
#define TIME_BALL_NF_MS		700 //Maxmimum allowed time [ms] of ball out of sight
#define TIME_FORCETURN_MS	100 //Force turn for given [ms] if dubious reads are obtained
#define MAX_TIME_FINDBALL_MS	6000 //time [ms] to settle and find ball
//Temporal parameters during charging
#define MAX_TIME_OFS_MS 		1000 //900 before, to change if wanted
#define TIMEOUT_SHOOT_MS		100
#define MAX_TIME_SHOOT_MS	(1000+MAX_TIME_OFS_MS) //means max shot dist is just above 20mm

//Regulator parameters
#define KP					0.5f
#define KI 					0.15f
#define KI_PRIOR				0.93f
#define MAX_SUM_ERROR 		(MOTOR_SPEED_LIMIT/(5*KI))
#define GOAL_DISTANCE 		(IMAGE_BUFFER_SIZE/2)
#define MEAS_POTENTIAL		20

//Precision of alignment
#define ROTATION_THRESHOLD	10	//pxl, cannot align perfectly anyway
#define ALIGNED_CNT			12

//Speeds
#define MANUAL_TURN_SPEED	(MOTOR_SPEED_LIMIT/2)
#define CHARGE_SPEED 		(MOTOR_SPEED_LIMIT-100)

//Distance parameters for charge mode
#define COLOR_CORRECTION_MM	60 //empirical value, TO CHANGE IF BALL COLOR IS CHANGED
#define CORRECTION_FACTOR	2 //Empirical value, put 1 (ONE) if ball has a clear color.
#define BALL_RADIUS_MM		20
#define THRESHOLD_MM			5
#define MAX_DIST_OFS_MM		60 //If the E-Putt is closer than that, the ball becomes out of sight
#define MIN_DIST_MM			(CORRECTION_FACTOR*BALL_RADIUS_MM+THRESHOLD_MM)
#define DIST_DETECT_MM		(MAX_DIST_OFS_MM+BALL_RADIUS_MM+COLOR_CORRECTION_MM+THRESHOLD_MM)

// needs to be global to file as it is reset if certain conditions are met
static int16_t sum_error = 0; //integral error

/* pi_regulator(System output, desired System output)
 * Simple implementation of a discrete PI regulator, (de)activated by the main regulator.
 */
int16_t pi_regulator(uint16_t distance, uint16_t goal){

	int16_t error = 0;
	int16_t speed = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	if(abs(error) < ROTATION_THRESHOLD){
		return 0;
	}

	//We multiply the previous accumulated error with a constant < 1. System forgets with time
	//It's a recommended thing to do with discrete PI regs.
	sum_error = sum_error*KI_PRIOR + error;

	//Anti-Reset-Windup
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;
    return speed;
}

/*THREAD: Regulator*/
static THD_WORKING_AREA(waRegulator, 256);
static THD_FUNCTION(Regulator, arg){

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    enum eputtState curr_state;
    
    while(1)
    {
		time = chVTGetSystemTime();
		curr_state = getState();

        if(curr_state == SEARCH_BALL || curr_state == BALL_LOCKED || curr_state == CHARGE_BALL)
        {
        		if(curr_state != CHARGE_BALL)
        			regulator_position(false);
        		if(curr_state != SEARCH_BALL)
        			distance_stop(false);
        }
        else
        {
        		regulator_position(true);
        		distance_stop(true);
        }
		chThdSleepUntilWindowed(time, time + MS2ST(TIME_MS_PIDREG));
    }
}

/*
* This is the main regulator. It behaves differently depending if the ball is in sight or not.
* BALL IN SIGHT: lets PI regulator calculate rotation speed
* BALL OUT OF SIGHT:
* 		A) if the ball recently been out of sight been out of sight is small, still let PI controller
* 			maybe it will find it again in the meantime (this allow false read from camera)
* 		B) if the ball has been gone for too long, switch to constant speed turn in hope to find it again
*
* It is still called during the ball charge until the ball disappear or become too close
*/
void regulator_position(bool reset){

    static systime_t time_nf, time_falsefound, time_max_exec = 0;

    static int16_t speed = 0;
    static uint16_t speed_offset = 0;
    static uint8_t aligned_cnt = 0, measure_potential = 0;
    static bool ball_nf = false, manual_turn = false, forceturn = false;

	if (reset)
	{
		time_max_exec = chVTGetSystemTime();
		speed = sum_error = 0;
		speed_offset = 0;
		aligned_cnt = 0;
		ball_nf = manual_turn = forceturn = false;
		return;
	}

	if (!ballSeenLast() && !ball_nf)
	{
		time_nf = chVTGetSystemTime();
		ball_nf=true;
	}
	else if (ballSeenLast() && !forceturn)
	{
		measure_potential = MEAS_POTENTIAL;
		ball_nf = manual_turn = false;
	}
	//if the ball has been reported missing, decrease the potential of current measure
	//if it has been missing for too long (either unseen or dubious read), go manual turn.
	else if (ball_nf && getState() == SEARCH_BALL)
	{
		if(measure_potential > 0)
		{
			--measure_potential;
			if(measure_potential==0)
			{
				forceturn = manual_turn = true;
				time_falsefound = chVTGetSystemTime();
			}
		}
		if (chVTGetSystemTime() - time_nf > MS2ST(TIME_BALL_NF_MS))
			manual_turn = true;

		if(forceturn && (chVTGetSystemTime() - time_falsefound > MS2ST(TIME_FORCETURN_MS)))
			forceturn = false;
	}

	if (!manual_turn)
		speed = pi_regulator(getBallPos(), (IMAGE_BUFFER_SIZE/2));
	else if (getState() == SEARCH_BALL)
		speed = MANUAL_TURN_SPEED;

	//if the ball is nearly in front of the camera, don't rotate and validate
	if(abs(speed) <= ROTATION_THRESHOLD && speed_offset == 0 && ballSeenLast())
		aligned_cnt++;
	else if (speed_offset == 0)
		aligned_cnt = 0;

	//if we're aligned on multiple read, it's time to charge
	if(aligned_cnt > ALIGNED_CNT && speed_offset==0)
	{
		if(getState() == SEARCH_BALL)
			switchState(true);
		speed_offset = CHARGE_SPEED;
		aligned_cnt = 0;
		sum_error=0;
	}
	//ball has gotten out of sight (ie: too near or removed by a mean user), just charge straight
	else if (getState() == BALL_LOCKED && !ballSeenLast())
	{
		speed=0;
		switchState(true);
		speed_offset=CHARGE_SPEED;
	}

	if(getState() == SEARCH_BALL && (chVTGetSystemTime() - time_max_exec > MS2ST(MAX_TIME_FINDBALL_MS)))
	{
		speed = 0;
		switchState(false);
	}

	if (getState() == CHARGE_BALL)
		speed = 0;

	right_motor_set_speed(speed_offset-speed);
	left_motor_set_speed(speed+speed_offset);
}

/*
* This handles the charging when the ball is: in sight and out of sight.
* Try to see if there's an object nearby with Time of Flight sensor.
* 	A) First distance calculated should be a distance when the ball becomes out of sight to the camera
* 			switch to charge and assume successful shot incoming.
* 	B) if the ball has been gone for too long, switch to constant speed turn in hope to find it again
*/
void distance_stop(bool reset){

    static systime_t time_start_shoot = 0, timer_stop = 0, time_shootdelay = 0;
    static bool isCharging = false;
    static bool isOutOfSight = false;
    static bool isShooting = false;

    uint16_t dist = 0;

	if (reset)
	{
		time_start_shoot = timer_stop = time_shootdelay = 0;
		isShooting = isCharging = isOutOfSight = false;
		return;
	}

	if(!isCharging)
	{
		isCharging=true;
		time_start_shoot = chVTGetSystemTime();
	}

	dist = VL53L0X_get_dist_mm();

	if (dist > 0 && dist < DIST_DETECT_MM)
	{
		if(getState() == BALL_LOCKED)
			switchState(true); //failsafe, should not happen before the one in controller
		timer_stop = chVTGetSystemTime();
		isOutOfSight = true;
		right_motor_set_speed(CHARGE_SPEED);
		left_motor_set_speed(CHARGE_SPEED);

	}

	//If we're guaranteed to hit the ball, extend slightly the shoot time to give more inertia
	if(dist > 0 && dist < MIN_DIST_MM && !isShooting)
	{
		time_shootdelay = chVTGetSystemTime();
		isShooting = true;
	}
	else if(isShooting && (chVTGetSystemTime() - time_shootdelay > MS2ST(TIMEOUT_SHOOT_MS)))
	{
		if(getState() == CHARGE_BALL)
			switchState(true);
		right_motor_set_speed(0);
		left_motor_set_speed(0);
	}

	/*If ball has been sighted by TOF but either these happen:
     *  -time to hit the ball has expired
	 *	-elapsed time between high and low distance read is not respected (=>ball has vanished)
	 *	-total time allowed to shoot has expired (ball is way too far or the ball was removed)
	 */
	if((isShooting && (chVTGetSystemTime() - time_start_shoot > MS2ST(MAX_TIME_SHOOT_MS+time_shootdelay) ||
		(/*isOutOfSight &&*/ (chVTGetSystemTime() - timer_stop > MS2ST(MAX_TIME_OFS_MS+time_shootdelay))))) ||
		(chVTGetSystemTime() - time_start_shoot > MS2ST(MAX_TIME_SHOOT_MS+MAX_TIME_OFS_MS+time_shootdelay)))
	{
		switchState(false);
		right_motor_set_speed(0);
		left_motor_set_speed(0);
	}
}

void regulator_start(void){
	chThdCreateStatic(waRegulator, sizeof(waRegulator), NORMALPRIO, Regulator, NULL);
}
