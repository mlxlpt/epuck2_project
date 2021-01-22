#include "ch.h"
#include "hal.h"
#include <math.h>

#include <arm_const_structs.h>
#include <arm_math.h>

#include <audio/microphone.h>

#include <main.h>
#include <motors.h>
#include <audio_processing.h>

#define SPEED_WAIT_COMMAND	400
#define SPEED_MV_COMMAND		700
#define MIN_VALUE_THRESHOLD	17500

//Y=aX+b with a = 0.064015827312713 and b = 0.01881259825829
#define MIN_FREQ		68 // 1000Hz - we don't analyze before this index to not use resources for nothing
#define FRQ_RESET	70 // 1100Hz - frequency to start operations
#define FRQ_FORWARD 	80 // 1250Hz - when the E-Putt hear this, he moves forward.
#define FRQ_SEARCH	90 // 1400Hz - frequency to indicate putt to search for the ball.
#define MAX_FREQ		92 // 1440Hz - we don't analyze after this index to not use resources for nothing

#define THRSH_FREQ 		3

#define FRQ_FORWARD_MIN	(FRQ_FORWARD-THRSH_FREQ)
#define FRQ_FORWARD_MAX	(FRQ_FORWARD+THRSH_FREQ)
#define FRQ_SEARCH_MIN	(FRQ_SEARCH-THRSH_FREQ)
#define FRQ_SEARCH_MAX	(FRQ_SEARCH+THRSH_FREQ)
#define FRQ_RESET_MIN	(FRQ_RESET-THRSH_FREQ)
#define FRQ_RESET_MAX	(FRQ_RESET+THRSH_FREQ)

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micBack_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micBack_output[FFT_SIZE];

//used to identify startup of the system, to discard a read
static bool firstread = true;

/* sound_remote(FFT of microphone)
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_remote(float* data)
{
	static uint16_t prev_freq = 0;

	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;
	bool shouldTurn = true;

	//On the first read, the microphones often returns garbage, discard it.
	if(firstread)
	{
		firstread=false;
		return;
	}

	//search for the highest peak
	for(uint16_t i = MIN_FREQ; i <= MAX_FREQ ; i++)
	{
		if(data[i] > max_norm)
		{
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	//start moving (only after power-on or successful shot
	if(getState() != MANUAL_MOVE && max_norm_index >= FRQ_RESET_MIN && max_norm_index <= FRQ_RESET_MAX)
		switchState(true);
	else if(getState()==MANUAL_MOVE)
	{
		//Check for two consecutives read with nearly equal frequencies
		if(abs(prev_freq-max_norm_index)<=THRSH_FREQ)
		{
			//move forward when there's a continuous pitch at given freq
			if(max_norm_index >=  FRQ_FORWARD_MIN && max_norm_index <= FRQ_FORWARD_MAX){
				left_motor_set_speed(SPEED_MV_COMMAND);
				right_motor_set_speed(SPEED_MV_COMMAND);
				shouldTurn=false;
			}
			//Start to search for ball if freq is matching
			else if(max_norm_index >= FRQ_SEARCH_MIN && max_norm_index <= FRQ_SEARCH_MAX){
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				switchState(true);
				shouldTurn=false;
			}
			else
				shouldTurn=true;
		}
		//turn around while waiting for command
		if(shouldTurn){
			left_motor_set_speed(-SPEED_WAIT_COMMAND);
			right_motor_set_speed(SPEED_WAIT_COMMAND);
		}
	}
	prev_freq = max_norm_index;
}

void doFFT_optimized(uint16_t size, float* complex_buffer){
	if(size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples)
{
	static uint16_t nb_samples = 0;

	//do not process any audio if its not the time to, save (lots of) cycles
	if(getState() != MANUAL_MOVE && getState() != STARTUP)
	{
		firstread = true;
		return;
	}

	//loop to fill the buffer, only take sample for the back microphone
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		nb_samples++;
		micBack_cmplx_input[nb_samples] = 0;
		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		//Calculate FFT and get associated magnitude in frequency domain for each sample
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);
		nb_samples = 0;
		sound_remote(micBack_output);
	}
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name)
{
	if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}
