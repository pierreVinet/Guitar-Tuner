#include <ch.h>
#include <hal.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <audio/microphone.h>
#include <arm_math.h>
#include <arm_const_structs.h>

#include "audio_processing.h"
#include "main.h"
#include "communications.h"

// frequency for each string of the guitar
#define SIXTH_STRING_FREQ_MIN 76
#define SIXTH_STRING_FREQ 82.41
#define SIXTH_STRING_FREQ_MAX 88

#define FIFTH_STRING_FREQ_MIN 103
#define FIFTH_STRING_FREQ 110.00
#define FIFTH_STRING_FREQ_MAX 117

#define FOURTH_STRING_FREQ_MIN 138
#define FOURTH_STRING_FREQ 146.83
#define FOURTH_STRING_FREQ_MAX 156

#define THIRD_STRING_FREQ_MIN 184
#define THIRD_STRING_FREQ 196.00
#define THIRD_STRING_FREQ_MAX 208

#define SECOND_STRING_FREQ_MIN 232
#define SECOND_STRING_FREQ 246.94
#define SECOND_STRING_FREQ_MAX 262

#define FIRST_STRING_FREQ_MIN 310
#define FIRST_STRING_FREQ 329.63
#define FIRST_STRING_FREQ_MAX 350

// minimum value for intensity to detect frequency
#define MIN_VALUE_THRESHOLD 1000
// we don't analyze before this index to not use resources for nothing
#define MIN_FREQ 100
// we don't analyze after this index to not use resources for nothing
#define MAX_FREQ 500

// semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

static float frequency;
static GUITAR_STRING guitar_string;

// input buffers for each microphones coded in 32bits
// 2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
// The data are arranged like [real0, imag0, real1, imag1, etc...]
// 2 * FTT_SIZE * 4Bytes (32bits) = 8192 Bytes / microphone
static float micLeft_cmplx_input[2 * FFT_SIZE];
// outupt buffers coded in 32bits
// Arrays containing the computed magnitude of the complex numbers
// FTT_SIZE * 4Bytes (32bits) = 4096 Bytes / microphone
static float micLeft_output[FFT_SIZE];

/*
 *	Wrapper to call a very optimized fft function provided by ARM
 *	which uses a lot of tricks to optimize the computations
 */
void doFFT_optimized(uint16_t size, float *complex_buffer)
{
    if (size == 1024)
        arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);
}

/*
 *	Simple function used to detect the highest value in a buffer
 *	and to execute a motor command depending on it
 */
uint16_t find_highest_peak(float *data)
{
    float max_norm = MIN_VALUE_THRESHOLD;
    int16_t max_norm_index = 0;

    // search for the highest peak
    for (uint16_t i = MIN_FREQ; i <= MAX_FREQ; i++)
    {
        if (data[i] > max_norm)
        {
            max_norm = data[i];
            max_norm_index = i;
        }
    }

    return max_norm_index;
}

GUITAR_STRING find_guitar_note(void)
{
    if (frequency > SIXTH_STRING_FREQ_MIN && frequency < SIXTH_STRING_FREQ_MAX)
    {
        return SIXTH_STRING;
    }
    else if (frequency > FIFTH_STRING_FREQ_MIN && frequency < FIFTH_STRING_FREQ_MAX)
    {
        return FIFTH_STRING;
    }
    else if (frequency > FOURTH_STRING_FREQ_MIN && frequency < FOURTH_STRING_FREQ_MAX)
    {
        return FOURTH_STRING;
    }
    else if (frequency > THIRD_STRING_FREQ_MIN && frequency < THIRD_STRING_FREQ_MAX)
    {
        return THIRD_STRING;
    }
    else if (frequency > SECOND_STRING_FREQ_MIN && frequency < SECOND_STRING_FREQ_MAX)
    {
        return SECOND_STRING;
    }
    else if (frequency > FIRST_STRING_FREQ_MIN && frequency < FIRST_STRING_FREQ_MAX)
    {
        return FIRST_STRING;
    }
    return NO_STRING;
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
    // checks if the Finite State Machine is in the correct state
    if (get_FSM_state() == FREQUENCY_DETECTION)
    {
        /*
         *
         *	We get 160 samples per mic every 10ms
         *	So we fill the samples buffers to reach
         *	1024 samples, then we compute the FFTs.
         *
         */

        static uint16_t nb_samples = 0;

        // loop to fill the input buffers with the sample for the real part and 0 for the imaginary part
        // data contains the sample of all 4 mics: [mic1, mic2, mic3, mic4, mic1, mic2...]
        // if I want only the samples of 1 mic at 16kHz: i+= 4*1
        // if I want only the samples of 1 mic at 800Hz: i+= 4*20 (=80)
        for (uint16_t i = 0; i < num_samples; i += 80)
        {
            // construct an array of complex numbers. Put 0 to the imaginary part
            micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];

            nb_samples++;

            micLeft_cmplx_input[nb_samples] = 0;

            nb_samples++;

            // stop when buffer is full
            if (nb_samples >= (2 * FFT_SIZE))
            {
                break;
            }
        }

        if (nb_samples >= (2 * FFT_SIZE))
        {
            /*	FFT proccessing
             *
             *	This FFT function stores the results in the input buffer given.
             *	This is an "In Place" function.
             */
            doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);

            /*	Magnitude processing
             *
             *	Computes the magnitude of the complex numbers and
             *	stores them in a buffer of FFT_SIZE because it only contains
             *	real numbers.
             */
            arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

            frequency = find_highest_peak(micLeft_output) * FREQUENCY_PRECISION;

            guitar_string = find_guitar_note();

            if (guitar_string != NO_STRING)
            {
                // change state of the FSM to the next step
                set_FSM_state(PERPENDICULAR_CALIBRATION);
            }

            chprintf((BaseSequentialStream *)&SD3, "frequency = %f\n", frequency);
            // chprintf((BaseSequentialStream *)&SD3, "highest peak = %d\n", find_highest_peak(micLeft_output));
            nb_samples = 0;
        }
    }
}

void wait_send_to_computer(void)
{
    chBSemWait(&sendToComputer_sem);
}

float *get_audio_buffer_ptr(BUFFER_NAME_t name)
{
    if (name == LEFT_CMPLX_INPUT)
    {
        return micLeft_cmplx_input;
    }
    else if (name == LEFT_OUTPUT)
    {
        return micLeft_output;
    }
    else
    {
        return NULL;
    }
}

float get_frequency(void)
{
    return frequency;
}

GUITAR_STRING get_guitar_string(void)
{
    return guitar_string;
}