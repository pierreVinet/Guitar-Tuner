#include <ch.h>
#include <hal.h>
#include <usbcfg.h>
#include <audio/microphone.h>
#include <arm_math.h>
#include <arm_const_structs.h>
#include <leds.h>

#include "audio_processing.h"
#include "main.h"


#define FFT_SIZE 1024

// minimum value of intensity to detect a frequency
#define MIN_INTENSITY_THRESHOLD 5000
// we don't analyze before this index to not use resources for nothing
#define MIN_INDEX 100
// we don't analyze after this index to not use resources for nothing
#define MAX_INDEX 500
#define FREQUENCY_PRECISION 0.765517

// frequency for each string of the guitar and their respective range
#define SIXTH_STRING_FREQ_MIN 76
#define SIXTH_STRING_FREQ 82.41f
#define SIXTH_STRING_FREQ_MAX 88

#define FIFTH_STRING_FREQ_MIN 103
#define FIFTH_STRING_FREQ 110.00f
#define FIFTH_STRING_FREQ_MAX 117

#define FOURTH_STRING_FREQ_MIN 138
#define FOURTH_STRING_FREQ 146.83f
#define FOURTH_STRING_FREQ_MAX 156

#define THIRD_STRING_FREQ_MIN 184
#define THIRD_STRING_FREQ 196.00f
#define THIRD_STRING_FREQ_MAX 208

#define SECOND_STRING_FREQ_MIN 232
#define SECOND_STRING_FREQ 246.94f
#define SECOND_STRING_FREQ_MAX 262

#define FIRST_STRING_FREQ_MIN 310
#define FIRST_STRING_FREQ 329.63f
#define FIRST_STRING_FREQ_MAX 350

// float array containing the theoretical frequencies of each string of the guitar
static float string_frequency[] = {FIRST_STRING_FREQ, SECOND_STRING_FREQ, THIRD_STRING_FREQ, FOURTH_STRING_FREQ, FIFTH_STRING_FREQ, SIXTH_STRING_FREQ};
static float frequency;
static GUITAR_STRING previous_guitar_string = NO_STRING;
static GUITAR_STRING guitar_string = NO_STRING;

/*
 *  Input complex buffer for the left microphone.
 *  2 times FFT_SIZE because this array contain complex numbers (real + imaginary).
 *  The data are arranged like [real0, imag0, real1, imag1, etc...].
 */
static float micLeft_cmplx_input[2 * FFT_SIZE];
// outupt buffer containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];

/*
 *	Wrapper to call a very optimized fft function provided by ARM.
 *
 *	params :
 *	uint16_t size		        Size of the FFT must be equak to 1024.
 *  float *complex_buffer       Input complex buffer to apply the FFT to.
 */
void doFFT_optimized(uint16_t size, float *complex_buffer)
{
    if (size == 1024)
        arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);
}

/*
 *	Returns the index associated at the frequency with the highest amplitude.
 *
 *	params :
 *	float *data			outupt buffer containing the computed magnitude
 *                      of the complex numbers. Size: FTT_SIZE.
 */
uint16_t find_highest_peak(float *data)
{
    float max_norm = MIN_INTENSITY_THRESHOLD;
    int16_t max_norm_index = 0;

    // search for the highest peak
    for (uint16_t i = MIN_INDEX; i <= MAX_INDEX; i++)
    {
        if (data[i] > max_norm)
        {
            max_norm = data[i];
            max_norm_index = i;
        }
    }
    return max_norm_index;
}

/*
 *	Search and return the string affiliated with the frequency detected.
 */
GUITAR_STRING find_guitar_string(void)
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
 *	Returns the frequency detected.
 */
float get_frequency(void)
{
    return frequency;
}

/*
 *	Returns the string of the frequency detected.
 */
GUITAR_STRING get_guitar_string(void)
{
    return guitar_string;
}

/*
 *	Returns the theoretical frequency of the string detected.
 */
float get_string_frequency(void)
{
    return string_frequency[guitar_string - 1];
}

/*
 *	If the pitch is higher than the theoretical string frequency, returns true
 *  If the pitch is lower than the theoretical string frequency, returns false
 */
bool get_pitch(void)
{
    if (frequency - string_frequency[guitar_string - 1] >= 0)
    {
        return true;
    }
    else
    {
        return false;
    }
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
        set_all_rgb_leds(0, 0, 255);
        static uint16_t nb_samples = 0;

        /*
         *  Loop to fill the input buffers with the sample for the real part and 0 for the imaginary part.
         *  Data contains the sample of all 4 mics: [mic1, mic2, mic3, mic4, mic1, mic2...].
         *  To get the samples of 1 mic at 800Hz: i+= 4*20 (= 80).
         */
        for (uint16_t i = 0; i < num_samples; i += 80)
        {
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
            // This FFT function stores the results in the input buffer given.
            doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
            // Computes the magnitude of the complex numbers and stores them in a buffer of FFT_SIZE
            arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

            frequency = find_highest_peak(micLeft_output) * FREQUENCY_PRECISION;
            previous_guitar_string = guitar_string;
            guitar_string = find_guitar_string();

            if (guitar_string != NO_STRING)
            {
                switch (get_FSM_previous_state())
                {
                case FREQUENCY_DETECTION:
                    clear_rgb_leds();
                    increment_FSM_state();
                    break;

                case FREQUENCY_POSITION:
                    if (previous_guitar_string == guitar_string)
                    {
                        clear_rgb_leds();
                        set_FSM_state(FREQUENCY_POSITION);
                    }
                    else
                    {
                        clear_rgb_leds();
                        set_FSM_state(STRING_CENTER);
                    }
                    break;
                default:
                    clear_rgb_leds();
                    set_FSM_state(DO_NOTHING);
                    break;
                }
            }
            nb_samples = 0;
        }
    }
}