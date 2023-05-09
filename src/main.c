#include "main.h"
#include "audio_processing.h"
#include "communications.h"
#include "motors_commands.h"
#include "process_image.h"
#include "line_tracking.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <arm_math.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <leds.h>
#include <camera/po8030.h>

// uncomment to send values to the computer
// #define DEBUG

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;
static FSM_STATE state = 0;
// tableau avec les frequences exactes de chaque corde de la guitare (en ordre: de la première corde à la sixième)
static float string_frequency[] = {FIRST_STRING_FREQ, SECOND_STRING_FREQ, THIRD_STRING_FREQ, FOURTH_STRING_FREQ, FIFTH_STRING_FREQ, SIXTH_STRING_FREQ};
static uint16_t string_coeff[] = {43, 37, 29, 22, 17, 13};

static void serial_start(void)
{
    static SerialConfig ser_cfg = {
        115200,
        0,
        0,
        0,
    };

    sdStart(&SD3, &ser_cfg); // UART3.
}

FSM_STATE get_FSM_state(void)
{
    return state;
}

void set_FSM_state(FSM_STATE new_state)
{
    state = new_state;
}

void increment_FSM_state(void)
{
    state++;
}

void handle_string_position(void)
{
    uint16_t distance_string = get_guitar_string() * DISTANCE_STRING;
    advance_until_distance_reached(distance_string, (uint8_t)TOF_PRECISION);
}

void handle_frequency_position(void)
{
    GUITAR_STRING string_number = get_guitar_string();
    float frequency_measured = get_frequency();
    float distance_frequency = 300.00 + (string_frequency[string_number - 1] - frequency_measured) * string_coeff[string_number - 1];
    advance_until_distance_reached(distance_frequency, (uint8_t)TOF_PRECISION);
}

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();
    // clear_leds();

    serial_start();
    // starts the USB communication
    usb_start();
    // starts the camera
    dcmi_start();
    po8030_start();
    // inits the motors
    motors_init();

    // send_tab is used to save the state of the buffer to send (double buffering)
    // to avoid modifications of the buffer while sending it
#ifdef DEBUG
    static float send_tab[FFT_SIZE];
#endif /*DEBUG*/

    // starts the microphones processing thread.
    // it calls the callback given in parameter when samples are ready
    // It initializes the interfaces used to sample the microphones and the thread used to demodulate the signals
    mic_start(&processAudioData);
    // inits time of flight
    VL53L0X_start();

    // init color detection mode: see process_image.h for values
    select_color_detection(BLUE_COLOR);

    // disable motors by default. Can be enable from plotImage Python code
    set_enabled_motors(true);

    // stars the threads for the pi regulator and the processing of the image
    pi_regulator_start();
    process_image_start();

    while (1)
    {
#ifdef DEBUG
        // waits until a result must be sent to the computer
        wait_send_to_computer();
        // we copy the buffer to avoid conflicts
        arm_copy_f32(get_audio_buffer_ptr(LEFT_OUTPUT), send_tab, FFT_SIZE);
        SendFloatToComputer((BaseSequentialStream *)&SD3, send_tab, FFT_SIZE);
#endif /* DEBUG */
        switch (state)
        {
        case STRING_POSITION:
            handle_string_position();
            break;
        case FREQUENCY_POSITION:
            select_color_detection(RED_COLOR);
            handle_frequency_position();
            break;
        default:
            break;
        }
        chThdSleepMilliseconds(200);
    }
}

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
