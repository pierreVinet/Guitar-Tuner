#include "main.h"
#include "audio_processing.h"
#include "communications.h"
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

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();
    clear_leds();

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
            // set_body_led(1);
            break;
        case FREQUENCY_POSITION:
            // set_front_led(1);
            break;
        case ROTATION:
            robot_rotate(90,0);
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
