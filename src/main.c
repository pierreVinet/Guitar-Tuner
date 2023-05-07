#include "main.h"
#include "perpendicular_calibration.h"
#include "audio_processing.h"
#include "communications.h"

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

// uncomment to send values to the computer
// #define DEBUG

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;
static FSM_STATE state;

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

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();
    clear_leds();
    set_body_led(0);
    set_front_led(0);

    serial_start();
    // starts the USB communication
    usb_start();
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

    perpendicular_calibration_start();

    // set the state of the FSM to the first state
    set_FSM_state(0);
    while (1)
    {
#ifdef DEBUG
        // waits until a result must be sent to the computer
        wait_send_to_computer();
        // we copy the buffer to avoid conflicts
        arm_copy_f32(get_audio_buffer_ptr(LEFT_OUTPUT), send_tab, FFT_SIZE);
        SendFloatToComputer((BaseSequentialStream *)&SD3, send_tab, FFT_SIZE);
#endif /* DEBUG */
        // chThdSleepMilliseconds(200);
    }
}

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
