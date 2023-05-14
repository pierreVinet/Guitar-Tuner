#include "main.h"
#include "audio_processing.h"
#include "image_processing.h"
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
// In order to be able to use the RGB LEDs and User button
// These funtcions are handled by the ESP32 and the communication with the uC is done via SPI
#include <spi_comm.h>

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

static FSM_STATE previous_state = 0;
static FSM_STATE state = 0;

/*
 *	Starts the serial communication.
 */
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

/*
 *	Returns the current state of the FSM (Finite-State Machine).
 */
FSM_STATE get_FSM_state(void)
{
	return state;
}

/*
 *	Returns the previous state of the FSM.
 */
FSM_STATE get_FSM_previous_state(void)
{
	return previous_state;
}

/*
 *	Set the previous state of the FSM with the current one,
 * 	and set the current state with the new one.
 */
void set_FSM_state(FSM_STATE new_state)
{
	previous_state = state;
	state = new_state;
}

/*
 *	Set the previous state of the FSM with the current one,
 * 	and increment the current state.
 */
void increment_FSM_state(void)
{
	previous_state = state;
	state++;
}

/*
 *	Clear all the rgb leads.
 */
void clear_rgb_leds(void)
{
	set_rgb_led(LED2, 0, 0, 0);
	set_rgb_led(LED4, 0, 0, 0);
	set_rgb_led(LED6, 0, 0, 0);
	set_rgb_led(LED8, 0, 0, 0);
}

int main(void)
{
	halInit();
	chSysInit();
	mpu_init();
	// clear all leds
	clear_leds();
	clear_rgb_leds();

	serial_start();
	// starts the USB communication
	usb_start();
	// starts the camera
	dcmi_start();
	po8030_start();
	// init color detection mode of the camera
	select_color_detection(BLUE_COLOR);
	// inits the motors
	motors_init();
	// starts RGB LEDS and User button managment
	spi_comm_start();
	// inits time of flight
	VL53L0X_start();
	// starts the microphones processing thread. It calls the callback given in parameter when samples are ready
	mic_start(&processAudioData);
	// stars the threads for the pi regulator
	line_tracking_start();
	// starts the thread for the processing of the image
	image_processing_start();

	while (1)
	{
		chThdSleepSeconds(1);
	}
}

void __stack_chk_fail(void)
{
	chSysHalt("Stack smashing detected");
}
