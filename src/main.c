#include "main.h"
#include "audio_processing.h"
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
// In order to be able to use the RGB LEDs and User button
// These funtcions are handled by the ESP32 and the communication with the uC is done via SPI
#include <spi_comm.h>

// uncomment to send values to the computer
// #define DEBUG

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

static FSM_STATE previous_state = 0;
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

FSM_STATE get_FSM_previous_state(void)
{
	return previous_state;
}

void set_FSM_state(FSM_STATE new_state)
{
	previous_state = state;
	state = new_state;
}

void increment_FSM_state(void)
{
	previous_state = state;
	state++;
}

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
	clear_leds();
	clear_rgb_leds();

	serial_start();
	// starts the USB communication
	usb_start();
	// starts the camera
	dcmi_start();
	po8030_start();
	// inits the motors
	motors_init();

	// starts RGB LEDS and User button managment
	spi_comm_start();
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

	set_rgb_led(LED2, 0, 0, 255);
	set_rgb_led(LED4, 0, 255, 0);
	set_rgb_led(LED6, 255, 0, 0);
	set_rgb_led(LED8, 255, 255, 0);

	while (1)
	{
		switch (state)
		{
		case FREQUENCY_DETECTION:
			break;
		case STRING_POSITION:
			break;
		case FREQUENCY_POSITION:
			break;
		case ROTATION:
			break;
		case STRING_CENTER:
			break;
		default:
			// set_body_led(1);
			break;
		}
		chThdSleepMilliseconds(200);
	}
}

void __stack_chk_fail(void)
{
	chSysHalt("Stack smashing detected");
}
