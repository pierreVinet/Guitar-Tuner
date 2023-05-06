#include "main.h"
#include "perpendicular_calibration.h"
#include "leds.h"

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

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

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
    // inits time of flight
    VL53L0X_start();

    perpendicular_calibration_start();

    while (1)
    {
        chThdSleepMilliseconds(1000);
    }
}

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
