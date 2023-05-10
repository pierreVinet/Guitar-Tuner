// inclusion de tout les headers du kernel de ChibiOS (threads...)
#include <ch.h>
// voir main
#include <hal.h>
// fonctions pour print
#include <chprintf.h>
// start the UART for the USB
#include <usbcfg.h>
// fonctions e config et define de la camera (nb de pixels etc...)
#include <camera/po8030.h>

// RGB LED used for interaction with plotImage Python code
#include <leds.h>

// dans le main.h il y a l'inclusion de dcmi_camera.h
#include "main.h"
#include "process_image.h"

// static float distance_cm = 0;
// position of the line (middle, left, right ...)
static uint16_t line_position = IMAGE_BUFFER_SIZE / 2; // middle
// = 1 if a line was found, = 0 if no line was found
static uint8_t line_found = 0;

// Extracts only the red pixels by default
// But the color can be changed with plotImage Python code
static color_detection_t detect_color = RED_COLOR;

// semaphore
// name: image_ready_sem
// initial value: TRUE (the semaphore is taken = red light)
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

void SendUint8ToComputer(uint8_t *data, uint16_t size)
{
    chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t *)"START", 5);
    chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t *)&size, sizeof(uint16_t));
    chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t *)data, size);
}

/*
 *  Returns true (1) if a line is detected or false (0) if no line wider MIN_LINE_WIDTH is detected
 *  If a line is detected, the static variable line_position is updated
 */
void line_detection(uint8_t *buffer)
{
    // i = iterate until IMAGE_BUFFER_SIZE, begin = i (pixel when descending slope found), end = i (pixel when the end of ascending slope found)
    uint16_t i = 0, begin = 0, end = 0;
    // stop = 1 exit the while, wrong_line = 1 width of the line to small (continues searching)
    uint8_t stop = 0, wrong_line = 0;
    // mean intensity of all the pixels (IMAGE_BUFFER_SIZE)
    uint32_t mean = 0;

    // last width
    // static uint16_t last_width = 0;

    // performs an average (intensity pixel average)
    for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++)
    {
        mean += buffer[i];
    }
    mean /= IMAGE_BUFFER_SIZE;

    do
    {
        line_found = 0;
        wrong_line = 0;
        // search for a begin = search for an descending slope
        while (stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
        {
            // the slope must at least be WIDTH_SLOPE wide
            // the beginning of the slope must be higher in intensity than the mean
            // the end of the slope (i + WIDTH_SLOPE) must be lower in intensity than the mean
            if (buffer[i] > mean && buffer[i + WIDTH_SLOPE] < mean)
            {
                begin = i;
                stop = 1;
            }
            i++;
        }
        // if a begin (descending slope) was found, search for an end (ascending slope)
        if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
        {
            stop = 0;

            while (stop == 0 && i < IMAGE_BUFFER_SIZE)
            {
                // the slope must at least be WIDTH_SLOPE wide
                // the end of the slope must be higher in intensity than the mean
                // the beginning of the slope (i + WIDTH_SLOPE) must be lower in intensity than the mean
                if (i > WIDTH_SLOPE && buffer[i] > mean && buffer[i - WIDTH_SLOPE] < mean)
                {
                    end = i;
                    stop = 1;
                    line_found = 1;
                }
                i++;
            }
            // if an end was not found
            // if (i > IMAGE_BUFFER_SIZE || !end)
            // {
            //     line_not_found = 1;
            // }
        }
        // else // if no begin was found
        // {
        //     line_not_found = 1;
        // }

        // if a line too small has been detected, continues the search
        if (line_found && (end - begin) < MIN_LINE_WIDTH)
        {
            i = end;
            begin = 0;
            end = 0;
            stop = 0;
            wrong_line = 1;
        }
    } while (wrong_line);

    // if a line is found
    if (line_found)
    {
        // gives the line position (left, middle, right).
        line_position = (begin + end) / 2;
        // SendUint16ToRealTerm(line_position);
        // returns true (= 1)
    }
    else
    {
        // if no line is found, line_position is update to it's initial value (the robot motors continues straight)
        line_position = IMAGE_BUFFER_SIZE / 2; // middle
    }

    // sets a maximum width or returns the measured width
    // if ((PXTOCM / width) > MAX_DISTANCE)
    // {
    //     return PXTOCM / MAX_DISTANCE;
    // }
    // else
    // {
    //     return width;
    // }
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg)
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    // Takes pixels 0 to IMAGE_BUFFER_SIZE of the lines USED_LINE and USED_LINE + 1 (minimum 2 lines because reasons)
    po8030_advanced_config(FORMAT_RGB565, 0, USED_LINE, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
    dcmi_enable_double_buffering();
    dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
    dcmi_prepare();

    while (1)
    {
        FSM_STATE current_state = get_FSM_state();
        if (current_state == STRING_POSITION || current_state == FREQUENCY_POSITION)
        {
            // starts a capture
            dcmi_capture_start();
            // waits for the capture to be done
            wait_image_ready();
            // the semaphore (image_ready_sem) goes from red to green light: signals an image has been captured
            chBSemSignal(&image_ready_sem);
        }
        chThdSleepMilliseconds(200);
    }
}

static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg)
{
    // set thread name
    chRegSetThreadName(__FUNCTION__);
    // cast arg to void to cancel the warnings of non used variables/parameters that are passed to the thread
    (void)arg;

    // points to the array filled with the last image in RGB565
    // the array with the image:
    // 0x00:    [R4 R3 R2 R1 R0 G5 G4 G3]   1st pixel
    // 0x01:    [G2 G1 G0 B4 B3 B2 B1 B0]   1st pixel
    // 0x02:    [R4 R3 R2 R1 R0 G5 G4 G3]   2nd pixel
    // 0x03:    [G2 G1 G0 B4 B3 B2 B1 B0]   2nd pixel

    uint8_t *img_buff_ptr;
    // all the elements are initialized to 0
    uint8_t image[IMAGE_BUFFER_SIZE] = {0};
    // uint16_t lineWidth = 0;

    bool send_to_computer = true;

    while (1)
    {
        // if (get_FSM_state() == LINE_TRACKING)
        // {
        // semaphore (image_ready_sem) waiting until an image has been captured
        chBSemWait(&image_ready_sem);
        // gets the pointer to the array filled with the last image in RGB565
        img_buff_ptr = dcmi_get_last_image_ptr();

        switch (detect_color)
        {
        case RED_COLOR:
            // toggle_rgb_led(USED_RGB_LED, RED_LED, INTENSITY_RGB_LED);
            // Extracts only the red pixels
            for (uint16_t i = 0; i < (2 * IMAGE_BUFFER_SIZE); i += 2)
            {
                // R4 R3 R2 R1 R0 G5 G4 G3 G2 G1 G0 B4 B3 B2 B1 B0
                // BIG ENDIAN:
                // first byte: R4 R3 R2 R1 R0 G5 G4 G3
                // second byte: G2 G1 G0 B4 B3 B2 B1 B0

                // extracts 5 MSbits of the MSbyte (First byte in big-endian format)
                // takes nothing from the second byte

                // In C, arrays and pointers are closely related, and in many cases they can be used interchangeably.
                // When you declare a pointer in C and initialize it to an array, the pointer acts as an array.
                // Therefore, if img_buff_ptr is a pointer to an array of uint8_t elements, img_buff_ptr[i]
                // can be used to access the i-th element of the array.

                image[i / 2] = (uint8_t)img_buff_ptr[i] & 0xF8;

                //       R4 R3 R2 R1 R0 G5 G4 G3        (uint8_t)img_buff_ptr[i]
                //  &     1  1  1  1  1  0  0  0        0xF8
                //  =   [R4 R3 R2 R1 R0  0  0  0]       image[i/2]
            }
            break;
        case GREEN_COLOR:
            toggle_rgb_led(USED_RGB_LED, GREEN_LED, INTENSITY_RGB_LED);
            // Extracts only the green pixels
            for (uint16_t i = 0; i < (2 * IMAGE_BUFFER_SIZE); i += 2)
            {
                // extracts 3 LSbits of the first byte and the 3 MSbits of second byte
                image[i / 2] = (((uint8_t)img_buff_ptr[i] & 0x07) << 5) + (((uint8_t)img_buff_ptr[i + 1] & 0xE0) >> 3);
            }
            break;
        case BLUE_COLOR:
            toggle_rgb_led(USED_RGB_LED, BLUE_LED, INTENSITY_RGB_LED);
            // Extracts only the blue pixels
            for (uint16_t i = 0; i < (2 * IMAGE_BUFFER_SIZE); i += 2)
            {
                // extracts 5 LSbits of the LSByte (Second byte in big-endian format)
                // and rescale to 8 bits
                // takes nothing from the first byte
                image[i / 2] = ((uint8_t)img_buff_ptr[i + 1] & 0x1F) << 3;
            }
            break;
        }

        // search for a line in the image and gets its width in pixels
        // lineWidth = extract_line_width(image);
        line_detection(image);

        // converts the width into a distance between the robot and the camera
        // if (lineWidth)
        // {
        //     distance_cm = PXTOCM / lineWidth;
        // }

        if (send_to_computer)
        {
            // sends to the computer the image
            SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
        }
        // invert the bool
        send_to_computer = !send_to_computer;
        // }
        // chThdSleepMilliseconds(1000);
    }
}

// float get_distance_cm(void)
// {
//     return distance_cm;
// }

uint8_t get_line_detection(void)
{
    return line_found;
}

uint16_t get_line_position(void)
{
    return line_position;
}

void process_image_start(void)
{
    chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
    chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

void select_color_detection(color_detection_t choice_detect_color)
{
    // Set off the RGB LED
    // set_rgb_led(USED_RGB_LED, 0, 0, 0);
    detect_color = choice_detect_color;
}