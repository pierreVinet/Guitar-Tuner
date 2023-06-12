#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <camera/dcmi_camera.h>
#include <msgbus/messagebus.h>
#include <parameter/parameter.h>

    /** Robot wide IPC bus. */
    extern messagebus_t bus;

    extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

// Different states of the FSM
typedef enum
{
    FREQUENCY_DETECTION = 0,
    STRING_POSITION,
    ROTATION,
    FREQUENCY_POSITION,
    STRING_CENTER,
    DO_NOTHING
} FSM_STATE;

FSM_STATE get_FSM_state(void);
FSM_STATE get_FSM_previous_state(void);
void set_FSM_state(FSM_STATE new_state);
void increment_FSM_state(void);
void set_all_rgb_leds(uint8_t red_val, uint8_t green_val, uint8_t blue_val);
void clear_rgb_leds(void);

#endif /* MAIN_H */

#endif