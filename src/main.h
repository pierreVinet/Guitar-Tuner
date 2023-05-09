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

typedef enum
{
    FREQUENCY_DETECTION = 0,
    STRING_POSITION,
    FREQUENCY_POSITION
} FSM_STATE;

#define SPEED_MOTORS 300
// in mm
#define DISTANCE_STRING 100
// in mm
#define TOF_PRECISION 5
// in mm
#define DISTANCE_FREQUENCY

FSM_STATE get_FSM_state(void);
void set_FSM_state(FSM_STATE new_state);
void increment_FSM_state(void);

#endif /* MAIN_H */
