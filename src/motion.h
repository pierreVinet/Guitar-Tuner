#ifndef MOTION_H
#define MOTION_H

// wall faced by the TOF of the EPuck2
typedef enum
{
    WALL_0 = 0,
    WALL_1,
    WALL_2,
    WALL_3,

} WALL_FACED;

void motion_start(void);

#endif /* MOTION_H */