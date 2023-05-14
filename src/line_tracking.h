#ifndef LINE_TRACKING_H
#define LINE_TRACKING_H

// wall faced by the TOF of the EPuck2
typedef enum
{
    WALL_0 = 0,
    WALL_1,
    WALL_2,
    WALL_3,

} WALL_FACED;

void line_tracking_start(void);

#endif /* LINE_TRACKING_H */