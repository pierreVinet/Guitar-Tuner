#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

// Different strings of the guitar
typedef enum
{
    FIRST_STRING = 1,
    SECOND_STRING,
    THIRD_STRING,
    FOURTH_STRING,
    FIFTH_STRING,
    SIXTH_STRING,
    NO_STRING

} GUITAR_STRING;

float get_frequency(void);
GUITAR_STRING get_guitar_string(void);
float get_string_frequency(void);
bool get_pitch(void);
void processAudioData(int16_t *data, uint16_t num_samples);

#endif /* AUDIO_PROCESSING_H */