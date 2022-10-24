
#include <Arduino.h>
#include <inttypes.h>
#include "util.h"
#include "parameters.h"

typedef enum driving_direction
{
    FORWARD,
    STOP,
} DRIVING_DIRECTION;

typedef enum steering_direction
{
    RIGHT,
    STRAIGHT,
    LEFT,
} STEERING_DIRECTION;

// *(10 ms)
#define STEERING_DURATION 25
#define DRIVE_DURATION 200
#define STOP_DURATION 0

#define INITIALIZE_DRIVE()      \
    pinMode(WHEEL_L_F, OUTPUT); \
    pinMode(WHEEL_L_B, OUTPUT); \
    pinMode(WHEEL_R_F, OUTPUT); \
    pinMode(WHEEL_R_B, OUTPUT); \
    diff_drive(STOP, STRAIGHT);

inline void diff_drive(DRIVING_DIRECTION drive, STEERING_DIRECTION steer)
{
    if (drive == STOP)
    {
        digitalWrite(WHEEL_L_F, LOW);
        digitalWrite(WHEEL_L_B, LOW);
        digitalWrite(WHEEL_R_F, LOW);
        digitalWrite(WHEEL_R_B, LOW);
        return;
    }

    switch (steer)
    {
    case LEFT:
        digitalWrite(WHEEL_L_F, HIGH);
        digitalWrite(WHEEL_L_B, LOW);
        digitalWrite(WHEEL_R_F, LOW);
        digitalWrite(WHEEL_R_B, HIGH);
        return;
    case RIGHT:
        digitalWrite(WHEEL_L_F, LOW);
        digitalWrite(WHEEL_L_B, HIGH);
        digitalWrite(WHEEL_R_F, HIGH);
        digitalWrite(WHEEL_R_B, LOW);
        return;
    case STRAIGHT:
        digitalWrite(WHEEL_L_F, LOW);
        digitalWrite(WHEEL_L_B, HIGH);
        digitalWrite(WHEEL_R_F, LOW);
        digitalWrite(WHEEL_R_B, HIGH);
        return;
    }
}