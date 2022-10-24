#ifndef _PARAMETERS_H
#define _PARAMETERS_H

/******************************
 *  Pin definition for motors  *
 ******************************/
#define WHEEL_L_F 10 // D5
#define WHEEL_L_B 3  // D6
#define WHEEL_R_F 6  // D9
#define WHEEL_R_B 5  // D10

#define GNSS_BAUD_RATE 9600

/*
 * OFFSET CALIBRATION
 */
#define COMPASS
/*
 * CONTROLLER LOOP
 */
#define KP 1

// The bearing threshold is the amount of degrees the vehicle can be off its desired bearing before adjusting the course
#define HEADING_THRESHOLD 10 // [DEGREES]
// Distance threshold is the amount of meters away from target coordinate, that when exceeded it will stop.
#define DISTANCE_THRESHOLD_M 0.3   // [METERS]
#define DISTANCE_THRESHOLD_ERR_M 2 // [METERS]
#define DISTANCE_SLOW 7            // [METERS]

#define DRIVE_SLOW_DELAY 100 // [*10 ms]
#define DRIVE_FAST_DELAY 200 // [*10 ms]

// error between 0 and 180
// divide by two for prop constant.
// divide 15 dry, 5 wet
#define TURNING_DELAY(error) (abs(error) / 5 + 10)

// Derived from testing with the 50 Hz PWM configuration.
#define DRIVE_PWM_MAX_VALUE 156
#define DRIVE_PWM_MIN_VALUE (DRIVE_PWM_MAX_VALUE / 8)
#define DRIVE_SPEED DRIVE_PWM_MAX_VALUE // (DRIVE_PWM_MAX_VALUE / 7)

#define GNSS_PORT 8, 9
#define SATELLITES_MIN 6
#define NMEAGPS_INTERRUPT_PROCESSING
// #define GPS_FIX_HDOP
// #define FAST_GNSS

#define KM_M_INT(distance_k) int(distance_k * 1000)

#define ULTRASONIC_SENSOR

#ifdef ULTRASONIC_SENSOR
#define US_TRIGGER_PIN 13       // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define US_ECHO_PIN 11          // Arduino pin tied to echo pin on the ultrasonic sensor.
#define US_MAX_DISTANCE 500     // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define US_SEARCH_TIME 300      // [*10 ms]
#define US_SEARCH_ANGLE_ITER 10 // [*10 ms]
#endif

#endif // _PARAMETERS_H