

#ifndef _UTIL_H
#define _UTIL_H

#include <Arduino.h>
#include <inttypes.h>
#include "user_config.h"
#include "parameters.h"
#include "util.cpp"

#ifdef FAST_GNSS
// USE U-Center to get the UBX binary command.
const unsigned char UBX_UPDATE_5HZ[] PROGMEM =
    {0x06, 0x08, 0x06, 0x00, 200, 0x00, 0x01, 0x00, 0x01, 0x00};

extern inline void send_ubx(NeoSWSerial *gps_port, const unsigned char *progmemBytes, uint16_t len);
#endif

extern inline int16_t normalize_angle(int16_t angle);

#ifdef DEBUG
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINTLN(...) ;
#endif

#ifdef DEBUG
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) ;
#endif

#define INITIALIZE_PWM_PIN(pin_name) \
    pinMode((pin_name), OUTPUT);     \
    analogWrite((pin_name), 0);

#endif