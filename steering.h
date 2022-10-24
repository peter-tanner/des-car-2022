#ifndef STEERING_H
#define STEERING_H

#include <inttypes.h>
#include "parameters.h"
#include "util.h"
#include "steering.cpp"

extern inline void diff_drive(DRIVING_DIRECTION drive, STEERING_DIRECTION steer);

#endif