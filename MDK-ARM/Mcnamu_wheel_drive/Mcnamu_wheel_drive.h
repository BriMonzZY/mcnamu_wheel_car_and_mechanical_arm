#ifndef __WHEEL_H
#define __WHEEL_H

#include "main.h"
#include "tim.h"


void wheel1_speed_set(float speed);
void wheel2_speed_set(float speed);
void wheel3_speed_set(float speed);
void wheel4_speed_set(float speed);
void all_directions(int v, int angle, int angular_velocity);

#endif
