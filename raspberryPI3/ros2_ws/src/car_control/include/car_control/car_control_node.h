#ifndef __car_control_node_H
#define __car_control_node_H

#include <stdint.h>
#include <string.h>  


#define PERIOD_UPDATE_CMD 2ms //Period to update proupulsion and steering command in [ms]

#define STOP 50               //PWM value to stop motors

// Steering constants
#define SERVO_FULL_LEFT -1
#define SERVO_ZERO 0
#define SERVO_FULL_RIGHT 1

#define STEERING_MAX_LEFT  -0.6108652 // radian
#define STEERING_CENTER    0.0f        // radian
#define STEERING_MAX_RIGHT 0.6108652 // radian

#define SOURCE_JOYSTICK 0
#define SOURCE_HMI      1

#define MODE_MANUAL         0
#define MODE_AUTONOMOUS     1
#define MODE_CALIBRATION    2

#endif /*__ car_control_node_H */