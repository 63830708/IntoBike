#ifndef NEW_PWM_OUT_H_
#define NEW_PWM_OUT_H_
#include "application.h"
#include "wiring_analog.h"
#define MAX_DUTY_VALUE 255

void outputPWM(uint16_t pin, uint32_t duty, uint32_t pwm_frequency);

#endif