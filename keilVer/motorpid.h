#ifndef __MOTORPID_H
#define __MOTORPID_H

#include "lptmr.h"                         //lptmr head
#include "pit.h"                           //lptmr needed pit head
#include "motor.h"
#include "ftm.h"
#include "gpio.h"
#include "uart.h"
#include "led.h"
#include "common.h"

void motor_init(void);
static void PIT0_CallBack(void);
void motorpid_Init(void);
void set_target(int16_t a,int16_t b);
void circle_right(void);
void circle_left(void);
void bluetooth_back(void) ;

#endif
