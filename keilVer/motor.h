#ifndef __MOTOR_H
#define __MOTOR_H
#include "common.h"



 void motor_init(void);
 void direct_init(void);
 void right();
 void left();
  void straight();
	void steering_pid(int error);
	void motor_straight_pid();
	void make_a_circle();

#endif

