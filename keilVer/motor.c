#include "motor.h"
#include "common.h"
#include "ftm.h"

 void motor_init()
 {
   FTM_PWM_QuickInit(FTM0_CH0_PC01, kPWM_EdgeAligned, 5000);  /*电机频率5khz*/
	 FTM_PWM_QuickInit(FTM0_CH1_PC02, kPWM_EdgeAligned, 5000);  /*电机频率5khz*/
	 FTM_PWM_QuickInit(FTM0_CH2_PC03, kPWM_EdgeAligned, 5000);  /*电机频率5khz*/
	 FTM_PWM_QuickInit(FTM0_CH3_PC04, kPWM_EdgeAligned, 5000);  /*电机频率5khz*/
	 FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH0 , 0);            //left
	 FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH1 , 0);             //left r
	 FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH2 , 0);             //right
	 FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH3 , 0);                //right r
 }
 
 
 /*舵机左边打死1120*/
 /*舵机右边打死963*/
 void direct_init()
 {
	 FTM_PWM_QuickInit(FTM1_CH0_PA12, kPWM_EdgeAligned, 50);   /*舵机频率50hz*/
   FTM_PWM_ChangeDuty(HW_FTM1, HW_FTM_CH0  , 620);  
 }
 
 void left()
 {
   FTM_PWM_ChangeDuty(HW_FTM1, HW_FTM_CH0  , 695); 
 }
 
 void right()
 {
   FTM_PWM_ChangeDuty(HW_FTM1, HW_FTM_CH0  , 545); 
 }
 
 void straight()
 {
   FTM_PWM_ChangeDuty(HW_FTM1, HW_FTM_CH0  , 620);  
 }
 
 void steering_pid(int error)
 {
    FTM_PWM_ChangeDuty(HW_FTM1, HW_FTM_CH0  , 620-error*1.25);  
 }

 