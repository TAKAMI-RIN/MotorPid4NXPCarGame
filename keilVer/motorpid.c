#include "motorpid.h"

int16_t target ;                      //  �� -80�������ת��� +80�������ת���
int16_t target1 ;                     //  �ң� -80�������ת��� +80�������ת���
int16_t g;                            //     �м����L
int16_t g1 ;                          //     �м����R
int16_t Kp = 11;                      //     ����ϵ��
int16_t Ki = 7;                       //     ����ϵ��
int16_t Kd = 1;                       //     ΢��ϵ��
int16_t e_0=0;                        //     һ�����������
int16_t e_1=0;                        //     �������������
int16_t e_2=0;                        //     �������������
int16_t out, count, kr ;              //     �м���
int16_t ep, ei, ed;                   //     �м���
int16_t feedback;                     //     �������������������ֵ����
int16_t feedback1;                    //     �������������������ֵ���ң�
int16_t e_01=0;                       //     һ����������ң�
int16_t e_11=0;                       //     ������������ң�
int16_t e_21=0;                       //     ������������ң�
int16_t out1, count1, kr1 ;           //     �м���
int16_t ep1, ei1, ed1;                //     �м���
int16_t amg=5;                       
int16_t rrs;                          //used for start key

void motor_init(void){
	FTM_PWM_QuickInit(FTM0_CH0_PC01 , kPWM_EdgeAligned , 5000);   //���Ƶ��5khz
	FTM_PWM_QuickInit(FTM0_CH1_PC02 , kPWM_EdgeAligned , 5000);   //���Ƶ��5khz
	FTM_PWM_QuickInit(FTM0_CH2_PC03 , kPWM_EdgeAligned , 5000);   //���Ƶ��5khz
	FTM_PWM_QuickInit(FTM0_CH3_PC04 , kPWM_EdgeAligned , 5000);   //���Ƶ��5khz
	FTM_PWM_ChangeDuty(HW_FTM0 , HW_FTM_CH0 , 0);                 //left
	FTM_PWM_ChangeDuty(HW_FTM0 , HW_FTM_CH1 , 0);                 //left r
	FTM_PWM_ChangeDuty(HW_FTM0 , HW_FTM_CH2 , 0);                 //right
	FTM_PWM_ChangeDuty(HW_FTM0 , HW_FTM_CH3 , 0);                 //right r
}

void motorpid_Init(void){
	target = 0;                                       //  ��target��ֵ�������������ת��
	target1 = 0;                                      //  ��target1��ֵ���������Һ���ת��
	LPTMR_PC_QuickInit(LPTMR_ALT1_PA19);
	
	PIT_QuickInit(HW_PIT_CH0, 2000);                   //��ʼ��PITģ���0ͨ��������2ms���ж�
	PIT_CallbackInstall(HW_PIT_CH0, PIT0_CallBack);    //ע��ص�����
	PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF,ENABLE);   //����ģ��0ͨ���ж�
	
	GPIO_QuickInit(HW_GPIOC, 12, kGPIO_Mode_IFT);      //�ұ߱���������ת��־λ
	GPIO_QuickInit(HW_GPIOC, 11, kGPIO_Mode_IFT);      //��߱���������ת��־λ
	if(target<=80 &&target>=7 ){
		g = (target *88+37)/10;
	}
	else if(target>=-80 && target<=-7){
		g = (target *88-37)/10;
	}
	else{
		g=0;
	}
	if(target1<=80 &&target1>=7 ){
		g1 = (target1 *88+37)/10;
	}
	else if(target1>=-80 && target1<=-7){
		g1 = (target1 *88-37)/10;
	}
	else{
		g1=0;
	}
}

static void PIT0_CallBack(void){
	if(amg==5){
		count =  LPTMR_PC_ReadCounter();                      //��ȡ�����������������
		LPTMR_PC_QuickInit(LPTMR_ALT2_PC05);
		if(GPIO_ReadBit(HW_GPIOC, 11)==1){
			feedback = count;                                   //lptmr workpoint
		}
		else{
			feedback = 0-count;                                 //lptmr workpoint
		}
		e_0 = target - feedback ;                             //lptmr workpoint
		ep = e_0  - e_1;                                      //lptmr workpoint
		ei = e_0;                                             //lptmr workpoint
		ed = e_0 - 2*e_1 + e_2;                               //lptmr workpoint
		kr = (Kp*ep + Ki*ei + Kd*ed + g)*10 ;                                                  		  
		amg++;                                              
		if(kr>=10000){
			FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH0  , 9000);   
			FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH1  , 0);		  	
		}
		else if(kr<0 && target>=7){
			FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH0  , 0);     
			FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH1  , 0);      
		}
		else if(kr<0){
			FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH0  , 0);  
			FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH1  , 0-kr); 
		}
		else{
			FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH0  , kr);   
			FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH1  , 0);     
		}
		e_2 = e_1;                                          
		e_1 =e_0;                                         
		LPTMR_ClearCounter();                         
	}
	else{
		count1 =  LPTMR_PC_ReadCounter();                   
		LPTMR_PC_QuickInit(LPTMR_ALT1_PA19);	              
		if(GPIO_ReadBit(HW_GPIOC, 12)==0){
			feedback1 = count1;                            
		}
		else{
			feedback1 = 0-count1;                             
		}
		e_01 = target1 - feedback1 ;    
		ep1 = e_01  - e_11;                         
		ei1 = e_01;                                    
		ed1 = e_01 - 2*e_11 + e_21;                        
		kr1 = (Kp*ep1 + Ki*ei1 + Kd*ed1 + g1)*10 ;                     
		amg--;		                                         
		if(kr1>=10000){
			FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH2  , 9000);   
			FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH3  , 0);      
		}
		else if(kr1<0 && target1>=7){
			FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH2  , 0);  
			FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH3  , 0);     
		}
		else if(kr1<0){
			FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH2  , 0);      
			FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH3  , 0-kr1);  
		}
		else{
			FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH2  , kr1);    
			FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH3  , 0);   
		}
		e_21 = e_11;                                          //lptmr workpoint
		e_11 =e_01;	                                          //lptmr workpoint
		LPTMR_ClearCounter();                                
	}
}

void set_target(int16_t a,int16_t b){
	target = a;
	target1 = b;
	if(target<=80 && target>=7 ){
		g = (target *88+37)/10;
	}
	else if(target>=-80 && target<=-7){
		g = (target *88-37)/10;
	}
	else{
		g=0;
	}

	if(target1<=80 && target1>=7 ){
		g1 = (target1 *88+37)/10;
	}
	else if(target1>=-80 && target1<=-7){
		g1 = (target1 *88-37)/10;
	}
	else{
		g1=0;
	}

}

void circle_right(void){
	FTM_PWM_ChangeDuty(HW_FTM1, HW_FTM_CH0 , 960);
	set_target(72,45);
}
 
void circle_left(void){
	FTM_PWM_ChangeDuty(HW_FTM1, HW_FTM_CH0 , 1070);
	set_target(8,9);
}

void bluetooth_back(void){
	printf("%d %d %d %d\r\n", feedback, feedback1, kr,kr1);
}
			//20170407  reverse pid and g value fixed  LBH
      //20170925  break code need to be upgraded
			//20180413  simplified the callback