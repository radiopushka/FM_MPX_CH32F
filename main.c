/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2020/04/30
 * Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 *@Note
 USART Print debugging routine:
 USART1_Tx(PA9).
 This example demonstrates using USART1(PA9) as a print debug port output.

*/

#include "debug.h"
#include "ch32v10x.h"
#include "ch32v10x_rcc.h"
#include "ch32v10x_rtc.h"
#include "ch32v10x_adc.h"
#include "ch32v10x_gpio.h"
#include "ch32v10x_misc.h"

/* Global typedef */

/* Global define */
#define MAX_ST_AMPLITUDE 4081 //min:1023 max:65535
#define MAX_PILOT_AMPLITUDE 4096
#define MAX_38_PHASE 26
#define MAX_19_PHASE 52

/* Global Variable */
int m19khz[]={2048,2294,2538,2774,2999,3211,3406,3580,3733,3861,3962,4036,4081,4096,4081,4036,3962,3861,3733,3580,3406,3211,2999,2774,2538,2294,2048,1802,1558,1322,1097,885,690,516,363,235,134,60,15,0,15,60,134,235,363,516,690,885,1097,1322,1558,1802};
int m38khz[]={2048,2538,2999,3406,3733,3962,4081,4081,3962,3733,3406,2999,2538,2048,1558,1097,690,363,134,15,15,134,363,690,1097,1558};

int stereo_amp=0;
int pilot_amp=6;//this percent of signal width

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
*/
void setup_pins(){
//no need ADC and DAC need analog input mode which is 00 or default

}
void setup_dac(){
	RCC->APB2PRSTR|=(1<<29);//reset DAC
	RCC->APB1PCENR|=(1<<29);//DAC clock enable
	DAC->CTLR=(4<<3)|(1<<2)|3;//TIM2 trigger,channel 1 enabled
	//DAC->R12BDHR1; is the input register and expects a 12 bit value

}
//channel 0 ADC1 - right channel, channel 1 ADC1 - left channel
void setup_adc(){
	RCC->APB2PCENR=0;
	RCC->APB2PRSTR|=(1<<9);//reset ADC
	RCC->APB2PCENR|=(1<<9);//enable adc clock
	ADC1->CTLR1=(1<<5);//enable software status
	ADC1->CTLR2=(15<<17)|1;//software trigger
	ADC1->RSQR1=0;//only convert one channel
	ADC1->RSQR1=0;//channel 0 ADC1
	ADC1->CTLR2|=(1<<22);//start conversion
}
//timer is positive edge triggered, our clock frequency is 144 MHz
//interrupts are triggered half way up and at 0
//we are dividing by 144 so we have a sampling frequency of 1 MHz
int PWM_freq=144;
void setup1mhz_timer(){//can run 64 commands to process the audio
	RCC->APB2PRSTR|=1;//using timer 2, reset
	RCC->APB1PCENR|=1;//clock enable
	TIM2->CTLR1=(1<<2)|1;//enable edge alighnment counting up, counter overflow and underflow interrupt
	TIM2->DMAINTENR=4;//interupt for capture compare timer2 compare capture 2
	TIM2->CHCTLR1=(6<<12);//count up PWM mode
	TIM2->CCER=(1<<4);//enable output just in case
	TIM2->ATRLR=PWM_freq;
	TIM2->CH2CVR=PWM_freq/2;
	NVIC_EnableIRQ(44);//enable interrupt for TIM2


}

//the Dac data
int dac_data=0;
int left_channel=0;//max value:65535
int right_channel=0;
int d38khz_phase=0;
int d19khz_phase=0;
int stdata_shift=0;
int mult_st_data_shift=0;
int st_pilot_shift=0;
int st_pilot_mult=1;
int global_shift=0;
//---
int phase;
int sum;
int diff;
int mpx_phase;
void timer2_isr(){
  //this takes 4 cycles:
  //if(TIM2->INTFR&4==0){return;}
  DAC->R12BDHR1=dac_data&4095;
  TIM2->INTFR&=~4;//clear interrupt flag
  //begin processor
  //next two lines, 2 cycles
  sum=left_channel+right_channel;//max value: 65535*2
  diff=right_channel-left_channel;//max_value: 65535
  phase=0;//1 cycle
  if(diff<0){//6 cycles
	  phase=MAX_ST_AMPLITUDE;
	  diff=-diff;
  }
  diff=diff>>stdata_shift;//1 cycle
  if(d38khz_phase>MAX_38_PHASE){//5cycles
	  d38khz_phase=0;
  }else{
	  d38khz_phase++;
  }
  if(d19khz_phase>MAX_19_PHASE){//5 cycles
  	  d19khz_phase=0;
  }else{
  	  d19khz_phase++;
  }
  mpx_phase=((phase-m38khz[d38khz_phase])*diff)>>mult_st_data_shift;//5 cycles
  dac_data=((mpx_phase+sum)+(m19khz[d19khz_phase]*st_pilot_mult)>>st_pilot_shift)>>global_shift;//7 cycles
  //processing took:23 clock cycles
  //taken: 28 clock cycles, lets just make it 30 for good measure
  if(ADC1->STATR&2!=0){//if ADC conversion is complete 4
	  	if(ADC1->RSQR1==0){//4
	  			ADC1->RSQR1=1;//1
	  			right_channel=ADC1->RDATAR;//1
	  	}else{
	  			ADC1->RSQR1=0;//1
	  			left_channel=ADC1->RDATAR;//1
	  	}
	  	ADC1->STATR&=~2;//2
	  	ADC1->CTLR2|=(1<<22);//2
  }//ADC extract takes 14 clock cycles
	//total 50 cycles
}
int calculate_shift(int mdata,int targdata){
	int max=mdata;
	int mult=1;
	int count;
	for(count=0;(max/mult)>=targdata+1;count++){
		mult=mult*2;
	}
	return count;
}
void calculate_processing_constraints(){
	stdata_shift=calculate_shift(65535,MAX_ST_AMPLITUDE);
	int mpx_shift=(65535>>stdata_shift)*MAX_ST_AMPLITUDE;
	mult_st_data_shift=calculate_shift(mpx_shift,65535*2)-stereo_amp;
	if(mult_st_data_shift<0){
		mult_st_data_shift=0;
	}
	int total_st_mono=(mpx_shift>>mult_st_data_shift)+65535*2;
	float dp=pilot_amp/100.0;
	float fval=total_st_mono*dp;
	int bsize=fval;

	if(MAX_PILOT_AMPLITUDE>bsize){
		 st_pilot_shift=calculate_shift(MAX_PILOT_AMPLITUDE,bsize);
		 st_pilot_mult=1;
	}else if(MAX_PILOT_AMPLITUDE<bsize){
		st_pilot_mult=bsize/MAX_PILOT_AMPLITUDE;
		st_pilot_shift=0;
	}
	int total_signal_size=((MAX_PILOT_AMPLITUDE*st_pilot_mult)>>st_pilot_shift)+total_st_mono;
	//12 bit dac so 4095 max
	global_shift=calculate_shift(total_signal_size,4095);

}
void TIM2_IRQHandler(void){
	timer2_isr();
}
int main(void)
{
	//set system PLL to 144 mhz in system_ch...
	setup_dac();
	setup_adc();
	calculate_processing_constraints();
	setup1mhz_timer();
	/*while(1){
		timer2_isr();
	}*/
    /*NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);

    printf("This is printf example\r\n");

    while(1)
    {
    }*/
}

