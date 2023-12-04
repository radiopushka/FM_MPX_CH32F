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


/* Global define */
#define MAX_ST_AMPLITUDE 4095 
#define MAX_PILOT_AMPLITUDE 4095
#define MAX_38_PHASE 19
#define MAX_19_PHASE 41

/* Global Variable */
//size:43
//int m19khz[]={,2047,2338,2623,2897,3153,3387,3594,3769,3909,4011,4073,4094,4073,4011,3909,3769,3594,3387,3153,2897,2623,2338,2047,1756,1471,1197,941,707,500,325,185,83,21,0,21,83,185,325,500,707,941,1197,1471,1756};
//size:21
//int m38khz[]={2058,2637,3170,3613,3930,4095,4095,3930,3613,3170,2637,2058,1479,946,503,186,21,21,186,503,946,1479};
//size:41
int m19khz[]={2050,2355,2654,2939,3204,3444,3652,3825,3958,4048,4094,4094,4048,3958,3825,3652,3444,3204,2939,2654,2355,2050,1745,1446,1161,896,656,448,275,142,52,6,6,52,142,275,448,656,896,1161,1446,1745};
//size: 19
int m38khz[]={2047,2679,3250,3703,3993,4094,3993,3703,3250,2679,2047,1415,844,391,101,0,101,391,844,1415};

int stereo_amp=0;
int pilot_amp=6;//this percent of signal width

int adc_cal=0;

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 * right -PA0
 * left - PA1
 * @return  none
*/
void setup_pins(){
//no need ADC and DAC need analog input mode which is 00 or default
    RCC->APB2PCENR=(1<<2)|1;//gpioa clock enable
    GPIOA->CFGLR=(0<<16)|(2<<6)|(3<<4);//50 mhz output speed
}
void setup_dac(){
    //GPIO_InitTypeDef ginit={0};

    RCC->APB1PCENR|=(1<<29);//DAC clock enable
    DAC->CTLR=1;//software trigger,channel 1 enabled
    //DAC->R12BDHR1; is the input register and expects a 12 bit value
    DAC->R12BDHR1=4000;

    //DAC->SWTR=1;
}
//channel 0 ADC1 - right channel, channel 1 ADC1 - left channel
void setup_adc(){
    RCC->CFGR0|=(3<<14);//prescale adc clock to 18 mhz
    ADC1->SAMPTR2=2|(2<<3);// sample time 7.5 cycles
    RCC->APB2PCENR|=(3<<9);//enable adc clock
    ADC1->CTLR1=(1<<5);//enable software status
    ADC1->CTLR2=(15<<17)|1;//software trigger
    int sleep;
    for(sleep=0;sleep<200;sleep++){}//wait for 4 ADC clock cycles
    ADC1->CTLR2|=(1<<3);
    while(((ADC1->CTLR2)&(1<<3))!=0);//init calibrate
    ADC1->CTLR2|=4;
    while(((ADC1->CTLR2)&4)!=0);//calibrate
    adc_cal=ADC1->RDATAR;
    ADC1->RSQR3=0;//only convert one channel, channel 0
    ADC1->CTLR2|=(1<<22);//start conversion
}
//timer is positive edge triggered, our clock frequency is 144 MHz
//interrupts are triggered half way up and at 0
//we are dividing by 144 so we have a sampling frequency of 1 MHz
int PWM_freq=144;// the interrupt is actually called at a frequency of 2 mhz
void setup1mhz_timer(){//can run 64 commands to process the audio
    RCC->APB2PRSTR|=1;//using timer 2, reset
    RCC->APB1PCENR|=1;//clock enable
    TIM2->CTLR1=(1<<2)|1;//enable edge alighnment counting up, counter overflow and underflow interrupt
    TIM2->DMAINTENR=4;//interupt for capture compare timer2 compare capture 2
    TIM2->CHCTLR1=(6<<12)|(1<<15);//count up PWM mode
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
__attribute__((interrupt()))
void TIM2_IRQHandler(void){
  //this takes 5 cycles:
  //if(TIM2->INTFR&4==0){return;}
  TIM2->INTFR=0;//reset interrupt flag
  if(dac_data>4095){
      dac_data=4095;
  }
  DAC->R12BDHR1=dac_data;//1
  //begin processor
  //next two lines, 4 cycles
  sum=left_channel+right_channel;//max value: 4095*2
  diff=right_channel-left_channel;//max_value: 4095
  phase=0;//1 cycle
  if(diff<0){//6 cycles
      phase=MAX_ST_AMPLITUDE;
  }
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
  mpx_phase=((m38khz[d38khz_phase]-phase)*diff)>>mult_st_data_shift;//5 cycles
  dac_data=((mpx_phase+sum)+((m19khz[d19khz_phase]*st_pilot_mult)>>st_pilot_shift))>>global_shift;//7 cycles
  //processing took:39 clock cycles
  if(((ADC1->STATR)&2)!=0){//if ADC conversion is complete 5
      if(ADC1->RSQR3==0){//4
          ADC1->RSQR3=1;//1
          right_channel=((ADC1->RDATAR)&4095);//2
      }else{
          ADC1->RSQR3=0;//1
          left_channel=((ADC1->RDATAR)&4095);//2
      }
      ADC1->STATR&=~2;//2
      ADC1->CTLR2|=(1<<22);//2
  }//ADC extract takes 17 clock cycles
    //total 56 cycles 
    //take in the 2 jumps we get 62/72
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
    stdata_shift=0;
    int mpx_shift=4095*MAX_ST_AMPLITUDE;
    mult_st_data_shift=12-stereo_amp;//you should keep max st around 4095
    if(mult_st_data_shift<0){
        mult_st_data_shift=0;
    }
    int total_st_mono=(mpx_shift>>mult_st_data_shift)+4095;
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
    global_shift=calculate_shift(total_signal_size,4095)-1;

}
int main(void)
{
    SystemCoreClockUpdate();
    //set system PLL to 144 mhz in system_ch...
    Delay_Init();
    setup_pins();
    setup_dac();
    setup_adc();
    calculate_processing_constraints();
    setup1mhz_timer();
    //USART_Printf_Init(115200);
    while(1){
      //  printf("%d\n",dac_data);
       // DAC->R12BDHR1=3000;
    }
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
