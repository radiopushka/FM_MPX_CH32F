/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
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
#define MAX_38_PHASE 21
#define MAX_19_PHASE 43

/* Global Variable */
//highest speed set will only work at overclocking past 144 mhz
//size:51
//int m19khz[]={2047,2293,2536,2772,2998,3209,3404,3579,3731,3859,3960,4034,4079,4094,4079,4034,3960,3859,3731,3579,3404,3209,2998,2772,2536,2293,2047,1801,1558,1322,1096,885,690,515,363,235,134,60,15,0,15,60,134,235,363,515,690,885,1096,1322,1558,1801};
//size:25
//int m38khz[]={2047,2536,2998,3404,3731,3960,4079,4079,3960,3731,3404,2998,2536,2047,1558,1096,690,363,134,15,15,134,363,690,1096,1558};
//size:43
int m19khz[]={2047,2338,2623,2897,3153,3387,3594,3769,3909,4011,4073,4094,4073,4011,3909,3769,3594,3387,3153,2897,2623,2338,2047,1756,1471,1197,941,707,500,325,185,83,21,0,21,83,185,325,500,707,941,1197,1471,1756};
//size:21
int m38khz[]={2058,2637,3170,3613,3930,4095,4095,3930,3613,3170,2637,2058,1479,946,503,186,21,21,186,503,946,1479};
//size:39
//int m19khz[]={2047,2367,2679,2976,3250,3494,3703,3870,3993,4068,4095,4068,3993,3870,3703,3494,3250,2976,2679,2367,2047,1727,1415,1118,844,600,391,224,101,26,0,26,101,224,391,600,844,1118,1415,1727};
//size: 19
//int m38khz[]={2047,2679,3250,3703,3993,4095,3993,3703,3250,2679,2047,1415,844,391,101,0,101,391,844,1415};

int stereo_amp=1;
int pilot_amp=18;//this percent of signal width
int extra_st_att=0;

int adc_cal=0;

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 * right -PA0
 * left - PA1
 * PWM out- PA1
 * @return  none
*/
void setup_pins(){
//no need ADC and DAC need analog input mode which is 00 or default
    RCC->APB2PCENR=(4<<2)|1;//gpioa gpioc clock enable
    //GPIOA->CFGLR=(2<<6)|(2<<2);//make the ADC inputs pulled to half(no effect)
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
    ADC1->SAMPTR2=2|(2<<3);// sample time 13.5 cycles
    RCC->APB2PCENR|=(3<<9);//enable adc clock
    ADC1->CTLR1=(1<<5)|(1<<26)|(1<<27);//enable software status, x4 gain amplifier, this seemed to work best for my laptop's sound card, do not use cheap sound cards though, their stereo output is low quality
    ADC1->CTLR2=(15<<17)|1;//software trigger
    int sleep;
    for(sleep=0;sleep<200;sleep++){}//wait for 4 ADC clock cycles
    ADC1->CTLR2|=(1<<3);
    while(((ADC1->CTLR2)&(1<<3))!=0);//init calibrate
    ADC1->CTLR2|=4;
    while(((ADC1->CTLR2)&4)!=0);//calibrate
    ADC1->RSQR3=0;//only convert one channel, channel 0
    ADC1->CTLR2|=(1<<22);//start conversion
    //use 2 ADCs for better audio

}
//timer is positive edge triggered, our clock frequency is 144 MHz
//interrupts are triggered half way up and at 0
//we are dividing by 72 so we have a sampling frequency of 1 MHz
int PWM_freq=172;//188 for slow set, 172 or medium
void setup1mhz_timer(){//can run 64 commands to process the audio
    RCC->APB2PRSTR|=1;//using timer 2, reset
    RCC->APB1PCENR|=1;//clock enable
    TIM2->CTLR1=(1<<2)|1;//enable edge alighnment counting up, counter overflow and underflow interrupt
    TIM2->DMAINTENR=4;//interupt for capture compare timer2 compare capture 2
    TIM2->CHCTLR1=(6<<12)|(1<<15);//count up PWM mode
    TIM2->CCER=(1<<4);//enable output just in case
    TIM2->ATRLR=PWM_freq;
    TIM2->CH2CVR=(PWM_freq/2);
    NVIC_EnableIRQ(44);//enable interrupt for TIM2


}

//the Dac data
int dac_data=0;
int sleft_channel=0;//max value:65535
int sright_channel=0;
int sd38khz_phase=0;
int sd19khz_phase=0;
int stdata_shift=0;
int mult_st_data_shift=0;
int st_pilot_shift=0;
int st_pilot_mult=1;
int global_shift=0;
//---

int spullphase38=0;
int sechannel=0;
int spullphase19=0;
__attribute__((interrupt()))
void TIM2_IRQHandler(void){
  //trying to find ways to optimize speed, giving up on counting clock cycles
  register int gshift=global_shift;//1
  register int right_channel=sright_channel;//1
  register int left_channel=sleft_channel;//1
  register int edac_data=dac_data;//1
  register int d38khz_phase=sd38khz_phase;//1
  register int pullphase38=spullphase38;//1
  register int pullphase19=spullphase19;//1
  register int d19khz_phase=sd19khz_phase;//1
  TIM2->INTFR=0;//reset interrupt flag 1 cycle
  register int diff=right_channel-left_channel;//max_value: 4095 1cycle
  edac_data=edac_data>>gshift;//1
  //begin processor
  d38khz_phase++;//1 cycle
  d19khz_phase++;//1 cycle
  register int dtemp=diff<0;//1 cycle
  register int sum=left_channel+right_channel;//max value: 4095*2 1 cycle
  register int phase=0;//1 cycle
  register int j38=d38khz_phase>MAX_38_PHASE;//1 cycle
  register int pmult_st_data_shift=mult_st_data_shift;//1
  if(dtemp){//3 cycles
      phase=MAX_ST_AMPLITUDE;
  }
  register int j19=d19khz_phase>MAX_19_PHASE;//1
  register int pst_pilot_shift=st_pilot_shift;//1
  if(j38){//3cycles
      d38khz_phase=0;
  }
  register int ddovf=edac_data>4095;//1
  if(j19){//3 cycles
      d19khz_phase=0;
  }
  register int pmultext=st_pilot_mult;//1
  register int mpx_phase=pullphase38-phase;//1 cycles
  pullphase38=m38khz[d38khz_phase];//1
  if(ddovf){//3 cycles
      edac_data=4095;
  }
  register int echannel=sechannel;//1
  mpx_phase=mpx_phase*diff;//1
  register int pilotphase=pullphase19*pmultext;//1
  pullphase19=m19khz[d19khz_phase];//1
  register int adccv=(ADC1->STATR);//1
  register int sval=(1<<22);//1
  register int pullctl=ADC1->CTLR2;//1
  DAC->R12BDHR1=edac_data;//1
  register int cmpa=adccv&2;//1
  pullctl=pullctl|sval;//1
  mpx_phase=mpx_phase>>pmult_st_data_shift;//1
  register int tshift=pilotphase>>pst_pilot_shift;//1
  if(cmpa!=0){//if ADC conversion is complete 3
      if(echannel==0){//2
          right_channel=ADC1->RDATAR;//1
          ADC1->RSQR3=1;//1
          sechannel=1;//1
          ADC1->STATR=0;//1
          right_channel=right_channel&4095;//1
          ADC1->CTLR2=pullctl;//1
      }else{//2

          left_channel=ADC1->RDATAR;//1
          ADC1->RSQR3=0;//1
          sechannel=0;//1
          ADC1->STATR=0;//1
          left_channel=left_channel&4095;//1
          ADC1->CTLR2=pullctl;//1
      }
  }
  register int tphase=mpx_phase+sum;//1
  spullphase19=pullphase19;//1
  spullphase38=pullphase38;//1
  sright_channel=right_channel;//1
  edac_data=tphase+tshift;//1 cycles
  sd19khz_phase=d19khz_phase;//1
  sd38khz_phase=d38khz_phase;//1
  sleft_channel=left_channel;//1
  dac_data=edac_data;//1 cycles
  // takes over 69 clock cycles

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
    mult_st_data_shift=mult_st_data_shift+extra_st_att;

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
