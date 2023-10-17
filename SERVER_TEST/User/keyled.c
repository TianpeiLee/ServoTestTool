#include "keyled.h"

extern volatile uint8_t OUT_MODE;

struct Button KEY_SET;   //设置键

uint8_t get_key_value(uint8_t btn_id)
{
    switch(btn_id)
   {
       case 0:
           if(GPIOA->INDR & GPIO_Pin_1) return 1;  //PA1
           else return 0;
           break;
       default:
           return 1;
           break;
   }
}



//设置键单击
void KEY_SET_SINGLE_CLICK_Handler(void* btn)
{
    static uint8_t mode_pwm=0;
    if((OUT_MODE&0xf) == 0)
    {
        OUT_MODE &= 0x0F;
        OUT_MODE |= (mode_pwm<<4);
        mode_pwm++;
        if(mode_pwm>2) mode_pwm = 0;

        printf("pwm_mode:%d\r\n",mode_pwm);
    }
}

//设置键长按
void KEY_SET_LONG_PRESS_START_Handler(void* btn)
{
    static uint8_t mode_output=0;

    mode_output++;
    if(mode_output>2) mode_output=0;
    if(mode_output==0)
    {
        TIM_Cmd( TIM2, DISABLE );
        TIM2->PSC = 48-1;
        TIM2->ATRLR = 20000-1;
        TIM_DMACmd( TIM2, TIM_DMA_CC4, DISABLE);
        TIMER_DMA_INIT( );
        TIM_Cmd( TIM2, ENABLE );
        OUT_MODE &= 0xF0;
    }
    else if(mode_output == 1)  //DSHOT 300
    {
        TIM_Cmd( TIM2, DISABLE );
        TIM2->PSC = 8-1;
        TIM2->ATRLR = 20-1;
        TIMER_DMA_INIT( );
        TIM_DMACmd( TIM2, TIM_DMA_CC4, ENABLE);
        TIM_Cmd( TIM2, ENABLE );
        OUT_MODE &= 0xF0;
        OUT_MODE |= 0x1;
    }
    else if(mode_output == 2)   //  DSHOT 600
    {
        TIM_Cmd( TIM2, DISABLE );
        TIM2->PSC = 4-1;
        TIM2->ATRLR = 20-1;
        TIMER_DMA_INIT( );
        TIM_DMACmd( TIM2, TIM_DMA_CC4, ENABLE);
        TIM_Cmd( TIM2, ENABLE );
        OUT_MODE &= 0xF0;
        OUT_MODE |= 0x2;
    }

}




void KEY_ENVENT_INIT(void)
{
    button_init(&KEY_SET, get_key_value, 0, 0);
    button_attach(&KEY_SET, SINGLE_CLICK,     KEY_SET_SINGLE_CLICK_Handler);     //按下设置，切换参数
    button_attach(&KEY_SET, LONG_PRESS_START, KEY_SET_LONG_PRESS_START_Handler); //长按,进入设置
    button_start(&KEY_SET);
}



void KEYLED_INIT(void)
{

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOC, ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE );
    GPIOA->BSHR = GPIO_Pin_1;
    GPIOA->CFGLR &= 0xFFFFFF0F; //PA1
    GPIOA->CFGLR |= (8<<4);     //PA1   上下拉输入
    KEY_ENVENT_INIT( );



    GPIOC->CFGLR &= 0xFFF0F0FF;             //C2->MODE(PWM,DSHOT300,DSHOT600)/man,mid,auto  C4->mid  D5->auto
    GPIOC->CFGLR |= 0x00030300;             //C2,C4推挽输出
    GPIOC->BSHR = GPIO_Pin_2|GPIO_Pin_4;    //输出高

    GPIO_PinRemapConfig( GPIO_Remap_SDI_Disable, ENABLE );
    GPIOD->CFGLR &= 0xFF0FFFFF;
    GPIOD->CFGLR |= 0x00300000;
    GPIOD->BSHR  = GPIO_Pin_5;
}
