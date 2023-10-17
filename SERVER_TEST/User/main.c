/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/08
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "debug.h"
#include "multi_button.h"
#include "keyled.h"

#define DSHOT_BIT_0  7
#define DSHOT_BIT_1  15

uint16_t ADC_VALUE=0;
uint16_t DSHOT_CCR[17];  //最后为低电平
volatile uint16_t TIMER_CNT_5MS=0;
volatile uint16_t FLASHING_TIMER=0;
volatile uint32_t DELAY_TIMER=0;
volatile uint8_t OUT_MODE=0;           //低四位为output模式，高四位为PWM状态下模式（手动0x10、中间0x20、循环0x30）
volatile uint8_t TIMER_FLAG=0;
volatile uint8_t FLASHING_2Hz=0;
volatile uint8_t FLASHING_1Hz=0;






long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    if(x < in_min){
        x = in_min;
    }
    if(x > in_max){
        x = in_max;
    }
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}

//square wave output initialize
void TIMER_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2  , ENABLE );
    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2,ENABLE);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //Multiplex push-pull output
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    TIM_TimeBaseInitStructure.TIM_Period = 20000-1;       //50Hz
    TIM_TimeBaseInitStructure.TIM_Prescaler = 48-1;       //1MHz
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM2, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init( TIM2, &TIM_OCInitStructure );

    TIM_CtrlPWMOutputs(TIM2, ENABLE );
    TIM_OC4PreloadConfig( TIM2, TIM_OCPreload_Enable );
    TIM_ARRPreloadConfig( TIM2, ENABLE );
    TIM_Cmd( TIM2, ENABLE );
}


void TIMER_DMA_INIT(void)
{
    DMA_InitTypeDef DMA_InitStructure = {0};
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM2->CH4CVR);
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)(&DSHOT_CCR[0]);
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize         = 17;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel7, &DMA_InitStructure);
    DMA_ClearFlag(DMA1_FLAG_TE7|DMA1_FLAG_HT7|DMA1_FLAG_TC7|DMA1_FLAG_GL7);
    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);
    NVIC_SetPriority(DMA1_Channel7_IRQn, 0x70);
    NVIC_EnableIRQ(DMA1_Channel7_IRQn);
    DMA_Cmd(DMA1_Channel7,ENABLE);
}


void ADC_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    ADC_InitTypeDef  ADC_InitStructure = {0};

    //enable ADC and GPIOA clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOA, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div4);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);  //PA2 set as Analog input


    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_Calibration_Vol(ADC1, ADC_CALVOL_50PERCENT);
    ADC_Cmd(ADC1, ENABLE);

    //After calibration is completed, the calibration value is automatically added to the ADC sampling value
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

}



uint16_t get_adc_val(uint8_t ch)
{
    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_15Cycles);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(!(ADC1->STATR & ADC_FLAG_EOC));
    return (uint16_t)(ADC1->RDATAR);
}



void ADC2PWM(void)
{
   uint16_t tem;
   tem = get_adc_val( ADC_Channel_0 );
   ADC_VALUE = (3*ADC_VALUE + tem) >> 2;
   tem = map( ADC_VALUE, 0, 1023, 1000, 2000 );
   TIM2->CH4CVR = (uint16_t)tem;
}



void SYSTICK_INIT(void)
{
    SysTick->CTLR = 0;
    SysTick->CNT = 0;
    SysTick->SR = 0;
    SysTick->CMP = SystemCoreClock / 500 - 1;   //定时采样
    SysTick->CTLR = 0x0F;
    NVIC_SetPriority(SysTicK_IRQn, 0x70);
    NVIC_EnableIRQ(SysTicK_IRQn);
}


uint16_t DSHOT_CALC_CRC(uint16_t speed, uint8_t reqTelem)
{
    volatile uint16_t dshot_data = ( speed <<1 ) | (reqTelem? 1 : 0);
    uint16_t crc=0,crc_data = dshot_data;
    for(uint8_t i=0;i<3;i++)
    {
        crc ^= crc_data;
        crc_data >>= 4;
    }
    crc &= 0xF;
    return ((dshot_data<<4) | crc);
}


void DSHOT_MAKE_PACKEG(uint16_t data, uint8_t reqTelem)
{
    data = DSHOT_CALC_CRC(data, reqTelem);
    DSHOT_CCR[0]  = (data & 0x8000)? DSHOT_BIT_1:DSHOT_BIT_0;
    DSHOT_CCR[1]  = (data & 0x4000)? DSHOT_BIT_1:DSHOT_BIT_0;
    DSHOT_CCR[2]  = (data & 0x2000)? DSHOT_BIT_1:DSHOT_BIT_0;
    DSHOT_CCR[3]  = (data & 0x1000)? DSHOT_BIT_1:DSHOT_BIT_0;
    DSHOT_CCR[4]  = (data & 0x0800)? DSHOT_BIT_1:DSHOT_BIT_0;
    DSHOT_CCR[5]  = (data & 0x0400)? DSHOT_BIT_1:DSHOT_BIT_0;
    DSHOT_CCR[6]  = (data & 0x0200)? DSHOT_BIT_1:DSHOT_BIT_0;
    DSHOT_CCR[7]  = (data & 0x0100)? DSHOT_BIT_1:DSHOT_BIT_0;
    DSHOT_CCR[8]  = (data & 0x0080)? DSHOT_BIT_1:DSHOT_BIT_0;
    DSHOT_CCR[9]  = (data & 0x0040)? DSHOT_BIT_1:DSHOT_BIT_0;
    DSHOT_CCR[10] = (data & 0x0020)? DSHOT_BIT_1:DSHOT_BIT_0;
    DSHOT_CCR[11] = (data & 0x0010)? DSHOT_BIT_1:DSHOT_BIT_0;
    DSHOT_CCR[12] = (data & 0x0008)? DSHOT_BIT_1:DSHOT_BIT_0;
    DSHOT_CCR[13] = (data & 0x0004)? DSHOT_BIT_1:DSHOT_BIT_0;
    DSHOT_CCR[14] = (data & 0x0002)? DSHOT_BIT_1:DSHOT_BIT_0;
    DSHOT_CCR[15] = (data & 0x0001)? DSHOT_BIT_1:DSHOT_BIT_0;
    DSHOT_CCR[16] = 0; //idle is 0
}

void ADC2DSHOT(void)
{
    uint16_t tem;
    tem = get_adc_val( ADC_Channel_0 );
    ADC_VALUE = (3*ADC_VALUE + tem) >> 2;
    tem = map( ADC_VALUE, 0, 1023, 47, 2047 );
    DSHOT_MAKE_PACKEG(tem, 0);
}

void DMA_RESTART(void)
{
    DMA1_Channel7->CFGR &= ~(0x1);
    DMA1_Channel7->CNTR = 17;
    DMA1_Channel7->MADDR = (uint32_t)&DSHOT_CCR[0];
    DMA1_Channel7->CFGR |= 0x1;
    TIM_Cmd( TIM2, ENABLE );
}
/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    USART_Printf_Init(921600);
    ADC_INIT( );
    TIMER_INIT( );
    SYSTICK_INIT( );
    KEYLED_INIT( );
    while(1)
    {

        if(TIMER_FLAG)
        {
            TIMER_FLAG = 0;
            switch(OUT_MODE & 0x0f)
            {
                case 0x0:
                     switch(OUT_MODE&0xF0)
                    {
                        case 0x00:
                            GPIOC->BCR  = GPIO_Pin_2;  //mode
                            GPIOD->BSHR = GPIO_Pin_5;  //auto
                            GPIOC->BSHR = GPIO_Pin_4;  //mid
                            ADC2PWM( );  //手动采集电位器输出PWM占空比
                            break;
                        case 0x10:       //中间位置
                            GPIOD->BSHR = GPIO_Pin_5;
                            GPIOC->BSHR = GPIO_Pin_2;
                            GPIOC->BCR  = GPIO_Pin_4;
                            TIM2->CH4CVR = 1500;
                            break;
                        case 0x20:       //自动扫描1000----2000
                            GPIOD->BCR  = GPIO_Pin_5;
                            GPIOC->BSHR = GPIO_Pin_2|GPIO_Pin_4;
                            for(uint16_t i=1000;i<=2000;i++)
                            {
                                TIM2->CH4CVR = i;
                                DELAY_TIMER = 0;
                                while(DELAY_TIMER<1);
                            }
                            break;
                        default:
                            break;
                    }
                    break;
                case 0x1:
                case 0x2:
                    if((OUT_MODE & 0x0f) == 0x1)
                    {
                        GPIO_WriteBit(GPIOC, GPIO_Pin_2,FLASHING_1Hz);
                        GPIOC->BSHR = GPIO_Pin_4;
                        GPIOD->BSHR = GPIO_Pin_5;
                    }
                    else if((OUT_MODE & 0x0f) == 0x2)
                    {
                        GPIO_WriteBit(GPIOC, GPIO_Pin_2,FLASHING_2Hz);
                        GPIOC->BSHR = GPIO_Pin_4;
                        GPIOD->BSHR = GPIO_Pin_5;
                    }
                    ADC2DSHOT( );
                    DMA_RESTART( );
                    break;
                default:
                    break;
            }
        }
    }
}




void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SysTick_Handler(void)
{
    TIMER_FLAG = 1;
    TIMER_CNT_5MS++;
    FLASHING_TIMER++;
    DELAY_TIMER++;
    if((FLASHING_TIMER % 125) == 0)
    {
        FLASHING_2Hz ^= 1;
    }
    if(FLASHING_TIMER > 250)
    {
        FLASHING_1Hz ^= 1;
        FLASHING_TIMER=0;
    }

    if(TIMER_CNT_5MS >= 2)
    {
        button_ticks( );
        TIMER_CNT_5MS=0;
    }
    SysTick->SR = 0;
}


void DMA1_Channel7_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel7_IRQHandler(void)
{
    if((DMA1->INTFR & DMA1_IT_TC7))
    {
        TIM_Cmd( TIM2, DISABLE );
        DMA1_Channel7->CFGR &= ~(0x1);
        DMA1->INTFCR = DMA1_IT_TC7;
    }
}
