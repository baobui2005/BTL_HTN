#include "driver_gate_sensor.h"

static volatile uint16_t ic_val1_in = 0, ic_val2_in = 0, diff_in = 0;
static volatile uint8_t  is_first_in = 0;

static volatile uint16_t ic_val1_out = 0, ic_val2_out = 0, diff_out = 0;
static volatile uint8_t  is_first_out = 0;

/* Tao tre micro-giay bang thanh ghi dem TIM2 (1 tick = 1us) */
static void delay_us(uint16_t us) {
    uint16_t start = TIM2->CNT;
    while ((uint16_t)(TIM2->CNT - start) < us);
}

void GateSensor_HardwareInit(void) {
    /* 1. Cau hinh TIM2 (72MHz / 72 = 1MHz -> 1 tick = 1us) cho HC-SR04 */
    TIM2->PSC = 72 - 1;
    TIM2->ARR = 0xFFFF;
    
    // Kenh 2 (PA1) va Kenh 4 (PA3) o che do Input Capture (CC2S = 01, CC4S = 01)
    TIM2->CCMR1 = TIM_CCMR1_CC2S_0;
    TIM2->CCMR2 = TIM_CCMR2_CC4S_0;
    
    // Bat Capture Kenh 2 & 4, bat suon len (Rising Edge) mac dinh
    TIM2->CCER = TIM_CCER_CC2E | TIM_CCER_CC4E;
    
    // Bat ngat Capture Kenh 2 & 4
    TIM2->DIER = TIM_DIER_CC2IE | TIM_DIER_CC4IE;
    NVIC_SetPriority(TIM2_IRQn, 6);
    NVIC_EnableIRQ(TIM2_IRQn);
    
    // Khoi chay TIM2
    TIM2->CR1 = TIM_CR1_CEN;

    /* 2. Cau hinh TIM1 (PWM 50Hz -> Chu ky 20000us) cho 2 Servo SG90 */
    TIM1->PSC = 72 - 1;       // 1 tick = 1us
    TIM1->ARR = 20000 - 1;    // 20ms (50Hz)
    
    // Kenh 1 (PA8) va Kenh 2 (PA9): PWM Mode 1 (OCxM = 110), bat Preload
    TIM1->CCMR1 = (6 << 4) | TIM_CCMR1_OC1PE | (6 << 12) | TIM_CCMR1_OC2PE;
    TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM1->BDTR  = TIM_BDTR_MOE; // Bat buoc phai bat MOE doi voi TIM1
    TIM1->CR1   = TIM_CR1_ARPE | TIM_CR1_CEN;
}

float HCSR04_Read_Entry(void) {
    GPIOA->BSRR = GPIO_BSRR_BS0; // PA0 = 1
    delay_us(10);
    GPIOA->BSRR = GPIO_BSRR_BR0; // PA0 = 0
    vTaskDelay(pdMS_TO_TICKS(15));
    return (float)diff_in * 0.034f / 2.0f;
}

float HCSR04_Read_Exit(void) {
    GPIOA->BSRR = GPIO_BSRR_BS2; // PA2 = 1
    delay_us(10);
    GPIOA->BSRR = GPIO_BSRR_BR2; // PA2 = 0
    vTaskDelay(pdMS_TO_TICKS(15));
    return (float)diff_out * 0.034f / 2.0f;
}

/* Trinh phuc vu ngat TIM2 do xung ECHO */
void TIM2_IRQHandler(void) {
    // Cong vao: PA1 (TIM2_CH2)
    if (TIM2->SR & TIM_SR_CC2IF) {
        if (is_first_in == 0) {
            ic_val1_in = TIM2->CCR2;
            is_first_in = 1;
            TIM2->CCER |= TIM_CCER_CC2P; // Chuyen sang bat suon xuong
        } else {
            ic_val2_in = TIM2->CCR2;
            diff_in = (uint16_t)(ic_val2_in - ic_val1_in);
            is_first_in = 0;
            TIM2->CCER &= ~TIM_CCER_CC2P; // Tra ve bat suon len
        }
        TIM2->SR &= ~TIM_SR_CC2IF;
    }

    // Cong ra: PA3 (TIM2_CH4)
    if (TIM2->SR & TIM_SR_CC4IF) {
        if (is_first_out == 0) {
            ic_val1_out = TIM2->CCR4;
            is_first_out = 1;
            TIM2->CCER |= TIM_CCER_CC4P;
        } else {
            ic_val2_out = TIM2->CCR4;
            diff_out = (uint16_t)(ic_val2_out - ic_val1_out);
            is_first_out = 0;
            TIM2->CCER &= ~TIM_CCER_CC4P;
        }
        TIM2->SR &= ~TIM_SR_CC4IF;
    }
}

void Servo_SetAngle_Entry(uint8_t angle) {
    TIM1->CCR1 = 500 + ((uint32_t)angle * 2000) / 180;
}

void Servo_SetAngle_Exit(uint8_t angle) {
    TIM1->CCR2 = 500 + ((uint32_t)angle * 2000) / 180;
}
