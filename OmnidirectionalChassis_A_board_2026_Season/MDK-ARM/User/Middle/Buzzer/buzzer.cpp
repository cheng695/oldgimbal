#include "buzzer.hpp"
void B_star()
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 125);
    delay_ms(150);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
    delay_ms(150);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 180);
    delay_ms(150);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
    delay_ms(150);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 125);
    delay_ms(400);    
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);

    delay_ms(200);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 125);
    delay_ms(50);    
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
    delay_ms(50); 
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 125);
    delay_ms(50);    
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);

}

void B_stop()
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 180);
    delay_ms(400);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
    delay_ms(150);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 180);
    delay_ms(400);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
    delay_ms(150);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 125);
    delay_ms(150);    
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);

    delay_ms(200);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 125);
    delay_ms(50);    
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
    delay_ms(50); 
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 125);
    delay_ms(50);    
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
}

void B_IMU()
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 180);
    delay_ms(100);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
    delay_ms(100);
}

void delay_ms(uint32_t ms) 
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms) 
    {
        // µÈ´ý
    }
}
