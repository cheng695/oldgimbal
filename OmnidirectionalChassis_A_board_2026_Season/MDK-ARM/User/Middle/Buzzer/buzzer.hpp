#ifndef _BUZEZER_HPP_
#define _BUZEZER_HPP_ 

#include "tim.h"
#include <cmsis_os.h>
#include "FreeRTOS.h"
#include "task.h"

void delay_ms(uint32_t ms);
void B_star();
void B_stop();
void B_IMU();

#endif
