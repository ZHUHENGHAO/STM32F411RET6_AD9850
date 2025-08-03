#include "delay.h"

// 微秒级延时函数（直接使用TIM2计数器）
void delay_us(uint32_t us) 
{
    uint32_t start = TIM2->CNT;
    uint32_t ticks = us;  // 直接使用 us，因 CNT 每 1us 加1

    while (1) {
        uint32_t now = TIM2->CNT;
        
        // 处理计数器溢出
        if (now >= start) {
            if ((now - start) >= ticks) break;
        } else {
            if ((0xFFFFFFFF - start + now) >= ticks) break;
        }

        // 长延时时释放 CPU（FreeRTOS 环境）
        #ifdef USE_FREERTOS
        if (ticks > 1000) {
            vTaskDelay(1);  // 释放 CPU 约 1ms
            ticks -= 1000;
            start = TIM2->CNT;  // 重置起始点
        }
        #endif
    }
} 


