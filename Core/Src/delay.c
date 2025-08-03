#include "delay.h"

// ΢�뼶��ʱ������ֱ��ʹ��TIM2��������
void delay_us(uint32_t us) 
{
    uint32_t start = TIM2->CNT;
    uint32_t ticks = us;  // ֱ��ʹ�� us���� CNT ÿ 1us ��1

    while (1) {
        uint32_t now = TIM2->CNT;
        
        // ������������
        if (now >= start) {
            if ((now - start) >= ticks) break;
        } else {
            if ((0xFFFFFFFF - start + now) >= ticks) break;
        }

        // ����ʱʱ�ͷ� CPU��FreeRTOS ������
        #ifdef USE_FREERTOS
        if (ticks > 1000) {
            vTaskDelay(1);  // �ͷ� CPU Լ 1ms
            ticks -= 1000;
            start = TIM2->CNT;  // ������ʼ��
        }
        #endif
    }
} 


