/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"



/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
#include "AD985x.h"
#include "adc.h"
#include "usart.h"
#include "tim.h"
#include "dma.h"
#include "pwm.h"
#include "delay.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint16_t adc_buf[250] = {0};     // ����
/* USER CODE END Variables */

osThreadId LEDTaskHandle;
osThreadId ButtonTaskHandle;
osThreadId AD9850initTaskHandle;
osThreadId AD9850CTRTaskHandle;
osThreadId ADCTaskHandle;
osThreadId ADCProcessTaskHandle;
osThreadId PWMTaskHandle;

osMessageQId Delayms_QueueHandle;
uint8_t Dealyms_QueueBuffer[ 1 * sizeof( uint16_t ) ];
osStaticMessageQDef_t Dealyms_QueueControlBlock;
osMessageQId Freq_QueueHandle;
uint8_t Freq_QueueBuffer[ 1 * sizeof( double ) ];
osStaticMessageQDef_t Freq_QueueControlBlock;

osSemaphoreId ADCDMA_SemHandle;  // ����
osStaticSemaphoreDef_t ADCDMA_SemControlBlock;  // ����
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LED_Task(void const * argument);
void Button_Task(void const * argument);
void AD9850init_Task(void const * argument);
void AD9850CTR_Task(void const * argument);
void ADC_Init_Task(void const *argument);  // ������ADC��ʼ������
void ADC_Process_Task(void const *argument);  // ������ADC���ݴ�������
void PWM_Task(void const * argument);


void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	// �����ź���������������DMA���֪ͨ��
  osSemaphoreStaticDef(ADCDMA_Sem, &ADCDMA_SemControlBlock);
  ADCDMA_SemHandle = osSemaphoreCreate(osSemaphore(ADCDMA_Sem), 1);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of Delayms_Queue */
  osMessageQStaticDef(Delayms_Queue, 1, uint16_t, Dealyms_QueueBuffer, &Dealyms_QueueControlBlock);
  Delayms_QueueHandle = osMessageCreate(osMessageQ(Delayms_Queue), NULL);

  /* definition and creation of Freq_Queue */
  osMessageQStaticDef(Freq_Queue, 1, double, Freq_QueueBuffer, &Freq_QueueControlBlock);
  Freq_QueueHandle = osMessageCreate(osMessageQ(Freq_Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LEDTask */
  osThreadDef(LEDTask, LED_Task, osPriorityNormal, 0, 128);
  LEDTaskHandle = osThreadCreate(osThread(LEDTask), NULL);

  /* definition and creation of ButtonTask */
  osThreadDef(ButtonTask, Button_Task, osPriorityAboveNormal, 0, 128);
  ButtonTaskHandle = osThreadCreate(osThread(ButtonTask), NULL);

  /* definition and creation of AD9850initTask */
  osThreadDef(AD9850initTask, AD9850init_Task, osPriorityBelowNormal, 0, 128);
  AD9850initTaskHandle = osThreadCreate(osThread(AD9850initTask), NULL);

  /* definition and creation of AD9850CTRTask */
  osThreadDef(AD9850CTRTask, AD9850CTR_Task, osPriorityNormal, 0, 128);
  AD9850CTRTaskHandle = osThreadCreate(osThread(AD9850CTRTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	  // ������ADC������� ��ʼ�ɼ����� ���һ����ɾ��
  osThreadDef(ADCTask, ADC_Init_Task, osPriorityHigh, 0, 128);
  ADCTaskHandle = osThreadCreate(osThread(ADCTask), NULL);
	  //�����ɼ���Ϣ���ٴ���������
  osThreadDef(ADCProcessTask, ADC_Process_Task, osPriorityNormal, 0, 256);
  ADCProcessTaskHandle = osThreadCreate(osThread(ADCProcessTask), NULL);
	
	 osThreadDef(PWMTask, PWM_Task, osPriorityAboveNormal, 0, 128);
   PWMTaskHandle = osThreadCreate(osThread(PWMTask), NULL);
   /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_LED_Task */
/**
  * @brief  Function implementing the LEDTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LED_Task */
void LED_Task(void const * argument)
{
  /* USER CODE BEGIN LED_Task */
		uint32_t current_delay = 200;  // Ĭ����ʱ200ms

  /* Infinite loop */
  for(;;)
  {
		/* ���������Ƿ�(�ǿ�)���µ���ʱֵ �Ƿǿ���Ѷ����е����µ�ֵ��ȡ��current_delay */
		if(xQueueReceive(Delayms_QueueHandle,&current_delay,0) == pdTRUE)
		{
		}
	
		 /* ��תLED״̬ */
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN); 
		osDelay(current_delay);
  }
  /* USER CODE END LED_Task */
}

/* USER CODE BEGIN Header_Button_Task */
/**
* @brief Function implementing the ButtonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Button_Task */
void Button_Task(void const * argument)
{
  /* USER CODE BEGIN Button_Task */
	uint8_t last_Button_state = GPIO_PIN_SET;
	uint8_t Current_Button_state;
	uint32_t start_times;
	uint32_t stop_times;
	uint8_t current_index = 0;
	uint16_t delay_option[4] = {200,500,1000,2000};
	double DDS_Freq = 10000;
	uint8_t flag = 0;
  /* Infinite loop */
  for(;;)
  {
		Current_Button_state =HAL_GPIO_ReadPin(BUTTON_GPIO_PORT,BUTTON_PIN);
		
		if(last_Button_state == GPIO_PIN_SET && Current_Button_state == GPIO_PIN_RESET)//�½��� ��������
		{
			start_times = xTaskGetTickCount();//ticks 1ms��1
		}
		if(last_Button_state == GPIO_PIN_RESET && Current_Button_state == GPIO_PIN_SET)//������ �����ͷ�
		{
			osDelay(20);
			stop_times = xTaskGetTickCount()-start_times;

			if(stop_times <= 1000)//�̰�1s��
			{
//				flag++;
				current_index = (current_index+1)%(sizeof(delay_option)/sizeof(delay_option[0]));
				xQueueSend(Delayms_QueueHandle,&delay_option[current_index],pdMS_TO_TICKS(10));
//				if(flag%2 == 1) TIM_SetTIM1Compare1(50);
//				else TIM_SetTIM1Compare1(0);
			}
			if(stop_times >= 2000 && stop_times <= 3000)//�̳���2s-3s��
			{
				DDS_Freq = 1000000;
				xQueueSend(Freq_QueueHandle,&DDS_Freq,pdMS_TO_TICKS(10));
			}
			if(stop_times >= 3000 && stop_times <= 5000)//�̳���3s-5s��
			{
				DDS_Freq = 10000;
				xQueueSend(Freq_QueueHandle,&DDS_Freq,pdMS_TO_TICKS(10));
			}
		}
		
//		TIM_SetTIM1Compare1(0);
		last_Button_state = Current_Button_state;//״̬����
		osDelay(10);//��Ϊ������������ȼ���ߣ�Ϊ�����������ȼ������ܹ������ȣ�������Ҫ��ÿ�ε�ѭ�����ó�CPU����ִ����������


		
  }
  /* USER CODE END Button_Task */
}

/* USER CODE BEGIN Header_AD9850init_Task */
/**
* @brief Function implementing the AD9850initTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AD9850init_Task */
void AD9850init_Task(void const * argument)
{
  /* USER CODE BEGIN AD9850init_Task */
	double default_freq = 10000.0;
  /* Infinite loop */
	AD985x_Init(AD9850,SERIAL);
	AD985x_SetFre_Phase(default_freq,0);//�������Ƶ��10KHz,��λ0��
	osDelay(1000);
	vTaskDelete(NULL);//��ʼ���ȶ�һ��ʱ����Ժ�ɾ����ʼ������
  /* USER CODE END AD9850init_Task */
	
}

/* USER CODE BEGIN Header_AD9850CTR_Task */
/**
* @brief Function implementing the AD9850CTRTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AD9850CTR_Task */
void AD9850CTR_Task(void const * argument)
{
  /* USER CODE BEGIN AD9850CTR_Task */
	double freq;
  /* Infinite loop */
  for(;;)
  {
    if(xQueueReceive(Freq_QueueHandle,&freq,portMAX_DELAY))
		{
			AD985x_SetFre_Phase(freq,0);//�������Ƶ��freqHz,��λ0��
		}
  }
  /* USER CODE END AD9850CTR_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void ADC_Init_Task(void const * argument)
{
	  // ����ADC+DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 250);  // ADC1�ɼ�250��
  HAL_TIM_Base_Start(&htim3);  // ����TIM3������ADC��

  // ������ɺ�ɾ������
  vTaskDelete(NULL);
	
}

void ADC_Process_Task(void const * argument)
{
  uint8_t tx_buf[500];  // ����USART���͵Ļ�������250���2�ֽڣ�

  for(;;) {
			
    // �ȴ�DMA����ź��� �ȴ��ź���Ϊ1�ż�������ִ�У������ź���-1
   if( osSemaphoreWait(ADCDMA_SemHandle, portMAX_DELAY) == osOK)
	 {
//    printf("ADC Data: ");  // ǰ׺��ʾ
    // ���ݴ���
    for(uint16_t i = 0; i < 250; i++) {
//      tx_buf[i*2] = (adc_buf[i] >> 8) & 0xFF;  // ��8λ
//      tx_buf[i*2+1] = adc_buf[i] & 0xFF;       // ��8λ
			printf("%d\r\n", adc_buf[i]); 
    }

//    // ͨ��USART1�������� 500���ֽ��� 100��100ms��Ϊ��ʱʱ��
//    HAL_UART_Transmit(&huart1, tx_buf, 500, 100);

    // ��������DMA��TIM3����ʼ��һ�ֲɼ�
    HAL_TIM_Base_Stop(&htim3);
    HAL_ADC_Stop_DMA(&hadc1);
    
    // �����־
    __HAL_DMA_CLEAR_FLAG(&hdma_adc1, DMA_FLAG_TCIF0_4);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
    
    // ��������
		HAL_ADC_Start(&hadc1); 
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 250);
    HAL_TIM_Base_Start(&htim3);
		osDelay(10);//�ó�CPU����ִ����������
  }
  }
}

// �жϷ������� ��stm32f4xx_it.c����
/* ������DMA����жϴ��� */
void DMA2_Stream0_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_adc1);
}

/* ������ADCת����ɻص����� */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if(hadc->Instance == ADC1) {
    // ֹͣTIM3�������������ADC��
    HAL_TIM_Base_Stop(&htim3);
    
    // �ͷ��ź�����֪ͨ�������� �ź�����1 ��Ϊ����״̬
    osSemaphoreRelease(ADCDMA_SemHandle);
  }
}


void PWM_Task(void const * argument)
{
  /* USER CODE BEGIN LED_Task */

  /* Infinite loop */
  for(;;)
  {
     Start_Pwm_Burst(50);
//		TIM_SetTIM1Compare1(25);		
//		Start_Pwm();
//		delay_us(50);
//		
////		TIM_SetTIM1Compare1(0);
//		Stop_Pwm();
		 osDelay(100);
  }
  /* USER CODE END LED_Task */
}
/* USER CODE END Application */
