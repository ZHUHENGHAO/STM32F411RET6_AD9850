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
uint16_t adc_buf[250] = {0};     // 新增
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

osSemaphoreId ADCDMA_SemHandle;  // 新增
osStaticSemaphoreDef_t ADCDMA_SemControlBlock;  // 新增
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LED_Task(void const * argument);
void Button_Task(void const * argument);
void AD9850init_Task(void const * argument);
void AD9850CTR_Task(void const * argument);
void ADC_Init_Task(void const *argument);  // 新增：ADC初始化任务
void ADC_Process_Task(void const *argument);  // 新增：ADC数据处理任务
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
	// 创建信号量（新增：用于DMA完成通知）
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
	  // 新增：ADC相关任务 初始采集任务 完成一次则删除
  osThreadDef(ADCTask, ADC_Init_Task, osPriorityHigh, 0, 128);
  ADCTaskHandle = osThreadCreate(osThread(ADCTask), NULL);
	  //处理采集信息、再次启动任务
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
		uint32_t current_delay = 200;  // 默认延时200ms

  /* Infinite loop */
  for(;;)
  {
		/* 检查队列中是否(非空)有新的延时值 是非空则把队列中的最新的值读取到current_delay */
		if(xQueueReceive(Delayms_QueueHandle,&current_delay,0) == pdTRUE)
		{
		}
	
		 /* 翻转LED状态 */
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
		
		if(last_Button_state == GPIO_PIN_SET && Current_Button_state == GPIO_PIN_RESET)//下降沿 按键按下
		{
			start_times = xTaskGetTickCount();//ticks 1ms加1
		}
		if(last_Button_state == GPIO_PIN_RESET && Current_Button_state == GPIO_PIN_SET)//上升沿 按键释放
		{
			osDelay(20);
			stop_times = xTaskGetTickCount()-start_times;

			if(stop_times <= 1000)//短按1s内
			{
//				flag++;
				current_index = (current_index+1)%(sizeof(delay_option)/sizeof(delay_option[0]));
				xQueueSend(Delayms_QueueHandle,&delay_option[current_index],pdMS_TO_TICKS(10));
//				if(flag%2 == 1) TIM_SetTIM1Compare1(50);
//				else TIM_SetTIM1Compare1(0);
			}
			if(stop_times >= 2000 && stop_times <= 3000)//短长按2s-3s内
			{
				DDS_Freq = 1000000;
				xQueueSend(Freq_QueueHandle,&DDS_Freq,pdMS_TO_TICKS(10));
			}
			if(stop_times >= 3000 && stop_times <= 5000)//短长按3s-5s内
			{
				DDS_Freq = 10000;
				xQueueSend(Freq_QueueHandle,&DDS_Freq,pdMS_TO_TICKS(10));
			}
		}
		
//		TIM_SetTIM1Compare1(0);
		last_Button_state = Current_Button_state;//状态更新
		osDelay(10);//因为按键任务的优先级最高，为了其他低优先级任务能够被调度，这里需要在每次的循环后让出CPU方便执行其他任务


		
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
	AD985x_SetFre_Phase(default_freq,0);//设置输出频率10KHz,相位0；
	osDelay(1000);
	vTaskDelete(NULL);//初始化稳定一段时间后以后删除初始化任务
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
			AD985x_SetFre_Phase(freq,0);//设置输出频率freqHz,相位0；
		}
  }
  /* USER CODE END AD9850CTR_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void ADC_Init_Task(void const * argument)
{
	  // 启动ADC+DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 250);  // ADC1采集250点
  HAL_TIM_Base_Start(&htim3);  // 启动TIM3（触发ADC）

  // 任务完成后删除自身
  vTaskDelete(NULL);
	
}

void ADC_Process_Task(void const * argument)
{
  uint8_t tx_buf[500];  // 用于USART发送的缓冲区（250点×2字节）

  for(;;) {
			
    // 等待DMA完成信号量 等待信号量为1才继续往下执行，并且信号量-1
   if( osSemaphoreWait(ADCDMA_SemHandle, portMAX_DELAY) == osOK)
	 {
//    printf("ADC Data: ");  // 前缀提示
    // 数据处理
    for(uint16_t i = 0; i < 250; i++) {
//      tx_buf[i*2] = (adc_buf[i] >> 8) & 0xFF;  // 高8位
//      tx_buf[i*2+1] = adc_buf[i] & 0xFF;       // 低8位
			printf("%d\r\n", adc_buf[i]); 
    }

//    // 通过USART1发送数据 500是字节数 100是100ms设为超时时间
//    HAL_UART_Transmit(&huart1, tx_buf, 500, 100);

    // 重新配置DMA和TIM3，开始下一轮采集
    HAL_TIM_Base_Stop(&htim3);
    HAL_ADC_Stop_DMA(&hadc1);
    
    // 清除标志
    __HAL_DMA_CLEAR_FLAG(&hdma_adc1, DMA_FLAG_TCIF0_4);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
    
    // 重新启动
		HAL_ADC_Start(&hadc1); 
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 250);
    HAL_TIM_Base_Start(&htim3);
		osDelay(10);//让出CPU方便执行其他任务
  }
  }
}

// 中断服务函数都 在stm32f4xx_it.c里面
/* 新增：DMA完成中断处理 */
void DMA2_Stream0_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_adc1);
}

/* 新增：ADC转换完成回调函数 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if(hadc->Instance == ADC1) {
    // 停止TIM3（避免继续触发ADC）
    HAL_TIM_Base_Stop(&htim3);
    
    // 释放信号量，通知处理任务 信号量＋1 变为可用状态
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
