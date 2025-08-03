#include "pwm.h"

TIM_HandleTypeDef 	TIM1_Handler;      	//��ʱ����� 
TIM_OC_InitTypeDef 	TIM1_CH1Handler;	//��ʱ��1ͨ��1���
TIM_BreakDeadTimeConfigTypeDef TIM1_DeadTimeHandler; //�������þ��

//TIM1 ����PWM���ֳ�ʼ�� 
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us
//Ft=��ʱ������Ƶ��,��λ:Mhz��TIM1������APB2�����100MHz��
void TIM1_PWM_Init(u16 arr,u16 psc)
{  
    TIM1_Handler.Instance=TIM1;         	//��ʱ��1���߼����ƶ�ʱ����
    TIM1_Handler.Init.Prescaler=psc;       	//��ʱ����Ƶ
    TIM1_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//���ϼ���ģʽ
    TIM1_Handler.Init.Period=arr;          	//�Զ���װ��ֵ
    TIM1_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//��Ƶ����
//    TIM1_Handler.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;//��ֹ�Զ���װ��Ԥ����
    TIM1_Handler.Init.RepetitionCounter=0;  //�ظ�������ֵ���߼���ʱ�����У�
    HAL_TIM_PWM_Init(&TIM1_Handler);       	//��ʼ��PWM
    
    //����CH1��ͨ����CH1N����ͨ��
    TIM1_CH1Handler.OCMode=TIM_OCMODE_PWM1; //ģʽѡ��PWM1
    TIM1_CH1Handler.Pulse=0;            //���ñȽ�ֵ,ռ�ձ�50%
    TIM1_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //��ͨ��������Ը�
    TIM1_CH1Handler.OCNPolarity=TIM_OCPOLARITY_HIGH;//����ͨ���������
    TIM1_CH1Handler.OCFastMode=TIM_OCFAST_DISABLE; //��ֹ����ģʽ
    TIM1_CH1Handler.OCIdleState=TIM_OCIDLESTATE_RESET; //����״̬��ͨ�������λ
    TIM1_CH1Handler.OCNIdleState=TIM_OCNIDLESTATE_RESET; //����״̬����ͨ�������λ
    HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH1Handler,TIM_CHANNEL_1);//����TIM1ͨ��1
	
//    //��������ʱ�䣨��ֹ�����ű�ֱͨ��
//    TIM1_DeadTimeHandler.OffStateRunMode=TIM_OSSR_DISABLE; //����ģʽ�¹ر�״̬ѡ��
//    TIM1_DeadTimeHandler.OffStateIDLEMode=TIM_OSSI_DISABLE; //����ģʽ�¹ر�״̬ѡ��
//    TIM1_DeadTimeHandler.LockLevel=TIM_LOCKLEVEL_OFF; //��������ر�
//    TIM1_DeadTimeHandler.DeadTime=10; //����ʱ�䣨��λ����ʱ��ʱ�����ڣ������ʵ�����������
//    TIM1_DeadTimeHandler.BreakState=TIM_BREAK_DISABLE; //��ֹɲ������
//    TIM1_DeadTimeHandler.BreakPolarity=TIM_BREAKPOLARITY_HIGH; //ɲ���źż���
//    TIM1_DeadTimeHandler.AutomaticOutput=TIM_AUTOMATICOUTPUT_DISABLE; //��ֹ�Զ����
//    HAL_TIMEx_ConfigBreakDeadTime(&TIM1_Handler,&TIM1_DeadTimeHandler);
	
    HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_1); //������ͨ��PWM
    HAL_TIMEx_PWMN_Start(&TIM1_Handler,TIM_CHANNEL_1); //��������ͨ��PWM
	 	   
}

//��ʱ���ײ�������ʱ��ʹ�ܣ���������
//�˺����ᱻHAL_TIM_PWM_Init()����
//htim:��ʱ�����
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_Initure;
	
    if(htim->Instance==TIM1)
	{
		__HAL_RCC_TIM1_CLK_ENABLE();			//ʹ�ܶ�ʱ��1ʱ�ӣ�APB2���ߣ�
		
		__HAL_RCC_GPIOA_CLK_ENABLE();			//����GPIOAʱ��
		
		//����PA8ΪTIM1_CH1����ͨ����
		GPIO_Initure.Pin=GPIO_PIN_8;            
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;          //����
		GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
		GPIO_Initure.Alternate=GPIO_AF1_TIM1;	//PA8����ΪTIM1_CH1��AF1��
		HAL_GPIO_Init(GPIOA,&GPIO_Initure); 	
		
		//����PA7ΪTIM1_CH1N������ͨ����
		GPIO_Initure.Pin=GPIO_PIN_7;            
		GPIO_Initure.Alternate=GPIO_AF1_TIM1;	//PA7����ΪTIM1_CH1N��AF1��
		HAL_GPIO_Init(GPIOA,&GPIO_Initure); 	
	}
}

//����TIM1ͨ��1��ռ�ձ�
//compare:�Ƚ�ֵ��0~arr��
void TIM_SetTIM1Compare1(u32 compare)
{
	TIM1->CCR1=compare; 
}

//��ʱ��1�жϷ�����
void TIM1_UP_TIM10_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM1_Handler);
}
