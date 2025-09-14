#include "pwm.h"

// TIM1����ṹ�壬�������úͿ���TIM1��ʱ��
TIM_HandleTypeDef 	TIM1_Handler;      	
// TIM1ͨ��1���ýṹ�壬��������CH1��PWM����
TIM_OC_InitTypeDef 	TIM1_CH1Handler;	
// TIM1ͨ��2N���ýṹ�壬��������CH2N��PWM���������������
TIM_OC_InitTypeDef 	TIM1_CH2NHandler;	
// TIM1����ʱ�����ýṹ�壬��������ɲ��������ʱ�����
TIM_BreakDeadTimeConfigTypeDef TIM1_DeadTimeHandler; 

// ����PWM���崮�����ı�����volatileȷ�����߳�/�ж��пɼ��ԣ�
volatile uint32_t pwm_burst_count = 0;
// ���PWM���崮�Ƿ񼤻�ı���
volatile uint8_t pwm_burst_active = 0;

/**
 * @brief  TIM1��ʱ��PWM��ʼ������
 * @param  arr: �Զ���װ��ֵ������PWM���� = (arr+1) * (psc+1) / ��ʱ��ʱ��Ƶ��
 * @param  psc: Ԥ��Ƶ��ֵ�����ڷ�Ƶ��ʱ��ʱ��
 * @retval ��
 */
void TIM1_PWM_Init(u16 arr,u16 psc)
{  
    // 1. ��ʼ��TIM1��������
    TIM1_Handler.Instance=TIM1;          // ѡ��TIM1��ʱ��
    TIM1_Handler.Init.Prescaler=psc;     // ����Ԥ��Ƶ��
    TIM1_Handler.Init.CounterMode=TIM_COUNTERMODE_UP; // ���ϼ���ģʽ
    TIM1_Handler.Init.Period=arr;        // �����Զ���װ��ֵ
    TIM1_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1; // ʱ�ӷ�Ƶ����Ϊ1
    TIM1_Handler.Init.RepetitionCounter=0;  // �ظ�������ֵΪ0����ʹ���ظ�������
    HAL_TIM_PWM_Init(&TIM1_Handler);       	// ��ʼ��PWM����

    // 2. ����CH1ͨ����PA8���ţ�ΪPWM1ģʽ�������������
    TIM1_CH1Handler.OCMode=TIM_OCMODE_PWM1; // PWM1ģʽ����������ֵ<�Ƚ�ֵʱ�������Ч��ƽ
    TIM1_CH1Handler.Pulse=arr/2;            // ��ʼռ�ձ�50%���Ƚ�ֵΪ�Զ���װ��ֵ��һ�룩
    TIM1_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; // �������Ϊ�ߣ���Ч��ƽΪ�ߵ�ƽ��
    TIM1_CH1Handler.OCFastMode=TIM_OCFAST_DISABLE; // ���ÿ��ٱȽ�ģʽ
    TIM1_CH1Handler.OCIdleState=TIM_OCIDLESTATE_SET; // ����״̬������ߵ�ƽ
    HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH1Handler,TIM_CHANNEL_1); // Ӧ��ͨ��1����
	
    // 3. ����CH2Nͨ����PB0���ţ�ΪPWM2ģʽ�����ڻ��������
    TIM1_CH2NHandler.OCMode=TIM_OCMODE_PWM2; // PWM2ģʽ����CH1������������ֵ>�Ƚ�ֵʱ�����Ч��ƽ
    TIM1_CH2NHandler.Pulse=arr/2;            // ��ʼռ�ձ���CH1��ͬ��50%��
    TIM1_CH2NHandler.OCNPolarity=TIM_OCNPOLARITY_HIGH; // �����������Ϊ��
    TIM1_CH2NHandler.OCFastMode=TIM_OCFAST_DISABLE; // ���ÿ��ٱȽ�ģʽ
    TIM1_CH2NHandler.OCNIdleState=TIM_OCNIDLESTATE_SET; // ����״̬�»�������ߵ�ƽ
    HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH2NHandler,TIM_CHANNEL_2); // Ӧ��ͨ��2����
	
    // 4. ��������ʱ�����
    TIM1_DeadTimeHandler.OffStateRunMode=TIM_OSSR_DISABLE; // ����ģʽ�¹ر�״̬ѡ�����
    TIM1_DeadTimeHandler.OffStateIDLEMode=TIM_OSSI_DISABLE; // ����ģʽ�¹ر�״̬ѡ�����
    TIM1_DeadTimeHandler.LockLevel=TIM_LOCKLEVEL_OFF; // �رռĴ�������
    TIM1_DeadTimeHandler.DeadTime=2; // ����ʱ������Ϊ2��ʱ�����ڣ���ֹ�����ű�ͬʱ��ͨ��
    TIM1_DeadTimeHandler.BreakState=TIM_BREAK_DISABLE; // ����ɲ������
    TIM1_DeadTimeHandler.BreakPolarity=TIM_BREAKPOLARITY_HIGH; // ɲ�����뼫��Ϊ��
    TIM1_DeadTimeHandler.AutomaticOutput=TIM_AUTOMATICOUTPUT_ENABLE; // ʹ���Զ����
    HAL_TIMEx_ConfigBreakDeadTime(&TIM1_Handler,&TIM1_DeadTimeHandler); // Ӧ����������
    
    // 5. ʹ��TIM1�����жϣ��������崮������
    __HAL_TIM_ENABLE_IT(&TIM1_Handler, TIM_IT_UPDATE);
}

/**
 * @brief  TIM_PWM��MSP��ʼ���ص�����
 * @note   �˺����ᱻHAL_TIM_PWM_Init()���ã����ڳ�ʼ���ײ�Ӳ����GPIO��ʱ�ӵȣ�
 * @param  htim: TIM���ָ��
 * @retval ��
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_Initure;
	
    if(htim->Instance==TIM1) // ������TIM1�ĳ�ʼ��
	{
		__HAL_RCC_TIM1_CLK_ENABLE();			// ʹ��TIM1ʱ��
		
		__HAL_RCC_GPIOA_CLK_ENABLE();			// ʹ��GPIOAʱ��
		__HAL_RCC_GPIOB_CLK_ENABLE();			// ʹ��GPIOBʱ��
		
		// ����PA8ΪTIM1_CH1���ܣ�PWM�����
		GPIO_Initure.Pin=GPIO_PIN_8;            
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	// �����������������PWM�źţ�
		GPIO_Initure.Pull=GPIO_PULLDOWN;        // �������裨Ĭ�ϵ͵�ƽ��
		GPIO_Initure.Speed=GPIO_SPEED_HIGH;     // ����ģʽ
		GPIO_Initure.Alternate=GPIO_AF1_TIM1;	// PA8����ΪTIM1_CH1
		HAL_GPIO_Init(GPIOA,&GPIO_Initure); 	
		
		// ����PB0ΪTIM1_CH2N���ܣ�����PWM�����
		GPIO_Initure.Pin=GPIO_PIN_0;            
		GPIO_Initure.Alternate=GPIO_AF1_TIM1;	// PB0����ΪTIM1_CH2N
		HAL_GPIO_Init(GPIOB,&GPIO_Initure); 	
		
		// ����NVIC��ʹ��TIM1�����ж�
		HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0); // �ж����ȼ�����ռ���ȼ�0�������ȼ�0
		HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn); // ʹ��TIM1_UP_TIM10�ж�ͨ��
	}
}

/**
 * @brief  ����TIM1ͨ��1�ıȽ�ֵ�����ڵ���PWMռ�ձȣ�
 * @param  compare: �Ƚ�ֵ����Χ0~arr��arrΪ��ʼ��ʱ���Զ���װ��ֵ��
 * @retval ��
 */
void TIM_SetTIM1Compare1(u32 compare)
{
	TIM1->CCR1=compare; // ֱ��д��ȽϼĴ���������ռ�ձ�
}

/**
 * @brief  TIM1�����жϷ�������ͬʱ����TIM10�жϣ�
 * @note   ����ʵ��PWM���崮�������ܣ���������0ʱ�Զ�ֹͣPWM
 * @retval ��
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
    // ���TIM1�����жϱ�־λ
    if(__HAL_TIM_GET_FLAG(&TIM1_Handler, TIM_FLAG_UPDATE) != RESET)
    {
        // ����ж�Դ�Ƿ�ΪTIM1�����ж�
        if(__HAL_TIM_GET_IT_SOURCE(&TIM1_Handler, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&TIM1_Handler, TIM_IT_UPDATE); // ����жϱ�־λ
            
            // ����PWM���崮����
            if(pwm_burst_active && pwm_burst_count > 0)
            {
                pwm_burst_count--; // ÿ����һ�θ����жϣ�һ��PWM���ڣ���������1
                
                // ����������0ʱ��ֹͣPWM���������״̬
                if(pwm_burst_count == 0)
                {
                    Stop_Pwm();
                    pwm_burst_active = 0;
                }
            }
        }
    }
}

/**
 * @brief  ����PWM���
 * @note   ͬʱ������ͨ����CH1���ͻ���ͨ����CH2N����PWM���
 * @retval ��
 */
void Start_Pwm(void)
{   
  	HAL_TIMEx_PWMN_Start(&TIM1_Handler, TIM_CHANNEL_2); // ����CH2N����ͨ��PWM
    HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_1); // ����CH1��ͨ��PWM
}

/**
 * @brief  ֹͣPWM���
 * @note   ͬʱֹͣ��ͨ����CH1���ͻ���ͨ����CH2N����PWM���
 * @retval ��
 */
void Stop_Pwm(void)
{
    HAL_TIMEx_PWMN_Stop(&TIM1_Handler,TIM_CHANNEL_2); // ֹͣCH2N����ͨ��PWM
    HAL_TIM_PWM_Stop(&TIM1_Handler,TIM_CHANNEL_1); // ֹͣCH1��ͨ��PWM
}

/**
 * @brief  ����PWM���崮�����ָ�������������Զ�ֹͣ��
 * @param  pulse_count: ��Ҫ�����PWM��������
 * @retval ��
 */
void Start_Pwm_Burst(uint32_t pulse_count)
{
    // �������崮����
    pwm_burst_count = pulse_count; // ������Ҫ�������������
    pwm_burst_active = 1; // �������崮ģʽ
    
    // ����PWM���
    __HAL_TIM_SET_COUNTER(&TIM1_Handler, 0); // ���ü�������ȷ���������׼ȷ
    Start_Pwm(); // ����PWM
}