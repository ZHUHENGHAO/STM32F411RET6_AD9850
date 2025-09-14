#include "pwm.h"

// TIM1句柄结构体，用于配置和控制TIM1定时器
TIM_HandleTypeDef 	TIM1_Handler;      	
// TIM1通道1配置结构体，用于配置CH1的PWM参数
TIM_OC_InitTypeDef 	TIM1_CH1Handler;	
// TIM1通道2N配置结构体，用于配置CH2N的PWM参数（互补输出）
TIM_OC_InitTypeDef 	TIM1_CH2NHandler;	
// TIM1死区时间配置结构体，用于配置刹车和死区时间参数
TIM_BreakDeadTimeConfigTypeDef TIM1_DeadTimeHandler; 

// 用于PWM脉冲串计数的变量（volatile确保多线程/中断中可见性）
volatile uint32_t pwm_burst_count = 0;
// 标记PWM脉冲串是否激活的变量
volatile uint8_t pwm_burst_active = 0;

/**
 * @brief  TIM1定时器PWM初始化函数
 * @param  arr: 自动重装载值，决定PWM周期 = (arr+1) * (psc+1) / 定时器时钟频率
 * @param  psc: 预分频器值，用于分频定时器时钟
 * @retval 无
 */
void TIM1_PWM_Init(u16 arr,u16 psc)
{  
    // 1. 初始化TIM1基本参数
    TIM1_Handler.Instance=TIM1;          // 选择TIM1定时器
    TIM1_Handler.Init.Prescaler=psc;     // 设置预分频器
    TIM1_Handler.Init.CounterMode=TIM_COUNTERMODE_UP; // 向上计数模式
    TIM1_Handler.Init.Period=arr;        // 设置自动重装载值
    TIM1_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1; // 时钟分频因子为1
    TIM1_Handler.Init.RepetitionCounter=0;  // 重复计数器值为0（不使用重复计数）
    HAL_TIM_PWM_Init(&TIM1_Handler);       	// 初始化PWM功能

    // 2. 配置CH1通道（PA8引脚）为PWM1模式（用于主输出）
    TIM1_CH1Handler.OCMode=TIM_OCMODE_PWM1; // PWM1模式：当计数器值<比较值时，输出有效电平
    TIM1_CH1Handler.Pulse=arr/2;            // 初始占空比50%（比较值为自动重装载值的一半）
    TIM1_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; // 输出极性为高（有效电平为高电平）
    TIM1_CH1Handler.OCFastMode=TIM_OCFAST_DISABLE; // 禁用快速比较模式
    TIM1_CH1Handler.OCIdleState=TIM_OCIDLESTATE_SET; // 空闲状态下输出高电平
    HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH1Handler,TIM_CHANNEL_1); // 应用通道1配置
	
    // 3. 配置CH2N通道（PB0引脚）为PWM2模式（用于互补输出）
    TIM1_CH2NHandler.OCMode=TIM_OCMODE_PWM2; // PWM2模式：与CH1互补，计数器值>比较值时输出有效电平
    TIM1_CH2NHandler.Pulse=arr/2;            // 初始占空比与CH1相同（50%）
    TIM1_CH2NHandler.OCNPolarity=TIM_OCNPOLARITY_HIGH; // 互补输出极性为高
    TIM1_CH2NHandler.OCFastMode=TIM_OCFAST_DISABLE; // 禁用快速比较模式
    TIM1_CH2NHandler.OCNIdleState=TIM_OCNIDLESTATE_SET; // 空闲状态下互补输出高电平
    HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH2NHandler,TIM_CHANNEL_2); // 应用通道2配置
	
    // 4. 配置死区时间参数
    TIM1_DeadTimeHandler.OffStateRunMode=TIM_OSSR_DISABLE; // 运行模式下关闭状态选择禁用
    TIM1_DeadTimeHandler.OffStateIDLEMode=TIM_OSSI_DISABLE; // 空闲模式下关闭状态选择禁用
    TIM1_DeadTimeHandler.LockLevel=TIM_LOCKLEVEL_OFF; // 关闭寄存器锁定
    TIM1_DeadTimeHandler.DeadTime=2; // 死区时间设置为2个时钟周期（防止上下桥臂同时导通）
    TIM1_DeadTimeHandler.BreakState=TIM_BREAK_DISABLE; // 禁用刹车功能
    TIM1_DeadTimeHandler.BreakPolarity=TIM_BREAKPOLARITY_HIGH; // 刹车输入极性为高
    TIM1_DeadTimeHandler.AutomaticOutput=TIM_AUTOMATICOUTPUT_ENABLE; // 使能自动输出
    HAL_TIMEx_ConfigBreakDeadTime(&TIM1_Handler,&TIM1_DeadTimeHandler); // 应用死区配置
    
    // 5. 使能TIM1更新中断（用于脉冲串计数）
    __HAL_TIM_ENABLE_IT(&TIM1_Handler, TIM_IT_UPDATE);
}

/**
 * @brief  TIM_PWM的MSP初始化回调函数
 * @note   此函数会被HAL_TIM_PWM_Init()调用，用于初始化底层硬件（GPIO、时钟等）
 * @param  htim: TIM句柄指针
 * @retval 无
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_Initure;
	
    if(htim->Instance==TIM1) // 仅处理TIM1的初始化
	{
		__HAL_RCC_TIM1_CLK_ENABLE();			// 使能TIM1时钟
		
		__HAL_RCC_GPIOA_CLK_ENABLE();			// 使能GPIOA时钟
		__HAL_RCC_GPIOB_CLK_ENABLE();			// 使能GPIOB时钟
		
		// 配置PA8为TIM1_CH1功能（PWM输出）
		GPIO_Initure.Pin=GPIO_PIN_8;            
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	// 复用推挽输出（用于PWM信号）
		GPIO_Initure.Pull=GPIO_PULLDOWN;        // 下拉电阻（默认低电平）
		GPIO_Initure.Speed=GPIO_SPEED_HIGH;     // 高速模式
		GPIO_Initure.Alternate=GPIO_AF1_TIM1;	// PA8复用为TIM1_CH1
		HAL_GPIO_Init(GPIOA,&GPIO_Initure); 	
		
		// 配置PB0为TIM1_CH2N功能（互补PWM输出）
		GPIO_Initure.Pin=GPIO_PIN_0;            
		GPIO_Initure.Alternate=GPIO_AF1_TIM1;	// PB0复用为TIM1_CH2N
		HAL_GPIO_Init(GPIOB,&GPIO_Initure); 	
		
		// 配置NVIC，使能TIM1更新中断
		HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0); // 中断优先级：抢占优先级0，子优先级0
		HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn); // 使能TIM1_UP_TIM10中断通道
	}
}

/**
 * @brief  设置TIM1通道1的比较值（用于调节PWM占空比）
 * @param  compare: 比较值，范围0~arr（arr为初始化时的自动重装载值）
 * @retval 无
 */
void TIM_SetTIM1Compare1(u32 compare)
{
	TIM1->CCR1=compare; // 直接写入比较寄存器，更新占空比
}

/**
 * @brief  TIM1更新中断服务函数（同时处理TIM10中断）
 * @note   用于实现PWM脉冲串计数功能，当计数到0时自动停止PWM
 * @retval 无
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
    // 检查TIM1更新中断标志位
    if(__HAL_TIM_GET_FLAG(&TIM1_Handler, TIM_FLAG_UPDATE) != RESET)
    {
        // 检查中断源是否为TIM1更新中断
        if(__HAL_TIM_GET_IT_SOURCE(&TIM1_Handler, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&TIM1_Handler, TIM_IT_UPDATE); // 清除中断标志位
            
            // 处理PWM脉冲串计数
            if(pwm_burst_active && pwm_burst_count > 0)
            {
                pwm_burst_count--; // 每产生一次更新中断（一个PWM周期），计数减1
                
                // 当计数减到0时，停止PWM并清除激活状态
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
 * @brief  启动PWM输出
 * @note   同时启动主通道（CH1）和互补通道（CH2N）的PWM输出
 * @retval 无
 */
void Start_Pwm(void)
{   
  	HAL_TIMEx_PWMN_Start(&TIM1_Handler, TIM_CHANNEL_2); // 启动CH2N互补通道PWM
    HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_1); // 启动CH1主通道PWM
}

/**
 * @brief  停止PWM输出
 * @note   同时停止主通道（CH1）和互补通道（CH2N）的PWM输出
 * @retval 无
 */
void Stop_Pwm(void)
{
    HAL_TIMEx_PWMN_Stop(&TIM1_Handler,TIM_CHANNEL_2); // 停止CH2N互补通道PWM
    HAL_TIM_PWM_Stop(&TIM1_Handler,TIM_CHANNEL_1); // 停止CH1主通道PWM
}

/**
 * @brief  启动PWM脉冲串输出（指定脉冲数量后自动停止）
 * @param  pulse_count: 需要输出的PWM脉冲数量
 * @retval 无
 */
void Start_Pwm_Burst(uint32_t pulse_count)
{
    // 配置脉冲串参数
    pwm_burst_count = pulse_count; // 设置需要输出的脉冲数量
    pwm_burst_active = 1; // 激活脉冲串模式
    
    // 启动PWM输出
    __HAL_TIM_SET_COUNTER(&TIM1_Handler, 0); // 重置计数器，确保脉冲计数准确
    Start_Pwm(); // 启动PWM
}