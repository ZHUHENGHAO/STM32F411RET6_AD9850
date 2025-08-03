#include "pwm.h"

TIM_HandleTypeDef 	TIM1_Handler;      	//定时器句柄 
TIM_OC_InitTypeDef 	TIM1_CH1Handler;	//定时器1通道1句柄
TIM_BreakDeadTimeConfigTypeDef TIM1_DeadTimeHandler; //死区配置句柄

//TIM1 互补PWM部分初始化 
//arr：自动重装值
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us
//Ft=定时器工作频率,单位:Mhz（TIM1挂载于APB2，最高100MHz）
void TIM1_PWM_Init(u16 arr,u16 psc)
{  
    TIM1_Handler.Instance=TIM1;         	//定时器1（高级控制定时器）
    TIM1_Handler.Init.Prescaler=psc;       	//定时器分频
    TIM1_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//向上计数模式
    TIM1_Handler.Init.Period=arr;          	//自动重装载值
    TIM1_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//分频因子
//    TIM1_Handler.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;//禁止自动重装载预加载
    TIM1_Handler.Init.RepetitionCounter=0;  //重复计数器值（高级定时器特有）
    HAL_TIM_PWM_Init(&TIM1_Handler);       	//初始化PWM
    
    //配置CH1主通道及CH1N互补通道
    TIM1_CH1Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM1_CH1Handler.Pulse=0;            //设置比较值,占空比50%
    TIM1_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //主通道输出极性高
    TIM1_CH1Handler.OCNPolarity=TIM_OCPOLARITY_HIGH;//互补通道输出极性
    TIM1_CH1Handler.OCFastMode=TIM_OCFAST_DISABLE; //禁止快速模式
    TIM1_CH1Handler.OCIdleState=TIM_OCIDLESTATE_RESET; //空闲状态主通道输出复位
    TIM1_CH1Handler.OCNIdleState=TIM_OCNIDLESTATE_RESET; //空闲状态互补通道输出复位
    HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH1Handler,TIM_CHANNEL_1);//配置TIM1通道1
	
//    //配置死区时间（防止上下桥臂直通）
//    TIM1_DeadTimeHandler.OffStateRunMode=TIM_OSSR_DISABLE; //运行模式下关闭状态选择
//    TIM1_DeadTimeHandler.OffStateIDLEMode=TIM_OSSI_DISABLE; //空闲模式下关闭状态选择
//    TIM1_DeadTimeHandler.LockLevel=TIM_LOCKLEVEL_OFF; //锁定级别关闭
//    TIM1_DeadTimeHandler.DeadTime=10; //死区时间（单位：定时器时钟周期，需根据实际需求调整）
//    TIM1_DeadTimeHandler.BreakState=TIM_BREAK_DISABLE; //禁止刹车功能
//    TIM1_DeadTimeHandler.BreakPolarity=TIM_BREAKPOLARITY_HIGH; //刹车信号极性
//    TIM1_DeadTimeHandler.AutomaticOutput=TIM_AUTOMATICOUTPUT_DISABLE; //禁止自动输出
//    HAL_TIMEx_ConfigBreakDeadTime(&TIM1_Handler,&TIM1_DeadTimeHandler);
	
    HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_1); //开启主通道PWM
    HAL_TIMEx_PWMN_Start(&TIM1_Handler,TIM_CHANNEL_1); //开启互补通道PWM
	 	   
}

//定时器底层驱动，时钟使能，引脚配置
//此函数会被HAL_TIM_PWM_Init()调用
//htim:定时器句柄
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_Initure;
	
    if(htim->Instance==TIM1)
	{
		__HAL_RCC_TIM1_CLK_ENABLE();			//使能定时器1时钟（APB2总线）
		
		__HAL_RCC_GPIOA_CLK_ENABLE();			//开启GPIOA时钟
		
		//配置PA8为TIM1_CH1（主通道）
		GPIO_Initure.Pin=GPIO_PIN_8;            
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
		GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
		GPIO_Initure.Alternate=GPIO_AF1_TIM1;	//PA8复用为TIM1_CH1（AF1）
		HAL_GPIO_Init(GPIOA,&GPIO_Initure); 	
		
		//配置PA7为TIM1_CH1N（互补通道）
		GPIO_Initure.Pin=GPIO_PIN_7;            
		GPIO_Initure.Alternate=GPIO_AF1_TIM1;	//PA7复用为TIM1_CH1N（AF1）
		HAL_GPIO_Init(GPIOA,&GPIO_Initure); 	
	}
}

//设置TIM1通道1的占空比
//compare:比较值（0~arr）
void TIM_SetTIM1Compare1(u32 compare)
{
	TIM1->CCR1=compare; 
}

//定时器1中断服务函数
void TIM1_UP_TIM10_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM1_Handler);
}
