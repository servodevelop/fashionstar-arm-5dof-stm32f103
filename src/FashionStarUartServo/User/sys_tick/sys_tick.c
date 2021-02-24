/*
 * 系统时间管理
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2020/05/19
 */
#include "sys_tick.h"

// 记录系统的时间 计时器
// __IO = volatile
static __IO u32 sysTickCnt;
static __IO u32 sysTimeStampMs; // 系统的时间戳, 累加 

// 系统定时器初始化
void SysTick_Init(void){
    // STM32F103系统内核的时钟频率是72MHZ
    // SystemCoreClock = SYSCLK_FREQ_72MHz = 72000000
    // SysTick_Config 函数里面传入的是计数为多少的时候产生一次系统中断
    // 1s中断一次  -> SystemCoreClock / 1
    // 1ms中断一次 -> SystemCoreClock / 1000, 1s分成1000个时间片段, 每个片段是1ms
    // 1us中断一次 -> SystemCoreClock / 1000000

    // 这里设置为1ms中断一次
	if(SysTick_Config(SystemCoreClock / 1000)){
		// 捕获异常
		while(1);
	}
    
    // 关闭嘀嗒定时器
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

// 等待计时完成
void SysTick_Wait(){
	// 定时器使能
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    // 等待直到计时器变为0
    while (sysTickCnt > 0);
	// 定时器失能
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

// 延时us
void SysTick_DelayUs(__IO TimeTypedef nTime){
    // 设置时钟中断为us级
    SysTick_Config(SystemCoreClock / 1000000);
    sysTickCnt = nTime;
    // 等待计时完成
    SysTick_Wait();
    // 重新设置系统中断为ms级
    SysTick_Config(SystemCoreClock / 1000);
	// 定时器失能
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

// 延时ms
void SysTick_DelayMs(__IO TimeTypedef nTime){
    sysTickCnt = nTime;
    SysTick_Wait();
}

// 延时s
void SysTick_DelayS(__IO TimeTypedef nTime){
    SysTick_DelayMs(nTime * 1000);
}

// 设置倒计时(非阻塞式)
void SysTick_CountdownBegin(__IO TimeTypedef nTime){
    // 这里设置为1ms中断一次
	sysTickCnt = nTime;
	// 定时器使能
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

// 撤销倒计时
void SysTick_CountdownCancel(void){
    // 重置嘀嗒计时器的计数值
    sysTickCnt = 0;
	// systick 定时器失能
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
	
}

// 判断倒计时是否超时
uint8_t SysTick_CountdownIsTimeout(void){
    return sysTickCnt == 0;
}

// 获取系统的时间戳
TimeTypedef SysTick_Millis(void){
	return sysTimeStampMs;
}

// 设置系统定时器中断的回调函数
void SysTick_Handler(void)
{
	if(sysTickCnt > 0){
        sysTickCnt--;
    }else{
		sysTickCnt = 0;
	}
	// 时间戳累加1
	sysTimeStampMs += 1; // 系统时间戳累加
}
