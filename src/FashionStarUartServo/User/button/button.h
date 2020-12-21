/*
 * 按键驱动
 * Author: Kyle
 */
 
#ifndef __BUTTON_H
#define __BUTTON_H

#include "stm32f10x.h"
#include "sys_tick.h"
#include "stdbool.h"

// 按键的个数
#define KEY_NUM 6

// 按键序号
#define KEY1 0
#define KEY2 1
#define KEY3 2
#define KEY4 3
#define KEYA 4
#define KEYB 5

// GPIO资源定义
#define KEY1_GPIO_Port GPIOB
#define KEY1_Pin GPIO_Pin_4
#define KEY2_GPIO_Port GPIOB
#define KEY2_Pin GPIO_Pin_5
#define KEY3_GPIO_Port GPIOB
#define KEY3_Pin GPIO_Pin_6
#define KEY4_GPIO_Port GPIOB
#define KEY4_Pin GPIO_Pin_7
#define KEYA_GPIO_Port GPIOB
#define KEYA_Pin GPIO_Pin_8
#define KEYB_GPIO_Port GPIOB
#define KEYB_Pin GPIO_Pin_9

// 按键电平定义
#define KEY_RELEASE RESET
#define KEY_PRESSED SET

// 按键扫描周期(单位ms)
#define KEY_SCAN_INTERVAL 10

// 按键事件定义
#define KEY_EVENT_NONE 0 				// 无事件发生
#define KEY_EVENT_SINGLE_CLICK 1 		// 按键单击
#define KEY_EVENT_DOUBLE_CLICK 2 		// 按键双击
#define KEY_EVENT_LONG_PRESSED_START 3 	// 按键长按开始
#define KEY_EVENT_LONG_PRESSED_END  4	// 按键长按结束

// 按键有限状态机 
#define KEY_FSM_RELEASED 0 		// 按键释放
#define KEY_FSM_DEBOUNCE 1 		// 消抖
#define KEY_FSM_SHORT_PRESSED 2	// 短按
#define KEY_FSM_LONG_PRESSED 3	// 长按

// 常量定义
#define KEY_DEBOUNCE_INTERVAL 10 		// 按键消抖的周期(单位 ms)
#define KEY_LONG_PRESSED_THRESHOLD 250 	// 单击与长按的判断阈值
										// 按下时间大于这个值就进入长按状态
#define KEY_DOUBLE_CLICK_THRESHOLD 200	// 如果连续两次单击事件相隔的时间小于这个值，就判定为双击

typedef struct{
	uint8_t fsmStatus; 				// 有限状态机的状态
	uint8_t eventFlag;	 			// 按键事件Flag
	uint8_t lastEventFlag; 			// 上次的事件FLAG
	uint8_t level; 				// 按键的电平状态
	TimeTypedef pressedTime;		// 按键按下的时间
	TimeTypedef releaseTime;		// 按键抬起的时间
	TimeTypedef lastPressedTime; 	// 上次按键按下的时间
	TimeTypedef lastReleaseTime; 	// 上次按键释放的时间
}KeyStatus;


extern KeyStatus keyStatusList[KEY_NUM];

// 按键GPIO配置
void Button_GPIO(void);

// 按键状态初始化
void Button_KeyStatusInit(void);

// 按键初始化
void Button_Init(void);

// 按键电平状态扫描
void Button_KeyLevelScan(void);

// 更新按键的状态
void Button_KeyStatusUpdate(uint8_t btnId);

// 更新所有按键的状态
void Button_KeyStatusUpdateAll(void);

#endif
