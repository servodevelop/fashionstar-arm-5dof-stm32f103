#include "button.h"


KeyStatus keyStatusList[KEY_NUM];

// 按键GPIO配置
void Button_GPIO(void){
	// 开启时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	// 配置管脚为上拉输入
	GPIO_InitTypeDef GPIO_InitStructure; // IO配置结构体
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // 上拉输入
	// 配置KEY1
	GPIO_InitStructure.GPIO_Pin = KEY1_Pin;
	GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStructure);
	// 配置KEY2
	GPIO_InitStructure.GPIO_Pin = KEY2_Pin;
	GPIO_Init(KEY2_GPIO_Port, &GPIO_InitStructure);
	// 配置KEY3
	GPIO_InitStructure.GPIO_Pin = KEY3_Pin;
	GPIO_Init(KEY3_GPIO_Port, &GPIO_InitStructure);
	// 配置KEY4
	GPIO_InitStructure.GPIO_Pin = KEY4_Pin;
	GPIO_Init(KEY4_GPIO_Port, &GPIO_InitStructure);
	// 配置KEYA
	GPIO_InitStructure.GPIO_Pin = KEYA_Pin;
	GPIO_Init(KEYA_GPIO_Port, &GPIO_InitStructure);
	// 配置KEYB
	GPIO_InitStructure.GPIO_Pin = KEYB_Pin;
	GPIO_Init(KEYB_GPIO_Port, &GPIO_InitStructure);
}

// 按键扫描,电平状态更新
void Button_KeyLevelScan(void){
	keyStatusList[KEY1].level = GPIO_ReadInputDataBit(KEY1_GPIO_Port, KEY1_Pin);
	keyStatusList[KEY2].level = GPIO_ReadInputDataBit(KEY2_GPIO_Port, KEY2_Pin);
	keyStatusList[KEY3].level = GPIO_ReadInputDataBit(KEY3_GPIO_Port, KEY3_Pin);
	keyStatusList[KEY4].level = GPIO_ReadInputDataBit(KEY4_GPIO_Port, KEY4_Pin);
	keyStatusList[KEYA].level = GPIO_ReadInputDataBit(KEYA_GPIO_Port, KEYA_Pin);
	keyStatusList[KEYB].level = GPIO_ReadInputDataBit(KEYB_GPIO_Port, KEYB_Pin);
}

// 更新按键状态
void Button_KeyStatusInit(void){
	for(uint8_t btnIdx=0; btnIdx < KEY_NUM; btnIdx++){
		keyStatusList[btnIdx].fsmStatus = KEY_FSM_RELEASED;
		keyStatusList[btnIdx].eventFlag = KEY_EVENT_NONE;
		keyStatusList[btnIdx].lastEventFlag = KEY_EVENT_NONE;
		keyStatusList[btnIdx].level = KEY_RELEASE;
		keyStatusList[btnIdx].pressedTime = 0;
		keyStatusList[btnIdx].releaseTime = 0;
		keyStatusList[btnIdx].lastPressedTime = 0;
		keyStatusList[btnIdx].lastReleaseTime = 0;
	}
}
// 更新按键的状态
void Button_KeyStatusUpdate(uint8_t btnId){
	KeyStatus* keyStatus = &(keyStatusList[btnId]);
	switch(keyStatus->fsmStatus){
		case KEY_FSM_RELEASED:
			// STATE1: 初始状态(按键未按下)
			if(keyStatus->level == KEY_PRESSED){
				keyStatus->pressedTime = SysTick_Millis();
				keyStatus->fsmStatus = KEY_FSM_DEBOUNCE;
			}
			break;
		case KEY_FSM_DEBOUNCE:
			// STATE2: 消抖状态
			if(keyStatus->level == KEY_RELEASE){
				// 抖动过滤, 噪声 恢复状态为按键未按下
				keyStatus->fsmStatus = KEY_FSM_RELEASED;
			}else if((SysTick_Millis()-keyStatus->pressedTime) > KEY_DEBOUNCE_INTERVAL){
				// 按键仍为按下状态 -> 切换状态为短按状态
				keyStatus->fsmStatus = KEY_FSM_SHORT_PRESSED;
			}
			break;
		case KEY_FSM_SHORT_PRESSED:
			// STATE3: 按键端按
			if ((SysTick_Millis() - keyStatus->pressedTime) > KEY_LONG_PRESSED_THRESHOLD){
				// 切换为按键长按状态
				keyStatus->fsmStatus = KEY_FSM_LONG_PRESSED;
				// 设置状态为长摁开始
				keyStatus->eventFlag =  KEY_EVENT_LONG_PRESSED_START;
			}else if(keyStatus->level == KEY_RELEASE){
				// 更新按键释放的时间
				keyStatus->releaseTime = SysTick_Millis();
				// 判断是否为Double Click
				if(keyStatus->lastEventFlag == KEY_EVENT_SINGLE_CLICK && \
					(keyStatus->pressedTime - keyStatus->lastReleaseTime) <= KEY_DOUBLE_CLICK_THRESHOLD){
					// 上次事件为单击，且上次释放的时间跟当前按下的时间中间的事件间隔小于(KEY_DOUBLE_CLICK_THRESHOLD)ms
					keyStatus->eventFlag = KEY_EVENT_DOUBLE_CLICK;
				}else{
					// 判定为单击事件
					keyStatus->eventFlag = KEY_EVENT_SINGLE_CLICK;
				}
				// 更新历史数据
				keyStatus->lastEventFlag = keyStatus->eventFlag;
				keyStatus->lastPressedTime = keyStatus->pressedTime;
				keyStatus->lastReleaseTime = keyStatus->releaseTime;
				// 状态切换为释放状态
				keyStatus->fsmStatus = KEY_FSM_RELEASED;
			}
			break;
		case KEY_FSM_LONG_PRESSED:
			// STATE4: 按键长按
			if (keyStatus->level == KEY_RELEASE){
				// 更新释放的事件
				keyStatus->releaseTime = SysTick_Millis();
				// 设置事件设置为长摁结束
				keyStatus->eventFlag =  KEY_EVENT_LONG_PRESSED_END;
				// 更新历史数据
				keyStatus->lastEventFlag = keyStatus->eventFlag;
				keyStatus->lastPressedTime = keyStatus->pressedTime;
				keyStatus->lastReleaseTime = keyStatus->releaseTime;
				// 状态切换为释放状态
				keyStatus->fsmStatus = KEY_FSM_RELEASED;
			}
			break;
	}
}

// 更新所有按键的状态
void Button_KeyStatusUpdateAll(void){
	for(uint8_t btnIdx=0; btnIdx < KEY_NUM; btnIdx++){
		Button_KeyStatusUpdate(btnIdx);
	}
}

// 按键初始化
void Button_Init(void){
	
	// 按键GPIO配置
	Button_GPIO();
	
	// 初始化keyStatusList
	Button_KeyStatusInit();
}


