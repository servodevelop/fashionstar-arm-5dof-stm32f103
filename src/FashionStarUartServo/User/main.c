///*
// * FashionStar五自由度机械臂-初始化测试
// * --------------------------
// * 作者: 阿凯|Kyle
// * 邮箱: xingshunkai@qq.com
// * 更新时间: 2020/05/26
// */
// 
//#include "stm32f10x.h"
//#include "usart.h"
//#include "sys_tick.h"
//#include "fashion_star_uart_servo.h"
//#include "arm5dof.h"

//// 使用串口1作为舵机控制的端口
//// <接线说明>
//// STM32F103 PA9(Tx)  <----> 串口舵机转接板 Rx
//// STM32F103 PA10(Rx) <----> 串口舵机转接板 Tx
//// STM32F103 GND 	  <----> 串口舵机转接板 GND
//// STM32F103 V5 	  <----> 串口舵机转接板 5V
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART1_ENABLE为1
//// 设置完成之后, 将下行取消注释
//Usart_DataTypeDef* servoUsart = &usart1; 

//int main (void)
//{
//	
//	SysTick_Init(); 			// 嘀嗒定时器初始化
//	Usart_Init(); 				// 串口初始化
//	FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点
//	while (1){
//	}
//}


///*
// * FashionStar五自由度机械臂-阻尼模式下角度回读
// * --------------------------
// * 作者: 阿凯|Kyle
// * 邮箱: xingshunkai@qq.com
// * 更新时间: 2020/05/26
// */
// 
//#include "stm32f10x.h"
//#include "usart.h"
//#include "sys_tick.h"
//#include "fashion_star_uart_servo.h"
//#include "arm5dof.h"

//// 使用串口1作为舵机控制的端口
//// <接线说明>
//// STM32F103 PA9(Tx)  <----> 串口舵机转接板 Rx
//// STM32F103 PA10(Rx) <----> 串口舵机转接板 Tx
//// STM32F103 GND 	  <----> 串口舵机转接板 GND
//// STM32F103 V5 	  <----> 串口舵机转接板 5V
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART1_ENABLE为1
//// 设置完成之后, 将下行取消注释
//Usart_DataTypeDef* servoUsart = &usart1; 

//// 使用串口2作为日志输出的端口
//// <接线说明>
//// STM32F103 PA2(Tx) <----> USB转TTL Rx
//// STM32F103 PA3(Rx) <----> USB转TTL Tx
//// STM32F103 GND 	 <----> USB转TTL GND
//// STM32F103 V5 	 <----> USB转TTL 5V (可选)
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART2_ENABLE为1
//Usart_DataTypeDef* loggingUsart = &usart2;

//// 重定向c库函数printf到串口，重定向后可使用printf函数
//int fputc(int ch, FILE *f)
//{
//	while((loggingUsart->pUSARTx->SR&0X40)==0){}
//	/* 发送一个字节数据到串口 */
//	USART_SendData(loggingUsart->pUSARTx, (uint8_t) ch);
//	/* 等待发送完毕 */
//	// while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);		
//	return (ch);
//}



//int main (void)
//{
//	
//	SysTick_Init(); 			// 嘀嗒定时器初始化
//	Usart_Init(); 				// 串口初始化
//	FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点
//	uint16_t power = 500; 		// 阻尼模式下的功率
//	FSARM_SetDamping(power);    // 设置舵机为阻尼模式
//	
//	FSARM_JOINTS_STATE_T servoAngles; // 舵机的角度
//	FSARM_JOINTS_STATE_T jointAngles; // 关节的角度

//	while (1){
//		// 查询舵机当前的角度
//		FSARM_QueryServoAngle(&servoAngles);		
//		// 查询当前关节的角度
//		FSARM_QueryJointAngle(&jointAngles);
//		// 打印日志
//		printf("[INFO] Servo Angles: [%.1f, %.1f, %.1f, %.1f, %.1f] \r\n", \
//			servoAngles.theta1, servoAngles.theta2, servoAngles.theta3, servoAngles.theta4, servoAngles.gripper);
//		printf("[INFO] Joint Angles: [%.1f, %.1f, %.1f, %.1f, %.1f] \r\n", \
//			jointAngles.theta1, jointAngles.theta2, jointAngles.theta3, jointAngles.theta4, jointAngles.gripper);
//		
//		// 等待500ms
//		SysTick_DelayMs(500);
//	}
//}


///*
// * FashionStar五自由度机械臂-设置关节角度
// * --------------------------
// * 作者: 阿凯|Kyle
// * 邮箱: xingshunkai@qq.com
// * 更新时间: 2020/05/26
// */
// 
//#include "stm32f10x.h"
//#include "usart.h"
//#include "sys_tick.h"
//#include "fashion_star_uart_servo.h"
//#include "arm5dof.h"

//// 使用串口1作为舵机控制的端口
//// <接线说明>
//// STM32F103 PA9(Tx)  <----> 串口舵机转接板 Rx
//// STM32F103 PA10(Rx) <----> 串口舵机转接板 Tx
//// STM32F103 GND 	  <----> 串口舵机转接板 GND
//// STM32F103 V5 	  <----> 串口舵机转接板 5V
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART1_ENABLE为1
//// 设置完成之后, 将下行取消注释
//Usart_DataTypeDef* servoUsart = &usart1; 


//int main (void)
//{
//	
//	SysTick_Init(); 			// 嘀嗒定时器初始化
//	Usart_Init(); 				// 串口初始化
//	FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点
//			
//	FSARM_JOINTS_STATE_T jointAngles; // 关节的角度
//	
//	while (1){
//		// 动作A
//		// 设置关节角度
//		jointAngles.theta1 = 45.0;
//		jointAngles.theta2 = -130.0;
//		jointAngles.theta3 = 90.0;
//		jointAngles.theta4 = 60.0;
//		jointAngles.gripper = 45.0; // 爪子的角度
//		
//		// 设置关节角度
//		FSARM_SetJointAngle(jointAngles);
//		FSARM_WaitAll();
//		SysTick_DelayMs(1000); // 等待1s
//		
//		// 动作B
//		jointAngles.theta1 = 90.0;
//		jointAngles.theta2 = -130.0;
//		jointAngles.theta3 = 90.0;
//		jointAngles.theta4 = 60.0;
//		jointAngles.gripper = 0.0; // 爪子的角度
//		
//		uint16_t interval = 1000; // 周期为2000ms
//		FSARM_SetJointAngle2(jointAngles, interval);
//		FSARM_WaitAll();
//		SysTick_DelayMs(1000); // 等待1s
//	}
//}


///*
// * FashionStar五自由度机械臂-机械臂正向运动学测试
// * --------------------------
// * 作者: 阿凯|Kyle
// * 邮箱: xingshunkai@qq.com
// * 更新时间: 2020/05/26
// */
// 
//#include "stm32f10x.h"
//#include "usart.h"
//#include "sys_tick.h"
//#include "fashion_star_uart_servo.h"
//#include "arm5dof.h"

//// 使用串口1作为舵机控制的端口
//// <接线说明>
//// STM32F103 PA9(Tx)  <----> 串口舵机转接板 Rx
//// STM32F103 PA10(Rx) <----> 串口舵机转接板 Tx
//// STM32F103 GND 	  <----> 串口舵机转接板 GND
//// STM32F103 V5 	  <----> 串口舵机转接板 5V
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART1_ENABLE为1
//// 设置完成之后, 将下行取消注释
//Usart_DataTypeDef* servoUsart = &usart1; 
//// 使用串口2作为日志输出的端口
//// <接线说明>
//// STM32F103 PA2(Tx) <----> USB转TTL Rx
//// STM32F103 PA3(Rx) <----> USB转TTL Tx
//// STM32F103 GND 	 <----> USB转TTL GND
//// STM32F103 V5 	 <----> USB转TTL 5V (可选)
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART2_ENABLE为1
//Usart_DataTypeDef* loggingUsart = &usart2;

//// 重定向c库函数printf到串口，重定向后可使用printf函数
//int fputc(int ch, FILE *f)
//{
//	while((loggingUsart->pUSARTx->SR&0X40)==0){}
//	/* 发送一个字节数据到串口 */
//	USART_SendData(loggingUsart->pUSARTx, (uint8_t) ch);
//	/* 等待发送完毕 */
//	// while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);		
//	return (ch);
//}


//int main (void)
//{
//	
//	SysTick_Init(); 			// 嘀嗒定时器初始化
//	Usart_Init(); 				// 串口初始化
//	FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点
//			
//	FSARM_JOINTS_STATE_T jointAngles; // 关节的角度
//	FSARM_POINT3D_T  toolPosi; 		  // 工具坐标系的坐标
//	float pitch; 					  // 末端的俯仰角
//	
//	// 设置关节角度
//	jointAngles.theta1 = 45.0;
//    jointAngles.theta2 = -130.0;
//    jointAngles.theta3 = 90.0;
//    jointAngles.theta4 = 60.0;
//    jointAngles.gripper = 0.0;
//	
//	// 设置关节角度
//	FSARM_SetJointAngle(jointAngles);
//	FSARM_WaitAll();	
//	SysTick_DelayMs(1000); // 等待1s
//	
//	// 测试正向运动学
//	FSARM_ForwardKinmatics(jointAngles, &toolPosi, &pitch);
//	// 打印当前末端的位置信息
//	printf("Forward Kinematics, ToolPosi = (%.1f, %.1f, %.1f) Pitch=%.1f", \
//		toolPosi.x, toolPosi.y, toolPosi.z, pitch);
//	
//	while (1){
//	}
//}



///*
// * FashionStar五自由度机械臂-机械臂逆向运动学测试
// * --------------------------
// * 作者: 阿凯|Kyle
// * 邮箱: xingshunkai@qq.com
// * 更新时间: 2020/05/27
// */
// 
//#include "stm32f10x.h"
//#include "usart.h"
//#include "sys_tick.h"
//#include "fashion_star_uart_servo.h"
//#include "arm5dof.h"

//// 使用串口1作为舵机控制的端口
//// <接线说明>
//// STM32F103 PA9(Tx)  <----> 串口舵机转接板 Rx
//// STM32F103 PA10(Rx) <----> 串口舵机转接板 Tx
//// STM32F103 GND 	  <----> 串口舵机转接板 GND
//// STM32F103 V5 	  <----> 串口舵机转接板 5V
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART1_ENABLE为1
//// 设置完成之后, 将下行取消注释
//Usart_DataTypeDef* servoUsart = &usart1; 
//// 使用串口2作为日志输出的端口
//// <接线说明>
//// STM32F103 PA2(Tx) <----> USB转TTL Rx
//// STM32F103 PA3(Rx) <----> USB转TTL Tx
//// STM32F103 GND 	 <----> USB转TTL GND
//// STM32F103 V5 	 <----> USB转TTL 5V (可选)
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART2_ENABLE为1
//Usart_DataTypeDef* loggingUsart = &usart2;

//// 重定向c库函数printf到串口，重定向后可使用printf函数
//int fputc(int ch, FILE *f)
//{
//	while((loggingUsart->pUSARTx->SR&0X40)==0){}
//	/* 发送一个字节数据到串口 */
//	USART_SendData(loggingUsart->pUSARTx, (uint8_t) ch);
//	/* 等待发送完毕 */
//	// while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);		
//	return (ch);
//}


//int main (void)
//{
//	SysTick_Init(); 			// 嘀嗒定时器初始化
//	Usart_Init(); 				// 串口初始化
//	FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点
//			
//	FSARM_JOINTS_STATE_T jointAngles; // 关节的角度
//	FSARM_POINT3D_T  toolPosi; 		  // 工具坐标系的坐标
//	float pitch;
//	
//	// 设置关节角度
//	jointAngles.theta1 = 45.0;
//    jointAngles.theta2 = -130.0;
//    jointAngles.theta3 = 90.0;
//    jointAngles.theta4 = 60.0;
//    jointAngles.gripper = 0.0;
//	
//	// 设置关节角度
//	FSARM_SetJointAngle(jointAngles);
//	FSARM_WaitAll();	
//	SysTick_DelayMs(1000); // 等待1s
//	
//	// 机械臂正向运动学
//	FSARM_ForwardKinmatics(jointAngles, &toolPosi, &pitch);
//	// 打印当前末端的位置信息
//	printf("Forward Kinematics, ToolPosi = (%.1f, %.1f, %.1f) Pitch = %.1f \r\n", toolPosi.x, toolPosi.y, toolPosi.z, pitch);
//	
//	// 关节角度2(存储逆向运动学的结果)
//	FSARM_JOINTS_STATE_T jointAngles2;
//	// 机械臂逆向运动
//	FSARM_STATUS status = FSARM_InverseKinematics(toolPosi, pitch, &jointAngles2);
//	// 打印逆向运动学的结果
//	printf("Inverse Kinematics, result code = %d\r\n", status);
//	printf("-> Joint Angles = (%.1f, %.1f, %.1f, %.1f) \r\n", jointAngles2.theta1, jointAngles2.theta2, jointAngles2.theta3, jointAngles2.theta4);
//	
//	while (1){
//	}
//}



///*
// * FashionStar五自由度机械臂-点控MoveP2P(自由轨迹)
// * --------------------------
// * 作者: 阿凯|Kyle
// * 邮箱: xingshunkai@qq.com
// * 更新时间: 2020/05/27
// */
// 
//#include "stm32f10x.h"
//#include "usart.h"
//#include "sys_tick.h"
//#include "fashion_star_uart_servo.h"
//#include "arm5dof.h"

//// 使用串口1作为舵机控制的端口
//// <接线说明>
//// STM32F103 PA9(Tx)  <----> 串口舵机转接板 Rx
//// STM32F103 PA10(Rx) <----> 串口舵机转接板 Tx
//// STM32F103 GND 	  <----> 串口舵机转接板 GND
//// STM32F103 V5 	  <----> 串口舵机转接板 5V
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART1_ENABLE为1
//// 设置完成之后, 将下行取消注释
//Usart_DataTypeDef* servoUsart = &usart1; 

//int main (void)
//{
//	SysTick_Init(); 			// 嘀嗒定时器初始化
//	Usart_Init(); 				// 串口初始化
//	FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点
//			
//	
//	
//	while (1){
//		FSARM_MoveP2P(13.5, 0, 5, 25.0);
//		FSARM_WaitAll();
//		SysTick_DelayMs(1000); // 等待1s
//		
//		FSARM_MoveP2P(14.0, 0, -4.0, 55.0);
//		FSARM_WaitAll();
//		SysTick_DelayMs(1000); // 等待1s
//		
//		FSARM_MoveP2P(9.5, 9.5, -4.0, 55.0);
//		FSARM_WaitAll();
//		SysTick_DelayMs(1000); // 等待1s
//	}
//}

/*
 * FashionStar五自由度机械臂-按键控制
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
 * 更新时间: 2020/06/20
 */
 
#include "stm32f10x.h"
#include "usart.h"
#include "sys_tick.h"
#include "fashion_star_uart_servo.h"
#include "arm5dof.h"
#include "button.h"

#define SINGLE_CLICK_ANGLE_STEP 5.0  // 单击角度增量
#define DOUBLE_CLICK_ANGLE_STEP 25.0 // 双击的角度增量
#define LONG_PRESSED_ANGLE_STEP 1.0	 // 长按模式下的角度增量


// 使用串口1作为舵机控制的端口
// <接线说明>
// STM32F103 PA9(Tx)  <----> 串口舵机转接板 Rx
// STM32F103 PA10(Rx) <----> 串口舵机转接板 Tx
// STM32F103 GND 	  <----> 串口舵机转接板 GND
// STM32F103 V5 	  <----> 串口舵机转接板 5V
// <注意事项>
// 使用前确保已设置usart.h里面的USART1_ENABLE为1
// 设置完成之后, 将下行取消注释
Usart_DataTypeDef* servoUsart = &usart1; 
// 使用串口2作为日志输出的端口
// <接线说明>
// STM32F103 PA2(Tx) <----> USB转TTL Rx
// STM32F103 PA3(Rx) <----> USB转TTL Tx
// STM32F103 GND 	 <----> USB转TTL GND
// STM32F103 V5 	 <----> USB转TTL 5V (可选)
// <注意事项>
// 使用前确保已设置usart.h里面的USART2_ENABLE为1
Usart_DataTypeDef* loggingUsart = &usart2;

// 重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	while((loggingUsart->pUSARTx->SR&0X40)==0){}
	/* 发送一个字节数据到串口 */
	USART_SendData(loggingUsart->pUSARTx, (uint8_t) ch);
	/* 等待发送完毕 */
	// while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);		
	return (ch);
}

uint8_t curServoId = 0; 		// 当前的JointID

// 切换当前按键控制的舵机ID
void ChangeServo(uint8_t servoId){
	// 设置当前的舵机角度ID
	curServoId = servoId; 
	// 查询舵机的角度
	FSUS_QueryServoAngle(armUsart, servoId, &(nextServoAngles[servoId]));
	// 日志输出
	printf("Cur Servo Id = %d Servo Angle=%.1f", curServoId, nextServoAngles[servoId]);
}

// 调整关节的角度
void AdjustServoAngle(uint8_t servoId, float dangle){
	float nextAngle = nextServoAngles[servoId] + dangle;	
	// 舵机角度范围约束
	if (nextAngle < servoAngleLowerb[servoId]){
		nextAngle = servoAngleLowerb[servoId];
	}else if(nextAngle > servoAngleUpperb[servoId]){
		nextAngle = servoAngleUpperb[servoId];
	}
	// 设置舵机角度,发送控制指令
	uint16_t interval = (uint16_t)fabs((nextServoAngles[servoId] - nextAngle)/armJointSpeed*1000);
	FSUS_SetServoAngle(armUsart, servoId, nextAngle, interval, 0, false);	
	// 更新舵机的目标角度
	nextServoAngles[servoId] = nextAngle;
}

// 处理按键1按下的逻辑
void Key1PressedHandler(void){
	uint8_t keyId = KEY1;
	
	if(keyStatusList[keyId].eventFlag == KEY_EVENT_NONE){
		return; // 什么也没有发生
	}
	
	// 根据不同的按键状态进行特殊处理
	switch(keyStatusList[keyId].eventFlag){
		case KEY_EVENT_SINGLE_CLICK: // 单击
			// 切换当前的舵机ID +1
			curServoId = (curServoId + 1)%FSARM_SERVO_NUM;
			ChangeServo(curServoId);
			break;
		case KEY_EVENT_DOUBLE_CLICK: // 双击			
			break;
		case KEY_EVENT_LONG_PRESSED_START: // 长按-开始
			break;
		case KEY_EVENT_LONG_PRESSED_END: // 长按-结束
			break;
	}
	
	// 清空标志位
	if(keyStatusList[keyId].eventFlag != KEY_EVENT_LONG_PRESSED_START){
		keyStatusList[keyId].eventFlag = KEY_EVENT_NONE;
	}
}

void Key2PressedHandler(void){
	uint8_t keyId = KEY2;
	
	if(keyStatusList[keyId].eventFlag == KEY_EVENT_NONE){
		return; // 什么也没有发生
	}
	
	// 根据不同的按键状态进行特殊处理
	switch(keyStatusList[keyId].eventFlag){
		case KEY_EVENT_SINGLE_CLICK: // 单击
			// 切换当前的舵机ID -1
			curServoId = (curServoId - 1)%FSARM_SERVO_NUM;
			ChangeServo(curServoId);
			break;
		case KEY_EVENT_DOUBLE_CLICK: // 双击			
			break;
		case KEY_EVENT_LONG_PRESSED_START: // 长按-开始
			break;
		case KEY_EVENT_LONG_PRESSED_END: // 长按-结束
			break;
	}
	
	// 清空标志位
	if(keyStatusList[keyId].eventFlag != KEY_EVENT_LONG_PRESSED_START){
		keyStatusList[keyId].eventFlag = KEY_EVENT_NONE;
	}
}

void Key3PressedHandler(void){
	uint8_t keyId = KEY3;
	
	if(keyStatusList[keyId].eventFlag == KEY_EVENT_NONE){
		return; // 什么也没有发生
	}
	
	// 根据不同的按键状态进行特殊处理
	switch(keyStatusList[keyId].eventFlag){
		case KEY_EVENT_SINGLE_CLICK: // 单击
			AdjustServoAngle(curServoId, SINGLE_CLICK_ANGLE_STEP); // 角度增加(小幅度)
			break;
		case KEY_EVENT_DOUBLE_CLICK: // 双击
			AdjustServoAngle(curServoId, DOUBLE_CLICK_ANGLE_STEP); // 角度增加(大幅度)			
			break;
		case KEY_EVENT_LONG_PRESSED_START: // 长按-开始
			AdjustServoAngle(curServoId, LONG_PRESSED_ANGLE_STEP); // 角度持续增加
			SysTick_DelayMs(20); // 延时， 给舵机反应的时间
			break;
		case KEY_EVENT_LONG_PRESSED_END: // 长按-结束
			// 查询舵机的角度
			FSUS_QueryServoAngle(armUsart, curServoId, &(nextServoAngles[curServoId]));
			// 设置为当前角度(刹车)
			FSUS_SetServoAngle(armUsart, curServoId, nextServoAngles[curServoId], 0, 0, false);	
			break;
	}
	
	// 清空标志位
	if(keyStatusList[keyId].eventFlag != KEY_EVENT_LONG_PRESSED_START){
		keyStatusList[keyId].eventFlag = KEY_EVENT_NONE;
	}
}

void Key4PressedHandler(void){
	uint8_t keyId = KEY4;
	
	if(keyStatusList[keyId].eventFlag == KEY_EVENT_NONE){
		return; // 什么也没有发生
	}
	
	// 根据不同的按键状态进行特殊处理
	switch(keyStatusList[keyId].eventFlag){
		case KEY_EVENT_SINGLE_CLICK: // 单击
			AdjustServoAngle(curServoId, -1*SINGLE_CLICK_ANGLE_STEP); // 角度增加(小幅度)
			break;
		case KEY_EVENT_DOUBLE_CLICK: // 双击
			AdjustServoAngle(curServoId, -1*DOUBLE_CLICK_ANGLE_STEP); // 角度增加(大幅度)			
			break;
		case KEY_EVENT_LONG_PRESSED_START: // 长按-开始
			AdjustServoAngle(curServoId, -1*LONG_PRESSED_ANGLE_STEP); // 角度持续增加
			SysTick_DelayMs(20); // 延时， 给舵机反应的时间
			break;
		case KEY_EVENT_LONG_PRESSED_END: // 长按-结束
			// 查询舵机的角度
			FSUS_QueryServoAngle(armUsart, curServoId, &(nextServoAngles[curServoId]));
			// 设置为当前角度(刹车)
			FSUS_SetServoAngle(armUsart, curServoId, nextServoAngles[curServoId], 0, 0, false);	
			break;
	}
	
	// 清空标志位
	if(keyStatusList[keyId].eventFlag != KEY_EVENT_LONG_PRESSED_START){
		keyStatusList[keyId].eventFlag = KEY_EVENT_NONE;
	}
}

void KeyAPressedHandler(void){
	uint8_t keyId = KEYA;
	
	if(keyStatusList[keyId].eventFlag == KEY_EVENT_NONE){
		return; // 什么也没有发生
	}
	
	// 根据不同的按键状态进行特殊处理
	switch(keyStatusList[keyId].eventFlag){
		case KEY_EVENT_SINGLE_CLICK: // 单击
			FSARM_Home(); 			// 回归机械臂机械零点
			FSARM_SetGripperAngle(0.0, 1000); // 设置爪子的关节角度 
			ChangeServo(FSARM_JOINT1);  // 重置JOINT ID, 更新角度
			break;
		case KEY_EVENT_DOUBLE_CLICK: // 双击			
			break;
		case KEY_EVENT_LONG_PRESSED_START: // 长按-开始
			break;
		case KEY_EVENT_LONG_PRESSED_END: // 长按-结束
			break;
	}
	
	// 清空标志位
	if(keyStatusList[keyId].eventFlag != KEY_EVENT_LONG_PRESSED_START){
		keyStatusList[keyId].eventFlag = KEY_EVENT_NONE;
	}
}

void KeyBPressedHandler(void){
	uint8_t keyId = KEYB;
	
	if(keyStatusList[keyId].eventFlag == KEY_EVENT_NONE){
		return; // 什么也没有发生
	}
	
	// 根据不同的按键状态进行特殊处理
	switch(keyStatusList[keyId].eventFlag){
		case KEY_EVENT_SINGLE_CLICK: // 单击
			break;
		case KEY_EVENT_DOUBLE_CLICK: // 双击			
			break;
		case KEY_EVENT_LONG_PRESSED_START: // 长按-开始
			// 设置舵机为阻尼模式
			FSARM_SetDamping(500);
			// 设置处理标志位(只执行一次) 
			keyStatusList[keyId].eventFlag = KEY_EVENT_NONE;
			break;
		case KEY_EVENT_LONG_PRESSED_END: // 长按-结束
			// 舵机上力
			FSARM_SetTorque(true);
			break;
	}
	
	// 清空标志位
	if(keyStatusList[keyId].eventFlag != KEY_EVENT_LONG_PRESSED_START){
		keyStatusList[keyId].eventFlag = KEY_EVENT_NONE;
	}
}


int main (void)
{
	SysTick_Init(); 			// 嘀嗒定时器初始化
	Usart_Init(); 				// 串口初始化
	FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点		
	ChangeServo(FSARM_JOINT1); 	// 切换当前的关节为JOINT1(初始化)
	Button_Init(); 				// 按键初始化
	SysTick_DelayMs(2000); 		// 等待2s	
	
	while (1){
		Button_KeyLevelScan(); // 按键电平状态扫描
		Button_KeyStatusUpdateAll(); // 更新所有的按键状态
		
		// 处理按键事件
		Key1PressedHandler(); // KEY1事件处理
		Key2PressedHandler(); // KEY2事件处理
		Key3PressedHandler(); // KEY3事件处理
		Key4PressedHandler(); // KEY4事件处理
		KeyAPressedHandler(); // KEYA事件处理
		KeyBPressedHandler(); // KEYB事件处理
		// 10ms扫描一次按键
		SysTick_DelayMs(10);
	}
}
