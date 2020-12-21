/*
 * FashionStar四自由度机械臂 STM32 SDK
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
 * 更新时间: 2020/05/26
 */
#ifndef __ARM5DOF_H
#define __ARM5DOF_H

#include "stm32f10x.h"
#include "stdlib.h"
#include "math.h"
#include "common.h"
#include "usart.h"
#include "fashion_star_uart_servo.h"
#include "sys_tick.h"

// 状态码
#define FSARM_STATUS uint8_t
#define FSARM_STATUS_SUCCESS 0 // 成功
#define FSARM_STATUS_FAIL 1 // 失败
#define FSARM_STATUS_JOINT1_OUTRANGE 2 // 关节1超出范围
#define FSARM_STATUS_JOINT2_OUTRANGE 3 // 关节2超出范围
#define FSARM_STATUS_JOINT3_OUTRANGE 4 // 关节3超出范围
#define FSARM_STATUS_JOINT4_OUTRANGE 5 // 关节4超出范围
#define FSARM_STATUS_TOOLPOSI_TOO_FAR 6 // 工具坐标目标点距离机械臂太遥远

// 机械臂常量
#define FSARM_SERVO_NUM 5 	// 机械臂舵机的个数
#define FSARM_JOINT1 0 		// 关节1对应的舵机ID
#define FSARM_JOINT2 1 		// 关节2对应的舵机ID
#define FSARM_JOINT3 2 		// 关节3对应的舵机ID
#define FSARM_JOINT4 3 		// 关节4对应的舵机ID
#define FSARM_GRIPPER 4     // 爪子对应的舵机ID

#define FSARM_LINK1 9.5     // 连杆1的长度 单位cm
#define FSARM_LINK2 8    	// 连杆2的长度 单位cm
#define FSARM_LINK3 7.6  	// 连杆3的长度 单位cm
#define FSARM_LINK4 13.6    // 连杆4的长度 单位cm(算上了爪子的长度)

// 舵机标定参数
#define FSARM_JOINT1_P90 -85.10  	//关节1为90°时的舵机原始角度
#define FSARM_JOINT1_N90 90.23   	//关节1为-90°时的舵机原始角度
#define FSARM_JOINT2_P0  90.80   	//关节2为0°时的舵机原始角度
#define FSARM_JOINT2_N90 0.90    	//关节2为-90°时的舵机原始角度
#define FSARM_JOINT3_P90 -45.5   	//关节3为90°时的舵机原始角度
#define FSARM_JOINT3_N90 130.90  	//关节3为-90°时的舵机原始角度
#define FSARM_JOINT4_P90 -93.4   	//关节4为90°时的舵机原始角度
#define FSARM_JOINT4_N90 84.3    	//关节4为-90°时的舵机原始角度
#define FSARM_GRIPPER_P0 -0.30		//爪子闭合的角度 关节角度为0
#define FSARM_GRIPPER_P90 93.80 	//爪子完全张开的角度 关节角度为90度 

// 设置关节角度的约束
#define FSARM_JOINT1_MIN -135.0
#define FSARM_JOINT1_MAX 135.0
#define FSARM_JOINT2_MIN -135.0
#define FSARM_JOINT2_MAX 0.0
#define FSARM_JOINT3_MIN -90.0
#define FSARM_JOINT3_MAX 160.0 
#define FSARM_JOINT4_MIN -135.0
#define FSARM_JOINT4_MAX 135.0
#define FSARM_GRIPPER_MIN 0.0
#define FSARM_GRIPPER_MAX 90.0

// HOME 机械臂机械零点的定义
#define FSARM_HOME_X 13.5
#define FSARM_HOME_Y 0
#define FSARM_HOME_Z 6.4
#define FSARM_HOME_PITCH 20.0

// 气泵的配置
#define PUMP_SERVO_ID 0xFE

// 串口舵机的角度控制死区(稳态误差)
#define FSUS_ANGLE_DEAD_BLOCK 0.2
// 在多少ms内，角度误差没有发生变化
#define FSUS_WAIT_TIMEOUT_MS 1000 

// 笛卡尔空间下的点
typedef struct{
    float x;
    float y;
    float z;
}FSARM_POINT3D_T;

// 关节状态
typedef struct{
	float theta1;
	float theta2;
    float theta3;
    float theta4;
	float gripper;
}FSARM_JOINTS_STATE_T;

extern Usart_DataTypeDef *armUsart; // 机械臂的Usart结构体

extern float kJoint2Servo[FSARM_SERVO_NUM]; 		// 机械臂关节角度到舵机原始角度的比例系数
extern float bJoint2Servo[FSARM_SERVO_NUM]; 		// 机械臂关节角度到舵机原始角度的偏移量
extern float jointAngleLowerb[FSARM_SERVO_NUM]; 	// 关节角度下限
extern float jointAngleUpperb[FSARM_SERVO_NUM]; 	// 关节角度上限
extern float curServoAngles[FSARM_SERVO_NUM]; 		// 当前的舵机的角度
extern float nextServoAngles[FSARM_SERVO_NUM];		// 目标舵机角度
extern float servoAngleLowerb[FSARM_SERVO_NUM]; 	// 舵机角度下限
extern float servoAngleUpperb[FSARM_SERVO_NUM]; 	// 舵机角度上限
extern float armJointSpeed; 							// 关节的旋转速度 单位dps

// 初始化机械臂
void FSARM_Init(Usart_DataTypeDef *usart);

// 机械臂关节标定
void FSARM_Calibration(void);

// 设置机械臂的关节范围
void FSARM_SetAngleRange(void);

// 设置是否开启扭矩
void FSARM_SetTorque(bool enable);

// 设置关节为阻尼模式
void FSARM_SetDamping(uint16_t power);

// 单个关节角度是否超出范围
bool FSARM_IsJointLegal(uint8_t jntIdx, float angle);

// 设置关节转速(估计)
void FSARM_SetSpeed(float speed);

// 关节角度转换为舵机角度 
void FSARM_JointAngle2ServoAngle(FSARM_JOINTS_STATE_T jointAngles, FSARM_JOINTS_STATE_T* servoAngles);

// 舵机角度转换为关节角度
void FSARM_ServoAngle2JointAngle(FSARM_JOINTS_STATE_T servoAngles,FSARM_JOINTS_STATE_T* jointAngles);

// 设置舵机的当前角度
void FSARM_SetCurServoAngle(FSARM_JOINTS_STATE_T servoAngles);

// 设置目标舵机角度
void FSARM_SetNextServoAngle(FSARM_JOINTS_STATE_T servoAngles);

// 批量读取舵机原始角度
void FSARM_QueryServoAngle(FSARM_JOINTS_STATE_T* servoAngles);

// 批量设置舵机原始角度
void FSARM_SetServoAngle(FSARM_JOINTS_STATE_T servoAngles);

// 批量设置舵机原始角度(带统一的周期)
void FSARM_SetServoAngle2(FSARM_JOINTS_STATE_T servoAngles, uint16_t interval);

// 设置爪子的角度
void FSARM_SetGripperAngle(float angle, uint16_t interval);

// 批量读取关节的角度
void FSARM_QueryJointAngle(FSARM_JOINTS_STATE_T* jointAngles);

// 批量设置关节的角度
void FSARM_SetJointAngle(FSARM_JOINTS_STATE_T jointAngles);

// 批量设置关节的角度(带统一的周期)
void FSARM_SetJointAngle2(FSARM_JOINTS_STATE_T jointAngles, uint16_t interval);
 
// 机械臂的正向运动学
void FSARM_ForwardKinmatics(FSARM_JOINTS_STATE_T jointAngles, FSARM_POINT3D_T* toolPosi, float* pitch);

// 机械臂逆向运动学
FSARM_STATUS  FSARM_InverseKinematics(FSARM_POINT3D_T toolPosi, float pitch, FSARM_JOINTS_STATE_T* jointAngles);

// 点控(自由轨迹)
FSARM_STATUS FSARM_MoveP2P(float x, float y, float z, float pitch);

// 归回到机械零点
void FSARM_Home(void);

// 等待当个关节旋转到目标角度
void FSARM_Wait(uint8_t jntIdx);

// 等待所有的关节旋转到目标角度
void FSARM_WaitAll(void);

// 计算末端工具的坐标 
void FSARM_GetToolPosi(FSARM_POINT3D_T *toolPosi, float * pitch);

// 更新机械臂的状态 
void FSARM_Update(void);

#endif
