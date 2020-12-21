/*
 * FashionStar四自由度机械臂 STM32 SDK
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
 * 更新时间: 2020/05/26
 */
#include "arm5dof.h"

Usart_DataTypeDef *armUsart; // 机械臂的Usart结构体
float kJoint2Servo[FSARM_SERVO_NUM]; 		// 机械臂关节角度到舵机原始角度的比例系数
float bJoint2Servo[FSARM_SERVO_NUM]; 		// 机械臂关节角度到舵机原始角度的偏移量
float jointAngleLowerb[FSARM_SERVO_NUM]; 	// 关节角度下限
float jointAngleUpperb[FSARM_SERVO_NUM]; 	// 关节角度上限
float curServoAngles[FSARM_SERVO_NUM]; 		// 当前的舵机的角度
float nextServoAngles[FSARM_SERVO_NUM];		// 目标舵机角度
float servoAngleLowerb[FSARM_SERVO_NUM]; 	// 舵机角度下限
float servoAngleUpperb[FSARM_SERVO_NUM]; 	// 舵机角度上限
float armJointSpeed;

// 初始化机械臂
void FSARM_Init(Usart_DataTypeDef *usart){
	armUsart = usart;		// 初始化Usart
	FSARM_Calibration();	// 加载机械臂关节标定的数据
	FSARM_SetAngleRange(); 	// 设置角度范围
	FSARM_SetSpeed(100.0); 	// 设置默认转速
	FSARM_SetGripperAngle(0.0, 1000); // 设置爪子的关节角度
	FSARM_Home(); 			// 回归机械臂机械零点
}

// 机械臂关节标定
void FSARM_Calibration(void){
	// 关节1
	kJoint2Servo[FSARM_JOINT1] = (FSARM_JOINT1_P90 - FSARM_JOINT1_N90) / (90.0 - (-90.0));
	bJoint2Servo[FSARM_JOINT1] = FSARM_JOINT1_P90 - kJoint2Servo[FSARM_JOINT1]*90.0;
	// 关节2
	kJoint2Servo[FSARM_JOINT2] = (FSARM_JOINT2_P0 - FSARM_JOINT2_N90) / (0 - (-90.0));
	bJoint2Servo[FSARM_JOINT2] = FSARM_JOINT2_P0 - kJoint2Servo[FSARM_JOINT2]*0.0;
	// 关节3
	kJoint2Servo[FSARM_JOINT3] = (FSARM_JOINT3_P90 - FSARM_JOINT3_N90) / (90.0 - (-90.0));
	bJoint2Servo[FSARM_JOINT3] = FSARM_JOINT3_P90 - kJoint2Servo[FSARM_JOINT3]*90.0;
	// 关节4
	kJoint2Servo[FSARM_JOINT4] = (FSARM_JOINT4_P90 - FSARM_JOINT4_N90) / (90.0 - (-90.0));
	bJoint2Servo[FSARM_JOINT4] = FSARM_JOINT4_P90 - kJoint2Servo[FSARM_JOINT4]*90.0;
	// 爪子
	kJoint2Servo[FSARM_GRIPPER] = (FSARM_GRIPPER_P90 - FSARM_GRIPPER_P0) / (90.0 - 0.0);
	bJoint2Servo[FSARM_GRIPPER] = FSARM_GRIPPER_P90 - kJoint2Servo[FSARM_GRIPPER]*90.0;
}

// 设置机械臂的关节范围
void FSARM_SetAngleRange(void){
	// 设置关节角度下界
	jointAngleLowerb[FSARM_JOINT1] = FSARM_JOINT1_MIN;
	jointAngleLowerb[FSARM_JOINT2] = FSARM_JOINT2_MIN;
	jointAngleLowerb[FSARM_JOINT3] = FSARM_JOINT3_MIN;
	jointAngleLowerb[FSARM_JOINT4] = FSARM_JOINT4_MIN;
	jointAngleLowerb[FSARM_GRIPPER] = FSARM_GRIPPER_MIN;
	// 设置关节角度上界
	jointAngleUpperb[FSARM_JOINT1] = FSARM_JOINT1_MAX;
	jointAngleUpperb[FSARM_JOINT2] = FSARM_JOINT2_MAX;
	jointAngleUpperb[FSARM_JOINT3] = FSARM_JOINT3_MAX;
	jointAngleUpperb[FSARM_JOINT4] = FSARM_JOINT4_MAX;
	jointAngleUpperb[FSARM_GRIPPER] = FSARM_GRIPPER_MAX;
	// 设置舵机角度下界与下界
	FSARM_JOINTS_STATE_T jointLowerb;
	FSARM_JOINTS_STATE_T jointUpperb;
	FSARM_JOINTS_STATE_T servoLowerb;
	FSARM_JOINTS_STATE_T servoUpperb;
	// 填充关节角度下限
	jointLowerb.theta1 = FSARM_JOINT1_MIN;
	jointLowerb.theta2 = FSARM_JOINT2_MIN;
	jointLowerb.theta3 = FSARM_JOINT3_MIN;
	jointLowerb.theta4 = FSARM_JOINT4_MIN;
	jointLowerb.gripper = FSARM_GRIPPER_MIN;
	// 填充关节角度上限
	jointUpperb.theta1 = FSARM_JOINT1_MAX;
	jointUpperb.theta2 = FSARM_JOINT2_MAX;
	jointUpperb.theta3 = FSARM_JOINT3_MAX;
	jointUpperb.theta4 = FSARM_JOINT4_MAX;
	jointUpperb.gripper = FSARM_GRIPPER_MAX;
	// 关节角度限制转换为舵机角度
	FSARM_JointAngle2ServoAngle(jointLowerb, &servoLowerb);
	FSARM_JointAngle2ServoAngle(jointUpperb, &servoUpperb);
	// 填充ServoAngleLowerb数组
	servoAngleLowerb[FSARM_JOINT1] = servoLowerb.theta1 < servoUpperb.theta1  ? servoLowerb.theta1 : servoUpperb.theta1;
	servoAngleLowerb[FSARM_JOINT2] = servoLowerb.theta2 < servoUpperb.theta2  ? servoLowerb.theta2 : servoUpperb.theta2;
	servoAngleLowerb[FSARM_JOINT3] = servoLowerb.theta3 < servoUpperb.theta3  ? servoLowerb.theta3 : servoUpperb.theta3;
	servoAngleLowerb[FSARM_JOINT4] = servoLowerb.theta4 < servoUpperb.theta4  ? servoLowerb.theta4 : servoUpperb.theta4;
	servoAngleLowerb[FSARM_GRIPPER] = servoLowerb.gripper < servoUpperb.gripper  ? servoLowerb.gripper : servoUpperb.gripper;
	// 填充ServoAngleUpperb数组
	servoAngleUpperb[FSARM_JOINT1] = servoLowerb.theta1 < servoUpperb.theta1  ? servoUpperb.theta1 : servoLowerb.theta1;
	servoAngleUpperb[FSARM_JOINT2] = servoLowerb.theta2 < servoUpperb.theta2  ? servoUpperb.theta2 : servoLowerb.theta2;
	servoAngleUpperb[FSARM_JOINT3] = servoLowerb.theta3 < servoUpperb.theta3  ? servoUpperb.theta3 : servoLowerb.theta3;
	servoAngleUpperb[FSARM_JOINT4] = servoLowerb.theta4 < servoUpperb.theta4  ? servoUpperb.theta4 : servoLowerb.theta4;
	servoAngleUpperb[FSARM_GRIPPER] = servoLowerb.gripper < servoUpperb.gripper  ? servoUpperb.gripper : servoLowerb.gripper;
}

// 设置是否开启扭矩
void FSARM_SetTorque(bool enable){
	if(!enable){
		FSARM_SetDamping(0);
	}else{
		FSARM_JOINTS_STATE_T thetas;
		FSARM_QueryJointAngle(&thetas);
		FSARM_SetJointAngle(thetas);
	}
}

// 设置关节为阻尼模式
void FSARM_SetDamping(uint16_t power){
	FSUS_DampingMode(armUsart, FSARM_JOINT1, power);
	FSUS_DampingMode(armUsart, FSARM_JOINT2, power);
	FSUS_DampingMode(armUsart, FSARM_JOINT3, power);
	FSUS_DampingMode(armUsart, FSARM_JOINT4, power);
	FSUS_DampingMode(armUsart, FSARM_GRIPPER, power);
}

// 单个关节角度是否超出范围
bool FSARM_IsJointLegal(uint8_t jntIdx, float angle){
	return angle >= jointAngleLowerb[jntIdx] && angle <= jointAngleUpperb[jntIdx];
}

// 设置关节转速(估计)
void FSARM_SetSpeed(float speed){
	armJointSpeed = speed;
}

// 关节角度转换为舵机角度
void FSARM_JointAngle2ServoAngle(FSARM_JOINTS_STATE_T jointAngles, FSARM_JOINTS_STATE_T* servoAngles){
	servoAngles->theta1 =  kJoint2Servo[FSARM_JOINT1] * jointAngles.theta1 + bJoint2Servo[FSARM_JOINT1];
	servoAngles->theta2 =  kJoint2Servo[FSARM_JOINT2] * jointAngles.theta2 + bJoint2Servo[FSARM_JOINT2];
	servoAngles->theta3 =  kJoint2Servo[FSARM_JOINT3] * jointAngles.theta3 + bJoint2Servo[FSARM_JOINT3];
	servoAngles->theta4 =  kJoint2Servo[FSARM_JOINT4] * jointAngles.theta4 + bJoint2Servo[FSARM_JOINT4];
	servoAngles->gripper = kJoint2Servo[FSARM_GRIPPER] * jointAngles.gripper + bJoint2Servo[FSARM_GRIPPER];
}

// 舵机角度转换为关节角度
void FSARM_ServoAngle2JointAngle(FSARM_JOINTS_STATE_T servoAngles,FSARM_JOINTS_STATE_T* jointAngles){
	jointAngles->theta1 = (servoAngles.theta1 - bJoint2Servo[FSARM_JOINT1]) / kJoint2Servo[FSARM_JOINT1];
	jointAngles->theta2 = (servoAngles.theta2 - bJoint2Servo[FSARM_JOINT2]) / kJoint2Servo[FSARM_JOINT2];
	jointAngles->theta3 = (servoAngles.theta3 - bJoint2Servo[FSARM_JOINT3]) / kJoint2Servo[FSARM_JOINT3];
	jointAngles->theta4 = (servoAngles.theta4 - bJoint2Servo[FSARM_JOINT4]) / kJoint2Servo[FSARM_JOINT4];
	jointAngles->gripper = (servoAngles.gripper - bJoint2Servo[FSARM_GRIPPER]) / kJoint2Servo[FSARM_GRIPPER];
}

// 设置舵机的当前角度
void FSARM_SetCurServoAngle(FSARM_JOINTS_STATE_T servoAngles){
	curServoAngles[FSARM_JOINT1] = servoAngles.theta1;
	curServoAngles[FSARM_JOINT2] = servoAngles.theta2;
	curServoAngles[FSARM_JOINT3] = servoAngles.theta3;
	curServoAngles[FSARM_JOINT4] = servoAngles.theta4;
	curServoAngles[FSARM_GRIPPER] = servoAngles.gripper;
}

// 设置舵机的目标角度
void FSARM_SetNextServoAngle(FSARM_JOINTS_STATE_T servoAngles){
	nextServoAngles[FSARM_JOINT1] = servoAngles.theta1;
	nextServoAngles[FSARM_JOINT2] = servoAngles.theta2;
	nextServoAngles[FSARM_JOINT3] = servoAngles.theta3;
	nextServoAngles[FSARM_JOINT4] = servoAngles.theta4;
	nextServoAngles[FSARM_GRIPPER] = servoAngles.gripper;
}

// 批量读取舵机原始角度
void FSARM_QueryServoAngle(FSARM_JOINTS_STATE_T* servoAngles){
	FSUS_QueryServoAngle(armUsart, FSARM_JOINT1, &(servoAngles->theta1));
	FSUS_QueryServoAngle(armUsart, FSARM_JOINT2, &(servoAngles->theta2));
	FSUS_QueryServoAngle(armUsart, FSARM_JOINT3, &(servoAngles->theta3));
	FSUS_QueryServoAngle(armUsart, FSARM_JOINT4, &(servoAngles->theta4));
	FSUS_QueryServoAngle(armUsart, FSARM_GRIPPER,&(servoAngles->gripper));
	// 更新当前舵机角度
	FSARM_SetCurServoAngle(*servoAngles);
}

// 批量设置舵机原始角度
void FSARM_SetServoAngle(FSARM_JOINTS_STATE_T servoAngles){
	FSARM_JOINTS_STATE_T curServoAngles;
	uint16_t interval;
	FSARM_QueryServoAngle(&curServoAngles);
	// 发送舵机角度控制指令
	interval = (uint16_t)fabs((servoAngles.theta1 - curServoAngles.theta1)/armJointSpeed*1000);
	FSUS_SetServoAngle(armUsart, FSARM_JOINT1, servoAngles.theta1, interval, 0, false);	
	interval = (uint16_t)fabs((servoAngles.theta2 - curServoAngles.theta2)/armJointSpeed*1000);
	FSUS_SetServoAngle(armUsart, FSARM_JOINT2, servoAngles.theta2, interval, 0, false);	
	interval = (uint16_t)fabs((servoAngles.theta3 - curServoAngles.theta3)/armJointSpeed*1000);
	FSUS_SetServoAngle(armUsart, FSARM_JOINT3, servoAngles.theta3, interval, 0, false);
	interval = (uint16_t)fabs((servoAngles.theta4 - curServoAngles.theta4)/armJointSpeed*1000);
	FSUS_SetServoAngle(armUsart, FSARM_JOINT4, servoAngles.theta4, interval, 0, false);
	interval = (uint16_t)fabs((servoAngles.gripper - curServoAngles.gripper)/armJointSpeed*1000);
	FSUS_SetServoAngle(armUsart, FSARM_GRIPPER, servoAngles.gripper, interval, 0, false);
	// 设置目标角度
	FSARM_SetNextServoAngle(servoAngles);
}

// 批量设置舵机原始角度(带统一的周期)
void FSARM_SetServoAngle2(FSARM_JOINTS_STATE_T servoAngles, uint16_t interval){
	FSARM_JOINTS_STATE_T curServoAngles;
	FSARM_QueryServoAngle(&curServoAngles);
	// 发送舵机角度控制指
	FSUS_SetServoAngle(armUsart, FSARM_JOINT1, servoAngles.theta1, interval, 0, false);	
	FSUS_SetServoAngle(armUsart, FSARM_JOINT2, servoAngles.theta2, interval, 0, false);	
	FSUS_SetServoAngle(armUsart, FSARM_JOINT3, servoAngles.theta3, interval, 0, false);
	FSUS_SetServoAngle(armUsart, FSARM_JOINT4, servoAngles.theta4, interval, 0, false);
	FSUS_SetServoAngle(armUsart, FSARM_GRIPPER, servoAngles.gripper, interval, 0, false);
	// 更新舵机目标角度值
	FSARM_SetNextServoAngle(servoAngles);
}

// 批量读取关节的角度
void FSARM_QueryJointAngle(FSARM_JOINTS_STATE_T* jointAngles){
	// 查询舵机原始角度
	FSARM_JOINTS_STATE_T curServoAngles;
	FSARM_QueryServoAngle(&curServoAngles);
	// 映射为关节角度
	FSARM_ServoAngle2JointAngle(curServoAngles, jointAngles);
}

// 批量设置关节的角度
void FSARM_SetJointAngle(FSARM_JOINTS_STATE_T jointAngles){
	FSARM_JOINTS_STATE_T servoAngles;
	// 映射为舵机角度
	FSARM_JointAngle2ServoAngle(jointAngles, &servoAngles);
	// 设置舵机角度
	FSARM_SetServoAngle(servoAngles);
}

// 批量设置关节的角度(带统一的周期)
void FSARM_SetJointAngle2(FSARM_JOINTS_STATE_T jointAngles, uint16_t interval){
	FSARM_JOINTS_STATE_T servoAngles;
	// 映射为舵机角度
	FSARM_JointAngle2ServoAngle(jointAngles, &servoAngles);
	// 设置舵机角度
	FSARM_SetServoAngle2(servoAngles, interval);
}

// 设置爪子的角度
void FSARM_SetGripperAngle(float angle, uint16_t interval){
	float servoAngle; // 爪子对应的舵机的角度
	servoAngle = kJoint2Servo[FSARM_GRIPPER] * angle + bJoint2Servo[FSARM_GRIPPER]; // 爪子的角度转换为舵机的角度
	// 设置舵机的角度
	FSUS_SetServoAngle(armUsart, FSARM_GRIPPER, servoAngle, interval, 0, false); 	// 设置爪子的角度
	nextServoAngles[FSARM_GRIPPER] = servoAngle;
}

// 机械臂的正向运动学
void FSARM_ForwardKinmatics(FSARM_JOINTS_STATE_T jointAngles, FSARM_POINT3D_T* toolPosi, float* pitch){
	// 关节角度->弧度
	float theta1 = radians(jointAngles.theta1);
	float theta2 = radians(jointAngles.theta2);
	float theta3 = radians(jointAngles.theta3);
	float theta4 = radians(jointAngles.theta4);
	*pitch = jointAngles.theta2 + jointAngles.theta3 + jointAngles.theta4;
	// 计算腕关节的坐标
	float d = FSARM_LINK2*cos(theta2)+FSARM_LINK3*cos(theta2+theta3)+FSARM_LINK4*cos(theta1+theta2+theta3);
    toolPosi->x = cos(theta1) * d;
    toolPosi->y = sin(theta1) * d;
    toolPosi->z = -FSARM_LINK2*sin(theta2)-FSARM_LINK3*sin(theta2+theta3)-FSARM_LINK4*sin(theta2+theta3+theta4);
}

// 机械臂逆向运动学
FSARM_STATUS  FSARM_InverseKinematics(FSARM_POINT3D_T toolPosi, float pitch, FSARM_JOINTS_STATE_T* jointAngles){
	// 关节弧度
    float theta1 = 0.0;
    float theta2 = 0.0;
    float theta3 = 0.0;
    float theta4 = 0.0;
	
	FSARM_POINT3D_T wristPosi; // 腕关节坐标
	
	// 根据腕关节原点距离机械臂基坐标系的直线距离
    float disO2Tool = sqrt(pow(toolPosi.x,2) + pow(toolPosi.y, 2) + pow(toolPosi.z, 2));
    if (disO2Tool > (FSARM_LINK2+FSARM_LINK3+FSARM_LINK4)){
        return FSARM_STATUS_TOOLPOSI_TOO_FAR;
    }
	// 判断腕关节的原点是否在机械臂坐标系的Z轴上
    if (toolPosi.x == 0 && toolPosi.y == 0){
        // 查询关节1的角度, 让theta1保持跟原来相同
		FSUS_QueryServoAngle(armUsart, FSARM_JOINT1, &(jointAngles->theta1));
		jointAngles->theta1 = (jointAngles->theta1 - bJoint2Servo[FSARM_JOINT1]) / kJoint2Servo[FSARM_JOINT1];
        theta1 = radians(jointAngles->theta1);
    }else{
        // 求解theta1
        theta1 = atan2(toolPosi.y, toolPosi.x);
        jointAngles->theta1 = degrees(theta1);
        // 判断theta1是否合法
        if (!FSARM_IsJointLegal(FSARM_JOINT1, jointAngles->theta1)){
            return FSARM_STATUS_JOINT1_OUTRANGE;
        }
    }
	// 俯仰角, 角度转弧度
	float pitch_rad = radians(pitch);
	// 计算腕关节的位置
	wristPosi.x = toolPosi.x - FSARM_LINK4*cos(pitch_rad)*cos(theta1);
    wristPosi.y = toolPosi.y - FSARM_LINK4*cos(pitch_rad)*sin(theta1);
    wristPosi.z = toolPosi.z + FSARM_LINK4*sin(pitch_rad);
	
	// 计算theta3
    float b;
    if(cos(theta1) !=0){
        b = wristPosi.x / cos(theta1);
    }else{
        b = wristPosi.y / sin(theta1);
    }
    float cos_theta3 = (pow(wristPosi.z, 2)+pow(b,2) - pow(FSARM_LINK2,2) - pow(FSARM_LINK3, 2))/(2*FSARM_LINK2*FSARM_LINK3);
    float sin_theta3 = sqrt(1 - pow(cos_theta3, 2));
    theta3 = atan2(sin_theta3, cos_theta3);
    jointAngles->theta3 = degrees(theta3);
    if(!FSARM_IsJointLegal(FSARM_JOINT3, jointAngles->theta3)){
        return FSARM_STATUS_JOINT3_OUTRANGE;
    }
    // 计算theta2
    float k1 = FSARM_LINK2 + FSARM_LINK3*cos(theta3);
    float k2 = FSARM_LINK3 * sin(theta3);
    float r = sqrt(pow(k1, 2) + pow(k2, 2));
    theta2 = atan2(-wristPosi.z/r, b/r) - atan2(k2/r, k1/r);
    jointAngles->theta2 = degrees(theta2);
    if(!FSARM_IsJointLegal(FSARM_JOINT2, jointAngles->theta2)){
        return FSARM_STATUS_JOINT2_OUTRANGE;
    }
    // 计算theta4
    theta4 = pitch_rad - (theta2 + theta3);
    jointAngles->theta4 = degrees(theta4);
    if(!FSARM_IsJointLegal(FSARM_JOINT4, jointAngles->theta4)){
        return FSARM_STATUS_JOINT4_OUTRANGE;
    }
    // 成功完成求解
    return FSARM_STATUS_SUCCESS;
}


// 点控(自由轨迹)
FSARM_STATUS FSARM_MoveP2P(float x, float y, float z, float pitch){
	FSARM_JOINTS_STATE_T jointAngles;
	FSARM_POINT3D_T toolPosi;
	toolPosi.x = x;
	toolPosi.y = y;
	toolPosi.z = z;
    FSARM_STATUS status = FSARM_InverseKinematics(toolPosi, pitch, &jointAngles); // 逆向运动学
    if(status == FSARM_STATUS_SUCCESS){
		
		FSARM_JOINTS_STATE_T nextServoAngle; // 设置目标舵机角度
        FSARM_JOINTS_STATE_T curServoAngles; // 设置舵机的角度
		uint16_t interval;					 // 时间间隔
		
		FSARM_JointAngle2ServoAngle(jointAngles, &nextServoAngle); // 目标角度转化为目标的舵机角度
		FSARM_QueryServoAngle(&curServoAngles);	// 查询当前的舵机角度
		
		// 发送舵机角度控制指令
		interval = (uint16_t)fabs((nextServoAngle.theta1 - curServoAngles.theta1)/armJointSpeed*1000);
		FSUS_SetServoAngle(armUsart, FSARM_JOINT1, nextServoAngle.theta1, interval, 0, false);	
		interval = (uint16_t)fabs((nextServoAngle.theta2 - curServoAngles.theta2)/armJointSpeed*1000);
		FSUS_SetServoAngle(armUsart, FSARM_JOINT2, nextServoAngle.theta2, interval, 0, false);	
		interval = (uint16_t)fabs((nextServoAngle.theta3 - curServoAngles.theta3)/armJointSpeed*1000);
		FSUS_SetServoAngle(armUsart, FSARM_JOINT3, nextServoAngle.theta3, interval, 0, false);
		interval = (uint16_t)fabs((nextServoAngle.theta4 - curServoAngles.theta4)/armJointSpeed*1000);
		FSUS_SetServoAngle(armUsart, FSARM_JOINT4, nextServoAngle.theta4, interval, 0, false);
		// 设置目标角度
		nextServoAngles[FSARM_JOINT1] = nextServoAngle.theta1;
		nextServoAngles[FSARM_JOINT2] = nextServoAngle.theta2;
		nextServoAngles[FSARM_JOINT3] = nextServoAngle.theta3;
		nextServoAngles[FSARM_JOINT4] = nextServoAngle.theta4;
    }
    return status;
}

// 归回到机械零点
void FSARM_Home(void){
	// 自由轨迹 运动到机械零点
	FSARM_MoveP2P(FSARM_HOME_X, FSARM_HOME_Y, FSARM_HOME_Z, FSARM_HOME_PITCH);
	FSARM_WaitAll();
}

// 更新机械臂的状态 
void FSARM_Update(void){
	// 更新舵机的角度
	FSARM_JOINTS_STATE_T servoAngles;
	FSARM_QueryServoAngle(&servoAngles);
	FSARM_SetCurServoAngle(servoAngles);
}

// 等待当个关节旋转到目标角度
void FSARM_Wait(uint8_t jntIdx){
	// 角度误差
	float dAngle = fabs(curServoAngles[jntIdx] - nextServoAngles[jntIdx]);
	TimeTypedef tStart;
	while(true){
		// printf("wait joint: %d curAngle: %.1f nextAngle: %.1f\r\n", jntIdx, curServoAngles[jntIdx], nextServoAngles[jntIdx]);
		FSARM_Update(); // 更新机械臂的状态
		if(fabs(curServoAngles[jntIdx] - nextServoAngles[jntIdx]) <= FSUS_ANGLE_DEAD_BLOCK){
			break;
		}else{
			// 判断是否发生卡死的情况
			// 误差差值保持不变
			if(fabs(dAngle) < 5.0 && fabs(dAngle - fabs(curServoAngles[jntIdx] - nextServoAngles[jntIdx])) <= 1.0){
				// 判断是否超时
				if(SysTick_Millis() - tStart >= FSUS_WAIT_TIMEOUT_MS){
					break;
				}
			}else{
				// 更新角度误差
				dAngle = fabs(dAngle - fabs(curServoAngles[jntIdx] - nextServoAngles[jntIdx]));
				// 开始倒计时
				// SysTick_CountdownBegin(FSUS_WAIT_TIMEOUT_MS);
				tStart = SysTick_Millis();
			}
		}
	}
}

// 等待所有的关节旋转到目标角度
void FSARM_WaitAll(void){
	for(uint8_t srvIdx=0; srvIdx < FSARM_SERVO_NUM; srvIdx++){
		FSARM_Wait(srvIdx);
	}
}

// 计算末端工具的坐标 
void FSARM_GetToolPosi(FSARM_POINT3D_T *toolPosi, float * pitch){
	FSARM_JOINTS_STATE_T curJointAngles; // 当前关节的角度
	FSARM_QueryJointAngle(&curJointAngles); // 查询关节角度
	FSARM_ForwardKinmatics(curJointAngles, toolPosi, pitch);// 正向运动学
}
