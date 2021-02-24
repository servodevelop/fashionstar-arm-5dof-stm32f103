/*
 * FashionStar�����ɶȻ�е�� STM32 SDK
 * --------------------------
 * ����: ����|Kyle
 * ��kyle.xing@fashionstar.com.hk.com
 * ����ʱ��: 2020/05/26
 */
#include "arm5dof.h"

Usart_DataTypeDef *armUsart; // ��е�۵�Usart�ṹ��
float kJoint2Servo[FSARM_SERVO_NUM]; 		// ��е�۹ؽڽǶȵ����ԭʼ�Ƕȵı���ϵ��
float bJoint2Servo[FSARM_SERVO_NUM]; 		// ��е�۹ؽڽǶȵ����ԭʼ�Ƕȵ�ƫ����
float jointAngleLowerb[FSARM_SERVO_NUM]; 	// �ؽڽǶ�����
float jointAngleUpperb[FSARM_SERVO_NUM]; 	// �ؽڽǶ�����
float curServoAngles[FSARM_SERVO_NUM]; 		// ��ǰ�Ķ���ĽǶ�
float nextServoAngles[FSARM_SERVO_NUM];		// Ŀ�����Ƕ�
float servoAngleLowerb[FSARM_SERVO_NUM]; 	// ����Ƕ�����
float servoAngleUpperb[FSARM_SERVO_NUM]; 	// ����Ƕ�����
float armJointSpeed;

// ��ʼ����е��
void FSARM_Init(Usart_DataTypeDef *usart){
	armUsart = usart;		// ��ʼ��Usart
	FSARM_Calibration();	// ���ػ�е�۹ؽڱ궨������
	FSARM_SetAngleRange(); 	// ���ýǶȷ�Χ
	FSARM_SetSpeed(100.0); 	// ����Ĭ��ת��
	FSARM_SetGripperAngle(0.0, 1000); // ����צ�ӵĹؽڽǶ�
	FSARM_Home(); 			// �ع��е�ۻ�е���
}

// ��е�۹ؽڱ궨
void FSARM_Calibration(void){
	// �ؽ�1
	kJoint2Servo[FSARM_JOINT1] = (FSARM_JOINT1_P90 - FSARM_JOINT1_N90) / (90.0 - (-90.0));
	bJoint2Servo[FSARM_JOINT1] = FSARM_JOINT1_P90 - kJoint2Servo[FSARM_JOINT1]*90.0;
	// �ؽ�2
	kJoint2Servo[FSARM_JOINT2] = (FSARM_JOINT2_P0 - FSARM_JOINT2_N90) / (0 - (-90.0));
	bJoint2Servo[FSARM_JOINT2] = FSARM_JOINT2_P0 - kJoint2Servo[FSARM_JOINT2]*0.0;
	// �ؽ�3
	kJoint2Servo[FSARM_JOINT3] = (FSARM_JOINT3_P90 - FSARM_JOINT3_N90) / (90.0 - (-90.0));
	bJoint2Servo[FSARM_JOINT3] = FSARM_JOINT3_P90 - kJoint2Servo[FSARM_JOINT3]*90.0;
	// �ؽ�4
	kJoint2Servo[FSARM_JOINT4] = (FSARM_JOINT4_P90 - FSARM_JOINT4_N90) / (90.0 - (-90.0));
	bJoint2Servo[FSARM_JOINT4] = FSARM_JOINT4_P90 - kJoint2Servo[FSARM_JOINT4]*90.0;
	// צ��
	kJoint2Servo[FSARM_GRIPPER] = (FSARM_GRIPPER_P90 - FSARM_GRIPPER_P0) / (90.0 - 0.0);
	bJoint2Servo[FSARM_GRIPPER] = FSARM_GRIPPER_P90 - kJoint2Servo[FSARM_GRIPPER]*90.0;
}

// ���û�е�۵Ĺؽڷ�Χ
void FSARM_SetAngleRange(void){
	// ���ùؽڽǶ��½�
	jointAngleLowerb[FSARM_JOINT1] = FSARM_JOINT1_MIN;
	jointAngleLowerb[FSARM_JOINT2] = FSARM_JOINT2_MIN;
	jointAngleLowerb[FSARM_JOINT3] = FSARM_JOINT3_MIN;
	jointAngleLowerb[FSARM_JOINT4] = FSARM_JOINT4_MIN;
	jointAngleLowerb[FSARM_GRIPPER] = FSARM_GRIPPER_MIN;
	// ���ùؽڽǶ��Ͻ�
	jointAngleUpperb[FSARM_JOINT1] = FSARM_JOINT1_MAX;
	jointAngleUpperb[FSARM_JOINT2] = FSARM_JOINT2_MAX;
	jointAngleUpperb[FSARM_JOINT3] = FSARM_JOINT3_MAX;
	jointAngleUpperb[FSARM_JOINT4] = FSARM_JOINT4_MAX;
	jointAngleUpperb[FSARM_GRIPPER] = FSARM_GRIPPER_MAX;
	// ���ö���Ƕ��½����½�
	FSARM_JOINTS_STATE_T jointLowerb;
	FSARM_JOINTS_STATE_T jointUpperb;
	FSARM_JOINTS_STATE_T servoLowerb;
	FSARM_JOINTS_STATE_T servoUpperb;
	// ���ؽڽǶ�����
	jointLowerb.theta1 = FSARM_JOINT1_MIN;
	jointLowerb.theta2 = FSARM_JOINT2_MIN;
	jointLowerb.theta3 = FSARM_JOINT3_MIN;
	jointLowerb.theta4 = FSARM_JOINT4_MIN;
	jointLowerb.gripper = FSARM_GRIPPER_MIN;
	// ���ؽڽǶ�����
	jointUpperb.theta1 = FSARM_JOINT1_MAX;
	jointUpperb.theta2 = FSARM_JOINT2_MAX;
	jointUpperb.theta3 = FSARM_JOINT3_MAX;
	jointUpperb.theta4 = FSARM_JOINT4_MAX;
	jointUpperb.gripper = FSARM_GRIPPER_MAX;
	// �ؽڽǶ�����ת��Ϊ����Ƕ�
	FSARM_JointAngle2ServoAngle(jointLowerb, &servoLowerb);
	FSARM_JointAngle2ServoAngle(jointUpperb, &servoUpperb);
	// ���ServoAngleLowerb����
	servoAngleLowerb[FSARM_JOINT1] = servoLowerb.theta1 < servoUpperb.theta1  ? servoLowerb.theta1 : servoUpperb.theta1;
	servoAngleLowerb[FSARM_JOINT2] = servoLowerb.theta2 < servoUpperb.theta2  ? servoLowerb.theta2 : servoUpperb.theta2;
	servoAngleLowerb[FSARM_JOINT3] = servoLowerb.theta3 < servoUpperb.theta3  ? servoLowerb.theta3 : servoUpperb.theta3;
	servoAngleLowerb[FSARM_JOINT4] = servoLowerb.theta4 < servoUpperb.theta4  ? servoLowerb.theta4 : servoUpperb.theta4;
	servoAngleLowerb[FSARM_GRIPPER] = servoLowerb.gripper < servoUpperb.gripper  ? servoLowerb.gripper : servoUpperb.gripper;
	// ���ServoAngleUpperb����
	servoAngleUpperb[FSARM_JOINT1] = servoLowerb.theta1 < servoUpperb.theta1  ? servoUpperb.theta1 : servoLowerb.theta1;
	servoAngleUpperb[FSARM_JOINT2] = servoLowerb.theta2 < servoUpperb.theta2  ? servoUpperb.theta2 : servoLowerb.theta2;
	servoAngleUpperb[FSARM_JOINT3] = servoLowerb.theta3 < servoUpperb.theta3  ? servoUpperb.theta3 : servoLowerb.theta3;
	servoAngleUpperb[FSARM_JOINT4] = servoLowerb.theta4 < servoUpperb.theta4  ? servoUpperb.theta4 : servoLowerb.theta4;
	servoAngleUpperb[FSARM_GRIPPER] = servoLowerb.gripper < servoUpperb.gripper  ? servoUpperb.gripper : servoLowerb.gripper;
}

// �����Ƿ���Ť��
void FSARM_SetTorque(bool enable){
	if(!enable){
		FSARM_SetDamping(0);
	}else{
		FSARM_JOINTS_STATE_T thetas;
		FSARM_QueryJointAngle(&thetas);
		FSARM_SetJointAngle(thetas);
	}
}

// ���ùؽ�Ϊ����ģʽ
void FSARM_SetDamping(uint16_t power){
	FSUS_DampingMode(armUsart, FSARM_JOINT1, power);
	FSUS_DampingMode(armUsart, FSARM_JOINT2, power);
	FSUS_DampingMode(armUsart, FSARM_JOINT3, power);
	FSUS_DampingMode(armUsart, FSARM_JOINT4, power);
	FSUS_DampingMode(armUsart, FSARM_GRIPPER, power);
}

// �����ؽڽǶ��Ƿ񳬳���Χ
bool FSARM_IsJointLegal(uint8_t jntIdx, float angle){
	return angle >= jointAngleLowerb[jntIdx] && angle <= jointAngleUpperb[jntIdx];
}

// ���ùؽ�ת��(����)
void FSARM_SetSpeed(float speed){
	armJointSpeed = speed;
}

// �ؽڽǶ�ת��Ϊ����Ƕ�
void FSARM_JointAngle2ServoAngle(FSARM_JOINTS_STATE_T jointAngles, FSARM_JOINTS_STATE_T* servoAngles){
	servoAngles->theta1 =  kJoint2Servo[FSARM_JOINT1] * jointAngles.theta1 + bJoint2Servo[FSARM_JOINT1];
	servoAngles->theta2 =  kJoint2Servo[FSARM_JOINT2] * jointAngles.theta2 + bJoint2Servo[FSARM_JOINT2];
	servoAngles->theta3 =  kJoint2Servo[FSARM_JOINT3] * jointAngles.theta3 + bJoint2Servo[FSARM_JOINT3];
	servoAngles->theta4 =  kJoint2Servo[FSARM_JOINT4] * jointAngles.theta4 + bJoint2Servo[FSARM_JOINT4];
	servoAngles->gripper = kJoint2Servo[FSARM_GRIPPER] * jointAngles.gripper + bJoint2Servo[FSARM_GRIPPER];
}

// ����Ƕ�ת��Ϊ�ؽڽǶ�
void FSARM_ServoAngle2JointAngle(FSARM_JOINTS_STATE_T servoAngles,FSARM_JOINTS_STATE_T* jointAngles){
	jointAngles->theta1 = (servoAngles.theta1 - bJoint2Servo[FSARM_JOINT1]) / kJoint2Servo[FSARM_JOINT1];
	jointAngles->theta2 = (servoAngles.theta2 - bJoint2Servo[FSARM_JOINT2]) / kJoint2Servo[FSARM_JOINT2];
	jointAngles->theta3 = (servoAngles.theta3 - bJoint2Servo[FSARM_JOINT3]) / kJoint2Servo[FSARM_JOINT3];
	jointAngles->theta4 = (servoAngles.theta4 - bJoint2Servo[FSARM_JOINT4]) / kJoint2Servo[FSARM_JOINT4];
	jointAngles->gripper = (servoAngles.gripper - bJoint2Servo[FSARM_GRIPPER]) / kJoint2Servo[FSARM_GRIPPER];
}

// ���ö���ĵ�ǰ�Ƕ�
void FSARM_SetCurServoAngle(FSARM_JOINTS_STATE_T servoAngles){
	curServoAngles[FSARM_JOINT1] = servoAngles.theta1;
	curServoAngles[FSARM_JOINT2] = servoAngles.theta2;
	curServoAngles[FSARM_JOINT3] = servoAngles.theta3;
	curServoAngles[FSARM_JOINT4] = servoAngles.theta4;
	curServoAngles[FSARM_GRIPPER] = servoAngles.gripper;
}

// ���ö����Ŀ��Ƕ�
void FSARM_SetNextServoAngle(FSARM_JOINTS_STATE_T servoAngles){
	nextServoAngles[FSARM_JOINT1] = servoAngles.theta1;
	nextServoAngles[FSARM_JOINT2] = servoAngles.theta2;
	nextServoAngles[FSARM_JOINT3] = servoAngles.theta3;
	nextServoAngles[FSARM_JOINT4] = servoAngles.theta4;
	nextServoAngles[FSARM_GRIPPER] = servoAngles.gripper;
}

// ������ȡ���ԭʼ�Ƕ�
void FSARM_QueryServoAngle(FSARM_JOINTS_STATE_T* servoAngles){
	FSUS_QueryServoAngle(armUsart, FSARM_JOINT1, &(servoAngles->theta1));
	FSUS_QueryServoAngle(armUsart, FSARM_JOINT2, &(servoAngles->theta2));
	FSUS_QueryServoAngle(armUsart, FSARM_JOINT3, &(servoAngles->theta3));
	FSUS_QueryServoAngle(armUsart, FSARM_JOINT4, &(servoAngles->theta4));
	FSUS_QueryServoAngle(armUsart, FSARM_GRIPPER,&(servoAngles->gripper));
	// ���µ�ǰ����Ƕ�
	FSARM_SetCurServoAngle(*servoAngles);
}

// �������ö��ԭʼ�Ƕ�
void FSARM_SetServoAngle(FSARM_JOINTS_STATE_T servoAngles){
	FSARM_JOINTS_STATE_T curServoAngles;
	uint16_t interval;
	FSARM_QueryServoAngle(&curServoAngles);
	// ���Ͷ���Ƕȿ���ָ��
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
	// ����Ŀ��Ƕ�
	FSARM_SetNextServoAngle(servoAngles);
}

// �������ö��ԭʼ�Ƕ�(��ͳһ������)
void FSARM_SetServoAngle2(FSARM_JOINTS_STATE_T servoAngles, uint16_t interval){
	FSARM_JOINTS_STATE_T curServoAngles;
	FSARM_QueryServoAngle(&curServoAngles);
	// ���Ͷ���Ƕȿ���ָ
	FSUS_SetServoAngle(armUsart, FSARM_JOINT1, servoAngles.theta1, interval, 0, false);	
	FSUS_SetServoAngle(armUsart, FSARM_JOINT2, servoAngles.theta2, interval, 0, false);	
	FSUS_SetServoAngle(armUsart, FSARM_JOINT3, servoAngles.theta3, interval, 0, false);
	FSUS_SetServoAngle(armUsart, FSARM_JOINT4, servoAngles.theta4, interval, 0, false);
	FSUS_SetServoAngle(armUsart, FSARM_GRIPPER, servoAngles.gripper, interval, 0, false);
	// ���¶��Ŀ��Ƕ�ֵ
	FSARM_SetNextServoAngle(servoAngles);
}

// ������ȡ�ؽڵĽǶ�
void FSARM_QueryJointAngle(FSARM_JOINTS_STATE_T* jointAngles){
	// ��ѯ���ԭʼ�Ƕ�
	FSARM_JOINTS_STATE_T curServoAngles;
	FSARM_QueryServoAngle(&curServoAngles);
	// ӳ��Ϊ�ؽڽǶ�
	FSARM_ServoAngle2JointAngle(curServoAngles, jointAngles);
}

// �������ùؽڵĽǶ�
void FSARM_SetJointAngle(FSARM_JOINTS_STATE_T jointAngles){
	FSARM_JOINTS_STATE_T servoAngles;
	// ӳ��Ϊ����Ƕ�
	FSARM_JointAngle2ServoAngle(jointAngles, &servoAngles);
	// ���ö���Ƕ�
	FSARM_SetServoAngle(servoAngles);
}

// �������ùؽڵĽǶ�(��ͳһ������)
void FSARM_SetJointAngle2(FSARM_JOINTS_STATE_T jointAngles, uint16_t interval){
	FSARM_JOINTS_STATE_T servoAngles;
	// ӳ��Ϊ����Ƕ�
	FSARM_JointAngle2ServoAngle(jointAngles, &servoAngles);
	// ���ö���Ƕ�
	FSARM_SetServoAngle2(servoAngles, interval);
}

// ����צ�ӵĽǶ�
void FSARM_SetGripperAngle(float angle, uint16_t interval){
	float servoAngle; // צ�Ӷ�Ӧ�Ķ���ĽǶ�
	servoAngle = kJoint2Servo[FSARM_GRIPPER] * angle + bJoint2Servo[FSARM_GRIPPER]; // צ�ӵĽǶ�ת��Ϊ����ĽǶ�
	// ���ö���ĽǶ�
	FSUS_SetServoAngle(armUsart, FSARM_GRIPPER, servoAngle, interval, 0, false); 	// ����צ�ӵĽǶ�
	nextServoAngles[FSARM_GRIPPER] = servoAngle;
}

// ��е�۵������˶�ѧ
void FSARM_ForwardKinmatics(FSARM_JOINTS_STATE_T jointAngles, FSARM_POINT3D_T* toolPosi, float* pitch){
	// �ؽڽǶ�->����
	float theta1 = radians(jointAngles.theta1);
	float theta2 = radians(jointAngles.theta2);
	float theta3 = radians(jointAngles.theta3);
	float theta4 = radians(jointAngles.theta4);
	*pitch = jointAngles.theta2 + jointAngles.theta3 + jointAngles.theta4;
	// ������ؽڵ�����
	float d = FSARM_LINK2*cos(theta2)+FSARM_LINK3*cos(theta2+theta3)+FSARM_LINK4*cos(theta1+theta2+theta3);
    toolPosi->x = cos(theta1) * d;
    toolPosi->y = sin(theta1) * d;
    toolPosi->z = -FSARM_LINK2*sin(theta2)-FSARM_LINK3*sin(theta2+theta3)-FSARM_LINK4*sin(theta2+theta3+theta4);
}

// ��е�������˶�ѧ
FSARM_STATUS  FSARM_InverseKinematics(FSARM_POINT3D_T toolPosi, float pitch, FSARM_JOINTS_STATE_T* jointAngles){
	// �ؽڻ���
    float theta1 = 0.0;
    float theta2 = 0.0;
    float theta3 = 0.0;
    float theta4 = 0.0;
	
	FSARM_POINT3D_T wristPosi; // ��ؽ�����
	
	// ������ؽ�ԭ������е�ۻ�����ϵ��ֱ�߾���
    float disO2Tool = sqrt(pow(toolPosi.x,2) + pow(toolPosi.y, 2) + pow(toolPosi.z, 2));
    if (disO2Tool > (FSARM_LINK2+FSARM_LINK3+FSARM_LINK4)){
        return FSARM_STATUS_TOOLPOSI_TOO_FAR;
    }
	// �ж���ؽڵ�ԭ���Ƿ��ڻ�е������ϵ��Z����
    if (toolPosi.x == 0 && toolPosi.y == 0){
        // ��ѯ�ؽ�1�ĽǶ�, ��theta1���ָ�ԭ����ͬ
		FSUS_QueryServoAngle(armUsart, FSARM_JOINT1, &(jointAngles->theta1));
		jointAngles->theta1 = (jointAngles->theta1 - bJoint2Servo[FSARM_JOINT1]) / kJoint2Servo[FSARM_JOINT1];
        theta1 = radians(jointAngles->theta1);
    }else{
        // ���theta1
        theta1 = atan2(toolPosi.y, toolPosi.x);
        jointAngles->theta1 = degrees(theta1);
        // �ж�theta1�Ƿ�Ϸ�
        if (!FSARM_IsJointLegal(FSARM_JOINT1, jointAngles->theta1)){
            return FSARM_STATUS_JOINT1_OUTRANGE;
        }
    }
	// ������, �Ƕ�ת����
	float pitch_rad = radians(pitch);
	// ������ؽڵ�λ��
	wristPosi.x = toolPosi.x - FSARM_LINK4*cos(pitch_rad)*cos(theta1);
    wristPosi.y = toolPosi.y - FSARM_LINK4*cos(pitch_rad)*sin(theta1);
    wristPosi.z = toolPosi.z + FSARM_LINK4*sin(pitch_rad);
	
	// ����theta3
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
    // ����theta2
    float k1 = FSARM_LINK2 + FSARM_LINK3*cos(theta3);
    float k2 = FSARM_LINK3 * sin(theta3);
    float r = sqrt(pow(k1, 2) + pow(k2, 2));
    theta2 = atan2(-wristPosi.z/r, b/r) - atan2(k2/r, k1/r);
    jointAngles->theta2 = degrees(theta2);
    if(!FSARM_IsJointLegal(FSARM_JOINT2, jointAngles->theta2)){
        return FSARM_STATUS_JOINT2_OUTRANGE;
    }
    // ����theta4
    theta4 = pitch_rad - (theta2 + theta3);
    jointAngles->theta4 = degrees(theta4);
    if(!FSARM_IsJointLegal(FSARM_JOINT4, jointAngles->theta4)){
        return FSARM_STATUS_JOINT4_OUTRANGE;
    }
    // �ɹ�������
    return FSARM_STATUS_SUCCESS;
}


// ���(���ɹ켣)
FSARM_STATUS FSARM_MoveP2P(float x, float y, float z, float pitch){
	FSARM_JOINTS_STATE_T jointAngles;
	FSARM_POINT3D_T toolPosi;
	toolPosi.x = x;
	toolPosi.y = y;
	toolPosi.z = z;
    FSARM_STATUS status = FSARM_InverseKinematics(toolPosi, pitch, &jointAngles); // �����˶�ѧ
    if(status == FSARM_STATUS_SUCCESS){
		
		FSARM_JOINTS_STATE_T nextServoAngle; // ����Ŀ�����Ƕ�
        FSARM_JOINTS_STATE_T curServoAngles; // ���ö���ĽǶ�
		uint16_t interval;					 // ʱ����
		
		FSARM_JointAngle2ServoAngle(jointAngles, &nextServoAngle); // Ŀ��Ƕ�ת��ΪĿ��Ķ���Ƕ�
		FSARM_QueryServoAngle(&curServoAngles);	// ��ѯ��ǰ�Ķ���Ƕ�
		
		// ���Ͷ���Ƕȿ���ָ��
		interval = (uint16_t)fabs((nextServoAngle.theta1 - curServoAngles.theta1)/armJointSpeed*1000);
		FSUS_SetServoAngle(armUsart, FSARM_JOINT1, nextServoAngle.theta1, interval, 0, false);	
		interval = (uint16_t)fabs((nextServoAngle.theta2 - curServoAngles.theta2)/armJointSpeed*1000);
		FSUS_SetServoAngle(armUsart, FSARM_JOINT2, nextServoAngle.theta2, interval, 0, false);	
		interval = (uint16_t)fabs((nextServoAngle.theta3 - curServoAngles.theta3)/armJointSpeed*1000);
		FSUS_SetServoAngle(armUsart, FSARM_JOINT3, nextServoAngle.theta3, interval, 0, false);
		interval = (uint16_t)fabs((nextServoAngle.theta4 - curServoAngles.theta4)/armJointSpeed*1000);
		FSUS_SetServoAngle(armUsart, FSARM_JOINT4, nextServoAngle.theta4, interval, 0, false);
		// ����Ŀ��Ƕ�
		nextServoAngles[FSARM_JOINT1] = nextServoAngle.theta1;
		nextServoAngles[FSARM_JOINT2] = nextServoAngle.theta2;
		nextServoAngles[FSARM_JOINT3] = nextServoAngle.theta3;
		nextServoAngles[FSARM_JOINT4] = nextServoAngle.theta4;
    }
    return status;
}

// ��ص���е���
void FSARM_Home(void){
	// ���ɹ켣 �˶�����е���
	FSARM_MoveP2P(FSARM_HOME_X, FSARM_HOME_Y, FSARM_HOME_Z, FSARM_HOME_PITCH);
	FSARM_WaitAll();
}

// ���»�е�۵�״̬ 
void FSARM_Update(void){
	// ���¶���ĽǶ�
	FSARM_JOINTS_STATE_T servoAngles;
	FSARM_QueryServoAngle(&servoAngles);
	FSARM_SetCurServoAngle(servoAngles);
}

// �ȴ������ؽ���ת��Ŀ��Ƕ�
void FSARM_Wait(uint8_t jntIdx){
	// �Ƕ����
	float dAngle = fabs(curServoAngles[jntIdx] - nextServoAngles[jntIdx]);
	TimeTypedef tStart;
	while(true){
		// printf("wait joint: %d curAngle: %.1f nextAngle: %.1f\r\n", jntIdx, curServoAngles[jntIdx], nextServoAngles[jntIdx]);
		FSARM_Update(); // ���»�е�۵�״̬
		if(fabs(curServoAngles[jntIdx] - nextServoAngles[jntIdx]) <= FSUS_ANGLE_DEAD_BLOCK){
			break;
		}else{
			// �ж��Ƿ������������
			// ����ֵ���ֲ���
			if(fabs(dAngle) < 5.0 && fabs(dAngle - fabs(curServoAngles[jntIdx] - nextServoAngles[jntIdx])) <= 1.0){
				// �ж��Ƿ�ʱ
				if(SysTick_Millis() - tStart >= FSUS_WAIT_TIMEOUT_MS){
					break;
				}
			}else{
				// ���½Ƕ����
				dAngle = fabs(dAngle - fabs(curServoAngles[jntIdx] - nextServoAngles[jntIdx]));
				// ��ʼ����ʱ
				// SysTick_CountdownBegin(FSUS_WAIT_TIMEOUT_MS);
				tStart = SysTick_Millis();
			}
		}
	}
}

// �ȴ����еĹؽ���ת��Ŀ��Ƕ�
void FSARM_WaitAll(void){
	for(uint8_t srvIdx=0; srvIdx < FSARM_SERVO_NUM; srvIdx++){
		FSARM_Wait(srvIdx);
	}
}

// ����ĩ�˹��ߵ����� 
void FSARM_GetToolPosi(FSARM_POINT3D_T *toolPosi, float * pitch){
	FSARM_JOINTS_STATE_T curJointAngles; // ��ǰ�ؽڵĽǶ�
	FSARM_QueryJointAngle(&curJointAngles); // ��ѯ�ؽڽǶ�
	FSARM_ForwardKinmatics(curJointAngles, toolPosi, pitch);// �����˶�ѧ
}
