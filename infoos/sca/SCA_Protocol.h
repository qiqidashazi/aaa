/**
  ******************************************************************************
  * @��	�� �� SCA_Protocol.h
  * @��	�� �� INNFOS Software Team
  * @��	�� �� V1.5.2
  * @��	�� �� 2019.08.20
  * @ժ	Ҫ �� INNFOS CAN ͨ��Э���
  ******************************************************************************/ 

#ifndef __SCA_PROTOCOL_H
#define __SCA_PROTOCOL_H

#include "rtthread.h"
#include "rtdevice.h"
#include "rtconfig.h"
#include "canfestival.h"

/* ���������º궨����Ϣ���������޸ģ����� */


//INNFOS CAN ͨ��Э��ָ��
//��һ���ȡָ��
#define R1_Heartbeat			0x00
#define R1_Mode					0x55
#define R1_LastState			0xB0
#define R1_CurrentFilterState	0X71
#define R1_VelocityFilterState	0x75
#define R1_PositionFilterState	0x79
#define R1_PositionLimitState	0x8B
#define R1_PowerState			0x2B

//�ڶ����ȡָ��
#define R2_Voltage				0x45
#define R2_Current_Max			0x53
#define R2_CurrentFilterValue	0x73
#define R2_VelocityFilterValue	0x77
#define R2_PositionFilterValue	0x7B
#define R2_MotorTemp			0x5F
#define R2_InverterTemp			0x60
#define R2_InverterProtectTemp	0x62
#define R2_InverterRecoverTemp	0x64
#define R2_MotorProtectTemp		0x6C
#define R2_MotorRecoverTemp		0x6E
#define R2_Error				0xFF

//�������ȡָ��
#define R3_Current				0x04
#define R3_Velocity				0x05
#define R3_Position				0x06
#define R3_CurrentFilterP		0x15
#define R3_CurrentFilterI		0x16
#define R3_VelocityFilterP		0x17
#define R3_VelocityFilterI		0x18
#define R3_PositionFilterP		0x19
#define R3_PositionFilterI		0x1A
#define R3_PositionFilterD		0X1B
#define R3_PPMaxVelocity		0x1C
#define R3_PPMaxAcceleration	0x1D
#define R3_PPMaxDeceleration	0x1E
#define R3_PVMaxVelocity		0x22
#define R3_PVMaxAcceleration	0x23
#define R3_PVMaxDeceleration	0x24
#define R3_CurrentFilterLimitL	0x34
#define R3_CurrentFilterLimitH	0x35
#define R3_VelocityFilterLimitL	0x36
#define R3_VelocityFilterLimitH	0x37
#define R3_PositionFilterLimitL	0x38
#define R3_PositionFilterLimitH	0x39
#define R3_CurrentLimit			0x59
#define R3_VelocityLimit		0x5B
#define R3_Inertia				0x7D
#define R3_PositionLimitH		0x85
#define R3_PositionLimitL		0x86
#define R3_PositionOffset		0x8A
#define R3_HomingCurrentLimitL	0x92
#define R3_HomingCurrentLimitH	0x93
#define R3_BlockEngy			0x7F

//�������ȡָ��
#define R4_CVP					0x94

//�������ȡָ��
#define R5_ShakeHands			0x02

//��һ��д������
#define W1_Mode					0x07
#define W1_CurrentFilterState	0X70
#define W1_VelocityFilterState	0x74
#define W1_PositionFilterState	0x78
#define W1_PositionLimitState	0x8C
#define W1_PowerState			0x2A

//�ڶ���д������
#define W2_CurrentFilterValue	0x72
#define W2_VelocityFilterValue	0x76
#define W2_PositionFilterValue	0x7A
#define W2_InverterProtectTemp	0x61
#define W2_InverterRecoverTemp	0x63
#define W2_MotorProtectTemp		0x6B
#define W2_MotorRecoverTemp		0x6D

//������д������
#define W3_Current				0x08
#define W3_Velocity				0x09
#define W3_Position				0x0A
#define W3_CurrentFilterP		0x0E
#define W3_CurrentFilterI		0x0F
#define W3_VelocityFilterP		0x10
#define W3_VelocityFilterI		0x11
#define W3_PositionFilterP		0x12
#define W3_PositionFilterI		0x13
#define W3_PositionFilterD		0X14
#define W3_PPMaxVelocity		0x1F
#define W3_PPMaxAcceleration	0x20
#define W3_PPMaxDeceleration	0x21
#define W3_PVMaxVelocity		0x25
#define W3_PVMaxAcceleration	0x26
#define W3_PVMaxDeceleration	0x27
#define W3_CurrentFilterLimitL	0x2E
#define W3_CurrentFilterLimitH	0x2F
#define W3_VelocityFilterLimitL	0x30
#define W3_VelocityFilterLimitH	0x31
#define W3_PositionFilterLimitL	0x32
#define W3_PositionFilterLimitH	0x33
#define W3_CurrentLimit			0x58
#define W3_VelocityLimit		0x5A
#define W3_PositionLimitH		0x83
#define W3_PositionLimitL		0x84
#define W3_HomingValue			0x87
#define W3_PositionOffset		0x89
#define W3_HomingCurrentLimitL	0x90
#define W3_HomingCurrentLimitH	0x91
#define W3_BlockEngy			0x7E

//������д������
#define W4_ClearError			0xFE
#define W4_ClearHome			0x88
#define W4_Save					0x0D

//������д������
#define W5_ChangeID				0x3D

//��������ֵ����
#define Velocity_Max	6000.0f			//�ٶ����ֵ���̶�Ϊ6000RPM������Ϊ�����ã�
#define BlkEngy_Scal	75.225f			//��ת��������ֵ
#define Profile_Scal	960.0f			//���β�������ֵ
#define IQ8				256.0f			//2^8
#define IQ10			1024.0f			//2^10
#define IQ24			16777216.0f		//2^24
#define IQ30			1073741824.0f	//2^30

/* IDΪCAN����֡ID��msgΪҪ���͵����ݣ���ַ��
   lenΪ�������ݵĳ��ȣ�����0�ɹ�����������ʧ�� */
typedef uint8_t (*Send_t)(uint8_t ID, uint8_t* msg, uint8_t len);
								
typedef struct				//CAN�˿ھ��
{
	//SCA ״̬��Ϣ
	uint8_t CanPort;		//ʹ�õ�CAN�˿ں�
	uint8_t Retry;			//����ʧ��ʱ�ط�����
	Send_t Send;			//���ͺ�������ʽ�μ�Send_t

}CAN_Handler_t;

typedef struct 						//SCA������Ϣ
{
	uint16_t Error_Code;			//�������
	
	/* ���屨����Ϣ��0��������1������ */
    uint8_t WARN_OVER_VOLT;  		//��ѹ�쳣
    uint8_t WARN_UNDER_VOLT;  		//Ƿѹ�쳣
    uint8_t WARN_LOCK_ROTOR;  		//��ת�쳣
    uint8_t WARN_OVER_TEMP;  		//�����쳣
    uint8_t WARN_RW_PARA;  			//��д�����쳣
    uint8_t WARN_MUL_CIRCLE;  		//��Ȧ�����쳣
    uint8_t WARN_TEMP_SENSOR_INV; 	//������¶ȴ������쳣
    uint8_t WARN_CAN_BUS;  			//CANͨѶ�쳣
    uint8_t WARN_TEMP_SENSOR_MTR;	//����¶ȴ������쳣
    uint8_t WARN_OVER_STEP;			//λ��ģʽ��Ծ����1
    uint8_t WARN_DRV_PROTEC;  		//DRV����
    uint8_t WARN_DVICE;  			//�豸�쳣
	
}SCA_Warn_t;

/* 
	SCA�������棬����д�����ʱ����Ŀ����������ɹ�����д������
	��ȡ��־λ������ʱʹ�ã��������ݿɸ�����Ŀ��Ҫ���вü�������
 */
typedef struct 
{
	/* ������Ϣ */
	uint8_t ID;						//SCA ��ID��
	
	/* ��һ�����ݱ��� */
	uint8_t Mode;					//��ǰ����ģʽ
	uint8_t Current_Filter_State;	//�������˲���״̬
	uint8_t Velocity_Filter_State;	//�ٶȻ��˲���״̬
	uint8_t Position_Filter_State;	//�ٶȻ��˲���״̬
	uint8_t Position_Limit_State;	//λ����λ״̬
	uint8_t Power_State;			//���ػ�״̬
	/* ��ȡ��־λ */
	uint8_t R_Mode;					//��ȡ���ݷ��ر�־λ 1Ϊ�����ݷ���
	uint8_t R_Last_State;	
	uint8_t R_Current_Filter_State;	
	uint8_t R_Velocity_Filter_State;
	uint8_t R_Position_Filter_State;
	uint8_t R_Position_Limit_State;	
	uint8_t R_Power_State;			
	
	/* �ڶ������ݱ��� */
	float Current_Filter_Value;		//�������˲�������
	float Velocity_Filter_Value;	//�ٶȻ��˲�������
	float Position_Filter_Value;	//λ�û��˲�������
	float Inverter_Protect_Temp;	//����������¶�
	float Inverter_Recover_Temp;	//������ָ��¶�
	float Motor_Protect_Temp;		//��������¶�
	float Motor_Recover_Temp;		//����ָ��¶�	
	/* ��ȡ��־λ */
	uint8_t R_Current_Filter_Value;	
	uint8_t R_Velocity_Filter_Value;
	uint8_t R_Position_Filter_Value;
	uint8_t R_Inverter_Protect_Temp;
	uint8_t R_Inverter_Recover_Temp;
	uint8_t R_Motor_Protect_Temp;	
	uint8_t R_Motor_Recover_Temp;	
	uint8_t R_Voltage;
	uint8_t R_Current_Max;
	uint8_t R_Motor_Temp;
	uint8_t R_Inverter_Temp;
	uint8_t R_Error_Code;
	
	/* ���������ݱ��� */
	float Current_Real;				//��ǰ��������λ��A��
	float Velocity_Real;			//��ǰ�ٶȣ���λ��RPM��
	float Position_Real;			//��ǰλ�ã���ʵֵ����λ��R��
	float Current_Filter_P;			//��������Pֵ
	float Current_Filter_I;			//��������Iֵ
	float Velocity_Filter_P;		//�ٶȻ���Pֵ
	float Velocity_Filter_I;		//�ٶȻ���Iֵ
	float Position_Filter_P;		//λ�û���Pֵ
	float Position_Filter_I;		//λ�û���Iֵ	
	//float Position_Filter_D;		//λ�û���Dֵ	
	float PP_Max_Velocity;			//λ�������ٶ����ֵ
	float PP_Max_Acceleration;		//λ�����μ��ٶ����ֵ
	float PP_Max_Deceleration;		//λ�����μ��ٶ����ֵ
	float PV_Max_Velocity;			//�ٶ������ٶ����ֵ
	float PV_Max_Acceleration;		//�ٶ����μ��ٶ����ֵ
	float PV_Max_Deceleration;		//�ٶ����μ��ٶ����ֵ
	//float Current_Filter_Limit_L;	//�������������
	//float Current_Filter_Limit_H;	//�������������
	float Velocity_Filter_Limit_L;	//�ٶȻ��������
	float Velocity_Filter_Limit_H;	//�ٶȻ��������
	float Position_Filter_Limit_L;	//λ�û��������
	float Position_Filter_Limit_H;	//λ�û��������
	float Position_Limit_H;			//ִ������λ������
	float Position_Limit_L;			//ִ������λ������
	float Current_Limit;			//���������޷�
	float Velocity_Limit;			//�ٶ������޷�
	float Homing_Value;				//ִ������Homingֵ
	float Position_Offset;			//ִ������λ��ƫ��
	float Homing_Current_Limit_L;	//�Զ������������
	float Homing_Current_Limit_H;	//�Զ������������
	float Blocked_Energy;			//��ת��������
	/* ��ȡ��־λ */
	uint8_t R_Current_Real;				
	uint8_t R_Velocity_Real;			
	uint8_t R_Position_Real;			
	uint8_t R_Current_Filter_P;			
	uint8_t R_Current_Filter_I;			
	uint8_t R_Velocity_Filter_P;		
	uint8_t R_Velocity_Filter_I;		
	uint8_t R_Position_Filter_P;		
	uint8_t R_Position_Filter_I;		
	//uint8_t R_Position_Filter_D;		
	uint8_t R_PP_Max_Velocity;			
	uint8_t R_PP_Max_Acceleration;		
	uint8_t R_PP_Max_Deceleration;		
	uint8_t R_PV_Max_Velocity;			
	uint8_t R_PV_Max_Acceleration;		
	uint8_t R_PV_Max_Deceleration;		
	//uint8_t R_Current_Filter_Limit_L;	
	//uint8_t R_Current_Filter_Limit_H;	
	uint8_t R_Velocity_Filter_Limit_L;	
	uint8_t R_Velocity_Filter_Limit_H;	
	uint8_t R_Position_Filter_Limit_L;	
	uint8_t R_Position_Filter_Limit_H;	
	uint8_t R_Position_Limit_H;			
	uint8_t R_Position_Limit_L;			
	uint8_t R_Current_Limit;			
	uint8_t R_Velocity_Limit;			
	uint8_t R_Homing_Value;				
	uint8_t R_Position_Offset;			
	uint8_t R_Homing_Current_Limit_L;	
	uint8_t R_Homing_Current_Limit_H;	
	uint8_t R_Blocked_Energy;		
	uint8_t R_CVP;		
	uint8_t R_Serial_Num;	
	uint8_t W_ClearHome;
	
}Para_Cache_t;

/* 
	SCA��Ϣ����������������������ֵ��
	�������ݿɸ�����Ŀ��Ҫ���вü�������
 */
typedef struct 
{
	/* Э�����ݱ����� */
	uint8_t ID;						//SCA��ID��
	uint8_t Serial_Num[4];			//���к�
	uint8_t Save_State;				//��������״̬��1Ϊ�ѱ���
	uint8_t Online_State;			//��ǰ����״̬��1Ϊ����
	uint8_t Update_State;			//�Ƿ��в���ˢ�£�1Ϊ�в���ˢ��
	CAN_Handler_t* Can;				//��ʹ�õ�CAN�˿�
	Para_Cache_t paraCache;			//��������
	
	/* �û����ݱ����� */
	
	/* ��һ�����ݱ��� */
	uint8_t Mode;					//��ǰ����ģʽ
	uint8_t Last_State;				//�ϴιػ����쳣״̬��1Ϊ����
	uint8_t Current_Filter_State;	//�������˲���״̬
	uint8_t Velocity_Filter_State;	//�ٶȻ��˲���״̬
	uint8_t Position_Filter_State;	//�ٶȻ��˲���״̬
	uint8_t Position_Limit_State;	//λ����λ״̬
	uint8_t Power_State;			//���ػ�״̬
	
	/* �ڶ������ݱ��� */
	float Voltage;					//��ǰ��ѹ����λ��V��
	float Current_Max;				//����������
	float Current_Filter_Value;		//�������˲�������
	float Velocity_Filter_Value;	//�ٶȻ��˲�������
	float Position_Filter_Value;	//λ�û��˲�������
	float Motor_Temp;				//����¶�
	float Inverter_Temp;			//������¶�
	float Inverter_Protect_Temp;	//����������¶�
	float Inverter_Recover_Temp;	//������ָ��¶�
	float Motor_Protect_Temp;		//��������¶�
	float Motor_Recover_Temp;		//����ָ��¶�	
	SCA_Warn_t SCA_Warn;			//���������Ϣ
	
	/* ���������ݱ��� */
	float Current_Real;				//��ǰ��������λ��A��
	float Velocity_Real;			//��ǰ�ٶȣ���λ��RPM��
	float Position_Real;			//��ǰλ�ã���ʵֵ����λ��R��
	float Current_Filter_P;			//��������Pֵ
	float Current_Filter_I;			//��������Iֵ
	float Velocity_Filter_P;		//�ٶȻ���Pֵ
	float Velocity_Filter_I;		//�ٶȻ���Iֵ
	float Position_Filter_P;		//λ�û���Pֵ
	float Position_Filter_I;		//λ�û���Iֵ	
	//float Position_Filter_D;		//λ�û���Dֵ	
	float PP_Max_Velocity;			//λ�������ٶ����ֵ
	float PP_Max_Acceleration;		//λ�����μ��ٶ����ֵ
	float PP_Max_Deceleration;		//λ�����μ��ٶ����ֵ
	float PV_Max_Velocity;			//�ٶ������ٶ����ֵ
	float PV_Max_Acceleration;		//�ٶ����μ��ٶ����ֵ
	float PV_Max_Deceleration;		//�ٶ����μ��ٶ����ֵ
	//float Current_Filter_Limit_L;	//�������������
	//float Current_Filter_Limit_H;	//�������������
	float Velocity_Filter_Limit_L;	//�ٶȻ��������
	float Velocity_Filter_Limit_H;	//�ٶȻ��������
	float Position_Filter_Limit_L;	//λ�û��������
	float Position_Filter_Limit_H;	//λ�û��������
	float Position_Limit_H;			//ִ������λ������
	float Position_Limit_L;			//ִ������λ������
	float Current_Limit;			//���������޷�
	float Velocity_Limit;			//�ٶ������޷�
	float Homing_Value;				//ִ������Homingֵ
	float Position_Offset;			//ִ������λ��ƫ��
	float Homing_Current_Limit_L;	//�Զ������������
	float Homing_Current_Limit_H;	//�Զ������������
	float Blocked_Energy;			//��ת��������
	
}SCA_Handler_t;

enum SCA_Error				//SCAͨ�Ŵ�������ö��
{
	SCA_NoError = 0,		//�޴���
	SCA_OverTime,			//ͨ�ŵȴ���ʱ
	SCA_SendError,			//���ݷ���ʧ��
	SCA_OperationFailed,	//����ʧ��
	SCA_UnknownID,			//δ�ҵ���ID��ִ�������
};

/* ���ݽ��սӿڣ����µ�CAN���ݰ�����ʱ����
  struct rt_can_msg ΪCAN���ݰ��Ľ������ͽṹ����ֲʱ������
  ����ƽ̨����CanRxMsg�ṹ���ͣ��˴�Ĭ��ʹ��STM32
  ��׼�⺯���еĽ��սṹ	*/	
void infoosCanDispatch(struct rt_can_msg* RxMsg);

/* ���º���ΪAPI����� */

/* ��ȡ����ӿ� */
uint8_t SCA_Read(SCA_Handler_t* pSCA, uint8_t cmd);

/* ����д������ */
uint8_t SCA_Write_1(SCA_Handler_t* pSCA, uint8_t cmd, uint8_t TxData);
uint8_t SCA_Write_2(SCA_Handler_t* pSCA, uint8_t cmd, float TxData);
uint8_t SCA_Write_3(SCA_Handler_t* pSCA, uint8_t cmd, float TxData);
uint8_t SCA_Write_4(SCA_Handler_t* pSCA, uint8_t cmd);
uint8_t SCA_Write_5(SCA_Handler_t* pSCA, uint8_t cmd, uint8_t TxData);

#endif
