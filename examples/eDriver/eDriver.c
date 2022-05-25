
#include "eDriver.h"
#include "rtconfig.h"


static eDriverMaster_t  eDriver;
static servo_sem_t  servo_conf[4];

// 读取的状态和位置的取值
static UNS16	status_test;
static UNS32	position_test;

///////////////////////////////////////----------------infoos 
#include "SCA_Protocol.h"
#include "SCA_API.h"
///////////////////////////////////////----------------infoos



int MInitialize_eDriver(void)
{
    eDriver.agv_board.busname = "0";
    eDriver.agv_board.baudrate = "1M";
    eDriver.semTable = servo_conf;
    eDriver.od_Data = &master402_Data;
    eDriver.MSetSYNC = MSetSYNC;

    return MCanopen_init();
}
//INIT_APP_EXPORT(MInitialize_eDriver);

static int MCanopen_init(void)
{
    // 注册canopen相关回调函数
	eDriver.od_Data->heartbeatError = master402_heartbeatError;
	eDriver.od_Data->initialisation = master402_initialisation;
	eDriver.od_Data->preOperational = master402_preOperational;
	eDriver.od_Data->operational = master402_operational;
	eDriver.od_Data->stopped   = master402_stopped;
	eDriver.od_Data->post_sync = master402_post_sync;
	eDriver.od_Data->post_TPDO = master402_post_TPDO;
	eDriver.od_Data->storeODSubIndex = (storeODSubIndex_t)master402_storeODSubIndex;
	eDriver.od_Data->post_emcy = (post_emcy_t)master402_post_emcy;

    //	打开主站can接收数据线程
	canOpen(&eDriver.agv_board, eDriver.od_Data);
	//	初始化定时器
    initTimer();

	//	定时器倒计时启动主站节点初始化
	StartTimerLoop(&MInitNodes);
	
	return 0;
}

//初始化主站节点
void MInitNodes(CO_Data* d, UNS32 id)
{
	setNodeId(eDriver.od_Data, 0x01);
	setState(eDriver.od_Data, Initialisation);
}

// 停止主站
void MInitNodeExit(CO_Data* d, UNS32 id)
{
    masterSendNMTstateChange(eDriver.od_Data, ALL_SERVO_NODE, NMT_Reset_Node);
	setState(eDriver.od_Data, Stopped);
}

// 从站自启线程
static void slaveBootupHdl(CO_Data* d, UNS8 nodeId)
{
	rt_thread_t tid;

	tid = rt_thread_create("Servo_cfg", MConfig_Single_Servo, (void *)(int)nodeId, 1024, 12 + nodeId, 2);
	if(tid == RT_NULL)
	{
		kdebug("canopen config servo thread start failed!\n");
	}
	else
	{
		rt_thread_startup(tid);
	}
}

// 主站预处理线程
void canopen_start_thread_entry(void *parameter)
{

	// thread delay
	MConfigServo(SERVO_1); //	配置伺服驱动1
	rt_thread_delay(200);
	MConfigServo(SERVO_2); //	配置伺服驱动2
	rt_thread_delay(200);
	MConfigServo(SERVO_3); //	配置伺服驱动3

	//	masterSendNMTnodeguard() 启动 slave自启线程
	eDriver.od_Data->post_SlaveBootup = slaveBootupHdl;	

	
	// 设置主站进入操作状态
	setState(eDriver.od_Data, Operational);  

	// 重置从站并设置三个从站进入操作状态
    masterSendNMTstateChange(eDriver.od_Data, ALL_SERVO_NODE, NMT_Reset_Node);
	rt_thread_delay(100);
    masterSendNMTstateChange(eDriver.od_Data, ALL_SERVO_NODE, NMT_Start_Node);

    // 同步发生器(4ms)  
	// 如果使用SDO方式发送需要关闭该项功能 
    MSetSYNC(SYNC_TIME);
}


//从站单次自启线程
static void MConfig_Single_Servo(void *parameter)
{
    uint32_t nodeId;
	nodeId = (uint32_t)parameter;
	MConfigServo(nodeId);
	masterSendNMTstateChange(eDriver.od_Data, nodeId, NMT_Start_Node);
}

// 配置从站
static void MConfigServo(uint8_t nodeId)
{
	do
	{
		if(nodeId <= 1) 
		{
			kdebug("MConfigServo: nodeid == 1 \n");
			break;
		}
		eDriver.semTable[nodeId - 2].case_cnt = 0;
		eDriver.semTable[nodeId - 2].retry_cnt = 0;
		eDriver.semTable[nodeId - 2].nodeId = nodeId;
		rt_sem_init(&(eDriver.semTable[nodeId - 2].finish_sem), "servocnf", 0, RT_IPC_FLAG_FIFO);

		// 只执行一次该函数，然后释放锁-等待回调函数的递归-释放信号量
		// 在信号量前释放锁--可能会出现sdo传输表不够用的情况，如果传输速度足够的快应该不会出现这个问题，为此设立了retry的机制
		EnterMutex();   // 进入排斥区
		MConfig_Servo_Param(nodeId, &eDriver.semTable[nodeId - 2]);     // 配置伺服从站
		LeaveMutex();   // 离开排斥区
		rt_sem_take(&(eDriver.semTable[nodeId - 2].finish_sem), RT_WAITING_FOREVER);
		rt_sem_detach(&(eDriver.semTable[nodeId - 2].finish_sem));
	} while (0);
}



static void MConfig_Servo_Param_Callback(CO_Data* d, UNS8 nodeId)
{
    UNS32 abortCode;
	UNS8 res;

    servo_sem_t* conf = RT_NULL;

    conf = &eDriver.semTable[nodeId - 2];

	res = getWriteResultNetworkDict(eDriver.od_Data, nodeId, &abortCode);
	closeSDOtransfer(eDriver.od_Data, nodeId, SDO_CLIENT);
	if(res != SDO_FINISHED)
	{
		conf->retry_cnt++;
		kdebug("MConfig_Servo_Param_Callback(): write SDO failed!  nodeId = %d, abortCode = 0x%08X\n", nodeId, abortCode);
		if(conf->retry_cnt < 3)
		{
			MConfig_Servo_Param(nodeId, conf);
		}
		else
		{
			rt_sem_release(&(conf->finish_sem));
			conf->case_cnt = 0;
			conf->retry_cnt = 0;
			conf->nodeId = 0;
			kdebug("MConfig_Servo_Param_Callback() : SDO config try count > 3, config failed!\n");
		}
	}
	else
	{
		conf->case_cnt++;
		conf->retry_cnt = 0;
		conf->nodeId = nodeId;
		MConfig_Servo_Param(nodeId, conf);
	}
}

static void MConfig_Servo_Param(uint8_t nodeId, servo_sem_t* conf)
{
	switch(nodeId)
	{
		case	SERVO_1: 
			(void)MServoConfig_Fir(conf);
			break;
		case	SERVO_2:
			(void)MServoConfig_Sec(conf);
			break;
		case	SERVO_3:
			(void)MServoConfig_Trd(conf);
			break;

		default: break;	//ignore
	}
}

///////////////////////
//	配置第一台从机
int MServoConfig_Fir(servo_sem_t* conf)
{
	switch(conf->case_cnt)
	{
//-------------------------------------从站SYNC的配置
		case 0:
			{	//	关闭同步
				UNS32 shutdown_SYNC = 0x80;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1005, 0, 4, uint32, &shutdown_SYNC, MConfig_Servo_Param_Callback, 0);
			}
 			break;
		case 1:
			{	//	开启同步周期  //如果使用SDO方式发送需要关闭该项功能
				UNS32 startSYNCPeriod = SYNC_TIME;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1006, 0, 4, uint32, &startSYNCPeriod, MConfig_Servo_Param_Callback, 0);
			}
			break;
//-------------------------------------从站RPDO的配置
		case 2:
			{ 	//	关闭RPDO权限	从站一使用 202 302 402 502的形式，以此类推 从站2使用 203 303 403 503 从站3：204 304 404 504 从站4：205 305 405 505
				UNS32 RPDO_COB_ID_shut = 0x80000200 + conf->nodeId;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1400, 1, 4, uint32, &RPDO_COB_ID_shut, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 3:
			{	//	同步模式 ：同步帧 (1帧)
				UNS8 RtransferType = TPDO_TRANSMISSION_SYNC(1);
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1400, 2, 1, uint8, &RtransferType, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 4:
			{	//	关闭RPDO的映射权限
				UNS8 RPDO_MAP_CNT_shut = 0;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1600, 0, 1, uint8, &RPDO_MAP_CNT_shut, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 5:
			{	//	映射插补模式低位
				UNS32 rpdo_map_01 = 0x60C10110;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1600, 1, 4, uint32, &rpdo_map_01, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 6:
			{	//	插补高位
				UNS32 rpdo_map_02 = 0x60C10210;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1600, 2, 4, uint32, &rpdo_map_02, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 7:
			{	//	控制字
				UNS32 rpdo_map_03 = 0x60400010;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1600, 3, 4, uint32, &rpdo_map_03, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 8:
			{ //  打开RPDO的映射权限
				UNS8 RPDO_MAP_CNT_open = 3;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1600, 0, 1, uint8, &RPDO_MAP_CNT_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 9:
			{ // 打开RPDO_COBID权限
				UNS32 RPDO_COB_ID_open = 0x00000200 + conf->nodeId;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1400, 1, 4, uint32, &RPDO_COB_ID_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
//-------------------------------------从站TPDO的配置
		case 10:
			{	// TPDO权限关闭		从站1：使用 182 282 382 482的形式，以此类推 从站2使用 183 283 383 483 从站3：184 284 384 484 从站4：185 285 385 485
				UNS32 TPDO_COBID_shut = 0x80000180 + conf->nodeId;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1800, 1, 4, uint32, &TPDO_COBID_shut, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 11:
			{	//	同步帧	1帧
				UNS8 TtransferType = TPDO_TRANSMISSION_SYNC(1);
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1800, 2, 1, uint8, &TtransferType, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 12:
			{	//	禁止时间
				UNS16 forbitTime = 3;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1800, 3, 2, uint16, &forbitTime, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 13:
			{	//	事件定时器
				UNS16 Event_Timer = 0;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1800, 5, 2, uint16, &Event_Timer, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 14:
			{ 	//	禁止TPDO的映射权限
				UNS8 highestSubIndex_shut = 0;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1A00, 0, 1, uint8, &highestSubIndex_shut, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 15:
			{ 	// 实际位置
				UNS32 actualPositon = 0x60640020;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1A00, 1, 4, uint32, &actualPositon, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 16:
			{	// 实际转矩 状态字
				UNS32 actual_torque = 0x60410010;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1A00, 2, 4, uint32, &actual_torque, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 17:
			{	//打开TDDO的映射权限
				UNS8 highestSubIndex_open = 2;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1A00, 0, 1, uint8, &highestSubIndex_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 18:
			{	// 打开TPDO的权限
				UNS32 TPDO_COBID_open = 0x00000180 + conf->nodeId;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1800, 1, 4, uint32, &TPDO_COBID_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
//-------------------------------------从站0x6060插补模式的配置
		case 19:
			{	//	配置模式为插补模式
				UNS8 Interpolation_open = INTERPOLATION_MODE;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x6060, 0, 1, uint8, &Interpolation_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
//------------------------------------------------------------结束循环配置
		case 20:
			{	//释放信号量 结束循环
				rt_sem_release(&(conf->finish_sem));
			}
			break;
//-------------------------------------do nothing: 以下不做任何操作，作为后续从站配置需要补充的选项
//-------------------------------------请将信号量释放移至末尾
		case 21:
			{ // 
				// UNS8 Interpolation_open = INTERPOLATION_MODE;
				// writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x6060, 0, 1, uint8, &Interpolation_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 22:
			{ // 
				// UNS8 Interpolation_open = INTERPOLATION_MODE;
				// writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x6060, 0, 1, uint8, &Interpolation_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 23:
			{ // 
				// UNS8 Interpolation_open = INTERPOLATION_MODE;
				// writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x6060, 0, 1, uint8, &Interpolation_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 24:
			{ // 
				// UNS8 Interpolation_open = INTERPOLATION_MODE;
				// writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x6060, 0, 1, uint8, &Interpolation_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 25:
			{ // 
				// UNS8 Interpolation_open = INTERPOLATION_MODE;
				// writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x6060, 0, 1, uint8, &Interpolation_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 26:
			{ // 
				// UNS8 Interpolation_open = INTERPOLATION_MODE;
				// writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x6060, 0, 1, uint8, &Interpolation_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 27:
			{ // 
				// UNS8 Interpolation_open = INTERPOLATION_MODE;
				// writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x6060, 0, 1, uint8, &Interpolation_open, MConfig_Servo_Param_Callback, 0);
			}
			break;	
		case 28:
			{ // 
				// UNS8 Interpolation_open = INTERPOLATION_MODE;
				// writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x6060, 0, 1, uint8, &Interpolation_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 29:
			{ // 
				// UNS8 Interpolation_open = INTERPOLATION_MODE;
				// writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x6060, 0, 1, uint8, &Interpolation_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 30:
			//rt_sem_release(&(conf->finish_sem));
			break;
		default:
			break;
	}
	return 0;
}

//	配置第二台
int MServoConfig_Sec(servo_sem_t* conf)
{
	switch(conf->case_cnt)
	{
//-------------------------------------从站SYNC的配置
		case 0:
			{	//	关闭同步
				UNS32 shutdown_SYNC = 0x80;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1005, 0, 4, uint32, &shutdown_SYNC, MConfig_Servo_Param_Callback, 0);
			}
 			break;
		case 1:
			{	//	开启同步周期	//如果使用SDO方式发送需要关闭该项功能
				UNS32 startSYNCPeriod = SYNC_TIME;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1006, 0, 4, uint32, &startSYNCPeriod, MConfig_Servo_Param_Callback, 0);
			}
			break;
//-------------------------------------从站RPDO的配置
		case 2:
			{ 	//	关闭RPDO权限	
				UNS32 RPDO_COB_ID_shut = 0x80000300 + conf->nodeId;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1401, 1, 4, uint32, &RPDO_COB_ID_shut, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 3:
			{	//	同步模式 ：同步帧 (1帧)
				UNS8 RtransferType = TPDO_TRANSMISSION_SYNC(1);
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1401, 2, 1, uint8, &RtransferType, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 4:
			{	//	关闭RPDO的映射权限
				UNS8 RPDO_MAP_CNT_shut = 0;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1601, 0, 1, uint8, &RPDO_MAP_CNT_shut, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 5:
			{	//	映射插补模式低位
				UNS32 rpdo_map_01 = 0x60C10110;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1601, 1, 4, uint32, &rpdo_map_01, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 6:
			{	//	插补高位
				UNS32 rpdo_map_02 = 0x60C10210;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1601, 2, 4, uint32, &rpdo_map_02, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 7:
			{	//	控制字
				UNS32 rpdo_map_03 = 0x60400010;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1601, 3, 4, uint32, &rpdo_map_03, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 8:
			{ //  打开RPDO的映射权限
				UNS8 RPDO_MAP_CNT_open = 3;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1601, 0, 1, uint8, &RPDO_MAP_CNT_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 9:
			{ // 打开RPDO_COBID权限
				UNS32 RPDO_COB_ID_open = 0x00000300 + conf->nodeId;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1401, 1, 4, uint32, &RPDO_COB_ID_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
//-------------------------------------从站TPDO的配置
		case 10:
			{	// TPDO权限关闭
				UNS32 TPDO_COBID_shut = 0x80000280 + conf->nodeId;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1801, 1, 4, uint32, &TPDO_COBID_shut, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 11:
			{	//	同步帧	1帧
				UNS8 TtransferType = TPDO_TRANSMISSION_SYNC(1);
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1801, 2, 1, uint8, &TtransferType, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 12:
			{	//	禁止时间
				UNS16 forbitTime = 3;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1801, 3, 2, uint16, &forbitTime, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 13:
			{	//	事件定时器
				UNS16 Event_Timer = 0;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1801, 5, 2, uint16, &Event_Timer, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 14:
			{ 	//	禁止TPDO的映射权限
				UNS8 highestSubIndex_shut = 0;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1A01, 0, 1, uint8, &highestSubIndex_shut, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 15:
			{ 	//	实际位置
				UNS32 actualPositon = 0x60640020;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1A01, 1, 4, uint32, &actualPositon, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 16:
			{	//	状态字
				UNS32 actual_torque = 0x60410010;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1A01, 2, 4, uint32, &actual_torque, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 17:
			{	//	打开TDDO的映射权限
				UNS8 highestSubIndex_open = 2;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1A01, 0, 1, uint8, &highestSubIndex_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 18:
			{	//	打开TPDO的权限
				UNS32 TPDO_COBID_open = 0x00000280 + conf->nodeId;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1801, 1, 4, uint32, &TPDO_COBID_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
//-------------------------------------从站0x6060插补模式的配置
		case 19:
			{	//	配置模式为插补模式
				UNS8 Interpolation_open = INTERPOLATION_MODE;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x6060, 0, 1, uint8, &Interpolation_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
//------------------------------------------------------------结束循环配置
		case 20:
			{	//释放信号量 结束循环
				rt_sem_release(&(conf->finish_sem));
			}
			break;
//-------------------------------------do nothing: 以下不做任何操作，作为后续从站配置需要补充的选项
//-------------------------------------请将信号量释放移至末尾
		default:
			break;
	}
	return 0;
}

//	配置第三台
int MServoConfig_Trd(servo_sem_t* conf)
{
	switch(conf->case_cnt)
	{
//-------------------------------------从站SYNC的配置
		case 0:
			{	//	关闭同步
				UNS32 shutdown_SYNC = 0x80;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1005, 0, 4, uint32, &shutdown_SYNC, MConfig_Servo_Param_Callback, 0);
			}
 			break;
		case 1:
			{	//	开启同步周期	//如果使用SDO方式发送需要关闭该项功能
				UNS32 startSYNCPeriod = SYNC_TIME;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1006, 0, 4, uint32, &startSYNCPeriod, MConfig_Servo_Param_Callback, 0);
			}
			break;
//-------------------------------------从站RPDO的配置
		case 2:
			{ 	//	关闭RPDO权限	
				UNS32 RPDO_COB_ID_shut = 0x80000400 + conf->nodeId;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1402, 1, 4, uint32, &RPDO_COB_ID_shut, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 3:
			{	//	同步模式 ：同步帧 (1帧)
				UNS8 RtransferType = TPDO_TRANSMISSION_SYNC(1);
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1402, 2, 1, uint8, &RtransferType, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 4:
			{	//	关闭RPDO的映射权限
				UNS8 RPDO_MAP_CNT_shut = 0;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1602, 0, 1, uint8, &RPDO_MAP_CNT_shut, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 5:
			{	//	映射插补模式低位
				UNS32 rpdo_map_01 = 0x60C10110;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1602, 1, 4, uint32, &rpdo_map_01, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 6:
			{	//	插补高位
				UNS32 rpdo_map_02 = 0x60C10210;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1602, 2, 4, uint32, &rpdo_map_02, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 7:
			{	//	控制字
				UNS32 rpdo_map_03 = 0x60400010;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1602, 3, 4, uint32, &rpdo_map_03, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 8:
			{ //  打开RPDO的映射权限
				UNS8 RPDO_MAP_CNT_open = 3;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1602, 0, 1, uint8, &RPDO_MAP_CNT_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 9:
			{ // 打开RPDO_COBID权限
				UNS32 RPDO_COB_ID_open = 0x00000400 + conf->nodeId;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1402, 1, 4, uint32, &RPDO_COB_ID_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
//-------------------------------------从站TPDO的配置
		case 10:
			{	// TPDO权限关闭
				UNS32 TPDO_COBID_shut = 0x80000380 + conf->nodeId;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1802, 1, 4, uint32, &TPDO_COBID_shut, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 11:
			{	//	同步帧	1帧
				UNS8 TtransferType = TPDO_TRANSMISSION_SYNC(1);
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1802, 2, 1, uint8, &TtransferType, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 12:
			{	//	禁止时间
				UNS16 forbitTime = 3;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1802, 3, 2, uint16, &forbitTime, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 13:
			{	//	事件定时器
				UNS16 Event_Timer = 0;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1802, 5, 2, uint16, &Event_Timer, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 14:
			{ 	//	禁止TPDO的映射权限
				UNS8 highestSubIndex_shut = 0;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1A02, 0, 1, uint8, &highestSubIndex_shut, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 15:
			{ 	// 实际位置
				UNS32 actualPositon = 0x60640020;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1A02, 1, 4, uint32, &actualPositon, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 16:
			{	// 实际转矩 状态字
				UNS32 actual_torque = 0x60410010;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1A02, 2, 4, uint32, &actual_torque, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 17:
			{	//打开TDDO的映射权限
				UNS8 highestSubIndex_open = 2;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1A02, 0, 1, uint8, &highestSubIndex_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
		case 18:
			{	// 打开TPDO的权限
				UNS32 TPDO_COBID_open = 0x00000380 + conf->nodeId;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x1802, 1, 4, uint32, &TPDO_COBID_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
//-------------------------------------从站0x6060插补模式的配置
		case 19:
			{	//	配置模式为插补模式
				UNS8 Interpolation_open = INTERPOLATION_MODE;
				writeNetworkDictCallBack(eDriver.od_Data, conf->nodeId, 0x6060, 0, 1, uint8, &Interpolation_open, MConfig_Servo_Param_Callback, 0);
			}
			break;
//------------------------------------------------------------结束循环配置
		case 20:
			{	//释放信号量 结束循环
				rt_sem_release(&(conf->finish_sem));
			}
			break;
//-------------------------------------do nothing: 以下不做任何操作，作为后续从站配置需要补充的选项
//-------------------------------------请将信号量释放移至末尾
		default:
			break;
	}
	return 0;
}


//////////////////////
//	激活节点同步发生器
void MSetSYNC(rt_uint32_t cycle)
{
    UNS8 data_type;
	UNS32 sync_id, size, period;

	data_type = uint32;
	size = 4;
	readLocalDict(eDriver.od_Data, 0x1005, 0, &sync_id, &size, &data_type, 0);

    // 1<<30 激活次高位设定 0x4000 0000
	sync_id |= (1 << 30);
	writeLocalDict(eDriver.od_Data, 0x1005, 0, &sync_id, &size, 0);

    // 同步通信周期字典设定
    period = (UNS32)cycle;
	writeLocalDict(eDriver.od_Data, 0x1006, 0, &period, &size, 0);
}

// 设置控制字
void MSetControlWord(rt_uint16_t val)
{
    Controlword = (UNS16)val;
   // SYNC_DELAY;
}

// 引导伺服驱动器
void MBootServoDriver(void)
{
	Controlword = 0x06;
	SYNC_DELAY;
	Controlword = 0x07;
	SYNC_DELAY;
	Controlword = 0x0f;
	SYNC_DELAY;
	//激活插补
	MActiveInterpolationMode();	
}

// 设置为插补模式 : 该功能在做从站初始化配置的时候通过网络字典配置进行更改
// void MSetInterpolationMode(){
//     Modes_of_operation = INTERPOLATION_MODE;
// }

// 激活插补
void MActiveInterpolationMode(void)
{
    Controlword = 0x1F;
}

// 设置心跳机制
void MSetHeartBeat(void)
{
    UNS32 size;
	UNS32 consumer_heartbeat_time;

    // ConsumerHeartbeatEntries[1] 132072
	consumer_heartbeat_time = (2 << 16) | CONSUMER_HEARTBEAT_TIME;  
	size = 4;
	writeLocalDict(eDriver.od_Data, 0x1016, 1, &consumer_heartbeat_time, &size, 0); // 节点2

	// ConsumerHeartbeatEntries[2] 197,608
	consumer_heartbeat_time = (3 << 16) | CONSUMER_HEARTBEAT_TIME;
	size = 4;
	writeLocalDict(eDriver.od_Data, 0x1016, 2, &consumer_heartbeat_time, &size, 0);	// 节点3

	// ConsumerHeartbeatEntries[3] 
	consumer_heartbeat_time = (4 << 16) | CONSUMER_HEARTBEAT_TIME;
	size = 4;
	writeLocalDict(eDriver.od_Data, 0x1016, 3, &consumer_heartbeat_time, &size, 0);	// 节点4
}

///////////////////////////////////////////////////////////////////////
// 从站执行完毕后的回调函数输出，或者通信失败提示
static void readStatus_Callback(CO_Data* d, UNS8 nodeId)
{
	UNS32 abortCode;
	UNS8 res;
    UNS16 status;
    UNS32  size;
	res = getReadResultNetworkDict(eDriver.od_Data, nodeId, (void*)&status, &size, &abortCode);
	status_test = status;
	closeSDOtransfer(eDriver.od_Data, nodeId, SDO_CLIENT);
	if(res != SDO_FINISHED)
	{
        kdebug(" Get statusword fail!");
	}
	else
	{
	    kdebug(" servo statusword %x interpolationmode:%d == 1? --- 1 is interpolaition mode \n", status, (status&0x800)==0x800);
    }

}

//获取当前位置
static void readPos_Callback(CO_Data* d, UNS8 nodeId)
{
	UNS32 abortCode;
	UNS8 res;
	UNS32	data;
    UNS32  size;
	res = getReadResultNetworkDict(eDriver.od_Data, nodeId, (void*)&data, &size, &abortCode);
	position_test = data;
	closeSDOtransfer(eDriver.od_Data, nodeId, SDO_CLIENT);
	if(res != SDO_FINISHED) kdebug(" Get statusword fail!");
}


// 获取三个从站的状态字	---调试用
int MGetServoStatus()
{
    UNS8  size;
    size = readNetworkDictCallback (eDriver.od_Data, 2, 0x6041, 0, uint16, readStatus_Callback, 0);
    if(size != 0)
	{
        kdebug(" Get statusword fail!");
    }
    size = readNetworkDictCallback (eDriver.od_Data, 3, 0x6041, 0, uint16, readStatus_Callback, 0);
    if(size != 0)
	{
        kdebug(" Get statusword fail!");
    }

    size = readNetworkDictCallback (eDriver.od_Data, 4, 0x6041, 0, uint16, readStatus_Callback, 0);
    if(size != 0)
	{
        kdebug(" Get statusword fail!");
    }
    return (int)size;
}

// 获取从站状态字
static void writeStatus_Callback(CO_Data* d, UNS8 nodeId)
{
	UNS32 abortCode;
	UNS8 res;
	res = getWriteResultNetworkDict(eDriver.od_Data, nodeId, &abortCode);
	closeSDOtransfer(eDriver.od_Data, nodeId, SDO_CLIENT);
	if(res != SDO_FINISHED)
        kdebug(" Get statusword fail!");

}

// 获取写入从站的60C1 位置
static void write60C1_Callback(CO_Data* d, UNS8 nodeId)
{
	UNS32 abortCode;
	UNS8 res;
	res = getWriteResultNetworkDict(eDriver.od_Data, nodeId,&abortCode);
	closeSDOtransfer(eDriver.od_Data, nodeId, SDO_CLIENT);
	if(res != SDO_FINISHED)
    	kdebug(" write60C1_Callback fail!");


}
///////////////////////////////////////////////////////////////////////

// 从站激活插补的引导流程
int MSetEDriverControl(rt_uint8_t nodeid)
{

	UNS8 size;
	UNS16	data_test;

	//进入引导流程
	UNS8 Interpolation_open = INTERPOLATION_MODE;
	writeNetworkDictCallBack(eDriver.od_Data, nodeid, 0x6060, 0, 1, uint8, &Interpolation_open, writeStatus_Callback, 0);
	SYNC_DELAY;
	
	data_test = 0x6;
	writeNetworkDictCallBack(eDriver.od_Data, nodeid, 0x6040, 0, 2, uint16, (void*)&data_test, writeStatus_Callback, 0);
	SYNC_DELAY;
	
	data_test = 0x7;
	writeNetworkDictCallBack(eDriver.od_Data, nodeid, 0x6040, 0, 2, uint16, (void*)&data_test, writeStatus_Callback, 0);
	SYNC_DELAY;
	
	data_test = 0xF;
	writeNetworkDictCallBack(eDriver.od_Data, nodeid, 0x6040, 0, 2, uint16, (void*)&data_test, writeStatus_Callback, 0);
	SYNC_DELAY;
	
	//读取状态字
	size = readNetworkDictCallback (eDriver.od_Data, nodeid, 0x6040, 0, uint16, readStatus_Callback, 0);
	if(size != 0){
		kdebug(" Get statusword fail!");
	}

	// 激活插补模式
	data_test = 0x1F;
	writeNetworkDictCallBack(eDriver.od_Data, nodeid, 0x6040, 0, 2, uint16, (void*)&data_test, writeStatus_Callback, 0);
	SYNC_DELAY;

	size = readNetworkDictCallback (eDriver.od_Data, nodeid, 0x6064, 0, uint16, readPos_Callback, 0);
	if(size != 0)
	{
		kdebug(" Get statusword fail!");
	}

	//查看插补模式
	kdebug(" ------------------- servo statusword %x  \n", 		status_test 	);
	kdebug(" ------------------- servo position_test %x  \n", 	position_test 	);


	size = readNetworkDictCallback (eDriver.od_Data, nodeid, 0x6064, 0, uint16, readPos_Callback, 0);
	if(size != 0)
	{
		kdebug(" Get statusword fail!");
	}

	return 0;
}

int MSDOSetPosition(rt_uint8_t nodeid, rt_uint32_t *position)
{
	rt_uint16_t pos1, pos2;
	UNS16 interpolation;
	
	// 入参检查
	if(!position)
	{
		kdebug(" position == NULL !");
		return 0;
	}

	pos1 = (rt_uint16_t)(*position & 0xFFFF);
	pos2 = (rt_uint16_t)((*position >> 16) & 0xFFFF);
	
	interpolation = 0x1F;
	writeNetworkDictCallBack(eDriver.od_Data, nodeid, 0x6040, 0, 2, uint16, &interpolation, writeStatus_Callback, 0);

	// 写入从机需要改变的位置节点
	if(nodeid == SERVO_1){
		writeNetworkDictCallBack(eDriver.od_Data, nodeid, 0x60C1, 1, 2, uint16, (void*)&pos1, write60C1_Callback, 0);
		writeNetworkDictCallBack(eDriver.od_Data, nodeid, 0x60C1, 2, 2, uint16, (void*)&pos2, write60C1_Callback, 0);
	}else if(nodeid == SERVO_2 ){
		writeNetworkDictCallBack(eDriver.od_Data, nodeid, 0x60C1, 1, 2, uint16, (void*)&pos1, write60C1_Callback, 0);
		writeNetworkDictCallBack(eDriver.od_Data, nodeid, 0x60C1, 2, 2, uint16, (void*)&pos2, write60C1_Callback, 0);
	}else if(nodeid == SERVO_3 ){
		writeNetworkDictCallBack(eDriver.od_Data, nodeid, 0x60C1, 1, 2, uint16, (void*)&pos1, write60C1_Callback, 0);
		writeNetworkDictCallBack(eDriver.od_Data, nodeid, 0x60C1, 2, 2, uint16, (void*)&pos2, write60C1_Callback, 0);
	}else{
		kdebug(" unknowwn nodeid :  %d!",nodeid);
	}
	return 0;
}

int MPDOSetPosition(rt_uint8_t	nodeID, rt_uint32_t *position)
{
	rt_uint16_t pos1, pos2;
	// 入参检查
	if(!position)
	{
		return 0;
	}

	pos1 = (rt_uint16_t)(*position & 0xFFFF);
	pos2 = (rt_uint16_t)((*position >> 16) & 0xFFFF);

	if(nodeID == SERVO_1){
		Interpolation_value[0] = (UNS16)pos1;
		Interpolation_value[1] = (UNS16)pos2;
	}else if(nodeID == SERVO_2){
		Interpolation_value[2] = (UNS16)pos1;
		Interpolation_value[3] = (UNS16)pos2;
	}else if(nodeID == SERVO_3){
		Interpolation_value[4] = (UNS16)pos1;
		Interpolation_value[5] = (UNS16)pos2;
	}else{
		return -1;
	}
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////
//初始化设备与canopen相关的回调函数
void master402_heartbeatError(CO_Data* d, UNS8 heartbeatID)
{
	kdebug("heartbeatError %d\n", heartbeatID);
}

void master402_initialisation(CO_Data* d)
{
	kdebug("canfestival enter initialisation state\n");
}

void master402_preOperational(CO_Data* d)
{
	rt_thread_t tid;
	kdebug("canfestival enter preOperational state\n");
	tid = rt_thread_create("co_cfg", canopen_start_thread_entry, RT_NULL, 1024, 12, 2);
	if(tid == RT_NULL)
	{
		kdebug("canfestival config thread start failed!\n");
	}
	else
	{
		rt_thread_startup(tid);
	}
}

void master402_operational(CO_Data* d)
{
	kdebug("canfestival enter operational state\n");
}

void master402_stopped(CO_Data* d)
{
	kdebug("canfestival enter stop state\n");
}

void master402_post_sync(CO_Data* d)
{
	kdebug("TestMaster_post_sync\n");
	kdebug("Master:  --(Modes_of_operation_display: %d)--  \n --(Statusword: %d)--  \n --(Position_actual_value: %d)-- \n",
	Modes_of_operation_display,
	Statusword,
	Position_actual_value);
}

void master402_post_TPDO(CO_Data* d)
{

}

void master402_storeODSubIndex(CO_Data* d, UNS16 wIndex, UNS8 bSubindex)
{
	/*TODO : 
	 * - call getODEntry for index and subindex, 
	 * - save content to file, database, flash, nvram, ...
	 * 
	 * To ease flash organisation, index of variable to store
	 * can be established by scanning d->objdict[d->ObjdictSize]
	 * for variables to store.
	 * 
	 * */
	kdebug("storeODSubIndex : %4.4x %2.2x\n", wIndex,  bSubindex);
}

void master402_post_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg, const UNS8 errSpec[5])
{
	kdebug("received EMCY message. Node: %2.2x  ErrorCode: %4.4x  ErrorRegister: %2.2x\n", nodeID, errCode, errReg);
}

///////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////----------------infoos 代码段

/* 
	初始化CAN端口信息定义，用于绑定SCA句柄，实现多端口控制。移植时根据实际数量定义
	设备ID： 8
	失败重发：2次
	底层发送接口
*/

CAN_Handler_t   can1 = {INFOOS_FIRST_DEVICE, 2, CAN1_Send_Msg};  

//	读写指针，可用于获取执行器参数或调用FAST型函数
SCA_Handler_t*   sca1 = RT_NULL;


//	初始化执行器设备。
void sca_init()
{
	
	/* 装载执行器的ID与所使用的CAN端口号 */
	setupActuators( INFOOS_FIRST_DEVICE, &can1);	//	绑定can1和ID：8的设备
	
	/* 获取ID 8 的参数句柄 */
	sca1 = getInstance(INFOOS_FIRST_DEVICE);
//	-----test
	//uint8_t buf[6] = {0x92,0x83,0x54,0x33,0x78,0x23};
	//test_canTransmit(sca1, buf, 6);
//	-----test
	/* 启动所有执行器 */
	enableAllActuators();
	
    /* 等待执行器稳定 */
    rt_thread_delay(500);

}

//  查找在线的设备
void sca_seek_device()
{
    lookupActuators(&can1);
}

//  执行器模式切换和归零
void sca_homing(void)
{
    //  未开机则不执行
    if((sca1->Power_State == Actr_Disable) || !sca1)
    {
        return;
    }   

    //  采用阻塞模式切换执行器到位置模式  (阻塞模式适用于切换模式等操作)
    activateActuatorMode(INFOOS_FIRST_DEVICE, SCA_Profile_Position_Mode, Block);

    /* 归零 1号执行器 */
    setPosition(INFOOS_FIRST_DEVICE, 0);

	/* 等待归零成功 */
	do
	{
		getPosition(INFOOS_FIRST_DEVICE, Unblock);
		rt_thread_delay(100);
	}
	while((sca1->Position_Real > 0.1f)||(sca1->Position_Real < -0.1f));

}

// 非阻塞执行参数保存操作   (非阻塞适用 参数刷新 保存等操作)
void sca_savesetting()
{
    saveAllParams(INFOOS_FIRST_DEVICE, Unblock);
}

//  关闭执行器
void sca_shutdown()
{
    disableAllActuators();
}

//  执行器写入设置的位置  ( 采用快速模式，直接通过指针执行)
void sca_set_position(float pos)
{
    if((sca1->Power_State == Actr_Disable) || !sca1)
    {
        return;
    }
    
    //  执行器切换成操作模式为位置模式并归零
    sca_homing();

    //  执行器设置位置
    setPositionFast(sca1, pos);
    rt_thread_delay(1000);
}

// 执行器写入设置的速度   ( 采用快速模式，直接通过指针执行)
void sca_set_speed(float speed)
{
    if((sca1->Power_State == Actr_Disable) || !sca1)
    {
        return;
    }

    activateActuatorMode(INFOOS_FIRST_DEVICE, SCA_Profile_Velocity_Mode, Block);

    setVelocityFast(sca1, speed);
    rt_thread_delay(1000);
}

//  执行器速度停止  ( 采用快速模式，直接通过指针执行)
void sca_speed_stop()
{
    if((sca1->Power_State == Actr_Disable ) || !sca1)
    {
        return;
    }

    setVelocityFast(sca1, 0);
}


///////////////////////////////////////----------------infoos
