// header file 
#ifndef  __EDRIVER_H__
#define  __EDRIVER_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "rtconfig.h"

#ifdef RT_USING_FINSH
#include <finsh.h>
#endif

#include "canfestival.h"
#include "timers_driver.h"
#include "objDictionary.h"

// 配置从站使用的宏
#define ALL_SERVO_NODE  0
#define SERVO_1    2
#define SERVO_2    3
#define SERVO_3    4

#define PRODUCER_HEARTBEAT_TIME 5000
#define CONSUMER_HEARTBEAT_TIME 10000

#define TPDO_TRANSMISSION_SYNC(x)  (x)  // 累计经历x个同步帧，发送PDO
#define TPDO_TRANSMISSION_SYNC_0    0   // 映射数据改变且收到一个同步帧就发送PDO
#define TPDO_TRANSMISSION_SYNC_254  254 // 映射数据改变或定时器到达就发送PDO
#define RPDO_TRANSMISSION_SYNC  0       // 接收到PDO，在下一个同步帧会更新到应用
#define RPDO_TRANSMISSION_ASYN  254     // 异步：接收到就立即更新到应用

// 伺服运行模式设定
#define INTERPOLATION_MODE 7 // 零差云控的伺服机器目前只支持 插补模式 60C1的低位和高位
#define ENABLE_INTERPOLATION_MODE    (0x1F) //在控制字加入插补模式


// 线程延时 20ms
#define SYNC_DELAY  rt_thread_delay(1000/50)
#define SYNC_TIME   (4000)

// 信号量/case/发送失败后的三次重试
typedef struct servo_config_state
{
	rt_uint8_t case_cnt;
	rt_uint8_t retry_cnt;
    uint8_t    nodeId;
	struct rt_semaphore finish_sem;
}servo_sem_t;

// 主站设备主结构体
typedef struct edriver_no_err_device{
    CO_Data*        od_Data;
    s_BOARD         agv_board;
    servo_sem_t       *semTable;
    void            (*MSetSYNC)(rt_uint32_t cycle);
} eDriverMaster_t;


// 初始化设备。在RT系统下自动启动
int MInitialize_eDriver(void);
static int MCanopen_init(void);

//配置主站和从站相关接口
void MInitNodes(CO_Data* d, UNS32 id);
void MInitNodeExit(CO_Data* d, UNS32 id);
static void MConfigServo(uint8_t nodeId);
static void MConfig_Single_Servo(void *parameter);
static void MConfig_Servo_Param(uint8_t nodeId, servo_sem_t* conf);
static void MConfig_Servo_Param_Callback(CO_Data* d, UNS8 nodeId);
void canopen_start_thread_entry(void *parameter);

// 主配置
int MServoConfig_Fir(servo_sem_t* conf);
int MServoConfig_Sec(servo_sem_t* conf);
int MServoConfig_Trd(servo_sem_t* conf);

// 激活节点同步发生器并设置同步周期(ms)
void MSetSYNC(rt_uint32_t cycle);

// 控制6000h对象组的接口
// 6040h
void MSetControlWord(rt_uint16_t val);
void MBootServoDriver(void);

/// 三个从站的心跳机制设定
void MSetHeartBeat(void);

// 获取三个从站的状态（调试用）
int MGetServoStatus(void);

// 配置模式设置插补模式 :该功能在做从站初始化配置的时候通过网络字典配置进行更改
//void MSetInterpolationMode();

// 激活插补
void MActiveInterpolationMode(void);

// SDO方式发送位置信息
int MSetEDriverControl(rt_uint8_t nodeid);
int MSDOSetPosition(rt_uint8_t nodeid, rt_uint32_t *position);
// PDO方式发送
int MPDOSetPosition(rt_uint8_t	nodeID, rt_uint32_t *position);

//

//canopen sdo 通信函数的封装
#define ReadDicCallBack(d,nodeid,mainindex,subIndexNbr,datatype,callback) \
    readNetworkDictCallback ( d, nodeid, mainindex, subIndexNbr, datatype, callback, 0);

#define GetReadResult(od_Data, nodeId, status, size, abortCode) \
    getReadResultNetworkDict(od_Data, nodeId, status, size,abortCode);

#define WriteDicCallBack(d,  nodeId,  mainindex, subIndex,  count,  dataType, data,  Callback)  \
    writeNetworkDictCallBack (d,  nodeId,  mainindex, subIndex,  count,  dataType, data,  Callback, 0);

#define GetWriteResult(od_Data, nodeId,  status,  size,  abortCode)    \
    getWriteResultNetworkDict(od_Data, nodeId,  status,  size,  abortCode);


// canopen相关注册回调函数
void master402_heartbeatError(CO_Data* d, UNS8 heartbeatID);
void master402_initialisation(CO_Data* d);
void master402_preOperational(CO_Data* d);
void master402_operational(CO_Data* d);
void master402_stopped(CO_Data* d);
void master402_post_sync(CO_Data* d);
void master402_post_TPDO(CO_Data* d);
void master402_storeODSubIndex(CO_Data* d, UNS16 wIndex, UNS8 bSubindex);
void master402_post_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg, const UNS8 errSpec[5]);


#ifdef CANFESTIVAL_USING_KPRINTF_DEBUG

#define kdebug(...)		rt_kprintf("DEBUG: "); rt_kprintf(__VA_ARGS__)

#else

#define kdebug(...)

#endif
///////////////////////////////////////----------------infoos

#define INFOOS_FIRST_DEVICE    0x08    //  infoos 驱动的 CAN-ID 固定定为8

void sca_init(void);
void sca_seek_device(void);
void sca_homing(void);
void sca_set_position(float pos);
void sca_set_speed(float speed);
void sca_savesetting(void);
void sca_shutdown(void);
void sca_speed_stop(void);

///////////////////////////////////////----------------infoos




#endif
