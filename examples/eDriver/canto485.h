/*
    本文件提供 485和can互转的接口，同objDictionary.h 对象字典文件紧密联系

    ##  CAN和485的互转流程：
    1. 首先修改了传输TPDO的传输类型为0，只在内存数据发生改变后进行传输。

    2. TPDO按照 节点ID+功能码+寄存器地址+寄存器数据+CRC效验 (1+1+2+2+2 一共8byte)。不过映射数据是按一个一个字节。
    (方便在接收到数据时进行内存对比，不用将发送的命令组装起来后再对比内存)

    3. RPDO也按照8字节进行接收，每一个映射数据同样是一个字节，响应命令包组成后通过内存对比来判断写入是否成功或是异常。

    4. 使用对象字典结构表里A000h—FFFFh 保留部分，来定义额外使用的映射索引：A100作为发送索引，A200作为接收索引

*/

#ifndef __CANTO485_H__
#define __CANTO485_H__

//  header file
#include "canfestival.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "rtconfig.h"
#include "objDictionary.h"

//  macro
#define MAX_RTU_CMD_LENGTH  8
#define IS_COBID_ILLEGAL(x)   (((x) & 0x80000000) == 0x80000000)

//  declaration
void mcanto485(rt_uint8_t *cmd);
rt_err_t m485tocan(void);
static rt_err_t sendOneRTUEvent(CO_Data *data, rt_uint16_t  pdoNum);

//  debug
#ifdef CANFESTIVAL_USING_KPRINTF_DEBUG

#define candebug(...)		rt_kprintf("DEBUG: "); rt_kprintf(__VA_ARGS__)

#else

#define candebug(...)

#endif



#endif
