
// can转485的实现

#include "canto485.h"

CO_Data*        od_Data = &master402_Data;

//  函数功能：  传入modbus rtu 写入单个寄存器的8个字节请求命令数据
//  备注：      只支持写入单个寄存器的命令
void mcanto485(rt_uint8_t *cmd)
{
    rt_uint32_t i;
    for(i = 0; i < MAX_RTU_CMD_LENGTH; i++)
    {
        can_to_rtu_send_storage[i] = cmd[i];
    } 
}

//  函数功能：  执行成功返回    同写入一样的数据，清除写入以及接收的数据
//  备注：      执行失败打印    地址码+02等错误码2个byte+CRC效验2个byte  (一共5byte)的错误数据
rt_err_t m485tocan(void)
{
    rt_err_t res = RT_EOK;

    if(rt_memcmp(can_to_rtu_send_storage, rtu_to_can_recv_storage, MAX_RTU_CMD_LENGTH) != 0)
    {
        candebug("485tocan() : slave response reply was not correct... ---ERROR MESSAGE:%s\n", rtu_to_can_recv_storage);
        if(rtu_to_can_recv_storage[1] == 0x5 || rtu_to_can_recv_storage[1] == 0x6 || rtu_to_can_recv_storage[1] == 0x8){
            sendOneRTUEvent(od_Data, 3);            
        }
        res = -RT_ERROR;
    }
    else
    {
        //rt_memset(can_to_rtu_send_storage, 0x00, MAX_RTU_CMD_LENGTH);
        rt_memset(rtu_to_can_recv_storage, 0x00, MAX_RTU_CMD_LENGTH);
        candebug("485tocan() : slave response reply was correct");
    }

    return res;
}

// 函数功能：   该函数只做发送单次PDO的功能
// 备注：       该接口不对PDO_INHIBITED进行判断, 但会存储当前的PDO更新到最近一次的传输pdo
static rt_err_t sendOneRTUEvent(CO_Data *data, rt_uint16_t  pdoNum)
{
    rt_err_t    res = RT_EOK;
    rt_uint16_t offset;
    rt_uint32_t cobid;
    Message pdomsg;

    if(!data->CurrentCommunicationState.csPDO)
    {
        candebug("cspdo does's work ! ");        
        return -RT_ERROR;
    }

    offset = (rt_uint16_t)(data->firstIndex->PDO_TRS + pdoNum);
    cobid = *((rt_uint32_t*)data->objdict[offset].pSubindex[1].pObject);

    if( IS_COBID_ILLEGAL(cobid))
    {
        candebug("cob-id illegal ! ");        
        return -RT_ERROR;
    }

    rt_memset(&pdomsg, 0x00, sizeof(Message));
    if(buildPDO(data, pdoNum, &pdomsg))
    {
        candebug(" buildPDO() failed !");        
        return -RT_ERROR;
    }
    //  更新Pdo
    data->PDO_status[pdoNum].last_message = pdomsg;
    //  底层发送
    canSend(data->canHandle, &pdomsg);
    
    return res;
}


