#include <stdint.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "canfestival.h"
#include "timers_driver.h"

struct can_app_struct
{
    const char *name;
    struct rt_semaphore sem;
};

static rt_device_t candev = RT_NULL;
static CO_Data * OD_Data = RT_NULL;
static rt_mutex_t canfstvl_mutex = RT_NULL;

static struct can_app_struct can_data =
{
    CANFESTIVAL_CAN_DEVICE_NAME
};

void EnterMutex(void)
{
	rt_mutex_take(canfstvl_mutex, RT_WAITING_FOREVER);
}

void LeaveMutex(void)
{
	rt_mutex_release(canfstvl_mutex);
}

static rt_err_t  can1ind(rt_device_t dev,  rt_size_t size)
{
    rt_sem_release(&can_data.sem);
    return RT_EOK;
}

unsigned char canSend(CAN_PORT notused, Message *m)
{
	struct rt_can_msg msg;

	msg.id = m->cob_id;		//	cob id
	msg.ide = 0;			// 	是否是标准格式
	msg.rtr = m->rtr;		//	是否是标准数据帧或远程帧
	msg.len = m->len;		//	数据长度
	memcpy(msg.data, m->data, m->len);	//	传输数据
    RT_ASSERT(candev != RT_NULL);
	rt_device_write(candev, 0, &msg, sizeof(msg));
	
    return 0;
}

// master thread recieve message and do can dispatch
void canopen_recv_thread_entry(void* parameter)
{
    struct can_app_struct *canpara = (struct can_app_struct *) parameter;
	struct rt_can_msg msg;
	Message co_msg;

    candev = rt_device_find(canpara->name);
    RT_ASSERT(candev);
    rt_sem_init(&can_data.sem, "co-rx", 0, RT_IPC_FLAG_FIFO);
    rt_device_open(candev, (RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX));
	
	//当串口有数据到达时，rt_device_set_rx_indicate就会调用回调函数，这里的回调函数就是释放信号量，使线程占用信号量，实现原子操作
    rt_device_set_rx_indicate(candev, can1ind);	

    while (1)
    {
        if (rt_sem_take(&can_data.sem, RT_WAITING_FOREVER) == RT_EOK)
        {
			while (rt_device_read(candev, 0, &msg, sizeof(msg)) == sizeof(msg))
			{
				co_msg.cob_id = msg.id;  // cobid 
				co_msg.len = msg.len;   //data len
				co_msg.rtr = msg.rtr;  //date frame
				memcpy(co_msg.data, msg.data, msg.len);	//raw data
				EnterMutex();
				canDispatch(OD_Data, &co_msg);   //  执行can设备接收到的数据。通过canOpen协议进报文解析，通过字典返回数据 
				LeaveMutex();
			}
		}
    }
}

CAN_PORT canOpen(s_BOARD *board, CO_Data * d)
{
	rt_thread_t tid;
	canfstvl_mutex = rt_mutex_create("canfstvl",RT_IPC_FLAG_FIFO);
    
	OD_Data = d;
    tid = rt_thread_create("cf_recv",
                           canopen_recv_thread_entry, &can_data,
                           1024, CANFESTIVAL_RECV_THREAD_PRIO, 20);
    if (tid != RT_NULL) rt_thread_startup(tid);

    return 0;
}


