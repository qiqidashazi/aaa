#include <stdbool.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "canfestival.h"
#include "timer.h"
#include "timers_driver.h"


/************************** Module variables **********************************/
static rt_sem_t canfstvl_timer_sem = RT_NULL;
static rt_device_t canfstvl_timer_dev=RT_NULL;
static rt_hwtimerval_t last_timer_val;


void setTimer(TIMEVAL value) // 5 usec
{
	rt_hwtimerval_t val;
	
	val.sec = value / 1000000;
	val.usec = value % 1000000;
	/// Avoid invalid 0 timeout.
	/// See here for detailed reference: https://club.rt-thread.org/ask/question/11194.html .
	if(val.usec < MIN_TIMER_TIMEOUT_US){
		val.usec = MIN_TIMER_TIMEOUT_US;
	}

	last_timer_val.sec = 0;
	last_timer_val.usec = 0;
	// 向can设备写入数据，猜测RT系统会在写入的这个操作进行释放信号量	也就是串口有数据产生的时候。
	// 读取数据的线程获取到信号量后，等待锁的释放。
	rt_device_write(canfstvl_timer_dev, 0, &val, sizeof(val));
}

TIMEVAL getElapsedTime(void)
{
	rt_hwtimerval_t val;
    
	rt_device_read(canfstvl_timer_dev, 0, &val, sizeof(val));

	return (val.sec - last_timer_val.sec) * 1000000 + (val.usec - last_timer_val.usec);
}

static void canopen_timer_thread_entry(void* parameter)
{	
	while(1)
	{
		rt_sem_take(canfstvl_timer_sem, RT_WAITING_FOREVER);	
		EnterMutex();							// 等待StartTimerLoop 写入数据时释放信号量的操作
		rt_device_read(canfstvl_timer_dev, 0, &last_timer_val, sizeof(last_timer_val));
		TimeDispatch();	//时间调度
		LeaveMutex();	//释放锁
	}

}


static rt_err_t timer_timeout_cb(rt_device_t dev, rt_size_t size)
{
	rt_sem_release(canfstvl_timer_sem);
    
    return RT_EOK;
}


void initTimer(void)
{
	rt_thread_t tid;
	rt_err_t err;
	rt_hwtimer_mode_t mode;
	int freq = 1000000;

	canfstvl_timer_sem = rt_sem_create("canfstvl", 0, RT_IPC_FLAG_FIFO);	// 初始化为0，线程挂起 等待唤醒  查阅RT内核代码得出

	canfstvl_timer_dev = rt_device_find(CANFESTIVAL_TIMER_DEVICE_NAME);
	RT_ASSERT(canfstvl_timer_dev != RT_NULL);
	err = rt_device_open(canfstvl_timer_dev, RT_DEVICE_OFLAG_RDWR);
	if (err != RT_EOK)
    {
        rt_kprintf("CanFestival open timer Failed! err=%d\n", err);
        return;
    }

	rt_device_set_rx_indicate(canfstvl_timer_dev, timer_timeout_cb);

	err = rt_device_control(canfstvl_timer_dev, HWTIMER_CTRL_FREQ_SET, &freq);
    if (err != RT_EOK)
    {
        rt_kprintf("Set Freq=%dhz Failed\n", freq);
    }

    mode = HWTIMER_MODE_ONESHOT;
    err = rt_device_control(canfstvl_timer_dev, HWTIMER_CTRL_MODE_SET, &mode);
	
	rt_device_read(canfstvl_timer_dev, 0, &last_timer_val, sizeof(last_timer_val));

	tid = rt_thread_create("cf_timer",
                           canopen_timer_thread_entry, RT_NULL,
                           1024, CANFESTIVAL_TIMER_THREAD_PRIO, 20);
    if (tid != RT_NULL) rt_thread_startup(tid);

}

static TimerCallback_t init_callback;

//set timer usecs
void StartTimerLoop(TimerCallback_t _init_callback) 
{
	init_callback = _init_callback;
	EnterMutex();			// 线程锁首先在这边进行获取 等待警报器执行完毕后 在释放锁让定时器线程进行获取
	SetAlarm(NULL, 0, init_callback, 5, 0);
	//释放锁，读定时器数据的线程此刻便获取
	LeaveMutex();
}


void StopTimerLoop(TimerCallback_t Callback){
	EnterMutex();			// 线程锁首先在这边进行获取 等待警报器执行完毕后 在释放锁让定时器线程进行获取
	SetAlarm(NULL, 0, Callback, 5, 0);
	//释放锁，读定时器数据的线程此刻便获取
	LeaveMutex();

	return;
}
