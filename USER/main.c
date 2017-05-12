#include "include.h" 
#include "iic_soft.h"
#include "iic_imu1.h"
#include "iic2.h"
#include "hml_sample.h"
#include "ms5611.h"
#include "alt_kf.h"
#include "flash.h"
#include "led_fc.h"
#include "bmp_adc.h"
#include "flash_w25.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "ucos_task.h"
#include "dog.h"
#include "ukf_task.h"
#include "stm32f4xx_dma.h"
#include "LSM303.h"
#include "LIS3MDL.h"
 /////////////////////////UCOSII启动任务设置///////////////////////////////////
//START 任务
//设置任务优先级
#define START_TASK_PRIO      			20 //开始任务的优先级设置为最低
//设置任务堆栈大小
#define START_STK_SIZE  				64
//任务堆栈	
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata);	
//////////////////////////////////////////////////////////////////////////////
    
OS_EVENT * msg_key;			//按键邮箱事件块	  
OS_EVENT * q_msg;			//消息队列

OS_FLAG_GRP * flags_key;	//按键信号量集
void * MsgGrp[256];			//消息队列存储地址,最大支持256个消息
uint16_t cpuGetFlashSize(void)
{

   return (*(__IO u16*)(0x1FFF7A22));
   // return (*(__IO u32*)(0x1FFF7A20))>>16;
}

//读取ChipID
u32 mcuID[3];
void cpuidGetId(void)
{
    mcuID[0] = *(__IO u32*)(0x1FFF7A10);
    mcuID[1] = *(__IO u32*)(0x1FFF7A14);
    mcuID[2] = *(__IO u32*)(0x1FFF7A18);
}


int main(void)
 { 
	NVIC_PriorityGroupConfig(NVIC_GROUP);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	Initial_Timer_SYS();
	Delay_ms(10);
  I2c_Soft_Init();					//初始化模拟I2C
  IIC_IMU1_Init();
	HMC5883L_SetUp();	
	if(mpu6050.good){
	Delay_ms(10);
	MS5611_Init();						//气压计初始化
	Delay_ms(10);						//延时
	MPU6050_Init(20);   			//加速度计、陀螺仪初始化，配置20hz低通
	Delay_ms(10);						//延时
	}
  //LSM303DLHC_AccInit(0);LSM303DLHC_MagInit(0);
	#if IMU_UPDATE
	LIS3MDL_enableDefault();
	Delay_ms(10);						//延时
	#endif
	altUkfInit();
	LED_Init();								//LED功能初始化
	Delay_ms(10);
//------------------------Uart Init-------------------------------------
	Usart1_Init(115200L);			//GPS_LINK
//	TIM7_Int_Init(100-1,8400-1);		//100ms中断
//	#if EN_DMA_UART1 
//	MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff1,SEND_BUF_SIZE1+2,1);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
//	#endif
	Usart2_Init(576000L);			//IMU_LINK
	#if EN_DMA_UART2
	MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)SendBuff2,SEND_BUF_SIZE2+2,1);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
	#endif
	#if USE_LASER_AVOID
	Usart4_Init(115200L);     //IMU2 Link
	#else
	Usart4_Init(256000L);     //IMU2 Link
	#endif
	#if EN_DMA_UART4 
	MYDMA_Config(DMA1_Stream4,DMA_Channel_4,(u32)&UART4->DR,(u32)SendBuff4,SEND_BUF_SIZE4+2,0);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
	#endif
	#if defined(SONAR_USE_UART)  
	Usart3_Init(9600L);    		//SONAR
	#else
	Ultrasonic_Init();
	#endif
	//#if EN_DMA_UART3
	//MYDMA_Config(DMA1_Stream3,DMA_Channel_4,(u32)&USART3->DR,(u32)SendBuff3,SEND_BUF_SIZE3+2,2);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
	// #endif
  Uart5_Init(115200L);			//FLOW
	Delay_ms(10);
//-------------------------Para Init------------------------------------	
	//Para_Init();							//参数初始化
	W25QXX_Init();			//W25QXX初始化
	while(W25QXX_ReadID()!=W25Q32&&W25QXX_ReadID()!=W25Q16)								//检测不到W25Q128
	{	LEDRGB_COLOR(RED);
		Delay_ms(100);
		LEDRGB_COLOR(BLACK);
		Delay_ms(100);
	}
	READ_PARM();//读取参数
	Delay_ms(10);
//-----------------------Mode &  Flag init--------------------	
//--system
	fly_ready=0;
	mode.en_imu_ekf=0;


	//-----------------DMA Init--------------------------
#if EN_DMA_UART4 
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
	MYDMA_Enable(DMA1_Stream4,SEND_BUF_SIZE4+2);     //开始一次DMA传输！
#endif
#if EN_DMA_UART2
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
	MYDMA_Enable(DMA1_Stream6,SEND_BUF_SIZE2+2);     //开始一次DMA传输！	
#endif
#if EN_DMA_UART3
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
	MYDMA_Enable(DMA1_Stream3,SEND_BUF_SIZE3+2);     //开始一次DMA传输！	
#endif
#if EN_DMA_UART1 
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
	MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE1+2);     //开始一次DMA传输！	 
#endif	
#if EN_TIM_INNER
  TIM3_Int_Init(25-1,8400-1);	//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数5000次为500ms    
#endif	
  TIM3_Int_Init(25-1,8400-1);	//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数5000次为500ms   
	Delay_ms(20);//上电延时
	IWDG_Init(4,1500); //与分频数为64,重载值为500,溢出时间为1s	
	//---------------初始化UCOSII--------------------------

//	while(1)
//	{Ultra_Duty(); 
//		//ukf_task(flow_matlab_data[2],flow_matlab_data[3],flow_matlab_data[0],flow_matlab_data[1],0.04);
//		
//			static char flag[2]={1,0};
//		oldx_nav(0,0,baro_matlab_data[0],
//		flow_matlab_data[2],flow_matlab_data[3],0,
//		flow_matlab_data[0],flow_matlab_data[1],baro_matlab_data[1],
//		0.04,flag,0);
//		Delay_ms(40);
//	}
//  TIM3_Int_Init(500-1,8400-1);	//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数5000次为500ms   
	OSInit();  	 				
	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
	OSStart();	    
}
 

//开始任务
void start_task(void *pdata)
{
  OS_CPU_SR cpu_sr=0;
	u8 err;	    	    
	pdata = pdata; 	
	msg_key=OSMboxCreate((void*)0);		//创建消息邮箱
	q_msg=OSQCreate(&MsgGrp[0],256);	//创建消息队列
 	flags_key=OSFlagCreate(0,&err); 	//创建信号量集		  
	  
	OSStatInit();					//初始化统计任务.这里会延时1秒钟左右
	//注册软件定时器
	tmr1=OSTmrCreate(10,10,OS_TMR_OPT_PERIODIC,(OS_TMR_CALLBACK)tmr1_callback,0,"tmr1",&err);		//100ms执行一次  cpu使用率
	OSTmrStart(tmr1,&err);//启动软件定时器1				 	
	tmr2=OSTmrCreate(10,5,OS_TMR_OPT_PERIODIC,(OS_TMR_CALLBACK)tmr2_callback,0,"tmr2",&err);		//50ms执行一次  LED&&MODE
	OSTmrStart(tmr2,&err);//启动软件定时器1				 	
 	OS_ENTER_CRITICAL();			//进入临界区(无法被中断打断)    
 	//注册线程 	
	OSTaskCreate(outer_task,(void *)0,(OS_STK*)&OUTER_TASK_STK[OUTER_STK_SIZE-1],OUTER_TASK_PRIO);
	#if !EN_TIM_INNER
	OSTaskCreate(inner_task,(void *)0,(OS_STK*)&INNER_TASK_STK[INNER_STK_SIZE-1],INNER_TASK_PRIO);
	#endif
	OSTaskCreate(ekf_task,(void *)0,(OS_STK*)&EKF_TASK_STK[EKF_STK_SIZE-1],EKF_TASK_PRIO);
	OSTaskCreate(flow_task1,(void *)0,(OS_STK*)&FLOW_TASK_STK[FLOW_STK_SIZE-1],FLOW_TASK_PRIO);
	OSTaskCreate(baro_task,(void *)0,(OS_STK*)&BARO_TASK_STK[BARO_STK_SIZE-1],BARO_TASK_PRIO);
	OSTaskCreate(sonar_task,(void *)0,(OS_STK*)&SONAR_TASK_STK[SONAR_STK_SIZE-1],SONAR_TASK_PRIO);	
	//OSTaskCreate(uart_task,(void *)0,(OS_STK*)&UART_TASK_STK[UART_STK_SIZE-1],UART_TASK_PRIO);
	OSTaskCreate(error_task,(void *)0,(OS_STK*)&ERROR_TASK_STK[ERROR_STK_SIZE-1],ERROR_TASK_PRIO);
	//--
 	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.
	OS_EXIT_CRITICAL();				//退出临界区(可以被中断打断)
}
   

//信号量集处理任务
void flags_task(void *pdata)
{	
	u16 flags;	
	u8 err;	    						 
	while(1)
	{
		flags=OSFlagPend(flags_key,0X001F,OS_FLAG_WAIT_SET_ANY,0,&err);//等待信号量
 		
		OSFlagPost(flags_key,0X001F,OS_FLAG_CLR,&err);//全部信号量清零
 	}
}
   		    


