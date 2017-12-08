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
#include "Soft_I2C_PX4.h"
#include "LIS3MDL.h"
#include "nav_ukf.h"
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
SYSTEM module;
uint16_t cpuGetFlashSize(void)
{
   return (*(__IO u16*)(0x1FFF7A22));
}

//读取ChipID
u32 mcuID[3];
void cpuidGetId(void)
{
    mcuID[0] = *(__IO u32*)(0x1FFF7A10);
    mcuID[1] = *(__IO u32*)(0x1FFF7A14);
    mcuID[2] = *(__IO u32*)(0x1FFF7A18);
}

float ekf_loop_time1;
int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_GROUP);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	Initial_Timer_SYS();
	LED_Init();								//LED功能初始化
	Delay_ms(100);
	//-------------------------Para Init------------------------------------	
	W25QXX_Init();			//W25QXX初始化
	while(W25QXX_ReadID()!=W25Q32&&W25QXX_ReadID()!=W25Q16)								//检测不到W25Q128
	{	LEDRGB_COLOR(RED);
		Delay_ms(100);
		LEDRGB_COLOR(BLACK);
		Delay_ms(100);
	}
	READ_PARM();//读取参数
	Delay_ms(4000);
  IIC_IMU1_Init();
	#if IMU_UPDATE
	LIS3MDL_enableDefault();
	Delay_ms(10);						//延时
	#endif
	altUkfInit();
	Delay_ms(500);
//------------------------Uart Init-------------------------------------
	Usart1_Init(115200L);			//GPS_LINK
	
	#if EN_DMA_UART1 
	MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff1,SEND_BUF_SIZE1+2,1);
	#endif
	Usart2_Init(576000L);			//IMU_LINK
	#if EN_DMA_UART2
	MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)SendBuff2,SEND_BUF_SIZE2+2,1);
	#endif
	#if defined(USE_WIFI_CONTROL)
	Usart4_Init(19200);
	#else
		#if USE_LASER_AVOID
		Usart4_Init(576000L);     //AVOID BOARD
		#else
			#if USE_M100_IMU
			Usart4_Init(115200L);     //IMU2 Link
			#else
			Usart4_Init(576000L);     //PI FLOW
			#endif
		#endif
	#endif
	
	#if EN_DMA_UART4 
	MYDMA_Config(DMA1_Stream4,DMA_Channel_4,(u32)&UART4->DR,(u32)SendBuff4,SEND_BUF_SIZE4+2,0);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
	#endif
	#if defined(SONAR_USE_UART)  
		#if defined(URM07)
		Usart3_Init(19200); 
		#else
		Usart3_Init(9600L);    	
		#endif
	#else
	Ultrasonic_Init();
	#endif
	#if EN_DMA_UART3
	MYDMA_Config(DMA1_Stream3,DMA_Channel_4,(u32)&USART3->DR,(u32)SendBuff3,SEND_BUF_SIZE3+2,2);
	#endif
	#if FLOW_USE_P5A
	Uart5_Init(19200);	
	#else
	#if USE_ANO_FLOW
	Uart5_Init(500000L);			
	#else
	#if FLOW_USE_IIC
	Soft_I2C_Init_PX4();      //FLOW PX4 IIC
	#else
  Uart5_Init(115200L);			//FLOW PX4
	#endif
	#endif
	#endif
	Delay_ms(10);
//-----------------------Mode &  Flag init--------------------	
//--system
	fly_ready=0;
	mode.en_imu_ekf=0;
	//-----------------DMA Init--------------------------
#if EN_DMA_UART4 
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);    
	MYDMA_Enable(DMA1_Stream4,SEND_BUF_SIZE4+2);     
#endif
#if EN_DMA_UART2
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE); 
	MYDMA_Enable(DMA1_Stream6,SEND_BUF_SIZE2+2);    
#endif
#if EN_DMA_UART3
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);   
	MYDMA_Enable(DMA1_Stream3,SEND_BUF_SIZE3+2);    
#endif
#if EN_DMA_UART1 
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);   
	MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE1+2);    
#endif	
#if EN_TIM_INNER
  TIM3_Int_Init(50-1,8400-1);
#endif	
  TIM3_Int_Init(50-1,8400-1);	
	Delay_ms(20);
	IWDG_Init(4,500*3); //与分频数为64,重载值为500,溢出时间为1s	
	#if !USE_UKF_FROM_AUTOQUAD
	#define NO_UCOS 0
	#else
	#define NO_UCOS 1
	#endif
	#if NO_UCOS
	while(1)
	{
	static u8 cnt1;
if(cnt1++>5){cnt1=0;		
	LEDRGB_STATE();	
	}
if(imu_feed_dog==1&&FC_CONNECT==1)
	IWDG_Feed();
  ekf_loop_time1 = Get_Cycle_T(GET_T_EKF);		
  static u8 cnt4,cnt3;
	
	LSM6_readAcc(0);
	LSM6_readGyro(0);
	if(cnt4++>=3){cnt4=0;
	LIS3MDL_read(0);//80hz
	}
	LIS_Data_Prepare(ekf_loop_time1)	;
	MPU6050_Data_Prepare( ekf_loop_time1 );
	
	if(imu_feed_dog==1&&FC_CONNECT==1)
	IWDG_Feed();//喂狗		
	static u8 cnt_sonar;
	if (cnt_sonar++>10){cnt_sonar=0;
		if(fly_ready||en_ble_debug)
		Ultra_Duty(); 
	}
	#define SEL_AHRS 1

#if SEL_AHRS
	madgwick_update_new(
	imu_fushion.Acc.x, imu_fushion.Acc.y, imu_fushion.Acc.z,
	my_deathzoom_2(imu_fushion.Gyro_deg.x,0.0)*DEG_RAD, my_deathzoom_2(imu_fushion.Gyro_deg.y,0.0)*DEG_RAD, my_deathzoom_2(imu_fushion.Gyro_deg.z,0.0)*DEG_RAD*1.2,
	imu_fushion.Mag_Val.x, imu_fushion.Mag_Val.y, imu_fushion.Mag_Val.z,
	&RollRm,&PitchRm,&YawRm,ekf_loop_time1);	
//	RollR=RollRm;
//	PitchR=PitchRm;
//	YawR=YawRm;
	reference_vr[0]=reference_vr_imd_down[0];
	reference_vr[1]=reference_vr_imd_down[1]; 
	reference_vr[2]=reference_vr_imd_down[2];
	q_nav[0]=ref_q[0]=ref_q_imd_down[0]; 		
	q_nav[1]=ref_q[1]=ref_q_imd_down[1]; 
	q_nav[2]=ref_q[2]=ref_q_imd_down[2]; 
	q_nav[3]=ref_q[3]=ref_q_imd_down[3]; 	

	ukf_pos_task_qr(0,0,Yaw,flow_matlab_data[2],flow_matlab_data[3],LIMIT(flow_matlab_data[0],-3,3),LIMIT(flow_matlab_data[1],-3,3),ekf_loop_time1);
  RollR=AQ_ROLL;
	PitchR=AQ_PITCH;
	YawR=AQ_YAW;
	Yaw_mid_down=Yaw=To_180_degrees(YawR);	
	Pitch_mid_down=Pitch=PitchR;
	Roll_mid_down=Roll=RollR;
#else	
	ukf_pos_task_qr(0,0,Yaw,flow_matlab_data[2],flow_matlab_data[3],LIMIT(flow_matlab_data[0],-3,3),LIMIT(flow_matlab_data[1],-3,3),ekf_loop_time1);
  RollR=RollRm=AQ_ROLL;
	PitchR=PitchRm=AQ_PITCH;
	YawR=YawRm=AQ_YAW;

	q_nav[0]=ref_q[0]=ref_q_imd_down[0]=UKF_Q1; 		
	q_nav[1]=ref_q[1]=ref_q_imd_down[1]=UKF_Q2; 
	q_nav[2]=ref_q[2]=ref_q_imd_down[2]=UKF_Q3; 
	q_nav[3]=ref_q[3]=ref_q_imd_down[3]=UKF_Q4; 
	reference_vr_imd_down[0] = 2*(ref_q_imd_down[1]*ref_q_imd_down[3] - ref_q_imd_down[0]*ref_q_imd_down[2]);
	reference_vr_imd_down[1] = 2*(ref_q_imd_down[0]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[3]);
	reference_vr_imd_down[2] = 1 - 2*(ref_q_imd_down[1]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[2]);	
	reference_vr[0]=reference_vr_imd_down[0];
	reference_vr[1]=reference_vr_imd_down[1]; 
	reference_vr[2]=reference_vr_imd_down[2];
	Yaw_mid_down=Yaw=To_180_degrees(YawR);	
	Pitch_mid_down=Pitch=PitchR;
	Roll_mid_down=Roll=RollR;
#endif	


//flow
float flow_height_fliter;		
static float acc_neo_off[3];
FLOW_RAD flow_rad_use;

	 #if FLOW_USE_IIC
		Read_Px4flow();		
	 #endif	
	 #if SENSOR_FORM_PI_FLOW&&!SENSOR_FORM_PI_FLOW_SONAR_NOT
	 if(!pi_flow.insert)
	 flow_height_fliter=0.666;
	 else if(pi_flow.z>4)
	 flow_height_fliter=4;
	 else
	 flow_height_fliter=pi_flow.z;
   #else
	 if(!ultra_ok)
	 flow_height_fliter=0.666;
	 else if(ALT_POS_SONAR3>4)
	 flow_height_fliter=4;
	 else
	 flow_height_fliter=ALT_POS_SONAR3;	
	 #endif
	 flow_sample();
	 #if FLOW_USE_P5A
	 qr.use_spd=1;
	 #endif
	 if(qr.use_spd==0)
	 {
	 flow_rad_use.time_usec=flow_rad_use.integration_time_us=flow_rad.integration_time_us;
	 flow_rad_use.integrated_xgyro=flow_rad.integrated_xgyro;
	 flow_rad_use.integrated_ygyro=flow_rad.integrated_ygyro;
   flow_rad_use.integrated_zgyro=flow_rad.integrated_zgyro;		 
	 flow_rad_use.integrated_x=flow_rad.integrated_x;
	 flow_rad_use.integrated_y=flow_rad.integrated_y;
	 }
	 else
	 {
	 flow_rad_use.integration_time_us=integration_timespan;
	 flow_rad_use.integrated_xgyro=accumulated_gyro_x;
	 flow_rad_use.integrated_ygyro=accumulated_gyro_y;
   flow_rad_use.integrated_zgyro=accumulated_gyro_z;		 
	 flow_rad_use.integrated_x=accumulated_flow_x;
	 flow_rad_use.integrated_y=accumulated_flow_y;
	 }	 
	  float flow_loop_time = Get_Cycle_T(GET_T_FLOW);			
	  if(flow_loop_time<0.001)flow_loop_time=0.01;
	  #if SENSOR_FORM_PI_FLOW
	  flow_ground_temp[0]=pi_flow.sensor.spdy;
		flow_ground_temp[1]=-pi_flow.sensor.spdx;
		flow_ground_temp[2]=pi_flow.sensor.spdy;
	  flow_ground_temp[3]=-pi_flow.sensor.spdx;
	  #else 
	  flow_pertreatment_oldx( &flow_rad_use , flow_height_fliter);
		if(module.pi_flow&&!module.flow&&!module.flow_iic){
		flow_ground_temp[0]=pi_flow.sensor.spdy*0.486;
		flow_ground_temp[1]=-pi_flow.sensor.spdx*0.486;
		flow_matlab_data[2]=pi_flow.sensor.spdy*0.486;
		flow_matlab_data[3]=-pi_flow.sensor.spdx*0.486;
		}else{
		flow_ground_temp[0]=flow_per_out[0];
		flow_ground_temp[1]=flow_per_out[1];
		#if FLOW_USE_P5A
		flow_matlab_data[2]=firstOrderFilter(-flow_per_out[2]*k_flow_devide,&firstOrderFilters[FLOW_LOWPASS_X],flow_loop_time);
		flow_matlab_data[3]=firstOrderFilter(-flow_per_out[3]*k_flow_devide,&firstOrderFilters[FLOW_LOWPASS_Y],flow_loop_time);
    #else
		flow_matlab_data[2]=firstOrderFilter(LIMIT(flow_per_out[2]*k_flow_devide,-6,6),&firstOrderFilters[FLOW_LOWPASS_X],flow_loop_time);
		flow_matlab_data[3]=firstOrderFilter(LIMIT(flow_per_out[3]*k_flow_devide,-6,6),&firstOrderFilters[FLOW_LOWPASS_Y],flow_loop_time);
		#endif
		}
		#endif
		static float a_br[3]={0};	
		static float acc_temp[3]={0};
		a_br[0] =(float) imu_fushion.Acc.x/4096.;
		a_br[1] =(float) imu_fushion.Acc.y/4096.;
		a_br[2] =(float) imu_fushion.Acc.z/4096.;
		// acc
	  if(fabs(a_br[0])<1.5&&fabs(a_br[1])<1.5){
		acc_temp[0] = a_br[1]*reference_vr[2]  - a_br[2]*reference_vr[1] ;
		acc_temp[1] = a_br[2]*reference_vr[0]  - a_br[0]*reference_vr[2] ;
	  acc_temp[2] =(reference_vr[2] *a_br[2] + reference_vr[0] *a_br[0] + reference_vr[1] *a_br[1]);
    if(fabs(acc_temp[0])<1.5&&fabs(acc_temp[1])<1.5){
		static float acc_neo_temp[3]={0};
		#if USE_UKF_FROM_AUTOQUAD
		float accIn[3];
    accIn[0] = IMU_ACCX + UKF_ACC_BIAS_X*1;
    accIn[1] = IMU_ACCY + UKF_ACC_BIAS_Y*1;
    accIn[2] = IMU_ACCZ + UKF_ACC_BIAS_Z*1;
    float acc[3];
    // rotate acc to world frame
    navUkfRotateVectorByQuat(acc, accIn, &UKF_Q1);
		acc_neo_temp[0]=-acc[0];
		acc_neo_temp[1]=-acc[1];
		acc_neo_temp[2]=-(acc[2]+9.87);	
		#else // body acc use AHRS
	  acc_neo_temp[0]=-acc_temp[0]*9.87;
		acc_neo_temp[1]=-acc_temp[1]*9.87;
		acc_neo_temp[2]=(acc_temp[2]-1.0f)*9.87;		
		#endif

		static float acc_neo_temp1[3]={0};
		static float acc_flt[3];
    acc_neo_temp1[0]=Moving_Median(5,5,acc_neo_temp[0]);
		acc_neo_temp1[1]=Moving_Median(6,5,acc_neo_temp[1]);
		acc_neo_temp1[2]=Moving_Median(7,5,acc_neo_temp[2]);	
		acc_flt[0]=firstOrderFilter(acc_neo_temp1[0],&firstOrderFilters[ACC_LOWPASS_X],flow_loop_time);
		acc_flt[1]=firstOrderFilter(acc_neo_temp1[1],&firstOrderFilters[ACC_LOWPASS_Y],flow_loop_time);
		acc_flt[2]=firstOrderFilter(acc_neo_temp1[2],&firstOrderFilters[ACC_LOWPASS_Z],flow_loop_time);		
		
		if(fabs(acc_neo_temp[0])<8.6&&fabs(acc_neo_temp[1])<8.6){
		acc_neo[0]=acc_flt[0];
		acc_neo[1]=acc_flt[1];
		acc_neo[2]=acc_flt[2];
	 	flow_matlab_data[0]=acc_neo[0];//acc
		flow_matlab_data[1]=acc_neo[1];}
	  }
	 }
		float temp_spd[2];
		imu_nav.flow.speed.x=Moving_Median(16,5,FLOW_VEL_X);
		imu_nav.flow.speed.y=Moving_Median(17,5,FLOW_VEL_Y);
	}
	#endif
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
	#if !USE_UKF_FROM_AUTOQUAD	
	OSTaskCreate(outer_task,(void *)0,(OS_STK*)&OUTER_TASK_STK[OUTER_STK_SIZE-1],OUTER_TASK_PRIO);
	#endif
	#if !EN_TIM_INNER
	OSTaskCreate(inner_task,(void *)0,(OS_STK*)&INNER_TASK_STK[INNER_STK_SIZE-1],INNER_TASK_PRIO);
	#endif
	#if !UKF_IN_ONE_THREAD
	OSTaskCreate(ekf_task,(void *)0,(OS_STK*)&EKF_TASK_STK[EKF_STK_SIZE-1],EKF_TASK_PRIO);
	#endif
	OSTaskCreate(flow_task1,(void *)0,(OS_STK*)&FLOW_TASK_STK[FLOW_STK_SIZE-1],FLOW_TASK_PRIO);
	#if !UKF_IN_ONE_THREAD
	OSTaskCreate(baro_task,(void *)0,(OS_STK*)&BARO_TASK_STK[BARO_STK_SIZE-1],BARO_TASK_PRIO);
	#endif
	OSTaskCreate(sonar_task,(void *)0,(OS_STK*)&SONAR_TASK_STK[SONAR_STK_SIZE-1],SONAR_TASK_PRIO);	
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
   		    


