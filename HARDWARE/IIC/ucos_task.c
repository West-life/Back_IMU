#include "include.h" 
#include "iic_soft.h"
#include "hml_sample.h"
#include "ms5611.h"
#include "ms5611_2.h"
#include "hml5833l.h"
#include "alt_kf.h"
#include "flash.h"
#include "led_fc.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "ucos_task.h"
#include "eso.h"
#include "neuron_pid.h"
#include "sonar_avoid.h"
#include "error.h"
#include "circle.h"
#include "bmp_adc.h"
#include "filter.h"
#include "gps.h"
#include "dog.h"
#include "ukf_task.h"
#include "ukf_baro.h"
#include "avoid.h"
#include "LIS3MDL.h"
//==============================传感器 任务函数==========================
u8 fly_ready;
float inner_loop_time_time;
float inner_loop_time_time_int;
u8 GOL_LINK_BUSY[2]={0,0};

//========================外环  任务函数============================
OS_STK INNER_TASK_STK[INNER_STK_SIZE];
#include "LSM303.h"
void inner_task(void *pdata)
{	static u8 cnt,cnt1,cnt2,cnt3,cnt4,cnt5;			
  static u16 init,cnt_init;
  u8 i,j;	
 	while(1)
	{	
	inner_loop_time_time = Get_Cycle_T(GET_T_MEMS); 	
	if(!init){
	if(cnt_init++>2)
	init=1;
	inner_loop_time_time=0.005;
	}
	else{
							//获取内环准确的执行周期
	if(inner_loop_time_time<0.000002)inner_loop_time_time=0.005;
	#if IMU_UPDATE	
	if(cnt5++>0)	
	#endif 
	{cnt5=0;
		if(mpu6050.good)
		MPU6050_Read(); 															//读取mpu6轴传感器
  if(cnt++>=3){cnt=0;
		if(mpu6050.good)
	  ANO_AK8975_Read();//75hz	
	}			//获取电子罗盘数据	
  if(cnt1++>=2){cnt1=0;
		if(mpu6050.good)
	MS5611_ThreadNew();//100hz
	}
  }
	
	#if IMU_UPDATE	
	LSM6_readAcc(0);
	LSM6_readGyro(0);
	if(cnt4++>=3){cnt4=0;
  LIS3MDL_read(0);//80hz
	}
	LIS_Data_Prepare(inner_loop_time_time)	;
	if(cnt3++>=7){cnt3=0;
		if(!mpu6050.good)
	LP_readbmp(0);//25hz
	}
	#endif
	MPU6050_Data_Prepare( inner_loop_time_time );			//mpu6轴传感器数据处理
  }
	delay_ms(5);
	}
}		



//========================外环  任务函数============================
OS_STK OUTER_TASK_STK[OUTER_STK_SIZE];
float YawR,PitchR,RollR;
float outer_loop_time;
void outer_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;			
  static u8 init,cnt_init;	
	static u8 cal_sel;
 	while(1)
	{	
	outer_loop_time = Get_Cycle_T(GET_T_OUTTER);								//获取外环准确的执行周期	
	if(!init){if(cnt_init++>2)
		init=1;
	outer_loop_time=0.01;
	}
	else{
 // if(cal_sel){cal_sel=0;
	if(outer_loop_time<=0.00002)outer_loop_time=0.01;	
	IMUupdate(0.5f *outer_loop_time,my_deathzoom_2(mpu6050.Gyro_deg.x,0.0), my_deathzoom_2(mpu6050.Gyro_deg.y,0.0), my_deathzoom_2(mpu6050.Gyro_deg.z,0.0),
		mpu6050.Acc.x, mpu6050.Acc.y, mpu6050.Acc.z,&RollR,&PitchR,&YawR);		
	if(mode.en_imu_ekf==0){
	Yaw=YawR;
	Pitch=PitchR;
	Roll=RollR;}
	//}else{cal_sel=1;
	#define SEL_1 1
	#if SEL_1//1 梯度
	MadgwickAHRSupdate(outer_loop_time,my_deathzoom_2(imu_fushion.Gyro_deg.x,0.5)/57.3, my_deathzoom_2(imu_fushion.Gyro_deg.y,0.5)/57.3, 
	my_deathzoom_2(imu_fushion.Gyro_deg.z,0.5)/57.3,(float)imu_fushion.Acc.x/4096., (float)imu_fushion.Acc.y/4096., (float)imu_fushion.Acc.z/4096.,
	0,0,0,
	&Roll_mid_down,&Pitch_mid_down,&Yaw_mid_down);
	#else //0 互补
	MahonyAHRSupdate(outer_loop_time,my_deathzoom_2(imu_fushion.Gyro_deg.x,0.5)/57.3, my_deathzoom_2(imu_fushion.Gyro_deg.y,0.5)/57.3, 
	my_deathzoom_2(imu_fushion.Gyro_deg.z,0.5)/57.3,(float)imu_fushion.Acc.x/4096., (float)imu_fushion.Acc.y/4096., (float)imu_fushion.Acc.z/4096.,
	0,0,0,
	&Roll_mid_down,&Pitch_mid_down,&Yaw_mid_down);
	#endif
	//}
  }
	IWDG_Feed();//喂狗
	delay_ms(10);
	}
}		


//========================EKF  任务函数============================
OS_STK EKF_TASK_STK[EKF_STK_SIZE];
float ekf_loop_time;

#include "FastMath.h"
#include "Quaternion.h"
#include "ekf_ins.h"
//#define USE_EKF
//#define USE_UKF
//#define USE_CKF
//#define USE_SRCKF
//#define USE_9AXIS_EKF
//#define USE_EKF_INS

#ifdef USE_EKF
	#include "EKF.h"
#elif defined USE_UKF
  #include "UKF.h"
#elif defined USE_CKF
	#include "CKF.h"
#elif defined USE_SRCKF
	#include "SRCKF.h"
#elif defined USE_9AXIS_EKF
  #include "miniARSH.h"
#endif


#ifdef USE_EKF
	EKF_Filter ekf;
#elif defined USE_UKF
	UKF_Filter ukf;
#elif defined USE_CKF
	CKF_Filter ckf;
#elif defined USE_SRCKF
	SRCKF_Filter srckf;
#endif
	float fRealGyro[3] = {0}, fRealAccel[3] = {0,0,0};
	float fRealMag[3] = {0}, fRealQ[4] = {0,0,0,0};
	long lQuat[4] = {0,0,0,0};
	float fRPY[3] = {0};
	float fQ[4] = {0};
	int flag_mag[3]={1,1,-1};
	float Kp_ekf=0.3f,Kp_ekf_pr=6; 
	float reference_v_ekf[3];
void ekf_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;			
  static u8 init,cnt_init;	
	float fRPYr[3] = {0};
 	while(1)
	{	
		
		ekf_loop_time = Get_Cycle_T(GET_T_EKF);			
if(cnt_init++>2&&!init){cnt_init=101;
		init=1;
#ifdef USE_EKF
	//Create a new EKF object;
	EKF_New(&ekf);
#elif defined USE_UKF
	//Create a new UKF object;
	UKF_New(&ukf);
#elif defined USE_CKF
	//Create a new CKF object;
	CKF_New(&ckf);
#elif defined USE_SRCKF
	SRCKF_New(&srckf);
#endif
#ifdef USE_EKF
				EKF_Init(&ekf, fRealQ, fRealGyro);
#elif defined USE_UKF
				UKF_Init(&ukf, fRealQ, fRealGyro);
#elif defined USE_CKF
				CKF_Init(&ckf, fRealQ, fRealGyro);
#elif defined USE_SRCKF
				SRCKF_Init(&srckf, fRealAccel, fRealMag);
#elif defined USE_6AXIS_EKF
				EKF_IMUInit(fRealAccel, fRealGyro);
#elif defined USE_6AXIS_FP_EKF
				FP_EKF_IMUInit(fRealAccel, fRealGyro);
#elif defined USE_9AXIS_EKF
				EKF_AHRSInit(fRealAccel, fRealMag);
#endif		

	}
	else{
	  if(ekf_loop_time<0.000002)ekf_loop_time=0.02;
//	u16 i,rxlen;
//	u16 lenx;
//	if(USART3_RX_STA&0X8000)		//接收到一次数据了
//		{
//			rxlen=USART3_RX_STA&0X7FFF;	//得到数据长度
//			for(i=0;i<rxlen;i++)USART1_TX_BUF[i]=USART3_RX_BUF[i];	   
// 			USART3_RX_STA=0;		   	//启动下一次接收
//			USART1_TX_BUF[i]=0;			//自动添加结束符
//			GPS_Analysis(&gpsx,(u8*)USART1_TX_BUF);//分析字符串
// 		}
	
		
	static u8 ekf_gps_cnt;	
	//if(ekf_gps_cnt++>1){ekf_gps_cnt=0;	
	//EKF_INS_GPS_Run(0.015);		
		//GpsUkfProcess(0.05);
	//}
		ukf_pos_task_qr(0,0,Yaw,flow_matlab_data[2],flow_matlab_data[3],LIMIT(flow_matlab_data[0],-3,3),LIMIT(flow_matlab_data[1],-3,3),ekf_loop_time);
 
}
	/*
		    fRealGyro[0] = my_deathzoom_2(mpu6050.Gyro_deg.x,0.0) * DEG_RAD*1;
				fRealGyro[1] = my_deathzoom_2(mpu6050.Gyro_deg.y,0.0) * DEG_RAD*1;
				fRealGyro[2] = my_deathzoom_2(mpu6050.Gyro_deg.z,0.0) * DEG_RAD*1;
			
				fRealAccel[0] = mpu6050.Acc.x/ 4096;
				fRealAccel[1] = mpu6050.Acc.y/ 4096;
				fRealAccel[2] = mpu6050.Acc.z/ 4096;

	 fRealMag[0]=flag_mag[0]*ak8975.Mag_Val.x;
	 fRealMag[1]=flag_mag[1]*ak8975.Mag_Val.y;
	 fRealMag[2]=flag_mag[2]*ak8975.Mag_Val.z;
		
		Quaternion_From6AxisData(fRealQ, fRealAccel, fRealMag);
		float fDeltaTime;		
				if(ekf_loop_time!=0)fDeltaTime=ekf_loop_time;
				else
				fDeltaTime=0.01;	
#ifdef USE_EKF
				EFK_Update(&ekf, fRealQ, fRealGyro, fRealAccel, fRealMag, fDeltaTime);
#elif defined USE_UKF
				UKF_Update(&ukf, fRealQ, fRealGyro, fRealAccel, fRealMag, fDeltaTime);
#elif defined USE_CKF
				CKF_Update(&ckf, fRealQ, fRealGyro, fRealAccel, fRealMag, fDeltaTime);
#elif defined USE_SRCKF
				SRCKF_Update(&srckf, fRealGyro, fRealAccel, fRealMag, fDeltaTime);
#elif defined USE_6AXIS_EKF
				EKF_IMUUpdate(fRealGyro, fRealAccel, fDeltaTime);
#elif defined USE_6AXIS_FP_EKF
				FP_EKF_IMUUpdate(fRealGyro, fRealAccel, fDeltaTime);
#elif defined USE_9AXIS_EKF
				EKF_AHRSUpdate(fRealGyro, fRealAccel, fRealMag, fDeltaTime);
#endif	
#ifdef USE_EKF
			EKF_GetAngle(&ekf, fRPYr);
			EKF_GetQ(&ekf, fQ);
#elif defined USE_UKF
			UKF_GetAngle(&ukf, fRPYr);
			UKF_GetQ(&ukf, fQ);
#elif defined USE_CKF
			CKF_GetAngle(&ckf, fRPYr);
			CKF_GetQ(&ckf, fQ);
#elif defined USE_SRCKF
			SRCKF_GetAngle(&srckf, fRPYr);
			SRCKF_GetQ(&srckf, fQ);
#elif defined USE_6AXIS_EKF
			EKF_IMUGetAngle(fRPYr);
			EKF_IMUGetQ(fQ);
#elif defined USE_6AXIS_FP_EKF
			FP_EKF_IMUGetAngle(fRPYr);
			FP_EKF_IMUGetQ(fQ);
#elif defined USE_9AXIS_EKF
			EKF_AHRSGetAngle(fRPYr);
			EKF_AHRSGetQ(fQ);
#endif
			lQuat[0] = (long)(fQ[0] * 2147483648.0f);
			lQuat[1] = (long)(fQ[1] * 2147483648.0f);
			lQuat[2] = (long)(fQ[2] * 2147483648.0f);
			lQuat[3] = (long)(fQ[3] * 2147483648.0f);
	 fRPY[0] += Kp_ekf_pr *0.1f *(-fRPY[0]+(fRPYr[0]));
	 fRPY[1] += Kp_ekf_pr *0.1f *(-fRPY[1]+(-fRPYr[1]));
	 fRPY[2]  =-fRPYr[2];
	reference_v_ekf[0]= 2*(fQ[1]*fQ[3] - fQ[0]*fQ[2]);
	reference_v_ekf[1]= 2*(fQ[0]*fQ[1] + fQ[2]*fQ[3]);
	reference_v_ekf[2] = 1 - 2*(fQ[1]*fQ[1] + fQ[2]*fQ[2]);
#if NOT_DEBUG_EKF
if(mode.en_imu_ekf) {
	q_nav[0]=fQ[0];
	q_nav[1]=fQ[1];
	q_nav[2]=fQ[2];
	q_nav[3]=fQ[3];
  reference_vr[0]=reference_v.x =reference_v_ekf[0];// 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
	reference_vr[1]=reference_v.y =reference_v_ekf[1];// 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
	reference_vr[2]=reference_v.z =reference_v_ekf[2];// 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);//ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] + r
	Pitch= fRPY[1] ;
	Roll=  fRPY[0] ;
	Yaw=   fRPY[2] ;
}
#endif
*/
	delay_ms(20);
	}
}		


//气压计 任务函数
OS_STK BARO_TASK_STK[BARO_STK_SIZE];
void baro_task(void *pdata)
{							  
 	while(1)
	{ static u8 cnt;
		altUkfProcess(0);
	  Laser_cal();
		delay_ms(20);  
	}
}	

//=======================超声波 任务函数==================
OS_STK SONAR_TASK_STK[SONAR_STK_SIZE];
void sonar_task(void *pdata)
{							  
 	while(1)
	{

		#if defined(SONAR_USE_SCL) 
			if(!Thr_Low)Ultra_Duty_SCL();delay_ms(100);
		#else
			if(fly_ready||en_ble_debug)
				Ultra_Duty(); 
		#if defined(USE_KS103)
			 #if defined(SONAR_SAMPLE1)
				delay_ms(40);
			 #elif defined(SONAR_SAMPLE2)
				delay_ms(100);
		   #elif defined(SONAR_SAMPLE3)
				delay_ms(70);
			 #endif
		#elif defined(USE_US100)
			
			  #if USE_FLOW_SONAR
			 	ultra_distance=flow.hight.originf*1000;//Moving_Median(1,5,temp);
		    sys.sonar=ultra_ok = 1;
			  #endif
				delay_ms(100);
		#endif
		#endif

	}
}

float focal_length_px = (16) / (4.0f * 6.0f) * 1000.0f; //original focal lenght: 12mm pixelsize: 6um, binning 4 enabled
float accumulated_flow_x = 0;
float accumulated_flow_y = 0;
float accumulated_gyro_x = 0;
float accumulated_gyro_y = 0;
float accumulated_gyro_z = 0;
uint16_t accumulated_framecount = 0;
uint16_t accumulated_quality = 0;
uint32_t integration_timespan = 0;
//float k_flow_acc=0.1;
float k_flow_acc[2]={0.1,0.1};
float k_gro_acc=0.1;
float flt_gro=0.03;//1;
float k_time_use=4.2;
void flow_sample(void)
{
uint32_t deltatime=Get_Cycle_T(GET_T_FLOW_SAMPLE)*1000000;
float x_rate = imu_fushion.Gyro_deg.y; // change x and y rates
float y_rate = imu_fushion.Gyro_deg.x;
float z_rate = -imu_fushion.Gyro_deg.z; // z is correct
if (1)
				{
					integration_timespan = deltatime*k_time_use;
					accumulated_flow_x = qr.spdx  / focal_length_px * 1.0f*k_flow_acc[0]; //rad axis swapped to align x flow around y axis
					accumulated_flow_y = qr.spdy / focal_length_px * 1.0f*k_flow_acc[1];//rad
					accumulated_gyro_x = LIMIT(x_rate * deltatime / 1000000.0f*k_gro_acc*flt_gro+accumulated_gyro_x*(1-flt_gro),-fabs(accumulated_flow_x*5),fabs(accumulated_flow_x*5));	//rad
					accumulated_gyro_y = LIMIT(y_rate * deltatime / 1000000.0f*k_gro_acc*flt_gro+accumulated_gyro_y*(1-flt_gro),-fabs(accumulated_flow_y*5),fabs(accumulated_flow_y*5));	//rad
					accumulated_gyro_z = z_rate * deltatime / 1000000.0f*k_gro_acc*flt_gro+accumulated_gyro_z*(1-flt_gro);	//rad
				}

}
void body_to_NEZ(float *vr, float *v, float *q) {
    float w, x, y, z;

    w = q[0];
    x = q[1];
    y = q[2];
    z = q[3];

    vr[0] = w*w*v[0] + 2.0f*y*w*v[2] - 2.0f*z*w*v[1] + x*x*v[0] + 2.0f*y*x*v[1] + 2.0f*z*x*v[2] - z*z*v[0] - y*y*v[0];
    vr[1] = 2.0f*x*y*v[0] + y*y*v[1] + 2.0f*z*y*v[2] + 2.0f*w*z*v[0] - z*z*v[1] + w*w*v[1] - 2.0f*x*w*v[2] - x*x*v[1];
    vr[2] = 2.0f*x*z*v[0] + 2.0f*y*z*v[1] + z*z*v[2] - 2.0f*w*y*v[0] - y*y*v[2] + 2.0f*w*x*v[1] - x*x*v[2] + w*w*v[2];
}

//=======================FLOW 任务函数==================
OS_STK FLOW_TASK_STK[FLOW_STK_SIZE];
float acc_neo[3],flow_ground_temp[4];
float flow_filter[4],flow_rad_fix[4];
float flow_rad_fix_k[2]={0.8,1},flow_rad_fix_k2=0.0;
float dead_rad_fix[2]={40,50};
float flow_height_fliter;
float wz_speed_flow[2];
float w_acc_spd=0.815;
float w_acc_fix=0.8;

#if USE_FLOW_FLY_ROBOT//使用飞行实验室的光流模块
#define K_PIX 1.8*0.7
float k_scale_pix=K_PIX;
float scale_pix=0.0055*K_PIX;//0.002;//.003;//0.005;
#else
#define K_PIX 1
float k_scale_pix=K_PIX;
float scale_pix=0.0055*K_PIX;//0.002;//.003;//0.005;
#endif

float k_acc_forward=0;
u8 MID_CNT_SPD=10;
float k_flp=1-0.1; 
//px4
double rate_threshold = 0.15f; 
float flow_k=0.15f;float k_gro_off=1;
float flow_m[3];  
float flow_module_offset_y=0,flow_module_offset_x=-0.05;//光流安装位移 单位米
float flow_gyrospeed[3];
u8 flow_px_sel=1;
float scale_px4_flow=1300;	float x_flow_orign_temp,y_flow_orign_temp;	
float flow_matlab_data[4];
float baro_matlab_data[2];
float flow_loop_time;
u8 flow_rad_sel;//=1;

void flow_task1(void *pdata)
{		float temp_sonar;			
 FLOW_RAD flow_rad_use;  
 	while(1)
	{float yaw_comp[2];
	 scale_pix=0.0055*k_scale_pix;
	 if(ALT_POS_SONAR2==0)
	 temp_sonar=0.7;
	 else if(ALT_POS_SONAR2>4)
	 temp_sonar=4;
	 else
	 temp_sonar=ALT_POS_SONAR2;
	 float spd_temp[2];
	 flow_height_fliter=temp_sonar;//sonar_filter_bmp((float)temp_sonar/1000,0.01);
	 //flow_height_fliter=sonar_filter_bmp((float)temp_sonar,0.01);
	 //px4
	  flow_sample();
	 if(qr.use_spd==0)//flow_rad_sel)
	 {
	 flow_rad_use.integration_time_us=flow_rad.integration_time_us;
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
	 flow_rad_use.integrated_y=accumulated_flow_y;//*1.3333333333333333333333333333333;
	 }	 
	 
	 
		flow_gyrospeed[0] = flow_rad_use.integrated_xgyro / (float)flow_rad_use.integration_time_us * 1000000.0f;  
		flow_gyrospeed[1] = flow_rad_use.integrated_ygyro / (float)flow_rad_use.integration_time_us * 1000000.0f;  
		flow_gyrospeed[2] = flow_rad_use.integrated_zgyro / (float)flow_rad_use.integration_time_us * 1000000.0f;  
	  static u8 n_flow;
	  static float gyro_offset_filtered[3],att_gyrospeed_filtered[3],flow_gyrospeed_filtered[3];
		float flow_ang[2];
		if(flow_rad_use.integration_time_us){
		//if (fabs(mpu6050.Gyro_deg.y/57.3) < rate_threshold) {  
		if (fabs(flow_gyrospeed[0]) < rate_threshold) {  
		flow_ang[0] = (flow_rad_use.integrated_x / (float)flow_rad_use.integration_time_us * 1000000.0f) * flow_k;//for now the flow has to be scaled (to small)  
		}  
		else {  
		//calculate flow [rad/s] and compensate for rotations (and offset of flow-gyro)  
		flow_ang[0] = ((flow_rad_use.integrated_x - LIMIT(flow_rad_use.integrated_xgyro*k_gro_off,-fabs(flow_rad_use.integrated_x),fabs(flow_rad_use.integrated_x))) / (float)flow_rad_use.integration_time_us * 1000000.0f  
		+ gyro_offset_filtered[0]*0) * flow_k;//for now the flow has to be scaled (to small)  
		}  

		//if (fabs(mpu6050.Gyro_deg.x/57.3) < rate_threshold) {  
		if (fabs(flow_gyrospeed[1]) < rate_threshold) {  
		flow_ang[1] = (flow_rad_use.integrated_y/ (float)flow_rad_use.integration_time_us  * 1000000.0f) * flow_k;//for now the flow has to be scaled (to small)  
		}  
		else {  
		flow_ang[1] = ((flow_rad_use.integrated_y- LIMIT(flow_rad_use.integrated_ygyro*k_gro_off,-fabs(flow_rad_use.integrated_y),fabs(flow_rad_use.integrated_y))) / (float)flow_rad_use.integration_time_us  * 1000000.0f  
		+ gyro_offset_filtered[1]*0) * flow_k;//for now the flow has to be scaled (to small)  
		}  
		

		yaw_comp[0] = - flow_module_offset_y * (flow_gyrospeed[2] - gyro_offset_filtered[2]);  
		yaw_comp[1] = flow_module_offset_x * (flow_gyrospeed[2] - gyro_offset_filtered[2]);  

		/* flow measurements vector */  

		flow_m[0] = -flow_ang[0];  
		flow_m[1] = -flow_ang[1] ;  
		
	
		if(flow_px_sel){
		x_flow_orign_temp=flow_m[1]*scale_px4_flow;
		y_flow_orign_temp=flow_m[0]*scale_px4_flow;		
		}else{
		x_flow_orign_temp=flow.flow_x.origin;
		y_flow_orign_temp=flow.flow_y.origin;		
		}
	}
		//x
		flow_rad_fix[0]=x_flow_orign_temp*sign_flow(mpu6050.Gyro_deg.x,dead_rad_fix[0])*flow_rad_fix_k[0];
		if(fabs(imu_fushion.Gyro_deg.x)<dead_rad_fix[0]*0.7)
		flow_rad_fix[0]=x_flow_orign_temp*sign_flow(mpu6050.Gyro_deg.x,0)*flow_rad_fix_k2; 

		//y
		flow_rad_fix[1]=y_flow_orign_temp*sign_flow(mpu6050.Gyro_deg.y,dead_rad_fix[0])*flow_rad_fix_k[0];
		if(fabs(imu_fushion.Gyro_deg.y)<dead_rad_fix[0]*0.7)
		flow_rad_fix[1]=y_flow_orign_temp*sign_flow(mpu6050.Gyro_deg.y,0)*flow_rad_fix_k2; 

		//z
		flow_rad_fix[2]=x_flow_orign_temp*sign_flow(mpu6050.Gyro_deg.z,dead_rad_fix[1])*flow_rad_fix_k[1];
		flow_rad_fix[3]=y_flow_orign_temp*sign_flow(mpu6050.Gyro_deg.z,dead_rad_fix[1])*flow_rad_fix_k[1];
		flow_filter[0]=Moving_Median(18,10,my_deathzoom_2(x_flow_orign_temp-flow_rad_fix[0]-flow_rad_fix[2],0));
		flow_filter[1]=Moving_Median(19,10,my_deathzoom_2(y_flow_orign_temp-flow_rad_fix[1]-flow_rad_fix[3],0));

		flow_ground_temp[0]=flow_filter[0]*flow_height_fliter*scale_pix;
		flow_ground_temp[1]=flow_filter[1]*flow_height_fliter*scale_pix;
		if (fabsf(flow_gyrospeed[2]) < rate_threshold) {
    flow_ground_temp[2]=my_deathzoom_2(x_flow_orign_temp-flow_rad_fix[0]-flow_rad_fix[2],0)*flow_height_fliter*scale_pix;
		flow_ground_temp[3]=my_deathzoom_2(y_flow_orign_temp-flow_rad_fix[1]-flow_rad_fix[3],0)*flow_height_fliter*scale_pix;
		} else {//偏航较大时，补偿其安装位置引起的偏差；
		flow_ground_temp[2]=my_deathzoom_2(x_flow_orign_temp-flow_rad_fix[0]-flow_rad_fix[2],0)*flow_height_fliter*scale_pix - yaw_comp[1] * flow_k;//1
		flow_ground_temp[3]=my_deathzoom_2(y_flow_orign_temp-flow_rad_fix[1]-flow_rad_fix[3],0)*flow_height_fliter*scale_pix - yaw_comp[0] * flow_k;//0
		//		flow_m[0] = -flow_ang[0] * flow_dist - yaw_comp[0] * params.flow_k;
		//		flow_m[1] = -flow_ang[1] * flow_dist - yaw_comp[1] * params.flow_k;
		}
										
		//flow_ground_temp[2]=my_deathzoom_2(x_flow_orign_temp-flow_rad_fix[0]-flow_rad_fix[2],0)*flow_height_fliter*scale_pix;
		//flow_ground_temp[3]=my_deathzoom_2(y_flow_orign_temp-flow_rad_fix[1]-flow_rad_fix[3],0)*flow_height_fliter*scale_pix;
		float a_br[3],tmp[3];	
		float acc_temp[3];
		a_br[0] =(float) imu_fushion.Acc.x/4096.;//16438.;
		a_br[1] =(float) imu_fushion.Acc.y/4096.;//16438.;
		a_br[2] =(float) imu_fushion.Acc.z/4096.;//16438.;
		// acc
		tmp[0] = a_br[0];
		tmp[1] = a_br[1];
		tmp[2] = a_br[2];

		acc_temp[0] = tmp[1]*reference_vr_imd_down[2]  - tmp[2]*reference_vr_imd_down[1] ;
		acc_temp[1] = tmp[2]*reference_vr_imd_down[0]  - tmp[0]*reference_vr_imd_down[2] ;
		float accIn[3],acc_body_temp[3];
		accIn[0] =(float) imu_fushion.Acc.x/4096.*9.8;//16438.;
		accIn[1] =(float) imu_fushion.Acc.y/4096.*9.8;//16438.;
		accIn[2] =(float) imu_fushion.Acc.z/4096.*9.8;//16438.;
    body_to_NEZ(acc_body_temp, accIn, ref_q_imd_down);
		
		
		acc_neo[0]=(float)(-acc_temp[0])*9.87;
		acc_neo[1]=(float)(-acc_temp[1])*9.87;
		flow_matlab_data[0]=acc_neo[0];
		flow_matlab_data[1]=acc_neo[1];
		acc_neo[2]=((float)((int)((acc_temp[2]-1.0f)*200)))/200*9.87;	
		flow_matlab_data[2]=flow_ground_temp[2];
		flow_matlab_data[3]=flow_ground_temp[3];
		//ukf_task(flow_matlab_data[2],flow_matlab_data[3],flow_matlab_data[0],flow_matlab_data[1],0.02);
		flow_loop_time = Get_Cycle_T(GET_T_FLOW);			
		//ukf_task(0,0,0,0,0.02);
		FlowUkfProcess(0);
		float temp_spd[2];
		temp_spd[0]=Moving_Median(16,MID_CNT_SPD,FLOW_VEL_X);//ALT_VEL_BMP+acc_neo[0]*k_acc_forward;
		temp_spd[1]=Moving_Median(17,MID_CNT_SPD,FLOW_VEL_Y);//ALT_VEL_SONAR+acc_neo[1]*k_acc_forward;
		imu_nav.flow.speed.x=imu_nav.flow.speed.x*(1-k_flp)+k_flp*temp_spd[0];
		imu_nav.flow.speed.y=imu_nav.flow.speed.y*(1-k_flp)+k_flp*temp_spd[1];
		delay_ms(20); 
	}
}	


//=======================DEBUG串口 任务函数===========================
OS_STK  UART_TASK_STK[UART_STK_SIZE];
u8 en_imu_debug,force_imu_debug;
void uart_task(void *pdata)
{	static u8 cnt[4];					 		
 	while(1)
	{
//			#if !EN_TIM_INNER
//		#if EN_DMA_UART2 			
//			     if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET)//等待DMA2_Steam7传输完成
//								{ 
//							DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);//清除DMA2_Steam7传输完成标志
//							clear_nrf_uart();		
//							GOL_LINK_TASK_DMA();
//					    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
//							MYDMA_Enable(DMA1_Stream6,SEND_BUF_SIZE2+2);     //开始一次DMA传输！	
//								}							
//					#else
//								 GOL_LINK_TASK();
//					#endif
//		
//		en_ble_debug=1;
//	if(cnt[1]++>1-1){cnt[1]=0;	
//							
//								if(en_ble_debug){//!GOL_LINK_BUSY[0]){
//								//GOL_LINK_BUSY[1]=1;
//								switch(UART_UP_LOAD_SEL)
//								{
//								case 0://海拔速度
//								Send_BLE_DEBUG((int16_t)(0),flow_rad.integrated_y*100, flow_rad.integrated_ygyro*100,
//								0,flow.flow_x.origin,flow.flow_y.origin,
//								0,flow_m[0]*1000,flow_m[1]*1000);break;
//								case 1://海拔速度
//								Send_BLE_DEBUG((int16_t)(0),flow.flow_x.origin,flow.flow_y.origin,
//								0,ALT_VEL_SONAR*1000,flow_ground_temp[3]*1000,
//								0,ALT_VEL_BMP*1000,flow_ground_temp[2]*1000);break;
//								case 2://海拔速度
//								Send_BLE_DEBUG((int16_t)(0),x_flow_orign_temp,0,
//								0,flow.flow_x.origin,0,
//								0,0,flow.flow_y.origin);break;
//								case 3://海拔速度
//								Send_BLE_DEBUG((int16_t)(baroAlt),ALT_POS_BMP*1000,ALT_POS_BMP_EKF*1000,
//								0,-ALT_VEL_BMP*100,ALT_VEL_BMP_EKF*100,
//								0,acc_z_view[1],acc_z_view[0]);break;		
//								case 4://海拔速度
//								Send_BLE_DEBUG((int16_t)(0),flow_matlab_data[0]*1000,0,
//								0,0*100,flow_matlab_data[1]*1000,
//								0,0,v_test[1]*1000);break;		
//								case 5://海拔速度
//								Send_BLE_DEBUG((int16_t)(0*100),X_ukf[0]*1000,X_ukf[3]*1000,
//								0,X_ukf[1]*1000,flow_matlab_data[2]*1000*K_spd_flow,
//								FLOW_VEL_Y*000,X_ukf[4]*1000,flow_matlab_data[3]*1000*K_spd_flow);break;	
//								case 6://海拔速度
//								Send_BLE_DEBUG((int16_t)(X_ukf_baro[3]*100),baro_matlab_data[0]*100,X_ukf_baro[0]*100,
//								X_ukf_baro[4]*100,X_ukf_baro[1]*100,ALT_VEL_BMP_EKF*100,
//								0,ALT_VEL_BMP_UNION*100,X_kf_sonar[1]*100);break;	
//								case 7:
//								Send_BLE_DEBUG((int16_t)(laser_buf[1]),Laser_avoid[1],Laser_avoid[2],
//								(int16_t)(Laser_avoid[3]),Laser_avoid[4],Laser_avoid[5],
//								(int16_t)(Laser_avoid[6]),Laser_avoid[7],Laser_avoid[8]);break;	
//								case 8:
//								Send_BLE_DEBUG(0,accumulated_flow_y*1000,flow_rad.integrated_y*1000,
//								0,accumulated_gyro_z*1000,flow_rad.integrated_zgyro*1000,
//								0,0,0);break;	
//								case 9:
//								Send_BLE_DEBUG(0,0,ALT_POS_SONAR3*1000,
//								0,ultra_distance,0,
//								0,0,ALT_POS_SONAR2*1000);break;
//								case 10:
//								Send_BLE_DEBUG(0,0,X_kf_yaw[0],
//								0,ultra_distance,0,
//								0,0,yaw_mag_view[4]);break;
//								case 11:
//								Send_BLE_DEBUG(velNorth*1000,velNorth_gps*1000,0*X_KF_NAV[0][1]*1000,
//								velEast*1000,velEast_gps*1000,0*X_KF_NAV[1][1]*1000,
//								flow_matlab_data[0]*100,flow_matlab_data[1]*100,Yaw);break;
//								case 12:
//								Send_BLE_DEBUG(flow_rad.integrated_xgyro*1000,accumulated_flow_x*1000,accumulated_gyro_x*1000,
//								0,flow_rad.integrated_xgyro*1000,flow_rad.integrated_x*1000,
//								0,flow_rad.integrated_x*1000,accumulated_flow_x*1000);break;
//								}				
//								//GOL_LINK_BUSY[1]=0;		
//								} 
//								
//								
//				}				
//	#endif
		delay_ms(5);  
	}
}	
u8 UART_UP_LOAD_SEL=6;//<------------------------------UART UPLOAD DATA SEL
float time_uart;
void TIM3_IRQHandler(void)
{
	OSIntEnter();static u16 cnt,cnt1,cnt_init,init;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
	time_uart = Get_Cycle_T(GET_T_UART); 	
	if(!init){
	if(cnt_init++>2)
	init=1;
	time_uart=0.0025;
	}
	else{
							//获取内环准确的执行周期
	if(time_uart<0.000002)time_uart=0.0025;
		#if EN_DMA_UART2 			
			     if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET)//等待DMA2_Steam7传输完成
								{ 
							DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);//清除DMA2_Steam7传输完成标志
							clear_nrf_uart();		
							//data_per_uart4(SEND_IMU_MEMS);
							//data_per_uart4(SEND_IMU_ATT);		
							//data_per_uart4(SEND_IMU_FLOW);
							GOL_LINK_TASK_DMA();
					    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
							MYDMA_Enable(DMA1_Stream6,nrf_uart_cnt+2);     //开始一次DMA传输！	
								}							
					#else
								 GOL_LINK_TASK();
					#endif
						 		
	        //en_ble_debug=1;
	if(cnt++>4-1){cnt=0;	
							
								if(en_ble_debug){//!GOL_LINK_BUSY[0]){
								GOL_LINK_BUSY[1]=1;
								switch(UART_UP_LOAD_SEL)
								{
								case 0://海拔速度
								Send_BLE_DEBUG((int16_t)(0),flow_rad.integrated_y*100, flow_rad.integrated_ygyro*100,
								0,flow.flow_x.origin,flow.flow_y.origin,
								0,flow_m[0]*1000,flow_m[1]*1000);break;
								case 1://海拔速度
								Send_BLE_DEBUG((int16_t)(0),flow.flow_x.origin,flow.flow_y.origin,
								0,ALT_VEL_SONAR*1000,flow_ground_temp[3]*1000,
								0,ALT_VEL_BMP*1000,flow_ground_temp[2]*1000);break;
								case 2://海拔速度
								Send_BLE_DEBUG((int16_t)(0),x_flow_orign_temp,0,
								0,flow.flow_x.origin,0,
								0,0,flow.flow_y.origin);break;
								case 3://海拔速度
								Send_BLE_DEBUG((int16_t)(baroAlt),ALT_POS_BMP*1000,ALT_POS_BMP_EKF*1000,
								0,-ALT_VEL_BMP*100,ALT_VEL_BMP_EKF*100,
								0,acc_z_view[1],acc_z_view[0]);break;		
								case 4://海拔速度
								Send_BLE_DEBUG((int16_t)(0),flow_matlab_data[0]*1000,0,
								0,0*100,flow_matlab_data[1]*1000,
								0,0,v_test[1]*1000);break;		
								case 5://海拔速度
								Send_BLE_DEBUG((int16_t)(0*100),X_ukf[0]*1000,X_ukf[3]*1000,
								0,X_ukf[1]*1000,flow_matlab_data[2]*1000*K_spd_flow,
								FLOW_VEL_Y*000,X_ukf[4]*1000,flow_matlab_data[3]*1000*K_spd_flow);break;	
								case 6://海拔速度
								Send_BLE_DEBUG((int16_t)(X_ukf_baro[3]*100),baro_matlab_data[0]*100,X_ukf_baro[0]*100,
								X_ukf_baro[4]*100,X_ukf_baro[1]*100,ALT_VEL_BMP_EKF*100,
								0,ALT_VEL_BMP_UNION*100,X_kf_sonar[1]*100);break;	
								case 7:
								Send_BLE_DEBUG((int16_t)(laser_buf[1]),Laser_avoid[1],Laser_avoid[2],
								(int16_t)(Laser_avoid[3]),Laser_avoid[4],Laser_avoid[5],
								(int16_t)(Laser_avoid[6]),Laser_avoid[7],Laser_avoid[8]);break;	
								case 8:
								Send_BLE_DEBUG(0,accumulated_flow_y*1000,flow_rad.integrated_y*1000,
								0,accumulated_gyro_z*1000,flow_rad.integrated_zgyro*1000,
								0,0,0);break;	
								case 9:
								Send_BLE_DEBUG(0,0,ALT_POS_SONAR3*1000,
								0,ultra_distance,0,
								0,0,ALT_POS_SONAR2*1000);break;
								case 10:
								Send_BLE_DEBUG(0,0,X_kf_yaw[0],
								0,ultra_distance,0,
								0,0,yaw_mag_view[4]);break;
								case 11:
								Send_BLE_DEBUG(velNorth*1000,velNorth_gps*1000,0*X_KF_NAV[0][1]*1000,
								velEast*1000,velEast_gps*1000,0*X_KF_NAV[1][1]*1000,
								flow_matlab_data[0]*100,flow_matlab_data[1]*100,Yaw);break;
								case 12:
								Send_BLE_DEBUG(flow_rad.integrated_xgyro*1000,accumulated_flow_x*1000,accumulated_gyro_x*1000,
								0,flow_rad.integrated_xgyro*1000,flow_rad.integrated_x*1000,
								0,flow_rad.integrated_x*1000,accumulated_flow_x*1000);break;	
								case 13:
								Send_BLE_DEBUG(lis3mdl.Alt*1000,0,0,
								(ultra_distance),0,ultra_distance,
								ALT_VEL_BMP_EKF*1000,ALT_POS_BMP_EKF*1000,ALT_POS_SONAR2*1000);break;
								}				
								GOL_LINK_BUSY[1]=0;		
								} 
							}								

	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
	}
	OSIntExit(); 
}

//=======================故障保护 任务函数==================
OS_STK ERROR_TASK_STK[ERROR_STK_SIZE];
void error_task(void *pdata)
{							  
 	while(1)
	{
		
		if(gps_loss_cnt++>1/0.2)
			gps_good=0;
		flow.rate=flow.flow_cnt;
	  flow.flow_cnt=0;
		if(fly_ready)
			Laser_start();
		else 
			Laser_stop();
		delay_ms(200); 
	}
}	

//------------------------------软件定时器--------------------------------//
OS_TMR   * tmr1;			//软件定时器1
OS_TMR   * tmr2;			//软件定时器2
OS_TMR   * tmr3;			//软件定时器3

//软件定时器1的回调函数	  OSCPUUsage
//每100ms执行一次,用于显示CPU使用率和内存使用率		
 u16 cpuusage=0;
void tmr1_callback(OS_TMR *ptmr,void *p_arg) 
{
	static u8 tcnt=0;	    

	if(tcnt==5)
	{
 		
		cpuusage=0;
		tcnt=0; 
	}
	cpuusage+=OSCPUUsage;
	tcnt++;				    
}

//软件定时器2的回调函数				  	   
void tmr2_callback(OS_TMR *ptmr,void *p_arg) 
{	
static u8 cnt;
	LEDRGB_STATE();
}
//软件定时器3的回调函数				  	   
void tmr3_callback(OS_TMR *ptmr,void *p_arg) 
{	
 
} 


//