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
#include "flow.h"
#include "filter.h"
#include "imu_oldx.h"
#include "m100.h"
#include "nav_ukf.h"
#include "oldx_ekf_imu.h"
#include "oldx_ekf_imu2.h"
//==============================������ ������==========================
u8 fly_ready;
float inner_loop_time_time;
float inner_loop_time_time_int;
u8 GOL_LINK_BUSY[2]={0,0};

//========================�⻷  ������============================
OS_STK INNER_TASK_STK[INNER_STK_SIZE];
u16 cnt_time[2];
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
	if(inner_loop_time_time<0.006)	
		cnt_time[0]++;
	else 
		cnt_time[1]++;
							//��ȡ�ڻ�׼ȷ��ִ������
	if(inner_loop_time_time<0.000002)inner_loop_time_time=0.005;
	
	#if IMU_UPDATE	
	LSM6_readAcc(0);
	LSM6_readGyro(0);
	if(cnt4++>=3){cnt4=0;
  LIS3MDL_read(0);//80hz
	}
	LIS_Data_Prepare(inner_loop_time_time)	;
	if(cnt3++>=5){cnt3=0;
	#if USE_VER_5
	LP_readbmp(0);//25hz
	#endif
	}
	#endif
	MPU6050_Data_Prepare( inner_loop_time_time );			//mpu6�ᴫ�������ݴ���
	
	#if UKF_IN_ONE_THREAD
	if(!lis3mdl.Mag_CALIBRATED)		
	ukf_pos_task_qr(0,0,Yaw,flow_matlab_data[2],flow_matlab_data[3],LIMIT(flow_matlab_data[0],-3,3),LIMIT(flow_matlab_data[1],-3,3),inner_loop_time_time);
	RollR=AQ_ROLL;
	PitchR=AQ_PITCH;
	YawR=AQ_YAW;

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
	if(imu_feed_dog==1&&FC_CONNECT==1)
	IWDG_Feed();//ι��
   
	static u8 cnt_bmp;
	if(cnt_bmp++>1){cnt_bmp=0;
	//altUkfProcess(0);
	}
	#endif
	
  }
  #if UKF_IN_ONE_THREAD
	delay_ms(5);
	#else
	delay_ms(5);
	#endif
	}
}		



//========================�⻷  ������============================
OS_STK OUTER_TASK_STK[OUTER_STK_SIZE];
float YawR,PitchR,RollR;
float YawRm,PitchRm,RollRm;
float outer_loop_time;
float k_gyro_z=1.2;
double X_ekf[7]={1,0,0,0}, P_ekf[49]={0};
double n_q=0.0001,  n_w=0.00001,  n_a=0.01,  n_m=1000;
double Att[4];
float z_k[9] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 9.81f, 0.2f, -0.2f, 0.2f};					/**< Measurement vector */
float x_apo[12]={0};		/**< states */
float P_apo[144] = {100.f, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
					 0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
					 0,   0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,   0,
					 0,   0,   0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,
					 0,   0,   0,   0,  100.f,  0,   0,   0,   0,   0,   0,   0,
					 0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,   0,   0,
					 0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,   0,
					 0,   0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,
					 0,   0,   0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,
					 0,   0,   0,   0,   0,   0,   0,   0,  0.0f, 100.0f,   0,   0,
					 0,   0,   0,   0,   0,   0,   0,   0,  0.0f,   0,   100.0f,   0,
					 0,   0,   0,   0,   0,   0,   0,   0,  0.0f,   0,   0,   100.0f};
float zFlag[3]={1,1,1};
float Rot_matrix[9]={0};
float param[7]={ 
1e-4,
0.08,
0.009,
0.005,
0.0008,
10000.0,
100.0};
float eulerAngles[3];
float q_ekf[4];
//#define AHRS_MAD
//#define AHRS_EKF1
#define AHRS_EKF2
void outer_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;			
  static u8 init,cnt_init;	
	static u8 cal_sel;
 	while(1)
	{	
	outer_loop_time = Get_Cycle_T(GET_T_OUTTER);								//��ȡ�⻷׼ȷ��ִ������	
	if(!init){if(cnt_init++>2)
		init=1;
	outer_loop_time=0.01;
	}
	else{
  #define AHRS_SEL 0
	if(outer_loop_time<=0.00002)outer_loop_time=0.01;	
	#if AHRS_SEL
	IMUupdate(0.5f *outer_loop_time,my_deathzoom_2(imu_fushion.Gyro_deg.x,0.5), my_deathzoom_2(imu_fushion.Gyro_deg.y,0.5), my_deathzoom_2(imu_fushion.Gyro_deg.z,0.5),imu_fushion.Acc.x, imu_fushion.Acc.y, imu_fushion.Acc.z,&RollR,&PitchR,&YawR);	
	#else
		#if defined(AHRS_EKF1)
		oldx_ekf_imu( X_ekf,  P_ekf,  0,  outer_loop_time, 
		imu_fushion.Gyro_deg.x*0.0173,imu_fushion.Gyro_deg.y*0.0173,imu_fushion.Gyro_deg.z*0.0173,
		imu_fushion.Acc.x, imu_fushion.Acc.y, imu_fushion.Acc.z,
		imu_fushion.Mag_Val.x, imu_fushion.Mag_Val.y, imu_fushion.Mag_Val.z,  
		n_q,  n_w,  n_a,  n_m, Att)	;
		#endif
		#if defined(AHRS_EKF2)
		z_k[0]= imu_fushion.Gyro_deg.x*0.0173;
		z_k[1]= imu_fushion.Gyro_deg.y*0.0173;
		z_k[2]= imu_fushion.Gyro_deg.z*0.0173;
		z_k[3]= imu_fushion.Acc.x/4096*9.8;
		z_k[4]= imu_fushion.Acc.y/4096*9.8;
		z_k[5]= imu_fushion.Acc.z/4096*9.8;
		z_k[6]= -imu_fushion.Mag_Val.x;
		z_k[7]= -imu_fushion.Mag_Val.y;
		z_k[8]= -imu_fushion.Mag_Val.z;
		oldx_ekf_imu2( x_apo,P_apo,zFlag,z_k,param,outer_loop_time,x_apo,P_apo,Rot_matrix, eulerAngles);
		RollR=eulerAngles[1];
		PitchR=eulerAngles[0];
		YawR=eulerAngles[2];
		Quaternion_FromRotationMatrix(Rot_matrix,q_ekf);
		ref_q_imd_down[0]= q_ekf[2];
		ref_q_imd_down[1]= -q_ekf[3];
		ref_q_imd_down[2]= -q_ekf[0];
		ref_q_imd_down[3]= q_ekf[1];
	  reference_vr_imd_down[0] = 2*(ref_q_imd_down[1]*ref_q_imd_down[3] - ref_q_imd_down[0]*ref_q_imd_down[2]);
	  reference_vr_imd_down[1] = 2*(ref_q_imd_down[0]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[3]);
	  reference_vr_imd_down[2] = 1 - 2*(ref_q_imd_down[1]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[2]);
		#endif
		#if defined(AHRS_MAD)
		madgwick_update_new(
		imu_fushion.Acc.x, imu_fushion.Acc.y, imu_fushion.Acc.z,
		my_deathzoom_2(imu_fushion.Gyro_deg.x,0.0)*DEG_RAD, my_deathzoom_2(imu_fushion.Gyro_deg.y,0.0)*DEG_RAD, my_deathzoom_2(imu_fushion.Gyro_deg.z,0.0)*DEG_RAD*k_gyro_z,
		imu_fushion.Mag_Val.x, imu_fushion.Mag_Val.y, imu_fushion.Mag_Val.z,
		&RollRm,&PitchRm,&YawRm,outer_loop_time);	
		#endif
		RollR=RollRm;
		PitchR=PitchRm;
		YawR=YawRm;
		reference_vr[0]=reference_vr_imd_down[0];
		reference_vr[1]=reference_vr_imd_down[1]; 
		reference_vr[2]=reference_vr_imd_down[2];
		q_nav[0]=ref_q[0]=ref_q_imd_down[0]; 		
		q_nav[1]=ref_q[1]=ref_q_imd_down[1]; 
		q_nav[2]=ref_q[2]=ref_q_imd_down[2]; 
		q_nav[3]=ref_q[3]=ref_q_imd_down[3]; 	

//  OLDX_AHRS(my_deathzoom_2(imu_fushion.Gyro_deg.x,0.5)*0.0173, my_deathzoom_2(imu_fushion.Gyro_deg.y,0.5)*0.0173, my_deathzoom_2(imu_fushion.Gyro_deg.z,0.5)*0.0173,
//						imu_fushion.Acc.x, imu_fushion.Acc.y, imu_fushion.Acc.z,
//						imu_fushion.Mag_Val.x, imu_fushion.Mag_Val.y, imu_fushion.Mag_Val.z,
//						1,&RollR,&PitchR,&YawR,outer_loop_time);
//						reference_vr[0]=reference_v.x=reference_v_m1[0];
//						reference_vr[1]=reference_v.y=reference_v_m1[1];
//						reference_vr[2]=reference_v.z=reference_v_m1[2];
//						q_nav[0]=ref_q[0]=ref_q_m1[0];
//						q_nav[1]=ref_q[1]=ref_q_m1[1];
//						q_nav[2]=ref_q[2]=ref_q_m1[2];
//						q_nav[3]=ref_q[3]=ref_q_m1[3];	
	#endif	
	//if(mode.en_imu_ekf==0){
		
		static float off_yaw; 
		if (pi_flow.insert==1&&module.pi_flow==1){
		if(pi_flow.connect==1&&pi_flow.check==1&&gpsx.pvt.PVT_fixtype==0)
		off_yaw=pi_flow.yaw_off=pi_flow.yaw-YawR;	
		//Yaw_mid_down=Yaw=To_180_degrees(YawR+pi_flow.yaw_off);
		}
		else if (m100.connect==1){
		if(m100.m100_data_refresh==1&&m100.Yaw!=0&&fabs(m100.Yaw-YawR)>10)
		off_yaw=m100.Yaw-YawR;	
		//Yaw_mid_down=Yaw=To_180_degrees(YawR+off_yaw_m100);
		}

		Yaw_mid_down=Yaw=To_180_degrees(YawR+off_yaw);	
		Pitch_mid_down=Pitch=PitchR;
		Roll_mid_down=Roll=RollR;
		
  }
	if(imu_feed_dog==1&&FC_CONNECT==1)
	IWDG_Feed();//ι��
	#if USE_M100_IMU
	m100_data(1);
	#endif
	delay_ms(5);
	}
}		


//========================EKF  ������============================
OS_STK EKF_TASK_STK[EKF_STK_SIZE];
float ekf_loop_time;

#include "FastMath.h"
#include "Quaternion.h"
#include "ekf_ins.h"

void ekf_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;			
  static u8 init,cnt_init;	
 	while(1)
	{	
	ekf_loop_time = Get_Cycle_T(GET_T_EKF);			
	if(cnt_init++>2&&!init){cnt_init=101;
		init=1;	
	}
	else{
	if(ekf_loop_time<0.000002)ekf_loop_time=0.02;
	static u8 ekf_gps_cnt;
	SINS_Prepare();
  if(!lis3mdl.Mag_CALIBRATED)		
	ukf_pos_task_qr(0,0,Yaw,flow_matlab_data[2],flow_matlab_data[3],LIMIT(flow_matlab_data[0],-3,3),LIMIT(flow_matlab_data[1],-3,3),ekf_loop_time);
  #if USE_UKF_FROM_AUTOQUAD	
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
	if(imu_feed_dog==1&&FC_CONNECT==1)
	IWDG_Feed();//ι��
	#endif
	
	}
	#if USE_UKF_FROM_AUTOQUAD
  delay_ms(10);
 #else
	delay_ms(5);
 #endif
	}
}		


//��ѹ�� ������
OS_STK BARO_TASK_STK[BARO_STK_SIZE];
void baro_task(void *pdata)
{							  
 	while(1)
	{ static u8 cnt;
		altUkfProcess(0);
		delay_ms(20);  
	}
}	

//=======================������ ������==================
OS_STK SONAR_TASK_STK[SONAR_STK_SIZE];
void sonar_task(void *pdata)
{static u16 cnt_ground;							  
 	while(1)
	{
		#if defined(SONAR_USE_SCL) 
			 if(fly_ready||en_ble_debug)
					Ultra_Duty_SCL(); 
				else if(cnt_ground++>1/0.1){cnt_ground=0;
					Ultra_Duty_SCL(); 
				}
		#else
				if(fly_ready||en_ble_debug)
					Ultra_Duty(); 
		#endif	  
		#if USE_FLOW_SONAR
		ultra_distance=flow.hight.originf*1000;//Moving_Median(1,5,temp);
		sys.sonar=ultra_ok = 1;
		#endif
		#if defined(SONAR_SAMPLE1)
		delay_ms(40);
		#elif defined(SONAR_SAMPLE2)
		delay_ms(100);
		#elif defined(SONAR_SAMPLE3)
		delay_ms(70);
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
float k_flow_acc[2]={0.1,0.1};
float k_gro_acc=0.1;
#if FLOW_USE_P5A
float flt_gro=0.066;//1;
#else
float flt_gro=0.03;//1;
#endif
float k_time_use=4.2;
void flow_sample(void)
{
uint32_t deltatime=Get_Cycle_T(GET_T_FLOW_SAMPLE)*1000000;
float x_rate = imu_fushion.Gyro_deg.y; // change x and y rates
float y_rate = imu_fushion.Gyro_deg.x;
float z_rate = -imu_fushion.Gyro_deg.z; // z is correct

integration_timespan = deltatime*k_time_use;
#if FLOW_USE_P5A
accumulated_flow_x = -flow_5a.flow_x_integral  / focal_length_px * 1.0f*k_flow_acc[0]; //rad axis swapped to align x flow around y axis
accumulated_flow_y = -flow_5a.flow_y_integral / focal_length_px * 1.0f*k_flow_acc[1];//rad	
#else	
accumulated_flow_x = qr.spdx  / focal_length_px * 1.0f*k_flow_acc[0]; //rad axis swapped to align x flow around y axis
accumulated_flow_y = qr.spdy / focal_length_px * 1.0f*k_flow_acc[1];//rad
#endif
accumulated_gyro_x = LIMIT(x_rate * deltatime / 1000000.0f*k_gro_acc*flt_gro+accumulated_gyro_x*(1-flt_gro),-fabs(accumulated_flow_x*8),fabs(accumulated_flow_x*8));	//rad
accumulated_gyro_y = LIMIT(y_rate * deltatime / 1000000.0f*k_gro_acc*flt_gro+accumulated_gyro_y*(1-flt_gro),-fabs(accumulated_flow_y*8),fabs(accumulated_flow_y*8));	//rad
accumulated_gyro_z = z_rate * deltatime / 1000000.0f*k_gro_acc*flt_gro+accumulated_gyro_z*(1-flt_gro);	//rad
}



//=======================FLOW ������==================
u8 imu_board_test=1;
OS_STK FLOW_TASK_STK[FLOW_STK_SIZE];
u8 MID_CNT_SPD=5;
float k_flp=1-0.1; 
static double b_IIR_acc[4+1] ={ 0.0004  ,  0.0017  ,  0.0025  ,  0.0017 ,   0.0004};  //ϵ��b
static double a_IIR_acc[4+1] ={ 1.0000   ,-3.1806   , 3.8612  , -2.1122  ,  0.4383};//ϵ��a
static double InPut_IIR_acc[3][4+1] = {0};
static double OutPut_IIR_acc[3][4+1] = {0};
float b[3] = {0.8122  ,  1.6244  ,  0.8122};
float a[3] = {1.0000  ,  1.5888  ,  0.6600};
float xBuf1[3];
float yBuf1[3];
float xBuf2[3];
float yBuf2[3];
float xBuf3[3];
float yBuf3[3];
float acc_neo[3],flow_ground_temp[4];
float flow_matlab_data[4];
float baro_matlab_data[2];
float flow_loop_time;

float k_flow_devide_pi=0.486;
void flow_task1(void *pdata)
{float flow_height_fliter;		
 static float acc_neo_off[3];
 FLOW_RAD flow_rad_use;  
 	while(1)
	{
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
	  flow_loop_time = Get_Cycle_T(GET_T_FLOW);			
	  if(flow_loop_time<0.001)flow_loop_time=0.01;
	  #if SENSOR_FORM_PI_FLOW
	  flow_ground_temp[0]=pi_flow.sensor.spdy;
		flow_ground_temp[1]=-pi_flow.sensor.spdx;
		flow_ground_temp[2]=pi_flow.sensor.spdy;
	  flow_ground_temp[3]=-pi_flow.sensor.spdx;
	  #else 
	  flow_pertreatment_oldx( &flow_rad_use , flow_height_fliter);
		if(module.pi_flow&&!module.flow&&!module.flow_iic){
		flow_ground_temp[0]=pi_flow.sensor.spdy*k_flow_devide_pi;
		flow_ground_temp[1]=-pi_flow.sensor.spdx*k_flow_devide_pi;
		flow_matlab_data[2]=pi_flow.sensor.spdy*k_flow_devide_pi;
		flow_matlab_data[3]=-pi_flow.sensor.spdx*k_flow_devide_pi;
		}else{
		flow_ground_temp[0]=flow_per_out[0];
		flow_ground_temp[1]=flow_per_out[1];
		#if FLOW_USE_P5A
		flow_matlab_data[2]=firstOrderFilter(-flow_per_out[2]*k_flow_devide,&firstOrderFilters[FLOW_LOWPASS_X],flow_loop_time);
		flow_matlab_data[3]=firstOrderFilter(-flow_per_out[3]*k_flow_devide,&firstOrderFilters[FLOW_LOWPASS_Y],flow_loop_time);
    #else
		flow_matlab_data[2]=firstOrderFilter(LIMIT(flow_per_out[2]*k_flow_devide,-6,6),&firstOrderFilters[FLOW_LOWPASS_X],flow_loop_time);
		flow_matlab_data[3]=firstOrderFilter(LIMIT(flow_per_out[3]*k_flow_devide,-6,6),&firstOrderFilters[FLOW_LOWPASS_Y],flow_loop_time);
		//	flow_ground_temp[2]=flow_per_out[2]*k_flow_devide;
		//	flow_ground_temp[3]=flow_per_out[3]*k_flow_devide;
		#endif
		}
		#endif
		static float a_br[3]={0};	
		static float acc_temp[3]={0};
		a_br[0] =(float) imu_fushion.Acc.x/4096.;//16438.;
		a_br[1] =(float) imu_fushion.Acc.y/4096.;//16438.;
		a_br[2] =(float) imu_fushion.Acc.z/4096.;//16438.;
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
		#else
	  acc_neo_temp[0]=-acc_temp[0]*9.87;
		acc_neo_temp[1]=-acc_temp[1]*9.87;
		acc_neo_temp[2]=(acc_temp[2]-1.0f)*9.87;		
		#endif
	  if(en_ble_debug||imu_board_test)
			;
		else if(!fly_ready){
	  acc_neo_off[0]+= ( 1 / ( 1 + 1 / ( 2.2f *3.14f *0.04 ) ) ) *(acc_neo_temp[0]- acc_neo_off[0]) ;
		acc_neo_off[1]+= ( 1 / ( 1 + 1 / ( 2.2f *3.14f *0.04 ) ) ) *(acc_neo_temp[1]- acc_neo_off[1]) ;
		acc_neo_off[2]+= ( 1 / ( 1 + 1 / ( 2.2f *3.14f *0.04 ) ) ) *(acc_neo_temp[2]- acc_neo_off[2]) ;
		}
		static float acc_neo_temp1[3]={0};
		static float acc_flt[3];
    acc_neo_temp1[0]=Moving_Median(5,5,acc_neo_temp[0]-acc_neo_off[0]);
		acc_neo_temp1[1]=Moving_Median(6,5,acc_neo_temp[1]-acc_neo_off[1]);
		acc_neo_temp1[2]=Moving_Median(7,5,acc_neo_temp[2]-acc_neo_off[2]);	
		acc_flt[0]=firstOrderFilter(acc_neo_temp1[0],&firstOrderFilters[ACC_LOWPASS_X],flow_loop_time);
		acc_flt[1]=firstOrderFilter(acc_neo_temp1[1],&firstOrderFilters[ACC_LOWPASS_Y],flow_loop_time);
		acc_flt[2]=firstOrderFilter(acc_neo_temp1[2],&firstOrderFilters[ACC_LOWPASS_Z],flow_loop_time);		
		
//		acc_flt[0]=acc_neo_temp1[0];
//		acc_flt[1]=acc_neo_temp1[1];
//		acc_flt[2]=acc_neo_temp1[2];//
	
		if(fabs(acc_neo_temp[0])<8.6&&fabs(acc_neo_temp[1])<8.6){
		acc_neo[0]=acc_flt[0];
		acc_neo[1]=acc_flt[1];
		acc_neo[2]=acc_flt[2];
	 	flow_matlab_data[0]=acc_neo[0];//acc
		flow_matlab_data[1]=acc_neo[1];}
	  }
	 }
		//flow_matlab_data[2]=flow_ground_temp[2];//spd
		//flow_matlab_data[3]=flow_ground_temp[3];
		
    #if !USE_UKF_FROM_AUTOQUAD
		FlowUkfProcess(0);//FK filter
		#endif
	 
		float temp_spd[2];
		temp_spd[0]=Moving_Median(16,MID_CNT_SPD,FLOW_VEL_X);
		temp_spd[1]=Moving_Median(17,MID_CNT_SPD,FLOW_VEL_Y);
		imu_nav.flow.speed.x=imu_nav.flow.speed.x*(1-k_flp)+k_flp*temp_spd[0];
		imu_nav.flow.speed.y=imu_nav.flow.speed.y*(1-k_flp)+k_flp*temp_spd[1];
		delay_ms(10); 
	}
}	



u8 UART_UP_LOAD_SEL=5;//<------------------------------UART UPLOAD DATA SEL
float time_uart;
float px4_test[4]={0};
void TIM3_IRQHandler(void)
{
	OSIntEnter();static u16 cnt,cnt1,cnt2,cnt3,cnt_init,init;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //����ж�
	{
	time_uart = Get_Cycle_T(GET_T_UART); 	
	if(!init){
	if(cnt_init++>2)
	init=1;
	time_uart=0.0025;
	}
	else{
		
	if(m100.control_connect)
		m100_contrl_px4(m100.control_spd[0],m100.control_spd[1],m100.control_spd[2],m100.control_yaw,m100.px4_tar_mode);
		//m100_contrl_px4(px4_test[0],px4_test[1],px4_test[2],px4_test[3],m100.px4_tar_mode);
	
	if(pi_flow.insert&&cnt2++>2){cnt2=0;
		Send_TO_FLOW_NAV_GPS();
		Send_TO_FLOW_PI();		
  }		
							//��ȡ�ڻ�׼ȷ��ִ������
	if(time_uart<0.000002)time_uart=0.005;
		#if EN_DMA_UART2 			
			     if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET)//�ȴ�DMA2_Steam7�������
								{ 
							DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);//���DMA2_Steam7������ɱ�־
							clear_nrf_uart();		
							GOL_LINK_TASK_DMA();
					    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����     
							MYDMA_Enable(DMA1_Stream6,nrf_uart_cnt+2);     //��ʼһ��DMA���䣡	
								}							
					#else
							GOL_LINK_TASK();
					#endif
	
	debug_pi_flow[0]=0;  
	en_ble_debug=1;								
if(debug_pi_flow[0])									
  en_ble_debug=1;
	if(cnt++>2-1){cnt=0;	
								if(en_ble_debug){
								GOL_LINK_BUSY[1]=1;
								switch(UART_UP_LOAD_SEL)
								{
								case 0://�����ٶ�
								Send_BLE_DEBUG((int16_t)(0),flow_rad.integrated_y*100, flow_rad.integrated_ygyro*100,
								0,flow.flow_x.origin,flow.flow_y.origin,
								acc_neo[0]*100,acc_neo[1]*100,acc_neo[2]*100);break;
								case 1://�����ٶ�
								Send_BLE_DEBUG((int16_t)(0),flow.flow_x.origin,flow.flow_y.origin,
								0,ALT_VEL_SONAR*1000,flow_ground_temp[3]*1000,
								0,ALT_VEL_BMP*1000,flow_ground_temp[2]*1000);break;
								case 2://�����ٶ�
								Send_BLE_DEBUG((int16_t)(0),0,0,
								0,flow.flow_x.origin,0,
								0,0,flow.flow_y.origin);break;
								case 3://�����ٶ�
								Send_BLE_DEBUG((int16_t)(baroAlt),ALT_POS_BMP*1000,ALT_POS_BMP_EKF*1000,
								0,-ALT_VEL_BMP*100,ALT_VEL_BMP_EKF*100,
								0,acc_z_view[1],acc_z_view[0]);break;		
								case 4://�����ٶ�
								Send_BLE_DEBUG((int16_t)(0),flow_matlab_data[0]*1000,0,
								0,0*100,flow_matlab_data[1]*1000,
								0,0,v_test[1]*1000);break;		
								case 5://�����ٶ�
								Send_BLE_DEBUG((int16_t)(SINS_Accel_Earth[Xr]*100),acc_neo[Xr]*100,X_ukf[3]*1000,
								FLOW_VEL_X*1000,flow_matlab_data[2]*1000*K_spd_flow,X_ukf[1]*1000,
								0,X_ukf[4]*1000,flow_matlab_data[3]*1000*K_spd_flow);break;	
								case 6://�����ٶ�
								Send_BLE_DEBUG((int16_t)(X_ukf_baro[3]*100),baro_matlab_data[0]*100,X_ukf_baro[0]*100,
								X_ukf_baro[4]*100,X_ukf_baro[1]*100,ALT_VEL_BMP_EKF*100,
								0,ALT_VEL_BMP_UNION*100,X_kf_sonar[1]*100);break;	
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
								Send_BLE_DEBUG(Global_GPS_Sensor.NED_Pos[0]*100,Global_GPS_Sensor.NED_Pos[1]*100,Global_GPS_Sensor.NED_Pos[2]*100,
								Global_GPS_Sensor.NED_Vel[0]*100,Global_GPS_Sensor.NED_Vel[1]*100,Global_GPS_Sensor.NED_Vel[2]*100,
								0,0,0);break;
								case 12:
								Send_BLE_DEBUG(flow_ground_temp[3]*100,accumulated_flow_x*1000,Global_GPS_Sensor.NED_Velf[Xr]*100,
								flow_ground_temp[2]*100,accumulated_flow_y*1000,Global_GPS_Sensor.NED_Velf[Yr]*100,
								Global_GPS_Sensor.NED_Acc[1]*100,X_ukf[4]*100,X_ukf[1]*100);break;	
								case 13:
								Send_BLE_DEBUG(0,gpsx.pvt.PVT_Down_speed*100,X_kf_baro[1]*100,
								(ultra_distance),m100.H*100,X_kf_baro[0]*100,
								m100.H_Spd*1000,Global_GPS_Sensor.NED_Pos[1]*100,Global_GPS_Sensor.NED_Pos[0]*100);break;
								case 14:
								Send_BLE_DEBUG(X_ukf[1]*100,flow_matlab_data[3]*100,imu_nav.flow.speed.x*100,
								X_ukf[4]*100,flow_matlab_data[2]*100,imu_nav.flow.speed.y*100,
								X_ukf[0]*100,Global_GPS_Sensor.NED_Posf_reg[Yr]*100,0);break;
								case 15:
								Send_BLE_DEBUG(X_ukf_global[1]*100,X_ukf_global[4]*100,UKF_VELD_F*100,
								Global_GPS_Sensor.NED_Vel[1]*100,Global_GPS_Sensor.NED_Vel[0]*100,gpsx.pvt.PVT_Down_speed*100,
								UKF_POSE*100, Global_GPS_Sensor.NED_Pos[0]*100, gpsx.pvt.PVT_height*100);break;
								case 16:
								Send_BLE_DEBUG(flow_rad.integrated_x,flow_rad.integrated_y,0,
								UKF_PRES_ALT*100,UKF_VELD_F*100,UKF_POSD*100,
								ALT_POS*100,-ALT_VEL*100,AQ_PRESSURE*100);break;		
								case 17:
								Send_BLE_DEBUG(0,0,flow_5a.flow_x_integral,
								0,0,flow_5a.flow_y_integral,
								ALT_POS*100,-ALT_VEL*100,AQ_PRESSURE*100);break;
							 }
								GOL_LINK_BUSY[1]=0;		
								} 
							}								
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
	}
	OSIntExit(); 
}

//=======================���ϱ��� ������==================
OS_STK ERROR_TASK_STK[ERROR_STK_SIZE];
void error_task(void *pdata)
{							  
 	while(1)
	{
		if(gps_loss_cnt++>1/0.2)
			gps_good=0;
		if(pi_flow.loss_cnt++>1/0.2)
			pi_flow.insert=0;
		if(pi_flow.sensor.loss_cnt++>1/0.2)
			pi_flow.sensor.connect=0;
	  if(fc_loss_cnt++>3/0.2)
			FC_CONNECT=0;
		if(m100.cnt_m100_data_refresh++>3/0.2)
		 m100.m100_data_refresh=0;
		if(m100.loss_cnt++>3/0.2)
		 m100.connect=0;
		if(m100.control_loss++>3/0.2)
		 m100.control_connect=0;
		if(qr.loss_cnt++>3/0.2)
		 qr.connect=0;

		flow.rate=flow.flow_cnt;
	  flow.flow_cnt=0;

		delay_ms(200); 
	}
}	

//------------------------------�����ʱ��--------------------------------//
OS_TMR   * tmr1;			//�����ʱ��1
OS_TMR   * tmr2;			//�����ʱ��2
OS_TMR   * tmr3;			//�����ʱ��3

//�����ʱ��1�Ļص�����	  OSCPUUsage
//ÿ100msִ��һ��,������ʾCPUʹ���ʺ��ڴ�ʹ����		
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

//�����ʱ��2�Ļص�����				  	   
void tmr2_callback(OS_TMR *ptmr,void *p_arg) 
{	
static u8 cnt;
	LEDRGB_STATE();
}
//�����ʱ��3�Ļص�����				  	   
void tmr3_callback(OS_TMR *ptmr,void *p_arg) 
{	
 
} 


//