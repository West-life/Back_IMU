#include "ukf_task.h"
#include "ukf_oldx.h"
#include "my_math.h"
#include "usart_fc.h"
#include "gps.h"
#include "KF_OLDX_NAV.h"
#include "hml5833l.h"
#include "alt_kf.h"
#include "insgps.h"
#include "time.h"
#include "nav_ukf.h"

#define DELAY_GPS 0.0//s

u8 OLDX_KF3(float *measure,float tau,float *r_sensor,u8 *flag_sensor,double *state,double *state_correct,float T)
{
float PosDealt;	
float SpeedDealt;
float K_ACC_Z;
float K_VEL_Z;
float K_POS_Z;

if(!flag_sensor[0]&&!flag_sensor[1]&&!flag_sensor[2])	
	return 0;
K_ACC_Z =(5.0f / (tau * tau * tau));
K_VEL_Z =(3.0f / (tau * tau));
K_POS_Z =(3.0f / tau);
//d spd	
if(flag_sensor[0]&&!flag_sensor[1])	
PosDealt=(measure[0]-state[0]);
else if(flag_sensor[0]&&!flag_sensor[1])
PosDealt=measure[1];
else if(flag_sensor[1]&&!flag_sensor[1])
PosDealt=(measure[0]-state[0])+state[1];
else 
return 0;	

state_correct[3*0+2] += r_sensor[0]*PosDealt* K_ACC_Z ;
state_correct[3*0+1] += r_sensor[1]*PosDealt* K_VEL_Z ;
state_correct[3*0+0] += r_sensor[2]*PosDealt* K_POS_Z ;

//acc correct
if(!flag_sensor[1]&&flag_sensor[2])	
state[2]=measure[2]+state_correct[0*3+2];
else if(flag_sensor[1]&&flag_sensor[2])	
state[2]=measure[1]+(measure[2]+state_correct[0*3+2]);
	
//d acc
SpeedDealt=state[2]*T;

//pos correct
state_correct[1*3+0]+=(state[1]+0.5*SpeedDealt)*T;
state[0]=state_correct[1*3+0]+state_correct[0*3+0];

//vel correct
state_correct[0*3+1]+=SpeedDealt;
state[1]=state_correct[1*3+1]+state_correct[0*3+1];

return 1;	
}


//For  Qr  mark
#define NAV_USE_KF 1
#if USE_FLOW_FLY_ROBOT
float q_flow[3]={0.005,0.005,0.005};//{1,1,1};//0.6;///1;
#else
float q_flow[3]={0.02,0.01,0.01};//{1,1,1};//0.6;///1;
#endif
float r_flow[3]={10,1,0.1};//{1,1,1};//0.6;///1;
float flow_gain=1;//2;
double X_ukf[6];
double X_ukf_baro[6];
int acc_flag_flow[2]={1,1};
float X_ukf_Pos[2];
float r1,r2;
float posNorth,posEast;
float local_Lat,local_Lon;//GPS局部坐标初始
float velEast,velNorth;
float GPS_J_F,GPS_W_F;//融合GPS
static void CalcEarthRadius(double lat) {
    double sinLat2;

    sinLat2 = sin(lat * (double)DEG_TO_RAD);
    sinLat2 = sinLat2 * sinLat2;

    r1 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD * ((double)1.0 - (double)NAV_E_2) / pow((double)1.0 - ((double)NAV_E_2 * sinLat2), ((double)3.0 / (double)2.0));
    r2 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD / sqrt((double)1.0 - ((double)NAV_E_2 * sinLat2)) * cos(lat * (double)DEG_TO_RAD);
}

static void CalcGlobalDistance(double lat, double lon) {
    posNorth = (lat - local_Lat) * r1;
    posEast =  (lon - local_Lon) * r2;
}


static void CalcGlobalLocation(float posNorth,float posEast){ 
    GPS_W_F=(float)posNorth/(float)(r1+0.1)+local_Lat;
    GPS_J_F=(float)posEast/(float)(r2+0.1)+local_Lon;
}

static float acc_body_buf[2][40];
static void feed_acc_buf(float in1,float in2)
{
u8 i,j;	
float reg[2][40];	
static u8 cnt;
 for(i=0;i<2;i++)
	for(j=0;j<40;j++)
   reg[i][j]=acc_body_buf[i][j];
	
 for(i=0;i<2;i++)
	for(j=0;j<40-1;j++)
   acc_body_buf[i][j]=reg[i][j-1];	
	
	acc_body_buf[0][0]=in1;
	acc_body_buf[1][0]=in2;
}	

static float get_acc_delay(u8 sel,float delay,float dt)
{
u8 id[2];
id[0]=(int)(delay/dt);
id[1]=id[0]+1;	
if(delay>0)	
return acc_body_buf[sel][id[0]]/2+acc_body_buf[sel][id[1]]/2;
else
return acc_body_buf[sel][0];	
}	

u8 kf_data_sel=1;//0->flow 1->gps 2->flow global 
double X_KF_NAV[2][3];
double P_KF_NAV[2][9];
float ga_nav= 0.1; 
float gwa_nav=0.1;
float g_pos_flow= 0.0086;//0.0051;
float g_spd_flow= 2.00000011e-005;//0.0006;

float K_pos_qr=0.01;
float K_spd_flow=0.86;//1.2;//0.86;
//gps

float K_acc_gps=1;  
float K_pos_gps=1;
float K_spd_gps=1;
#if  USE_M100_IMU
float g_pos_gps= 0.001;//10;
float g_spd_gps= 0.001;//0.1;//0.1; 
#else
float g_pos_gps= 0.03125;//10;
float g_spd_gps= 0.002125;//0.1;//0.1; 
#endif
float velNorth_gps,velEast_gps;
int flag_kf1[2]={1,1};
u8 force_test;
float Posx,Posy;
u8 gps_data_vaild=0,gps_init;
float  r_gps_new[4]={0.015,0.056,0.026,3};
double state_correct_posx[6];
double state_correct_posy[6];
double X_kf2_x[3];
double X_kf2_y[3];
void ukf_pos_task_qr(float Qr_x,float Qr_y,float Yaw,float flowx,float flowy,float accx,float accy,float T)
{
static int gps_h_off;	

float Sdpx,Accx;
float Sdpy,Accy;

u8 pos_vaild=0;
double A[9]=
			 {1,       0,    0,
				T,       1,    0,
				-T*T/2, -T,    1};

double B[3]={T*T/2,T,0}; 
double H[9]={
			 1,0,0,
       0,1,0,
       0,0,0}; 

#if USE_M100_IMU
gpsx.pvt.PVT_latitude=m100.Lat;
gpsx.pvt.PVT_longitude=m100.Lon;	
if(m100.connect&&m100.m100_data_refresh==1&&m100.Yaw!=0)	
{gpsx.pvt.PVT_numsv=3;gpsx.pvt.PVT_fixtype=3;}
#endif			 
			 
 if((gpsx.pvt.PVT_longitude!=0||force_test) && gps_init==0 && gpsx.pvt.PVT_numsv>=4&&gpsx.pvt.PVT_fixtype>=3){
 gps_init=1;
 local_Lat=gpsx.pvt.PVT_latitude;
 local_Lon=gpsx.pvt.PVT_longitude;
 gps_h_off=gpsx.pvt.PVT_height; 
 CalcEarthRadius(gpsx.pvt.PVT_latitude);

 }
#if USE_M100_IMU //<<----------------------------------------------------------------------
u8 kf_data_sel_temp=1;  
#else 
u8 kf_data_sel_temp=kf_data_sel; 
#endif 
 
if(module.gps&& gpsx.pvt.PVT_numsv>=1&&gpsx.pvt.PVT_fixtype>=3)
kf_data_sel_temp=1;	
else if(module.flow||module.flow_iic)
{kf_data_sel_temp=1;gps_init=0;}	
//kf_data_sel_temp=0;
#if NAV_USE_KF
//--------------------------------GPS_KF------------------------------------- 
if(kf_data_sel_temp==1){
   float ACCY=flow_matlab_data[1]*K_acc_gps;
   float ACCX=flow_matlab_data[0]*K_acc_gps;
	 float accEast=ACCY*sin(Yaw*0.0173)+ACCX*cos(Yaw*0.0173);
   float accNorth=ACCY*cos(Yaw*0.0173)-ACCX*sin(Yaw*0.0173);
	 feed_acc_buf(accEast,accNorth);
   #if USE_M100_IMU
	 if(m100.connect&&m100.m100_data_refresh==1&&m100.Yaw!=0)
	 {CalcEarthRadius(gpsx.pvt.PVT_latitude); gps_data_vaild=1;}
	 #else
	 if(gpsx.pvt.PVT_numsv>=4&&gpsx.pvt.PVT_fixtype>=1&&gpsx.pvt.PVT_latitude!=0)
	 {CalcEarthRadius(gpsx.pvt.PVT_latitude); gps_data_vaild=1;}
	 #endif

	 CalcGlobalDistance(gpsx.pvt.PVT_latitude,gpsx.pvt.PVT_longitude); 
	 
	 static float dposEast,dposNorth;
	 static u8 cnt;
	 if(cnt++>0.2/T){cnt=0;
	 velNorth_gps=(posNorth-dposNorth)/(0.2+0.00001);
	 velEast_gps=(posEast-dposEast)/(0.2+0.00001);
	 dposEast=posEast;
	 dposNorth=posNorth;
	 }
	 
	 #if !USE_M100_IMU
   velEast=LIMIT(gpsx.pvt.PVT_East_speed,-3,3);//LIMIT(-gpsx.spd*sin((gpsx.angle-180)*0.0173),-3,3);
   velNorth=LIMIT(gpsx.pvt.PVT_North_speed,-3,3);//LIMIT(-gpsx.spd*cos((gpsx.angle-180)*0.0173),-3,3);
	 #else
	 velEast=LIMIT(m100.spd[1],-3,3);//LIMIT(-gpsx.spd*sin((gpsx.angle-180)*0.0173),-3,3);
   velNorth=LIMIT(m100.spd[0],-3,3);//LIMIT(-gpsx.spd*cos((gpsx.angle-180)*0.0173),-3,3); 
	 #endif
   #if !USE_M100_IMU
	 Global_GPS_Sensor.NED_Pos[2]=gpsx.pvt.PVT_height-gps_h_off;
	 #else
   Global_GPS_Sensor.NED_Pos[2]=(float)(gpsx.altitude-gps_h_off)/10.;
	 #endif
	 Global_GPS_Sensor.NED_Vel[2]=gpsx.pvt.PVT_Down_speed;
   if((module.pi_flow&&pi_flow.insert)&&
		!(gpsx.pvt.PVT_numsv>=4&&gpsx.pvt.PVT_fixtype>=1&&gpsx.pvt.PVT_latitude!=0&&((gps_init&&gps_data_vaild)))) 
   ;
	 else{
   Global_GPS_Sensor.NED_Pos[1]=Posy=posNorth*K_pos_gps;//1-> west north
	 Global_GPS_Sensor.NED_Vel[1]=Sdpy=velNorth*K_spd_gps;
	 Global_GPS_Sensor.NED_Pos[0]=Posx=posEast*K_pos_gps;//0->  east
	 Global_GPS_Sensor.NED_Vel[0]=Sdpx=velEast*K_spd_gps;
	 }
	 
	 Global_GPS_Sensor.NED_Acc[1]=Accy=get_acc_delay(1,DELAY_GPS,T);
	 Global_GPS_Sensor.NED_Acc[0]=Accx=get_acc_delay(0,DELAY_GPS,T);
	 u8 flag_sensor[3]={1,0,1};
	 double Zx[3]={Posx,Sdpx,Accx};
	 double Zy[3]={Posy,Sdpy,Accy};
#if !USE_UKF_FROM_AUTOQUAD
	 if((gps_init&&gps_data_vaild)||force_test)//bei
   KF_OLDX_NAV( X_KF_NAV[1],  P_KF_NAV[1],  Zy,  Accy, A,  B,  H,  ga_nav,  gwa_nav, g_pos_gps,  g_spd_gps,  T);

	 if((gps_init&&gps_data_vaild)||force_test)//dong 
	 KF_OLDX_NAV( X_KF_NAV[0],  P_KF_NAV[0],  Zx,  Accx, A,  B,  H,  ga_nav,  gwa_nav, g_pos_gps,  g_spd_gps,  T);

	 float X_KF_NAV_TEMP[2][3];//0->x->east  1->y->north
	 X_KF_NAV_TEMP[0][0]=X_KF_NAV[0][0]+DELAY_GPS*X_KF_NAV[0][1]+1/2*pow(DELAY_GPS,2)*(Accx+X_KF_NAV[0][2]);
	 X_KF_NAV_TEMP[0][1]=X_KF_NAV[0][1]+DELAY_GPS*(Accx+X_KF_NAV[0][2]);
	 X_KF_NAV_TEMP[0][2]=X_KF_NAV[0][2];
	 X_KF_NAV_TEMP[1][0]=X_KF_NAV[1][0]+DELAY_GPS*X_KF_NAV[1][1]+1/2*pow(DELAY_GPS,2)*(Accy+X_KF_NAV[1][2]);
	 X_KF_NAV_TEMP[1][1]=X_KF_NAV[1][1]+DELAY_GPS*(Accy+X_KF_NAV[1][2]);
	 X_KF_NAV_TEMP[1][2]=X_KF_NAV[1][2];						
#else//<<---------------------------------UKF-AUTOQUAD----------------------------------	
  static u16 init_ukf,init_ukf_cnt;	 
	float X_KF_NAV_TEMP[2][3];//0->x->east  1->y->north 
	if(init_ukf_cnt++>320)init_ukf=1;
		
	if(init_ukf)
  runTaskCode(Global_GPS_Sensor.NED_Pos[1],Global_GPS_Sensor.NED_Pos[0],Global_GPS_Sensor.NED_Pos[2],
	            Global_GPS_Sensor.NED_Vel[1],Global_GPS_Sensor.NED_Vel[0],Global_GPS_Sensor.NED_Vel[2],T );
		
	X_KF_NAV_TEMP[0][0]=UKF_POSE;
	X_KF_NAV_TEMP[0][1]=UKF_VELE_F;
	//X_KF_NAV_TEMP[0][2]=UKF_ACC_BIAS_X;
	X_KF_NAV_TEMP[1][0]=UKF_POSN;
	X_KF_NAV_TEMP[1][1]=UKF_VELN_F;
	//X_KF_NAV_TEMP[1][2]=X_KF_NAV[1][2];	
	//}
#endif

	 X_ukf[0]=X_KF_NAV_TEMP[1][0];//North pos	
	 X_ukf[2]=X_KF_NAV_TEMP[1][2];
	 X_ukf[3]=X_KF_NAV_TEMP[0][0];//East  pos
	 X_ukf[5]=X_KF_NAV_TEMP[0][2];
	//global
	 X_ukf[1]=X_KF_NAV_TEMP[1][1];//North vel
	 X_ukf[4]=X_KF_NAV_TEMP[0][1];//East  vel
	//turn to body frame
	 X_ukf[4]=X_KF_NAV_TEMP[1][1]*cos(Yaw*0.0173)+X_KF_NAV_TEMP[0][1]*sin(Yaw*0.0173);//Y
	 X_ukf[1]=-X_KF_NAV_TEMP[1][1]*sin(Yaw*0.0173)+X_KF_NAV_TEMP[0][1]*cos(Yaw*0.0173);//X
	//output to fc 
	 X_ukf_Pos[1]=X_ukf[0];//North Pos
   X_ukf_Pos[0]=X_ukf[3];//East Pos  
   CalcGlobalLocation(X_ukf[0],X_ukf[3]);}
//--------=-----------------flow in global---------------------------------------------------------
	else if(kf_data_sel_temp==2){	 
	 static int qr_yaw_init;	
	 #if SENSOR_FORM_PI_FLOW	
	 float Yaw_qr=To_180_degrees(Yaw);
	 #else
	 float Yaw_qr=To_180_degrees(Yaw+yaw_qr_off);	
   #endif		
	 float ACCY=flow_matlab_data[1];
   float ACCX=flow_matlab_data[0];
	 float accEast=ACCY*sin(Yaw_qr*0.0173)+ACCX*cos(Yaw_qr*0.0173);
   float accNorth=ACCY*cos(Yaw_qr*0.0173)-ACCX*sin(Yaw_qr*0.0173);
   float SPDY=flowy*K_spd_flow;
	 float SPDX=flowx*K_spd_flow;
	 float acc_bias[2]={0};

	 velNorth=SPDY*cos(Yaw_qr*0.0173)-SPDX*sin(Yaw_qr*0.0173);
   velEast=SPDY*sin(Yaw_qr*0.0173)+SPDX*cos(Yaw_qr*0.0173);
	 #if SENSOR_FORM_PI_FLOW
	 if(pi_flow.check==0&&pi_flow.connect)
	 H[0]=0; 
	 #else
	 if(qr.check==0&&qr.connect)
	 H[0]=0; 
	 #endif
	 static float pos_reg[2];
	 #if SENSOR_FORM_PI_FLOW
	 Qr_y=-pi_flow.sensor.y;
	 Posy=Qr_y*K_pos_qr;
	 Qr_x=pi_flow.sensor.x;
	 Posx=Qr_x*K_pos_qr;
	 #else
   Qr_y=-qr.y;
	 Posy=Qr_y*K_pos_qr;
	 Qr_x=qr.x;
	 Posx=Qr_x*K_pos_qr;
	 #endif
	 Sdpy=velNorth*K_spd_gps;
	 Accy=accNorth*flag_kf1[1];
	 Sdpx=velEast*K_spd_gps;
	 Accx=accEast*flag_kf1[0];
	 static u8 state_init_flow_pos;
	 switch(state_init_flow_pos)
	 {
		 case 0:
			  if(qr.check&&qr.connect)
				{
				state_init_flow_pos=1;
				X_KF_NAV[1][0]=Posy;X_KF_NAV[0][0]=Posx;
				}
			 break;
		 case 1:
			 if(ALT_POS_SONAR2<0.15||!fly_ready)
				state_init_flow_pos=0;
			break; 
	 }
	 double Zy[3]={Posy,Sdpy,acc_bias[1]};
	 if(1)//bei 
   KF_OLDX_NAV( X_KF_NAV[1],  P_KF_NAV[1],  Zy,  Accy, A,  B,  H,  ga_nav,  gwa_nav, g_pos_flow,  g_spd_flow,  T);
	 double Zx[3]={Posx,Sdpx,acc_bias[0]};
	 if(1)//dong
   KF_OLDX_NAV( X_KF_NAV[0],  P_KF_NAV[0],  Zx,  Accx, A,  B,  H,  ga_nav,  gwa_nav, g_pos_flow,  g_spd_flow,  T);
	 X_ukf[0]=X_KF_NAV[0][0]+X_ukf[2]*T*15;//East pos
	 //X_ukf[1]=X_KF_NAV[0][1];//East vel
	 X_ukf[2]=X_KF_NAV[0][2];
	 X_ukf[3]=X_KF_NAV[1][0]+X_ukf[5]*T*15;//North  pos
	 //X_ukf[4]=X_KF_NAV[1][1];//North  vel
	 X_ukf[5]=X_KF_NAV[1][2];							
	 X_ukf[1]=-X_KF_NAV[1][1]*sin(Yaw_qr*0.0173)+X_KF_NAV[0][1]*cos(Yaw_qr*0.0173);//X
	 X_ukf[4]= X_KF_NAV[1][1]*cos(Yaw_qr*0.0173)+X_KF_NAV[0][1]*sin(Yaw_qr*0.0173);//Y
   if(fabs( X_ukf[0]-pos_reg[0])>1.5||fabs( X_ukf[3]-pos_reg[1])>1.5){
		  if(qr.connect)
			{X_KF_NAV[1][0]=Posy;X_KF_NAV[0][0]=Posx;}
			else
			{X_KF_NAV[1][0]= pos_reg[1];X_KF_NAV[0][0]= pos_reg[0];}	
		}
	 pos_reg[0]=X_ukf[0];
	 pos_reg[1]=X_ukf[3];	
 	 X_ukf_Pos[0]=X_ukf[0];//East Pos
   X_ukf_Pos[1]=X_ukf[3];//North Pos
	}else{
//-------------------------------------------flow in body-------------------------------------------------------------------------	
	 if(qr.check==0)
	 H[0]=0; 
   Qr_y=-qr.y;
	 Qr_x=qr.x;  
   Posx=Qr_x*K_pos_qr;
	 Sdpx=flowx*K_spd_flow;
	 Accx=accx*acc_flag_flow[0];
	 double Zx[3]={Posx,Sdpx,0};
   KF_OLDX_NAV( X_KF_NAV[0],  P_KF_NAV[0],  Zx,  Accx, A,  B,  H,  ga_nav,  gwa_nav, g_pos_flow,  g_spd_flow,  T);
	 Posy=Qr_y*K_pos_qr;
   Sdpy=flowy*K_spd_flow;
	 Accy=accy*acc_flag_flow[1];
	 double Zy[3]={Posy,Sdpy,0};
   KF_OLDX_NAV( X_KF_NAV[1],  P_KF_NAV[1],  Zy,  Accy, A,  B,  H,  ga_nav,  gwa_nav, g_pos_flow,  g_spd_flow,  T);
	 X_ukf[0]=X_KF_NAV[0][0];
	 X_ukf[1]=X_KF_NAV[0][1];
	 X_ukf[2]=X_KF_NAV[0][2];
	 X_ukf[3]=X_KF_NAV[1][0];
	 X_ukf[4]=X_KF_NAV[1][1];
	 X_ukf[5]=X_KF_NAV[1][2];
	 
 	 X_ukf_Pos[0]=X_ukf[0];//X
   X_ukf_Pos[1]=X_ukf[3];//Y
 }
	#endif
}

