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
#include "matlib.h"
#define DELAY_GPS 0.1//s
u8 QR_DELAY=20;
u8 GPS_DELAY_P=20;
u8 GPS_DELAY_V=10;
u8 FLOW_DELAY=2;
u8 DELAY_FIX_SEL=1;
float k_acc_f=0.86;
float test_k[3]={0.6,1.6,0.6};
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
PosDealt=(measure[0]-state[0]);

if(flag_sensor[0]){
state_correct[3*0+0] += r_sensor[0]*PosDealt* K_POS_Z ;//pos correct
state_correct[3*0+1] += r_sensor[1]*PosDealt* K_VEL_Z ;//spd correct
//state_correct[3*0+2] += r_sensor[2]*PosDealt* K_ACC_Z ;//acc correct
}

if(flag_sensor[1])
{
state_correct[3*0+1] += r_sensor[0]*(measure[1]-state[1])*33* K_VEL_Z ;//spd correct
state_correct[3*0+2] += r_sensor[1]*(measure[1]-state[1])*33* K_ACC_Z ;//acc correct
}

//acc correct
//����
state_correct[3*1+2]=0;
//����
state[2]=measure[2]*flag_sensor[2]+state_correct[3*0+2];

//vel correct
//����
state_correct[3*1+1]+=state[2]*T;
//����
state[1]=state_correct[3*1+1]+state_correct[3*0+1];

//pos correct
//����
state_correct[3*1+0]+=state[1]*T+0.5*state[2]*T*T;
//����
state[0]=state_correct[3*1+0]+state_correct[3*0+0];

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
double X_ukf[6],X_ukf_global[6];
double X_ukf_baro[6];
int acc_flag_flow[2]={1,1};
float X_ukf_Pos[2];
float r1,r2;
float posNorth,posEast;
double local_Lat,local_Lon;//GPS�ֲ������ʼ
float velEast,velNorth;
float GPS_J_F,GPS_W_F;//�ں�GPS
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
double X_KF_NAV[2][3],X_KF_NAV_HB[2][3];
double P_KF_NAV[2][9]={0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001};
float ga_nav= 0.1; 
float gwa_nav=0.1;
#if FLOW_USE_P5A
float g_pos_flow= 0.0086;//0.0051;
float g_spd_flow= 0.000368;//0.0006;
#else
float g_pos_flow= 0.0086*1.2;//0.0051;
float g_spd_flow= 0.000026;//0.0006;
#endif
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
float g_pos_gps= 0.036;//10;
float g_spd_gps= 0.000052125;//0.1;//0.1; 
#endif
float g_pos_use,g_spd_use;
float velNorth_gps,velEast_gps;
int flag_kf1[2]={1,1};
u8 force_test;
float Posx,Posy;
u8 gps_data_vaild=0,gps_init;
float  r_flow_new[4]={0.03,0.05,0.0153,4.56};
float  r_gps_new[4]={0.03,0.05,0.0153,5};
double state_correct_posx[6];
double state_correct_posy[6];
double X_kf2_x[3];
double X_kf2_y[3];
float X_f[3][1],P_f[3][3];
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
			 
 if((gpsx.pvt.PVT_longitude!=0||force_test) && gps_init==0 && gpsx.pvt.PVT_numsv>=6&&gpsx.pvt.PVT_fixtype>=3){
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
kf_data_sel_temp=2;
#if NAV_USE_KF
//--------------------------------GPS_KF------------------------------------- 
if(kf_data_sel_temp==1){
   float ACCY=flow_matlab_data[1]*K_acc_gps;
   float ACCX=flow_matlab_data[0]*K_acc_gps;
	 float accEast=ACCY*sin(Yaw*0.0173)+ACCX*cos(Yaw*0.0173);
   float accNorth=ACCY*cos(Yaw*0.0173)-ACCX*sin(Yaw*0.0173);
	 #if USE_UKF_FROM_AUTOQUAD
	 feed_acc_buf(-accy,-accx);
	 #else
	 feed_acc_buf(accEast,accNorth);
	 #endif
   #if USE_M100_IMU
	 if(m100.connect&&m100.m100_data_refresh==1&&m100.Yaw!=0)
	 {CalcEarthRadius(gpsx.pvt.PVT_latitude); gps_data_vaild=1;}
	 #else
	 if(gpsx.pvt.PVT_numsv>=6&&gpsx.pvt.PVT_fixtype>=1&&gpsx.pvt.PVT_latitude!=0)
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
   #if GPS_FROM_UBM
	   velEast=LIMIT(gpsx.ubm.velE,-6.3,6.3);//LIMIT(-gpsx.spd*sin((gpsx.angle-180)*0.0173),-3,3);
		 velNorth=LIMIT(gpsx.ubm.velN,-6.3,6.3);//LIMIT(-gpsx.spd*cos((gpsx.angle-180)*0.0173),-3,3);
		 #else
		 velEast=LIMIT(gpsx.pvt.PVT_East_speed,-6.3,6.3);//LIMIT(-gpsx.spd*sin((gpsx.angle-180)*0.0173),-3,3);
		 velNorth=LIMIT(gpsx.pvt.PVT_North_speed,-6.3,6.3);//LIMIT(-gpsx.spd*cos((gpsx.angle-180)*0.0173),-3,3);
		 #endif
	 #else
	 velEast=LIMIT(m100.spd[1],-3,3);//LIMIT(-gpsx.spd*sin((gpsx.angle-180)*0.0173),-3,3);
   velNorth=LIMIT(m100.spd[0],-3,3);//LIMIT(-gpsx.spd*cos((gpsx.angle-180)*0.0173),-3,3); 
	 #endif
   #if !USE_M100_IMU
	 Global_GPS_Sensor.NED_Pos[Zr]=gpsx.pvt.PVT_height-gps_h_off;
	 #else
   Global_GPS_Sensor.NED_Pos[Zr]=(float)(gpsx.altitude-gps_h_off)/10.;
	 #endif
	 Global_GPS_Sensor.NED_Vel[Zr]=gpsx.pvt.PVT_Down_speed;
   if((module.pi_flow&&pi_flow.insert)&&
		!(gpsx.pvt.PVT_numsv>=6&&gpsx.pvt.PVT_fixtype>=1&&gpsx.pvt.PVT_latitude!=0&&((gps_init&&gps_data_vaild)))) 
   ;
	 else{
   Global_GPS_Sensor.NED_Pos[Yr]=Posy=posNorth*K_pos_gps;//1-> west north
	 Global_GPS_Sensor.NED_Vel[Yr]=Sdpy=velNorth*K_spd_gps;
	 Global_GPS_Sensor.NED_Pos[Xr]=Posx=posEast*K_pos_gps;//0->  east
	 Global_GPS_Sensor.NED_Vel[Xr]=Sdpx=velEast*K_spd_gps;
	 }
	 
	 Global_GPS_Sensor.NED_Acc[Yr]=Accy=get_acc_delay(Yr,DELAY_GPS,T);
	 Global_GPS_Sensor.NED_Acc[Xr]=Accx=get_acc_delay(Xr,DELAY_GPS,T);
	 u8 flag_sensor[3]={1,0,1};
	
#if !USE_UKF_FROM_AUTOQUAD
	 double Zx[3]={Posx,Sdpx,Accx};
	 double Zy[3]={Posy,Sdpy,Accy};
	 if((gps_init&&gps_data_vaild)||force_test)//bei
   KF_OLDX_NAV( X_KF_NAV[Yr],  P_KF_NAV[Yr],  Zy,  Accy, A,  B,  H,  ga_nav,  gwa_nav, g_pos_gps,  g_spd_gps,  T,0);

	 if((gps_init&&gps_data_vaild)||force_test)//dong 
	 KF_OLDX_NAV( X_KF_NAV[Xr],  P_KF_NAV[Xr],  Zx,  Accx, A,  B,  H,  ga_nav,  gwa_nav, g_pos_gps,  g_spd_gps,  T,0);

	 float X_KF_NAV_TEMP[2][3];//0->x->east  1->y->north
	 X_KF_NAV_TEMP[Xr][0]=X_KF_NAV[Xr][0]+DELAY_GPS*X_KF_NAV[Xr][1]+1/2*pow(DELAY_GPS,2)*(Accx+X_KF_NAV[Xr][2]);
	 X_KF_NAV_TEMP[Xr][1]=X_KF_NAV[Xr][1]+DELAY_GPS*(Accx+X_KF_NAV[Xr][2]);
	 X_KF_NAV_TEMP[Xr][2]=X_KF_NAV[Xr][2];
	 X_KF_NAV_TEMP[Yr][0]=X_KF_NAV[Yr][0]+DELAY_GPS*X_KF_NAV[Yr][1]+1/2*pow(DELAY_GPS,2)*(Accy+X_KF_NAV[Yr][2]);
	 X_KF_NAV_TEMP[Yr][1]=X_KF_NAV[Yr][1]+DELAY_GPS*(Accy+X_KF_NAV[Yr][2]);
	 X_KF_NAV_TEMP[Yr][2]=X_KF_NAV[Yr][2];						
#else//<<---------------------------------UKF-AUTOQUAD----------------------------------	
  static u16 init_ukf,init_ukf_cnt;	 
	float X_KF_NAV_TEMP[2][3];//0->x->east  1->y->north 
	if(init_ukf_cnt++>320)init_ukf=1;
		
	if(init_ukf)
  runTaskCode(Global_GPS_Sensor.NED_Pos[Yr],Global_GPS_Sensor.NED_Pos[Xr],Global_GPS_Sensor.NED_Pos[Zr],
	            Global_GPS_Sensor.NED_Vel[Yr],Global_GPS_Sensor.NED_Vel[Xr],Global_GPS_Sensor.NED_Vel[Zr],T );
		
	 double Zx_kf[3]={0,0,0};
	 double Zy_kf[3]={0,0,0};
	 u8 flag_sensor_flow_gps[3]={1,1,1};
	 

	 static u8 gps_flow_switch_flag=2,has_off_flag;
	 static u16 loss_check_cnt;
	 static u16 cnt_off_yaw_correct;
	  force_test=1;		
	 if(gpsx.pvt.PVT_numsv>4&&gpsx.pvt.PVT_fixtype>=1&&gpsx.pvt.PVT_latitude!=0)
	 { gps_flow_switch_flag=1;
		 Zx_kf[0]=Posx;
		 Zx_kf[1]=UKF_VELE_F;
		 Zy_kf[0]=Posy;
		 Zy_kf[1]=UKF_VELN_F; 
		 g_pos_use=	 g_pos_gps;
		 g_spd_use=	 g_spd_gps;	 
	 }
	 else 
	 { 
		 force_test=1;		
		 gps_flow_switch_flag=0;
		//convert qr to nez
		 if(loss_check_cnt>10/T)
		 {
		 has_off_flag=0;
		 }
		 
		 if(pi_flow.connect==1&&pi_flow.check==1){
		 loss_check_cnt=0;	
		 cnt_off_yaw_correct++;
			 if(cnt_off_yaw_correct>333)
			 { has_off_flag=1;cnt_off_yaw_correct=0; }			
			 if(has_off_flag==0&&pi_flow.yaw!=0){
			 pi_flow.yaw_off=Moving_Median(0,19,Yaw-pi_flow.sensor.yaw);
			 }
		
			 if(pi_flow.yaw_off!=0){
			 Global_GPS_Sensor.NED_Posf[Yr]=-pi_flow.sensor.y*K_pos_qr*cos(pi_flow.yaw_off*0.0173)-pi_flow.sensor.x*K_pos_qr*sin(pi_flow.yaw_off*0.0173);
			 Global_GPS_Sensor.NED_Posf[Xr]=pi_flow.sensor.x*K_pos_qr*cos(pi_flow.yaw_off*0.0173)-pi_flow.sensor.y*K_pos_qr*sin(pi_flow.yaw_off*0.0173);
			 }
       Global_GPS_Sensor.NED_Posf[Zr]=pi_flow.sensor.z*K_pos_qr;
			 if(Global_GPS_Sensor.NED_Posf[Yr]!=0&&Global_GPS_Sensor.NED_Posf[Xr]!=0){
			 Global_GPS_Sensor.NED_Posf_reg[Xr]=Global_GPS_Sensor.NED_Posf[Xr];
			 Global_GPS_Sensor.NED_Posf_reg[Yr]=Global_GPS_Sensor.NED_Posf[Yr];
			 }
		 }else 
		 loss_check_cnt++;
		 
		 
		//convert flow to nez		 
		 Global_GPS_Sensor.NED_Velf[Yr]=flowy*K_spd_flow*cos(Yaw*0.0173)-flowx*K_spd_flow*sin(Yaw*0.0173);
		 Global_GPS_Sensor.NED_Velf[Xr]=flowy*K_spd_flow*sin(Yaw*0.0173)+flowx*K_spd_flow*cos(Yaw*0.0173);
		 Zx_kf[0]=Global_GPS_Sensor.NED_Posf_reg[Xr];
		 Zx_kf[1]=Global_GPS_Sensor.NED_Velf[Xr];
		 Zy_kf[0]=Global_GPS_Sensor.NED_Posf_reg[Yr];
		 Zy_kf[1]=Global_GPS_Sensor.NED_Velf[Yr]; 
		 g_pos_use=g_pos_flow;
		 g_spd_use=g_spd_flow;	 

		 if(pi_flow.check==0||!pi_flow.insert||pi_flow.yaw_off==0||pi_flow.sensor.y==0||pi_flow.sensor.x==0)
		 {flag_sensor_flow_gps[0]=0;
		 H[0]=0;} 
	 }
	 static u8 gps_flow_switch_flag_reg;
	 if(gps_flow_switch_flag==0&&gps_flow_switch_flag_reg==1)//gps->flow
	 {
	 X_KF_NAV[Xr][0]= Global_GPS_Sensor.NED_Posf_reg[Xr];
	 X_KF_NAV[Yr][0]= Global_GPS_Sensor.NED_Posf_reg[Yr];	 
	 }
	 else if(gps_flow_switch_flag==1&&gps_flow_switch_flag_reg==0)//flow->gps
	 {
	 X_KF_NAV[Xr][0]= Global_GPS_Sensor.NED_Pos[Xr];
	 X_KF_NAV[Yr][0]= Global_GPS_Sensor.NED_Pos[Yr];	
	 }
	 static u16 cnt_outrange;
	 if(fabs(X_KF_NAV[Xr][0]-Zx_kf[0])>1.5||fabs(X_KF_NAV[Yr][0]-Zy_kf[0])>1.5)
	  cnt_outrange++;
	 if(cnt_outrange>2/T){
			if(gps_flow_switch_flag==0&&pi_flow.connect&&1)//flow
			{cnt_outrange=0;
			X_KF_NAV[Xr][0]= Global_GPS_Sensor.NED_Posf_reg[Xr];
			X_KF_NAV[Yr][0]= Global_GPS_Sensor.NED_Posf_reg[Yr];	 
			}
		}
	 gps_flow_switch_flag_reg=gps_flow_switch_flag;
		
	 //KF	global
	 static u8 cnt_kf;
	 float Z_x[3]={Zx_kf[0],Zx_kf[1],Accx};
	 float Z_y[3]={Zy_kf[0],Zy_kf[1],Accy};
		
	 if(((gps_init&&gps_data_vaild)||force_test)&&init_ukf&&cnt_kf++>0){cnt_kf=0;
	 OLDX_KF3(Z_x,r_flow_new[3],r_flow_new,flag_sensor_flow_gps,X_KF_NAV[Xr],state_correct_posx,T);
	 OLDX_KF3(Z_y,r_flow_new[3],r_flow_new,flag_sensor_flow_gps,X_KF_NAV[Yr],state_correct_posy,T);
  // KF_OLDX_NAV( X_KF_NAV[1],  P_KF_NAV[1],  Zy_kf,  Accy, A,  B,  H,  ga_nav,  gwa_nav, g_pos_use,  g_spd_use,  T); 
	// KF_OLDX_NAV( X_KF_NAV[0],  P_KF_NAV[0],  Zx_kf,  Accx, A,  B,  H,  ga_nav,  gwa_nav, g_pos_use,  g_spd_use,  T);
   }
	 // NAN check 
  	if(isnan(X_KF_NAV[Xr][0]))
			X_KF_NAV[Xr][0]=Zx_kf[0];
    if(isnan(X_KF_NAV[Xr][1]))
			X_KF_NAV[Xr][1]=Zx_kf[1];	
	  if(isnan(X_KF_NAV[Xr][2]))
			X_KF_NAV[Xr][2]=Zx_kf[2];
    if(isnan(X_KF_NAV[Yr][0]))
			X_KF_NAV[Yr][0]=Zy_kf[0];
	  if(isnan(X_KF_NAV[Yr][1]))
			X_KF_NAV[Yr][1]=Zy_kf[1];
    if(isnan(X_KF_NAV[Yr][2]))
			X_KF_NAV[Yr][2]=Zy_kf[2];	 		
		
	 //0->x->east  1->y->north
	 X_KF_NAV_TEMP[Xr][0]=X_KF_NAV[Xr][0]+DELAY_GPS*X_KF_NAV[Xr][1];//+1/2*pow(DELAY_GPS,2)*(Accx-X_KF_NAV[Xr][2]);
	 X_KF_NAV_TEMP[Xr][1]=X_KF_NAV[Xr][1];//+DELAY_GPS*(Accx+X_KF_NAV[0][2]);
	 X_KF_NAV_TEMP[Xr][2]=X_KF_NAV[Xr][2];
	 X_KF_NAV_TEMP[Yr][0]=X_KF_NAV[Yr][0]+DELAY_GPS*X_KF_NAV[Yr][1];//+1/2*pow(DELAY_GPS,2)*(Accy-X_KF_NAV[Yr][2]);
	 X_KF_NAV_TEMP[Yr][1]=X_KF_NAV[Yr][1];//+DELAY_GPS*(Accy+X_KF_NAV[1][2]);
	 X_KF_NAV_TEMP[Yr][2]=X_KF_NAV[Yr][2];						

	// X_KF_NAV_TEMP[Yr][0]=Global_GPS_Sensor.NED_Posf_reg[Yr];
	// X_KF_NAV_TEMP[Xr][0]=Global_GPS_Sensor.NED_Posf_reg[Xr];
#endif

	 X_ukf[0]=X_KF_NAV_TEMP[Yr][0];//North pos	
	 X_ukf[2]=X_KF_NAV_TEMP[Yr][2];
	 X_ukf[3]=X_KF_NAV_TEMP[Xr][0];//East  pos
	 X_ukf[5]=X_KF_NAV_TEMP[Xr][2];
	//global
	 X_ukf_global[1]=X_ukf[1]=X_KF_NAV_TEMP[Yr][1];//North vel
	 X_ukf_global[4]=X_ukf[4]=X_KF_NAV_TEMP[Xr][1];//East  vel
	//turn to body frame
	 X_ukf[4]=X_KF_NAV_TEMP[Yr][1]*cos(Yaw*0.0173)+X_KF_NAV_TEMP[Xr][1]*sin(Yaw*0.0173);//Y
	 X_ukf[1]=-X_KF_NAV_TEMP[Yr][1]*sin(Yaw*0.0173)+X_KF_NAV_TEMP[Xr][1]*cos(Yaw*0.0173);//X
	//output to fc 
	 X_ukf_Pos[1]=X_ukf[0];//North Pos
   X_ukf_Pos[0]=X_ukf[3];//East Pos  
	 if((gps_init&&gps_data_vaild))
   CalcGlobalLocation(X_ukf[0],X_ukf[3]);
  }
//--------=-----------------flow in global---------------------------------------------------------
	else if(kf_data_sel_temp==2){	 
		static float pos_buf[2][30],spd_buf[2][30],acc_buf[2][30],acc_bufs[2][30];
		u8 i;
	
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
   //conver body flow spd to NEZ
	 velNorth=SPDY*cos(Yaw_qr*0.0173)-SPDX*sin(Yaw_qr*0.0173);
   velEast=SPDY*sin(Yaw_qr*0.0173)+SPDX*cos(Yaw_qr*0.0173);
	 #if SENSOR_FORM_PI_FLOW
	 if(pi_flow.check&&pi_flow.connect)
	 H[0]=1; 
	 else
	 H[0]=0; 	
	 #else
	 if(qr.check&&qr.connect)
	 H[0]=1; 
	 else
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
	 
	 //Accy=SINS_Accel_Earth[Yr];Accx=SINS_Accel_Earth[Xr];
	 //Accy=accNorth*0.5+0.5*SINS_Accel_Earth[Yr];Accx=accEast*0.5+0.5*SINS_Accel_Earth[Xr];
	 
	 //GPS init
	 #if USE_M100_IMU
	 if(m100.connect&&m100.m100_data_refresh==1&&m100.Yaw!=0)
	 {CalcEarthRadius(gpsx.pvt.PVT_latitude); gps_data_vaild=1;}
	 #else
	 if(gpsx.pvt.PVT_numsv>=6&&gpsx.pvt.PVT_fixtype>=1&&gpsx.pvt.PVT_latitude!=0)
	 {CalcEarthRadius(gpsx.pvt.PVT_latitude); gps_data_vaild=1;}
	 #endif

	 CalcGlobalDistance(gpsx.pvt.PVT_latitude,gpsx.pvt.PVT_longitude); 
	 #if !USE_M100_IMU
	 velEast=LIMIT(gpsx.pvt.PVT_East_speed,-6.3,6.3);
	 velNorth=LIMIT(gpsx.pvt.PVT_North_speed,-6.3,6.3);
	 Global_GPS_Sensor.NED_Pos[Zr]=gpsx.pvt.PVT_height-gps_h_off;
	 #else
	 velEast=LIMIT(m100.spd[1],-3,3);
   velNorth=LIMIT(m100.spd[0],-3,3);
   Global_GPS_Sensor.NED_Pos[Zr]=(float)(gpsx.altitude-gps_h_off)/10.;	 
	 #endif
	 Global_GPS_Sensor.NED_Vel[Zr]=gpsx.pvt.PVT_Down_speed;
	 
   if((module.pi_flow&&pi_flow.insert)&&
		!(gpsx.pvt.PVT_numsv>=5&&gpsx.pvt.PVT_fixtype>=1&&gpsx.pvt.PVT_latitude!=0&&((gps_init&&gps_data_vaild)))) 
   ;
	 else{
   Global_GPS_Sensor.NED_Pos[Yr]=Posy=posNorth;//1-> west north
	 Global_GPS_Sensor.NED_Vel[Yr]=Sdpy=velNorth;
	 Global_GPS_Sensor.NED_Pos[Xr]=Posx=posEast;//0->  east
	 Global_GPS_Sensor.NED_Vel[Xr]=Sdpx=velEast;
	 }	 
	 Global_GPS_Sensor.NED_Acc[Yr]=Accy;
	 Global_GPS_Sensor.NED_Acc[Xr]=Accx;
	 
	 //Qr first check initial
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
	  //restore
		for(i=30;i>0;i--)
		{
		pos_buf[Xr][i]=pos_buf[Xr][i-1];
		pos_buf[Yr][i]=pos_buf[Yr][i-1];

		spd_buf[Xr][i]=spd_buf[Xr][i-1];
		spd_buf[Yr][i]=spd_buf[Yr][i-1];
			
		acc_buf[Xr][i]=acc_buf[Xr][i-1];
		acc_buf[Yr][i]=acc_buf[Yr][i-1];
			
		acc_bufs[Xr][i]=acc_bufs[Xr][i-1];
		acc_bufs[Yr][i]=acc_bufs[Yr][i-1];
		}
		pos_buf[Xr][0]=X_KF_NAV[Xr][0];//�洢�˲����ֵ
		pos_buf[Yr][0]=X_KF_NAV[Yr][0];

		spd_buf[Xr][0]=X_KF_NAV[Xr][1];
		spd_buf[Yr][0]=X_KF_NAV[Yr][1];
		
		acc_bufs[Xr][0]=X_KF_NAV[Xr][2];
		acc_bufs[Yr][0]=X_KF_NAV[Yr][2];
		
		acc_buf[Xr][0]=Accx;
		acc_buf[Yr][0]=Accy;
	 //kalman filter
	  u8 sensor_sel=0;
	  u8 delay_sel[2]={0};	
	  if(gps_update==1)//����
		{
		H[0]=H[4]=1; 	
		gps_update=0;
		sensor_sel=1;	
		delay_sel[0]=GPS_DELAY_P;delay_sel[1]=GPS_DELAY_V;		
		}	
		else if(flow_update==1)//����
		{
		H[4]=1; 	
		flow_update=0;
		sensor_sel=2;	
		delay_sel[0]=QR_DELAY;delay_sel[1]=FLOW_DELAY;			
		}else
		H[4]=0;
		
	
   		
	 double Zx[3]={Posx,Sdpx,Accx};
	 double Zy[3]={Posy,Sdpy,Accy};
	 double Zx_delay[4]={pos_buf[Xr][delay_sel[0]],spd_buf[Xr][delay_sel[1]],acc_bufs[Xr][delay_sel[1]],DELAY_FIX_SEL};
	 double Zy_delay[4]={pos_buf[Yr][delay_sel[0]],spd_buf[Yr][delay_sel[1]],acc_bufs[Yr][delay_sel[1]],DELAY_FIX_SEL};
	 double Zx_fix[3]={Posx,Sdpx,1};
	 double Zy_fix[3]={Posy,Sdpy,1};
	 float posDelta[2],spdDelta[2];
	 if(FLOW_DELAY==0)
		Zx_delay[3]=Zy_delay[3]=Zy_fix[2]=Zx_fix[2]=0;
	
	 switch(DELAY_FIX_SEL){
		 case 1://WT
   Zx_fix[0]=Posx+(spd_buf[Xr][delay_sel[0]]*delay_sel[0]*T+
		 (acc_buf[Xr][delay_sel[0]]-acc_bufs[Xr][delay_sel[0]])*delay_sel[0]*T*delay_sel[0]*T)*Zx_fix[2];
	 Zx_fix[1]=Sdpx+((acc_buf[Xr][delay_sel[1]]-acc_bufs[Xr][delay_sel[1]])*delay_sel[1]*T)*Zx_fix[2];
		 
	 Zy_fix[0]=Posy+(spd_buf[Yr][delay_sel[0]]*delay_sel[0]*T+
		 (acc_buf[Yr][delay_sel[0]]-acc_bufs[Yr][delay_sel[0]])*delay_sel[0]*T*delay_sel[0]*T)*Zy_fix[2];
	 Zy_fix[1]=Sdpy+((acc_buf[Yr][delay_sel[1]]-acc_bufs[Yr][delay_sel[1]])*delay_sel[1]*T)*Zy_fix[2];
		 break;
	 }
	  //Filter_Horizontal( Posx, Sdpx, Accx, Posy, Sdpy, Accy, T);
	  //Strapdown_INS_Horizontal( Posx, Sdpx, Accx, Posy, Sdpy, Accy, T);
	 float gps_wqv=gpsx.ubm.sAcc *0.001* __sqrtf(gpsx.pvt.tDOP*0.01*gpsx.pvt.tDOP*0.01 + gpsx.pvt.nDOP*0.01*gpsx.pvt.nDOP*0.01) * UKF_GPS_VEL_M_N;
	 float gps_wqp=gpsx.ubm.hAcc *1.001* __sqrtf(gpsx.pvt.tDOP*0.01*gpsx.pvt.tDOP*0.01 + gpsx.pvt.nDOP*0.01*gpsx.pvt.nDOP*0.01) * UKF_GPS_POS_M_N;
	 float g_spd,g_pos;
	 float GPS_DOP[2]={gpsx.pvt.pDOP,gpsx.pvt.vDOP};
   switch(sensor_sel){
		 case 1:g_spd=g_spd_gps+gps_wqv;g_pos=g_pos_gps+gps_wqp;break;
     case 2:g_spd=g_spd_flow;g_pos=g_pos_flow;break;
		 default:g_spd=g_spd_flow;g_pos=g_pos_flow;break; 
	 }		 
	 KF_OLDX_NAV( X_KF_NAV[1],  P_KF_NAV[1],  Zy_fix,  Accy*k_acc_f, A,  B,  H,  ga_nav,  gwa_nav, g_pos,  g_spd,  T ,Zy_delay);
	 KF_OLDX_NAV( X_KF_NAV[0],  P_KF_NAV[0],  Zx_fix,  Accx*k_acc_f, A,  B,  H,  ga_nav,  gwa_nav, g_pos,  g_spd,  T ,Zx_delay);
  
	 X_ukf[0]=X_KF_NAV[0][0];//East pos
	 //X_ukf[1]=X_KF_NAV[0][1];//East vel
	 X_ukf[2]=X_KF_NAV[0][2];
	 X_ukf[3]=X_KF_NAV[1][0];//North  pos
	 //X_ukf[4]=X_KF_NAV[1][1];//North  vel
	 X_ukf[5]=X_KF_NAV[1][2];	
   //spd convert to body frame 	 
	 X_ukf[1]=-X_KF_NAV[1][1]*sin(Yaw_qr*0.0173)+X_KF_NAV[0][1]*cos(Yaw_qr*0.0173);//X
	 X_ukf[4]= X_KF_NAV[1][1]*cos(Yaw_qr*0.0173)+X_KF_NAV[0][1]*sin(Yaw_qr*0.0173);//Y
	 //out range fix
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
   KF_OLDX_NAV( X_KF_NAV[0],  P_KF_NAV[0],  Zx,  Accx, A,  B,  H,  ga_nav,  gwa_nav, g_pos_flow,  g_spd_flow,  T,0);
	 Posy=Qr_y*K_pos_qr;
   Sdpy=flowy*K_spd_flow;
	 Accy=accy*acc_flag_flow[1];
	 double Zy[3]={Posy,Sdpy,0};
   KF_OLDX_NAV( X_KF_NAV[1],  P_KF_NAV[1],  Zy,  Accy, A,  B,  H,  ga_nav,  gwa_nav, g_pos_flow,  g_spd_flow,  T,0);
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



float R_GPS[2]={0.05f,0.005f};
float Q_GPS[2]={2,3};
float R_Acce_bias[2]={0.0001,0.00001};
double Pre_conv_GPS[2][4]=
{
  0.0001 ,    0.00001,  0.00001    , 0.003,
  0.0001 ,    0.00001,  0.00001    , 0.003,
};//��һ��Э����
double K_GPS[2][2]={0};//�������
float Acce_Bias[2];
void  KalmanFilter_Horizontal_GPS(float Position_GPS,float Vel_GPS,float Position_Last,float Vel_Last,
                                   double *Position,double *Vel,
                                   float *Acce,float *R,
                                   float *Q,float dt,uint8_t Axis)
{

float Conv_Z=0;
float Z_Delta[2]={0};
float Conv_Temp=0;
double Temp_conv[4]={0};//����Э����
uint8 Label=0;
if(Axis=='X') Label=0;
else Label=1;
//����״̬ ��ǰ���ٶ�
*Position +=*Vel*dt+((*Acce+Acce_Bias[Label])*dt*dt)/2.0;
*Vel+=*Acce*dt;
//����Э����
Conv_Temp=Pre_conv_GPS[Label][1]+Pre_conv_GPS[Label][3]*dt;
Temp_conv[0]=Pre_conv_GPS[Label][0]+Pre_conv_GPS[Label][2]*dt+Conv_Temp*dt+R_GPS[0];
Temp_conv[1]=Conv_Temp;
Temp_conv[2]=Pre_conv_GPS[Label][2]+Pre_conv_GPS[Label][3]*dt;
//Temp_conv[1]=Conv_Temp+R_GPS[0]*0.5*0.00001;
//Temp_conv[2]=Temp_conv[1];
Temp_conv[3]=Pre_conv_GPS[Label][3]+R_GPS[1];

//���㿨��������
Conv_Z=1.0/(Temp_conv[0]+Q_GPS[0]*1)*(Temp_conv[3]+Q_GPS[1]*1)-Temp_conv[1]*Temp_conv[2];

//K_GPS[0][0]=( Temp_conv[0]*(Temp_conv[3]+Q_GPS[1])-Temp_conv[1]*Temp_conv[2])*Conv_Z;
//K_GPS[0][1]=(-Temp_conv[0]*Temp_conv[1]+Temp_conv[1]*(Temp_conv[0]+Q_GPS[0]))*Conv_Z;
//K_GPS[1][0]=( Temp_conv[2]*(Temp_conv[3]+Q_GPS[1])-Temp_conv[2]*Temp_conv[3])*Conv_Z;
//K_GPS[1][1]=(-Temp_conv[1]*Temp_conv[2]+Temp_conv[3]*(Temp_conv[0]+Q_GPS[0]))*Conv_Z;

//��������
K_GPS[0][0]=( Temp_conv[0]*(Temp_conv[3]+Q_GPS[1]*1)-Temp_conv[1]*Temp_conv[2])*Conv_Z;
K_GPS[0][1]=(Temp_conv[1]*Q_GPS[0]*1)*Conv_Z;
K_GPS[1][0]=(Temp_conv[2]*Q_GPS[1]*1)*Conv_Z;
K_GPS[1][1]=(-Temp_conv[1]*Temp_conv[2]+Temp_conv[3]*(Temp_conv[0]+Q_GPS[0]*1))*Conv_Z;

//�ں��������

//Z_Delta[0]=Position_GPS-*Position;//����ʱ
//Z_Delta[1]=Vel_GPS-*Vel;
//��ǰ�۲�-��ȥ�ں�
Z_Delta[0]=Position_GPS-Position_Last;
Z_Delta[1]=Vel_GPS-Vel_Last;


*Position +=K_GPS[0][0]*Z_Delta[0]
           +K_GPS[0][1]*Z_Delta[1];

*Vel +=K_GPS[1][0]*Z_Delta[0]
      +K_GPS[1][1]*Z_Delta[1];



Acce_Bias[Label]=R_Acce_bias[0]*Z_Delta[0]
                 +R_Acce_bias[1]*Z_Delta[1];



//����״̬Э�������

Pre_conv_GPS[Label][0]=(1-K_GPS[0][0])*Temp_conv[0]-K_GPS[0][1]*Temp_conv[2];
Pre_conv_GPS[Label][1]=(1-K_GPS[0][0])*Temp_conv[1]-K_GPS[0][1]*Temp_conv[3];
Pre_conv_GPS[Label][2]=(1-K_GPS[1][1])*Temp_conv[2]-K_GPS[1][0]*Temp_conv[0];
Pre_conv_GPS[Label][3]=(1-K_GPS[1][1])*Temp_conv[3]-K_GPS[1][0]*Temp_conv[1];

}
#define GPS_BUF_NUM 40
float Position_History[2][GPS_BUF_NUM]={0};
float Vel_History[2][GPS_BUF_NUM]={0};
uint16 GPS_Vel_Delay_Cnt=0;//100ms
uint16 GPS_Pos_Delay_Cnt=0;//20;//200ms
int16 GPS_Position_Cnt=0;
float Acce_bias[2]={0};
void Filter_Horizontal(float posx,float spdx,float accx,float posy,float spdy,float accy,float Dt)
{
int16 i=0;

for(i=GPS_BUF_NUM;i>0;i--)
{
 Position_History[Xr][i]=Position_History[Xr][i-1];
 Position_History[Yr][i]=Position_History[Xr][i-1];

 Vel_History[Xr][i]=Vel_History[Xr][i-1];
 Vel_History[Yr][i]=Vel_History[Yr][i-1];
}
 Position_History[Xr][0]=X_KF_NAV[Xr][0];//�洢�˲����ֵ
 Position_History[Yr][0]=X_KF_NAV[Yr][0];

 Vel_History[Xr][0]=X_KF_NAV[Xr][1];
 Vel_History[Yr][0]=X_KF_NAV[Yr][1];



if(flow_update==1)//���GPS����
{
KalmanFilter_Horizontal_GPS(posx,//pos gps
                            spdx,//vel gps
                            Position_History[Xr][GPS_Pos_Delay_Cnt],//delay pos
                            Vel_History[Xr][GPS_Vel_Delay_Cnt],//delay vel
                            &X_KF_NAV[Xr][0],//fuse pos
                            &X_KF_NAV[Xr][1],//fuse vel
                            &accx,//kalman u 
                            R_GPS,Q_GPS,Dt,'X');


KalmanFilter_Horizontal_GPS(posy,
                            spdy,
                            Position_History[Yr][GPS_Pos_Delay_Cnt],
                            Vel_History[Yr][GPS_Vel_Delay_Cnt],
                            &X_KF_NAV[Yr][0],
                            &X_KF_NAV[Yr][1],
                            &accy,
                            R_GPS,Q_GPS,Dt,'Y');
flow_update=0;
}
else//ֱ�ӻ���
{
	    X_KF_NAV[Xr][0] +=X_KF_NAV[Xr][1]*Dt
																		+((accx)*Dt*Dt)/2.0;
			X_KF_NAV[Xr][1]+=((accx))*Dt;

			X_KF_NAV[Yr][0] +=X_KF_NAV[Yr][1]*Dt
																		+((accy)*Dt*Dt)/2.0;
			X_KF_NAV[Yr][1]+=((accy))*Dt;
}

}

#define TIME_CONTANST_XY      2.5f
#define K_ACC_XY	     (1.0f / (TIME_CONTANST_XY * TIME_CONTANST_XY * TIME_CONTANST_XY))
#define K_VEL_XY             (3.0f / (TIME_CONTANST_XY * TIME_CONTANST_XY))															// XY????���䨤??�̨�y,3.0
#define K_POS_XY             (3.0f / TIME_CONTANST_XY)
float X_Delta=0,Y_Delta=0;
uint16_t GPS_Save_Period_Cnt=0;
uint16_t GPS_SINS_Delay_Cnt=20;//10ms
typedef struct
{
 float Pit;
 float Rol;
}Vector2_Body;


typedef struct
{
 float North;
 float East;
}Vector2_Earth;
Vector2_Body Pos_Err_On_Accel={0};
Vector2_Body  Accel_Correction_BF={0};
Vector2_Earth Accel_Correction_EF={0};
float pos_correction[3]={0,0,0};
float acc_correction[3]={0,0,0};
float vel_correction[3]={0,0,0};
float SpeedDealt[3]={0};
void Strapdown_INS_Horizontal(float posx,float spdx,float accx,float posy,float spdy,float accy,float Dt)
{
      uint16 Cnt=0,i;
    for(i=GPS_BUF_NUM;i>0;i--)
		{
		 Position_History[Xr][i]=Position_History[Xr][i-1];
		 Position_History[Yr][i]=Position_History[Xr][i-1];

		 Vel_History[Xr][i]=Vel_History[Xr][i-1];
		 Vel_History[Yr][i]=Vel_History[Yr][i-1];
		}
		 Position_History[Xr][0]=X_KF_NAV[Xr][0];//�洢�˲����ֵ
		 Position_History[Yr][0]=X_KF_NAV[Yr][0];

		 Vel_History[Xr][0]=X_KF_NAV[Xr][1];
		 Vel_History[Yr][0]=X_KF_NAV[Yr][1];
		float Sin_Yaw=sin(Yaw* 0.0173);
    float Cos_Yaw=cos(Yaw* 0.0173);
		float CNTLCYCLE=0.005;
		  static float Earth_Frame_To_XYZE;
		  static float Earth_Frame_To_XYZN;
		  Earth_Frame_To_XYZN+=spdy*Dt;
		  Earth_Frame_To_XYZE+=spdx*Dt;
      //GPS��������ϵ�£���������������λ��ƫ����SINS�������Ĳ��λcm
      X_Delta=Earth_Frame_To_XYZE-Position_History[Xr][GPS_SINS_Delay_Cnt];
      Y_Delta=Earth_Frame_To_XYZN-Position_History[Yr][GPS_SINS_Delay_Cnt];


      Pos_Err_On_Accel.Rol=X_Delta*Cos_Yaw+Y_Delta*Sin_Yaw;//����ϵRoll����    X��
      Pos_Err_On_Accel.Pit=-X_Delta*Sin_Yaw+Y_Delta*Cos_Yaw;//����ϵPitch����  Y��

      Accel_Correction_BF.Pit+=Pos_Err_On_Accel.Pit* K_ACC_XY*CNTLCYCLE;//�������ͷ���򣬼��ٶȽ�����
      Accel_Correction_BF.Rol+=Pos_Err_On_Accel.Rol* K_ACC_XY*CNTLCYCLE;//�����������򣬼��ٶȽ�����

      Accel_Correction_EF.North=Accel_Correction_BF.Rol*Cos_Yaw+Accel_Correction_BF.Pit*Sin_Yaw;//�����巽���ϼ��ٶ�����������ת������ϵ����  Y Axis
      Accel_Correction_EF.East=Accel_Correction_BF.Rol*Sin_Yaw-Accel_Correction_BF.Pit*Cos_Yaw;//�����巽���ϼ��ٶ�����������ת������ϵ����   X axis

      acc_correction[Xr] += X_Delta*K_ACC_XY*CNTLCYCLE;//���ٶȽ�����
      //acc_correction[Xr]  = Accel_Correction_EF.East;//���ٶȽ�����
      vel_correction[Xr] += X_Delta* K_VEL_XY*CNTLCYCLE;//�ٶȽ�����
      pos_correction[Xr] += X_Delta* K_POS_XY*CNTLCYCLE;//λ�ý�����

      //acc_correction[Yr]  = Accel_Correction_EF.North;//���ٶȽ�����
      acc_correction[Yr] += Y_Delta* K_ACC_XY*CNTLCYCLE;//���ٶȽ�����
      vel_correction[Yr] += Y_Delta* K_VEL_XY*CNTLCYCLE;//�ٶȽ�����
      pos_correction[Yr] += Y_Delta* K_POS_XY*CNTLCYCLE;//λ�ý�����

      /*************************************************************/
      //ˮƽ�˶����ٶȼ�У��
      X_KF_NAV[Xr][2]=accx+acc_correction[Xr];
      //�ٶ�������������£����ڸ���λ��
      SpeedDealt[Xr]=X_KF_NAV[Xr][2]*CNTLCYCLE;
      //ԭʼλ�ø���
      float OPositionx=posx+(X_KF_NAV[Xr][1]+0.5*SpeedDealt[Xr])*CNTLCYCLE;
      //λ�ý��������
      X_KF_NAV[Xr][0]=OPositionx+pos_correction[Xr];
      //ԭʼ�ٶȸ���
      float OSpeedx=spdx+SpeedDealt[Xr];
      //�ٶȽ��������
      X_KF_NAV[Xr][1]=OSpeedx+vel_correction[Xr];

        /*************************************************************/
      //ˮƽ�˶����ٶȼ�У��
      X_KF_NAV[Yr][2]=accy+acc_correction[Yr];
      //�ٶ�������������£����ڸ���λ��
      SpeedDealt[Yr]=X_KF_NAV[Yr][2]*CNTLCYCLE;
      //ԭʼλ�ø���
      float OPositiony=posx+(X_KF_NAV[Yr][1]+0.5*SpeedDealt[Yr])*CNTLCYCLE;
      //λ�ý��������
      X_KF_NAV[Yr][0]=OPositiony+pos_correction[Yr];
      //ԭʼ�ٶȸ���
      float OSpeedy=spdy+SpeedDealt[Yr];
      //�ٶȽ��������
      X_KF_NAV[Yr][1]=OSpeedy+vel_correction[Yr];
}




float SINS_Accel_Body[3];
float SINS_Accel_Earth[2]={0,0};
void  SINS_Prepare(void)
{

      float Sin_Pitch=sin(Pitch* 0.0173);
      float Cos_Pitch=cos(Pitch* 0.0173);
      float Sin_Roll=sin(-Roll* 0.0173);
      float Cos_Roll=cos(-Roll* 0.0173);
      float Sin_Yaw=sin(Yaw* 0.0173);
      float Cos_Yaw=cos(Yaw* 0.0173);
      float Acce_Control[3];
			Acce_Control[0]=LPButterworth(imu_fushion.Acc.y,
				&Butter_Buffer[0],&Butter_30HZ_Parameter_Acce);
			Acce_Control[1]=LPButterworth(imu_fushion.Acc.x
				,&Butter_Buffer[1],&Butter_30HZ_Parameter_Acce);
			Acce_Control[2]=LPButterworth(imu_fushion.Acc.z
				,&Butter_Buffer[2],&Butter_30HZ_Parameter_Acce);

			float oAcceleration[3];
      oAcceleration[2] =
                            -Sin_Roll* Acce_Control[0]
                              + Sin_Pitch *Cos_Roll * Acce_Control[1]
                                 + Cos_Pitch * Cos_Roll *Acce_Control[2];

      oAcceleration[Xr]=
                         Cos_Yaw* Cos_Roll * Acce_Control[0]
                              +(Sin_Pitch*Sin_Roll*Cos_Yaw-Cos_Pitch * Sin_Yaw) * Acce_Control[1]
                                +(Sin_Pitch * Sin_Yaw+Cos_Pitch * Sin_Roll * Cos_Yaw)*Acce_Control[2];

      oAcceleration[Yr]=
                         Sin_Yaw* Cos_Roll * Acce_Control[0]
                              +(Sin_Pitch * Sin_Roll * Sin_Yaw +Cos_Pitch * Cos_Yaw) * Acce_Control[1]
                                + (Cos_Pitch * Sin_Roll * Sin_Yaw - Sin_Pitch * Cos_Yaw)*Acce_Control[2];
																
      oAcceleration[2]*=9.8/4096.;
      oAcceleration[2]-=9.8;//��ȥ�������ٶ�

      oAcceleration[Xr]*=9.8/4096.;

      oAcceleration[Yr]*=9.8/4096.;


   /******************************************************************************/
   //�����˻��ڵ�������ϵ�µ���������������������˶����ٶ���ת����ǰ������˶����ٶ�:��ͷ(����)+���

      SINS_Accel_Earth[Xr]=-oAcceleration[Xr];//�ص�������ϵ�����������˶����ٶ�,��λΪCM
      SINS_Accel_Earth[Yr]=oAcceleration[Yr];//�ص�������ϵ�����������˶����ٶ�,��λΪCM


      SINS_Accel_Body[Xr]=-(SINS_Accel_Earth[Xr]*Cos_Yaw+SINS_Accel_Earth[Yr]*Sin_Yaw);  //��������˶����ٶ�  X������
      SINS_Accel_Body[Yr]=-(-SINS_Accel_Earth[Xr]*Sin_Yaw+SINS_Accel_Earth[Yr]*Cos_Yaw); //��ͷ�����˶����ٶ�  Y������
      SINS_Accel_Body[Zr]= oAcceleration[2];
}
