#include "ukf_task.h"
#include "ukf_oldx.h"
#include "my_math.h"
#include "usart_fc.h"
#include "gps.h"
#include "KF_OLDX_NAV.h"
#include "hml5833l.h"
#include "alt_kf.h"
#include "insgps.h"
//#include "ekf_ins.h"

navUkfStruct_t navUkfData;

float DELAY_GPS= 0.45;//0.2;//s
int   DELAY_GPS_STATE= -10000;//us
#define INSGPS_GPS_TIMEOUT 2   /* 2 seconds triggers reinit of position */
#define INSGPS_GPS_MINSAT  6   /* 2 seconds triggers reinit of position */
#define INSGPS_GPS_MINPDOP 3.5 /* minimum PDOP for postition updates    */
#define INSGPS_MAGLEN      1000
#define INSGPS_MAGTOL      0.5 /* error in magnetic vector length to use  */
//! Contains data from the GPS (via the SPI link)

struct gps_sensor gps_data;
struct mag_sensor mag_data;
//! Contains the data from the accelerometer
struct accel_sensor  accel_data;
//! Contains the data from the gyro
struct gyro_sensor gyro_data;
//! Conains the current estimate of the attitude
struct attitude_solution attitude_data;

int ekf_hml_flag[3]={1,1,1};
float angle_ins[3];


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
double X_ukf[6],X_ukf_global[6];
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
#define BUF_NUM 80
static float acc_body_buf[2][BUF_NUM];
static void feed_acc_buf(float in1,float in2)
{
u8 i,j;	
float reg[2][BUF_NUM];	
static u8 cnt;
 for(i=0;i<2;i++)
	for(j=0;j<BUF_NUM;j++)
   reg[i][j]=acc_body_buf[i][j];
	
 for(i=0;i<2;i++)
	for(j=0;j<BUF_NUM-1;j++)
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

u8 kf_data_sel=1;//0->flow 1->gps 2->flow global 3->openpilot
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
float g_pos_gps= 0.06;//10;
float g_spd_gps= 0.0004*1000;//0.000366*1000;
float g_spd_gps_use;
float g_spd_gps_state;
#endif
float velNorth_gps,velEast_gps;
int flag_kf1[2]={1,1};
u8 force_test;
float Posx,Posy;
u8 gps_data_vaild=0;
float  r_gps_new[4]={0.015,0.056,0.026,3};
double state_correct_posx[6];
double state_correct_posy[6];
double X_kf2_x[3];
double X_kf2_y[3];
u16 	histIndex ;
void ukf_pos_task_qr(float Qr_x,float Qr_y,float Yaw,float flowx,float flowy,float accx,float accy,float T)
{
static int gps_h_off;	
static u8 gps_init;
float Sdpx,Accx,Accx_now;
float Sdpy,Accy,Accy_now;

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
#if USE_M100_IMU
u8 kf_data_sel_temp=1;  
#else 
u8 kf_data_sel_temp=kf_data_sel; 
#endif 
 
if(module.gps&& gpsx.pvt.PVT_numsv>=1&&gpsx.pvt.PVT_fixtype>=3)
kf_data_sel_temp=1;	
else if(module.flow||module.flow_iic)
{kf_data_sel_temp=1;gps_init=0;}	
// kf_data_sel_temp=0;
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
	 
	 float SPDY=flowy*K_spd_flow;
	 float SPDX=flowx*K_spd_flow;
   float velEast_flow=SPDY*sin(Yaw*0.0173)+SPDX*cos(Yaw*0.0173);
	 float velNorth_flow=SPDY*cos(Yaw*0.0173)-SPDX*sin(Yaw*0.0173);
	 
	 #if !USE_M100_IMU
   velEast=LIMIT(gpsx.pvt.PVT_East_speed,-3,3);
   velNorth=LIMIT(gpsx.pvt.PVT_North_speed,-3,3);
	 if(!gps_init){
	 velEast=LIMIT(velEast_flow,-3,3);
   velNorth=LIMIT(velNorth_flow,-3,3);
   }		 
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
	 
   Global_GPS_Sensor.NED_Pos[1]=Posy=posNorth*K_pos_gps;
	 Global_GPS_Sensor.NED_Vel[1]=Sdpy=velNorth*K_spd_gps;
	 Global_GPS_Sensor.NED_Acc[1]=Accy=get_acc_delay(1,DELAY_GPS,T);//accNorth;
	 Global_GPS_Sensor.NED_Pos[0]=Posx=posEast*K_pos_gps;
	 Global_GPS_Sensor.NED_Vel[0]=Sdpx=velEast*K_spd_gps;
	 Global_GPS_Sensor.NED_Acc[0]=Accx=get_acc_delay(0,DELAY_GPS,T);//accEast;
	 Accy_now=get_acc_delay(1,0,T);
	 Accx_now=get_acc_delay(0,0,T);
	 u8 flag_sensor[3]={1,0,1};
	 double Zx[3]={Posx,Sdpx,Accx};
	 double Zy[3]={Posy,Sdpy,Accy};
	 
#define USE_STATE_FORWARD_UKF 	
//#define FORWARD_IN_MEASURE 	 
	//update speed & pos
	#if defined(USE_STATE_FORWARD_UKF)	 	 
 if(((gps_init&&gps_data_vaild)||force_test)&&1)//gpsx.pvt.gps_update)//bei
	 {
		g_spd_gps_state=0.001*gpsx.ubm.sAcc * __sqrtf(gpsx.pvt.tDOP*0.01*gpsx.pvt.tDOP*0.01 + gpsx.pvt.nDOP*0.01*gpsx.pvt.nDOP*0.01) * 0.030;
		g_spd_gps_use=0.001*g_spd_gps+g_spd_gps_state*1;	
		
  	float posDelta[3],velDelta[3];
		if(gpsx.pvt.gps_update||1)		 
		{	 
		gpsx.pvt.gps_update=0;
			
		histIndex = (micros() - (gpsx.pvt.last_update + DELAY_GPS_STATE)) / (int)(1e6f * T);	 	
		histIndex = navUkfData.navHistIndex - histIndex;
		if (histIndex < 0)
				histIndex += UKF_HIST;
		if (histIndex < 0 || histIndex >= UKF_HIST)
				histIndex = 0;		 
		posDelta[1] = UKF_POSN - navUkfData.posN[histIndex];
		posDelta[0] = UKF_POSE - navUkfData.posE[histIndex];
		// set current position state to historic data
		UKF_POSN = navUkfData.posN[histIndex];
		UKF_POSE = navUkfData.posE[histIndex];
		// calculate delta from current position
		velDelta[1] = UKF_VELN - navUkfData.velN[histIndex];
		velDelta[0] = UKF_VELE - navUkfData.velE[histIndex];
		// set current position state to historic data
		UKF_VELN = navUkfData.velN[histIndex];
		UKF_VELE = navUkfData.velE[histIndex];
					
		KF_OLDX_NAV( X_KF_NAV[1],  P_KF_NAV[1],  Zy,  Accy, A,  B,  H,  ga_nav,  gwa_nav, g_pos_gps,  g_spd_gps_use,  T);
	  KF_OLDX_NAV( X_KF_NAV[0],  P_KF_NAV[0],  Zx,  Accx, A,  B,  H,  ga_nav,  gwa_nav, g_pos_gps,  g_spd_gps_use,  T);
		
		// add the historic position delta back to the current state
		UKF_POSN += posDelta[1];
		UKF_POSE += posDelta[0];
    // add the historic position delta back to the current state
    UKF_VELN += velDelta[1];
    UKF_VELE += velDelta[0];		
		}	
		else{
		//ins update	
		H[0]=H[4]=0;
    KF_OLDX_NAV( X_KF_NAV[1],  P_KF_NAV[1],  Zy,  Accy, A,  B,  H,  ga_nav,  gwa_nav, 5,  5,  T);
    KF_OLDX_NAV( X_KF_NAV[0],  P_KF_NAV[0],  Zx,  Accx, A,  B,  H,  ga_nav,  gwa_nav, 5,  5,  T);	
		}	
		
    // store history
		navUkfData.posN[navUkfData.navHistIndex] = UKF_POSN;
		navUkfData.posE[navUkfData.navHistIndex] = UKF_POSE;
		navUkfData.posD[navUkfData.navHistIndex] = 0;

		navUkfData.velN[navUkfData.navHistIndex] = UKF_VELN;
		navUkfData.velE[navUkfData.navHistIndex] = UKF_VELE;
		navUkfData.velD[navUkfData.navHistIndex] = 0;	 
    navUkfData.navHistIndex = (navUkfData.navHistIndex + 1) % UKF_HIST;

	 }
#else	 
	 if(((gps_init&&gps_data_vaild)||force_test)&&1){
		 
	 #if defined(FORWARD_IN_MEASURE)	  
		 Zx[1]+=DELAY_GPS*Zx[2];
		 Zx[0]+=DELAY_GPS*Zx[1]+1/2*pow(DELAY_GPS,2)*(Zx[2]);
		
		 
		 
		 Zy[1]+=DELAY_GPS*Zy[2];
		 Zy[0]+=DELAY_GPS*Zy[1]+1/2*pow(DELAY_GPS,2)*(Zy[2]);
		 
		 Accy=Accy_now;
		 Accx=Accx_now;
	 #endif	 
	
//	 Zx[1]+=DELAY_GPS*Zx[2];
//	 Zy[1]+=DELAY_GPS*Zy[2];		 
	 g_spd_gps_state=0.001*gpsx.ubm.sAcc * __sqrtf(gpsx.pvt.tDOP*0.01*gpsx.pvt.tDOP*0.01 + gpsx.pvt.nDOP*0.01*gpsx.pvt.nDOP*0.01) * 0.030;
	 g_spd_gps_use=0.001*g_spd_gps+g_spd_gps_state*1;	 
	 KF_OLDX_NAV( X_KF_NAV[1],  P_KF_NAV[1],  Zy,  Accy, A,  B,  H,  ga_nav,  gwa_nav, g_pos_gps,  g_spd_gps_use,  T);
	 KF_OLDX_NAV( X_KF_NAV[0],  P_KF_NAV[0],  Zx,  Accx, A,  B,  H,  ga_nav,  gwa_nav, g_pos_gps,  g_spd_gps_use,  T);}
#endif	 
	 X_KF_NAV[0][1]=LIMIT(X_KF_NAV[0][1],-6.6,6.6);
	 X_KF_NAV[1][1]=LIMIT(X_KF_NAV[1][1],-6.6,6.6); 
	 

	 float X_KF_NAV_TEMP[2][3];//0->x->east  1->y->north
	 X_KF_NAV_TEMP[0][0]=X_KF_NAV[0][0]+gpsx.pvt.rx_dt*2*X_KF_NAV[0][1]+1/2*pow(gpsx.pvt.rx_dt*2,2)*(Accx-X_KF_NAV[0][2]);
	 X_KF_NAV_TEMP[0][1]=X_KF_NAV[0][1]+gpsx.pvt.rx_dt*2*(Accx-X_KF_NAV[0][2]);
	 X_KF_NAV_TEMP[0][2]=X_KF_NAV[0][2];
	 X_KF_NAV_TEMP[1][0]=X_KF_NAV[1][0]+gpsx.pvt.rx_dt*2*X_KF_NAV[1][1]+1/2*pow(gpsx.pvt.rx_dt*2,2)*(Accy-X_KF_NAV[1][2]);
	 X_KF_NAV_TEMP[1][1]=X_KF_NAV[1][1]+gpsx.pvt.rx_dt*2*(Accy-X_KF_NAV[1][2]);
	 X_KF_NAV_TEMP[1][2]=X_KF_NAV[1][2];
	 #if defined(FORWARD_IN_MEASURE)	
	 X_KF_NAV_TEMP[0][0]=X_KF_NAV[0][0];
	 X_KF_NAV_TEMP[0][1]=X_KF_NAV[0][1];
	 X_KF_NAV_TEMP[0][2]=X_KF_NAV[0][2];
	 X_KF_NAV_TEMP[1][0]=X_KF_NAV[1][0];
	 X_KF_NAV_TEMP[1][1]=X_KF_NAV[1][1];
	 X_KF_NAV_TEMP[1][2]=X_KF_NAV[1][2];
	 #endif
	 
	 X_ukf[0]=X_KF_NAV_TEMP[0][0];
	 X_ukf_global[1]=X_KF_NAV[0][1];
	 X_ukf[2]=X_KF_NAV_TEMP[0][2];
	 X_ukf[3]=X_KF_NAV_TEMP[1][0];
	 X_ukf_global[4]=X_KF_NAV[1][1];
	 X_ukf[5]=X_KF_NAV_TEMP[1][2];
	 //turn to body frame
	 X_ukf[4]=X_KF_NAV_TEMP[1][1]*cos(Yaw*0.0173)+X_KF_NAV_TEMP[0][1]*sin(Yaw*0.0173);//Y
	 X_ukf[1]=-X_KF_NAV_TEMP[1][1]*sin(Yaw*0.0173)+X_KF_NAV_TEMP[0][1]*cos(Yaw*0.0173);//X
	 
	 X_ukf_Pos[0]=X_ukf[0];//East Pos
	 X_ukf_Pos[1]=X_ukf[3];//North Pos							

   CalcGlobalLocation(X_ukf[0],X_ukf[3]);}
//---------------------------GPS _UKF-----------------------------------------------------------
else if(kf_data_sel_temp==4){   
	 float ACCY=flow_matlab_data[1]*K_acc_gps;
   float ACCX=flow_matlab_data[0]*K_acc_gps;
	 float accEast=ACCY*sin(Yaw*0.0173)+ACCX*cos(Yaw*0.0173);
   float accNorth=ACCY*cos(Yaw*0.0173)-ACCX*sin(Yaw*0.0173);

	 if(gpsx.gpssta>0&&gpsx.latitude!=0)
	 {CalcEarthRadius(gpsx.latitude); gps_data_vaild=1;}
	 	#if USE_M100_IMU
	 H[4]=0; 
	 if(m100.connect&&m100.m100_data_refresh==1&&m100.Yaw!=0)
	 gps_data_vaild=1;
	 #endif

	 CalcGlobalDistance(gpsx.latitude,gpsx.longitude); 
	 static float dposEast,dposNorth;
	 static u8 cnt;
	 
   velEast=LIMIT(-gpsx.spd*sin((gpsx.angle-180)*0.0173),-3,3);
   velNorth=LIMIT(-gpsx.spd*cos((gpsx.angle-180)*0.0173),-3,3);
	 
		#if !USE_M100_IMU
	 if(!gps_data_vaild)
	 H[0]=velEast=velNorth=0;
	 #endif
	 
   Posy=posNorth*K_pos_gps;
	 Sdpy=velNorth*K_spd_gps;
	 Accy=accNorth;
	 Posx=posEast*K_pos_gps;
	 Sdpx=velEast*K_spd_gps;
	 Accx=accEast;

	 if((gps_init&&gps_data_vaild)||force_test)//bei
	 {
	 
	  srcdkfTimeUpdate(gpsUkfData_n.kf, &Accy,T);//5000			    // us (200 Hz)
    srcdkfTimeUpdate(gpsUkfData_e.kf, &Accx,T);//5000			    // us (200 Hz)
	  float noise;        // measurement variance
    float y[2];            // measurment
    gpsUkfData_n.x = srcdkfGetState(gpsUkfData_n.kf);
		gpsUkfData_e.x = srcdkfGetState(gpsUkfData_e.kf);
	
		y[0] =LIMIT(Sdpy,-3,3);
		y[1] =LIMIT(Sdpx,-3,3);
	 srcdkfMeasurementUpdate(gpsUkfData_n.kf, 0, &y[0], 1, 1, &g_spd_gps, gpsUpdate);
	 srcdkfMeasurementUpdate(gpsUkfData_e.kf, 0, &y[1], 1, 1, &g_spd_gps, gpsUpdate);
	 }
   
	 X_ukf[0]=gpsUkfData_e.x[0];//North pos
	 //X_ukf[1]=X_KF_NAV[0][1];//North vel
	 X_ukf[2]=gpsUkfData_e.x[2];
	 X_ukf[3]=gpsUkfData_n.x[0];//East  pos
	 //X_ukf[4]=X_KF_NAV[1][1];//East  vel
	 X_ukf[5]=gpsUkfData_n.x[2];
									//bei												dong
	 X_ukf[1]=-gpsUkfData_e.x[1]*sin(Yaw*0.0173)+gpsUkfData_e.x[0]*cos(Yaw*0.0173);//X
	 X_ukf[4]=gpsUkfData_n.x[1]*cos(Yaw*0.0173)+gpsUkfData_e.x[0]*sin(Yaw*0.0173);//Y
	 								//bei												dong
	 X_ukf_Pos[0]=velNorth*cos(Yaw*0.0173)+velEast*sin(Yaw*0.0173);//Y 
	 X_ukf_Pos[1]=-velNorth*sin(Yaw*0.0173)+velEast*cos(Yaw*0.0173);//X
// 	 X_ukf_Pos[0]=X_ukf[0];//X
//   X_ukf_Pos[1]=X_ukf[3];//Y
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
	 //H[8]=1;  //no acc bias
//	 if(par[0]!=0)g_pos_flow=(float)par[0]/1000.;
//   if(par[1]!=0)
//		 g_spd_flow=(float)par[1]/1000.;
//	 if(par[2]!=0)K_spd_flow=(float)par[2]/1000.;
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
	 X_ukf_global[1]=X_KF_NAV[0][1];//East vel
	 X_ukf[2]=X_KF_NAV[0][2];
	 X_ukf[3]=X_KF_NAV[1][0]+X_ukf[5]*T*15;//North  pos
	 X_ukf_global[4]=X_KF_NAV[1][1];//North  vel
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
	#else 
   ukf_flow( flowx, flowy, accx, accy,T);
	#endif
}



