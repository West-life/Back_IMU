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
float local_Lat,local_Lon;
float velEast,velNorth;
float GPS_J_F,GPS_W_F;
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


static void CalcGlobalLocation(posNorth,posEast){ 
    GPS_W_F=(float)posNorth/(float)(r1+0.1)+local_Lat;
    GPS_J_F=(float)posEast/(float)(r2+0.1)+local_Lon;
}


u8 kf_data_sel=2;//0->flow 1->gps 2->flow global 3->openpilot
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
float g_pos_gps= 10;
float g_spd_gps= 0.1;//0.1;                          
float velNorth_gps,velEast_gps;
int flag_kf1[2]={1,1};
u8 force_test;
float Posx,Posy;
void ukf_pos_task_qr(float Qr_x,float Qr_y,float Yaw,float flowx,float flowy,float accx,float accy,float T)
{
static int gps_h_off;	
static u8 gps_init;
float Sdpx,Accx;
float Sdpy,Accy;
u8 gps_data_vaild=0;
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

 if((gpsx.latitude!=0||force_test) && gps_init==0){
 gps_init=1;
 local_Lon=gpsx.longitude;
 local_Lat=gpsx.latitude;
 gps_h_off=	gpsx.altitude; 
 CalcEarthRadius(gpsx.latitude);

 }
 
u8 kf_data_sel_temp=kf_data_sel; 
// kf_data_sel_temp=0;
#if NAV_USE_KF
 
if(kf_data_sel_temp==1){//GPS
   float ACCY=flow_matlab_data[1]*K_acc_gps;
   float ACCX=flow_matlab_data[0]*K_acc_gps;
	 float accEast=ACCY*sin(Yaw*0.0173)+ACCX*cos(Yaw*0.0173);
   float accNorth=ACCY*cos(Yaw*0.0173)-ACCX*sin(Yaw*0.0173);

	 if(gpsx.gpssta>0&&gpsx.latitude!=0)
	 {CalcEarthRadius(gpsx.latitude); gps_data_vaild=1;}
	 
//	 if(par[0]!=0)
//		 g_pos_gps=(float)par[0]/100.;
//   if(par[1]!=0)
//		 g_spd_gps=(float)par[1]/100.;
//	 if(par[2]!=0)
//		 K_acc_gps=(float)par[2]/100.;
	 CalcGlobalDistance(gpsx.latitude,gpsx.longitude); 
	 
	 static float dposEast,dposNorth;
	 static u8 cnt;
	 if(cnt++>0.2/T){cnt=0;
	 velNorth_gps=(posNorth-dposNorth)/(0.2+0.00001);
	 velEast_gps=(posEast-dposEast)/(0.2+0.00001);
	 dposEast=posEast;
	 dposNorth=posNorth;
	 }
	 
   velEast=LIMIT(-gpsx.spd*sin((gpsx.angle-180)*0.0173),-3,3);
   velNorth=LIMIT(-gpsx.spd*cos((gpsx.angle-180)*0.0173),-3,3);
	 
	 if(!gps_data_vaild)
	 H[0]=velEast=velNorth=0;
	 
   Global_GPS_Sensor.NED_Pos[2]=(float)(gpsx.altitude-gps_h_off)/10.;
   Global_GPS_Sensor.NED_Pos[1]=Posx=posNorth*K_pos_gps;
	 Global_GPS_Sensor.NED_Vel[1]=Sdpx=velNorth*K_spd_gps;
	 Accx=accNorth;
	 double Zx[3]={Posx,Sdpx,0};
	 if((gps_init&&gps_data_vaild)||force_test)//bei
   KF_OLDX_NAV( X_KF_NAV[1],  P_KF_NAV[1],  Zx,  Accx, A,  B,  H,  ga_nav,  gwa_nav, g_pos_gps,  g_spd_gps,  T);
	 Global_GPS_Sensor.NED_Pos[0]=Posy=posEast*K_pos_gps;
	 Global_GPS_Sensor.NED_Vel[0]=Sdpy=velEast*K_spd_gps;
	 Accy=accEast;
	 double Zy[3]={Posy,Sdpy,0};
	 if((gps_init&&gps_data_vaild)||force_test)//dong 
   KF_OLDX_NAV( X_KF_NAV[0],  P_KF_NAV[0],  Zy,  Accy, A,  B,  H,  ga_nav,  gwa_nav, g_pos_gps,  g_spd_gps,  T);
	 X_ukf[0]=X_KF_NAV[0][0];//North pos
	 //X_ukf[1]=X_KF_NAV[0][1];//North vel
	 X_ukf[2]=X_KF_NAV[0][2];
	 X_ukf[3]=X_KF_NAV[1][0];//East  pos
	 //X_ukf[4]=X_KF_NAV[1][1];//East  vel
	 X_ukf[5]=X_KF_NAV[1][2];
									//bei												dong
	 X_ukf[1]=X_KF_NAV[1][1]*cos(Yaw*0.0173)+X_KF_NAV[0][1]*sin(Yaw*0.0173);//Y
	 X_ukf[4]=-X_KF_NAV[1][1]*sin(Yaw*0.0173)+X_KF_NAV[0][1]*cos(Yaw*0.0173);//X
	 
	 								//bei												dong
	 X_ukf_Pos[0]=velNorth*cos(Yaw*0.0173)+velEast*sin(Yaw*0.0173);//Y 
	 X_ukf_Pos[1]=-velNorth*sin(Yaw*0.0173)+velEast*cos(Yaw*0.0173);//X
// 	 X_ukf_Pos[0]=X_ukf[0];//X
//   X_ukf_Pos[1]=X_ukf[3];//Y
   CalcGlobalLocation(X_ukf[0],X_ukf[3]);}
	if(kf_data_sel_temp==4){//-----------------------GPS _UKF-----------------------------------------------------------
   float ACCY=flow_matlab_data[1]*K_acc_gps;
   float ACCX=flow_matlab_data[0]*K_acc_gps;
	 float accEast=ACCY*sin(Yaw*0.0173)+ACCX*cos(Yaw*0.0173);
   float accNorth=ACCY*cos(Yaw*0.0173)-ACCX*sin(Yaw*0.0173);

	 if(gpsx.gpssta>0&&gpsx.latitude!=0)
	 {CalcEarthRadius(gpsx.latitude); gps_data_vaild=1;}
	 
//	 if(par[0]!=0)
//		 g_pos_gps=(float)par[0]/100.;
//   if(par[1]!=0)
//		 g_spd_gps=(float)par[1]/100.;
//	 if(par[2]!=0)
//		 K_acc_gps=(float)par[2]/100.;
	 CalcGlobalDistance(gpsx.latitude,gpsx.longitude); 
	 static float dposEast,dposNorth;
	 static u8 cnt;
	 
   velEast=LIMIT(-gpsx.spd*sin((gpsx.angle-180)*0.0173),-3,3);
   velNorth=LIMIT(-gpsx.spd*cos((gpsx.angle-180)*0.0173),-3,3);
	 
	 if(!gps_data_vaild)
	 H[0]=velEast=velNorth=0;
	 
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
  else if(kf_data_sel_temp==3){//---------------------pilot EKF in global--------------------------------------------------------
	 if(gpsx.gpssta>0&&gpsx.latitude!=0)
	 {CalcEarthRadius(gpsx.latitude);CalcGlobalDistance(gpsx.latitude,gpsx.longitude); gps_data_vaild=1;}
	   
	  gps_data.NED[0]=posNorth;
	  gps_data.NED[1]=posEast;
	  gps_data.NED[2]=0;
	 
		
  }
	else if(kf_data_sel_temp==2){//---------------------flow in global---------------------------------------------------------
	 static int qr_yaw_init;	
	 #if SENSOR_FORM_PI_FLOW	
	 float Yaw_qr=To_180_degrees(Yaw+pi_flow.yaw_off);
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
	}else{//--------------------------------------flow in body-------------------------------------------------------------------------	
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



