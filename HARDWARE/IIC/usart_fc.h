#ifndef _USART_H
#define _USART_H
#include "flow.h"
#include "stm32f4xx.h"

extern FLOW flow;
extern FLOW_RAD flow_rad,flow_rad_mine;
extern int par[12];
extern u8 Rx_Buf[], force_fly_ready,acc_3d_step;
void Usart2_Init(u32 br_num);
void Usart2_IRQ(void);
void Usart3_IRQ(void);
void Usart2_Send(unsigned char *DataToSend ,u8 data_num);

void Uart5_Init(u32 br_num);
void Uart5_IRQ(void);
void Uart5_Send(unsigned char *DataToSend ,u8 data_num);
extern float k_scale_pix;
typedef struct PID_STA{u16 OP,OI,OD,IP,II,ID,YP,YI,YD;}PID_STA;
extern PID_STA HPID,SPID,FIX_PID,NAV_PID;
extern float flow_matlab_data[4], baro_matlab_data[2];
typedef struct int16_rcget{
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
				int16_t AUX1;
				int16_t AUX2;
				int16_t AUX3;
				int16_t AUX4;
				int16_t AUX5;
				u8 RST;}RC_GETDATA;

extern RC_GETDATA Rc_Get;//���յ���RC����,1000~2000
				
				
struct _float{
	      float x;
				float y;
				float z;};

struct _int16{
       int16_t x;
	     int16_t y;
	     int16_t z;};		

struct _trans{
     struct _int16 origin;  //ԭʼֵ
	   struct _float averag;  //ƽ��ֵ
	   struct _float histor;  //��ʷֵ
	   struct _int16 quiet;   //��ֵ̬
	   struct _float radian;  //����ֵ 
          };

struct _alt{
int32_t altitude;
float altitude_f;
int32_t Temperature;
int32_t Pressure;
int Temperat;
};
					
struct _sensor{   
	struct _trans acc;
	struct _trans gyro;
	struct _trans hmc;
	struct _alt alt;
              };

extern struct _sensor sensor;	

	

struct _speed{   
	int altitude;
	int bmp;
	int pitch;
	int roll;
	int gps;
	int filter;
	int sonar;
	int acc;
              };

struct _altitude{   
	int bmp;
	int sonar;
	int gps;
	int acc;
	int filter;
              };
struct _angle{   
float pitch;
float roll;
float yaw;
              };

							
struct _get{   
	struct _speed speed;
	struct _altitude altitude;
	struct _angle AngE;
              };
struct _plane{   
	struct _speed speed;
	struct _altitude altitude;
	struct _get get;
              };

extern struct _plane plane;	
							
							
struct _slam{   
	 int16_t spd[5];
	 int16_t dis[5];
              };
extern struct _slam slam;	
							
							
struct _SPEED_FLOW_NAV{
float west;
float east;
float x;
float y;
float x_f;
float y_f;
};	
struct _POS_FLOW_NAV {
float	east;
float	west;
long LAT;
long LON;
long Weidu_Dig;
long Jingdu_Dig;
u8 flow_qual;
};
struct _POS_GPS_NAV {
long J;
long W;
long X_O;
long Y_O;
long X_UKF;
long Y_UKF;
u8 gps_mode;
u8 star_num;
};

struct _FLOW_NAV{
struct _SPEED_FLOW_NAV speed;
struct _SPEED_FLOW_NAV speed_h;	
struct _POS_FLOW_NAV position;
};	

struct _IMU_NAV{   
struct _FLOW_NAV flow;
struct _POS_GPS_NAV gps;
	
};
extern struct _IMU_NAV imu_nav;
	

struct _FLOW_DEBUG{   
int ax,ay,az,gx,gy,gz,hx,hy,hz;
u8 en_ble_debug;
};
extern struct _FLOW_DEBUG flow_debug;


struct _PID2{
float p;
float i;
float d;
float i_max;
float limit;
float dead;	
float dead2;	
};
struct _PID1{
struct _PID2 out;
struct _PID2 in;	
};
struct _PID_SET{
struct _PID1 nav;
struct _PID1 high;
struct _PID1 avoid;
struct _PID1 circle;
};
extern struct _PID_SET pid;

struct _MODE
{
u8 thr_fix;
u8 en_pid_out_pit;
u8 en_pid_out_rol;
u8 en_pid_out_yaw;
u8 en_pid_fuzzy_p;
u8 en_pid_sb_set;
u8 en_fuzzy_angle_pid;
u8 en_sensor_equal_flp;
u8 pit_rol_pid_out_flp;	
u8 en_pid_yaw_angle_control;	
u8 en_pid_yaw_control_by_pit_rol;	
u8 thr_add_pos;
u8 spid;
u8 mpu6050_bw_42;
u8 en_imu_q_update_mine;	
u8 en_moto_smooth;
u8 pid_mode;
u8 no_head;
u8 sonar_avoid;	
u8 yaw_sel;
u8 sb_smooth;
u8 flow_hold_position;
u8 flow_hold_position_use_global;
u8 flow_hold_position_high_fix;
u8 height_safe;
u8 hunman_pid;
u8 yaw_imu_control;	
u8 cal_sel;
u8 flow_sel;
u8 height_in_speed;
u8 height_upload;
u8 en_h_mode_switch;
u8 en_dj_cal;
u8 en_sd_save;
u8 hold_use_flow;
u8 en_sonar_avoid;
u8 thr_fix_test;
u8 en_imu_ekf;
u8 att_pid_tune;
u8 high_pid_tune;
u8 dj_lock;
u8 en_eso;
u8 en_eso_h_out;
u8 en_eso_h_in;
u8 en_circle_control;
u8 save_video;
u8 test1;
u8 test2;
u8 test3;
u8 test4;	
u8 mode_fly;

//flow
u8 en_flow_gro_fix;
u8 flow_size;
};
	

typedef struct{
	unsigned int x;//Ŀ���x����
	unsigned int y;//Ŀ���y����
	unsigned int w;//Ŀ��Ŀ��
	unsigned int h;//Ŀ��ĸ߶�
	
	u8 check;
}RESULT;//ʶ����

extern RESULT color;
extern u8 LOCK, KEY[8],KEY_SEL[4],NAV_BOARD_CONNECT;
extern struct _MODE mode;
extern void GOL_LINK_TASK(void);
extern void SD_LINK_TASK(void);
extern void Usart1_Init(u32 br_num);//SD_board
extern void Usart4_Init(u32 br_num);//-------SD_board
extern void Usart3_Init(u32 br_num);//-------CPU_board
extern u8 cnt_nav_board;
extern u16 data_rate_gol_link;
extern void UART2_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz);
extern void UART2_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);
void SD_LINK_TASK2(u8 sel);
extern void Send_MODE_SD(void);
void Send_IMU_NAV(void);
#define SEND_IMU 0
#define SEND_FLOW 1
#define SEND_GPS 2
#define SEND_ALT 3
extern float rate_gps_board;
void Send_IMU_TO_GPS(void);
#define SEND_BUF_SIZE1 64	//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.
extern u8 SendBuff1[SEND_BUF_SIZE1];	//�������ݻ�����

void data_per_uart1(int16_t ax,int16_t ay, int16_t az, int16_t gx,int16_t  gy, int16_t gz,int16_t hx, int16_t hy, int16_t hz,
	int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);


#define SEND_BUF_SIZE2 64*6	//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.
extern u8 SendBuff2[SEND_BUF_SIZE2];	//�������ݻ�����
void data_per_uart2(void);
extern u16 nrf_uart_cnt;
#define SEND_BUF_SIZE3 32	//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.
extern u8 SendBuff3[SEND_BUF_SIZE3];	//�������ݻ�����
void data_per_uart3(void);

#define SEND_BUF_SIZE4 64	//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.
extern u8 SendBuff4[SEND_BUF_SIZE4];	//�������ݻ�����
void data_per_uart4(u8 sel);
#define SEND_IMU_ATT 8
#define SEND_IMU_FLOW 1
#define SEND_IMU_GPS 2
#define SEND_IMU_ALT 3
#define SEND_IMU_PID 4
#define SEND_IMU_DEBUG 5
#define SEND_IMU_QR 6
#define SEND_IMU_MEMS 7
#define SEND_ALL 9
extern float sonar_fc,baroAlt_fc;
extern float k_flow_devide;
extern float flow_module_offset_y,flow_module_offset_x,flow_module_set_yaw;//������װλ�� ��λ��

void GOL_LINK_TASK_DMA(void);
extern u8 en_ble_debug;
extern void GOL_LINK_BLE_DEBUG(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz);

void Send_BLE_DEBUG(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz);

extern FLOW_RAD flow_rad,flow_rad_mine;


//GPS
#define USART3_MAX_RECV_LEN		800					//�����ջ����ֽ���
#define USART3_MAX_SEND_LEN		800					//����ͻ����ֽ���
extern u8 kf_data_sel;
extern u16 USART3_RX_STA;
extern u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//���ջ���,���USART3_MAX_RECV_LEN���ֽ�.
extern u8 USART1_TX_BUF[USART3_MAX_RECV_LEN]; 					//����1,���ͻ�����
extern __align(8) u8 USART3_TX_BUF[USART3_MAX_SEND_LEN]; 	//���ͻ���,���USART3_MAX_SEND_LEN�ֽ�
void clear_nrf_uart(void);


void Laser_start(void);
void Laser_stop(void);
extern u16 laser_buf[360];
extern u16 Laser_avoid[60];
void Laser_cal(void);
extern u8 gps_good;
extern u16 gps_loss_cnt;

struct _QR{
float x,y,z,pit,rol,yaw;
float spdx,spdy,spdz;
u8 check,use_spd,connect;
float yaw_off;
u8 insert,update,qual;
u32 last_update;
u16 loss_cnt;
float dt;
};
extern struct _QR qr;


struct _FLOW_PI{
float x,y,z,z_o,pit,rol,yaw;
float spdx,spdy,spdz;
float yaw_off;
u16 loss_cnt;
u8 check,use_spd,connect,insert,update;
uint32_t last_update;	
struct _QR sensor;
float acc[3];
float gyro[3];
int mark_map[10][5];	
};
extern struct _FLOW_PI pi_flow;
void Send_TO_FLOW_PI(void);
extern int debug_pi_flow[20];



 typedef struct
{
 float Pit,Rol,Yaw;
 double Lat,Lon;
 float H,H_Spd;	
 u8 GPS_STATUS;	//>3  5->Best
/*
Flight status val	status name
1	standby
2	take_off
3	in_air
4	landing
5	finish_landing
*/
 u8 STATUS;
 float Bat;	
 int Rc_pit,Rc_rol,Rc_yaw,Rc_thr,Rc_mode,Rc_gear;
 u8 connect,m100_data_refresh;
 u16 loss_cnt,cnt_m100_data_refresh;
 float rx_dt;
 float spd[3];
}M100;
extern M100 m100;
void UsartSend_M100(uint8_t ch);

void Ublox_PVT_Mode(void);

extern u8 en_px4_mapper,imu_feed_dog;
void Send_TO_FLOW_NAV_GPS(void);
extern u8 FC_CONNECT;
extern u16 fc_loss_cnt;
extern uint32_t gpsData_lastPosUpdate,gpsData_lastVelUpdate,gps_update;
#endif
