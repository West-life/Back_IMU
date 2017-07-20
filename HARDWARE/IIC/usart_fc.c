#include "LIS3MDL.h"
#include "include.h"
#include "usart_fc.h"
#include "ultrasonic.h"
#include "hml5833l.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "rc.h"
#include "att.h"
#include "height_ctrl.h"
#include "alt_kf.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "circle.h"
#include "error.h"
#include "flow.h"
#include "gps.h"
#include "ublox.h"
#include "flow.h"
void Usart2_Init(u32 br_num)//--GOL-link
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //����USART2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	//����PD5��ΪUSART2��Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//����PD6��ΪUSART2��Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

	

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = br_num;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART2, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���1 
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	

	//ʹ��USART2�����ж�
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//ʹ��USART2
	USART_Cmd(USART2, ENABLE); 
//	//ʹ�ܷ��ͣ�������λ���ж�
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}


}

struct _QR qr;
RESULT color;
float dt_flow_rx;
u16 data_rate_gol_link;
PID_STA HPID,SPID,FIX_PID,NAV_PID;
struct _PID_SET pid;
RC_GETDATA Rc_Get;
struct _plane plane;
struct _slam slam;
struct _IMU_NAV imu_nav;
struct _MODE mode;
u8 LOCK, KEY[8],KEY_SEL[4];
struct _FLOW_DEBUG flow_debug;
u8 NAV_BOARD_CONNECT=0;
u8 force_fly_ready;
int par[12];
float k_flow_opencv=0.88;
float sonar_fc,baroAlt_fc;
float fc_rx_time;
float flow_module_set_yaw;
u8 acc_3d_step,imu_feed_dog;
u8 FC_CONNECT;
u16 fc_loss_cnt=0;
void Data_Receive_Anl(u8 *data_buf,u8 num)
{ static u8 flag;
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//�ж�֡ͷ
   if(*(data_buf+2)==0x01)//IMU_FRAME  QR
  { FC_CONNECT=1;
		fc_loss_cnt=0;
		module.fc=1;
		fc_rx_time = Get_Cycle_T(GET_T_FC);			
    fly_ready=*(data_buf+10)||force_fly_ready||en_ble_debug;	
		qr.x=((vs16)(*(data_buf+11)<<8)|*(data_buf+12));
		qr.y=((vs16)(*(data_buf+13)<<8)|*(data_buf+14));
		qr.z=((vs16)(*(data_buf+15)<<8)|*(data_buf+16));
		qr.pit=((vs16)(*(data_buf+17)<<8)|*(data_buf+16));
		qr.rol=((vs16)(*(data_buf+19)<<8)|*(data_buf+20));
		qr.yaw=((vs16)(*(data_buf+21)<<8)|*(data_buf+22));
		qr.spdx=-((vs16)(*(data_buf+23)<<8)|*(data_buf+24))/10*k_flow_opencv;
		qr.spdy=-((vs16)(*(data_buf+25)<<8)|*(data_buf+26))/10*k_flow_opencv;
		qr.yaw_off=(float)((int16_t)(*(data_buf+27)<<8)|*(data_buf+28))/10.;
		qr.connect=*(data_buf+29);
		qr.check=*(data_buf+30);
		qr.use_spd=*(data_buf+31);
		sonar_fc=(float)((int16_t)(*(data_buf+32)<<8)|*(data_buf+33))/1000.;
    baroAlt_fc=(float)((int16_t)(*(data_buf+34)<<8)|*(data_buf+35));
		acc_3d_step=*(data_buf+36);
	}
		else if(*(data_buf+2)==0x14)//IMU_FRAME
  {
	
		if(!mpu6050.Acc_CALIBRATE&&*(data_buf+22)==1)
		mpu6050.Acc_CALIBRATE=1;
		if(!mpu6050.Gyro_CALIBRATE&&*(data_buf+23)==1)
		mpu6050.Gyro_CALIBRATE=1;
		if(!ak8975.Mag_CALIBRATED&&*(data_buf+24)==1)
		ak8975.Mag_CALIBRATED=1;
 
	}
		else if(*(data_buf+2)==0x82)//
  {
	  par[0]=(int16_t)(*(data_buf+4)<<8)|*(data_buf+5);//op
    par[1]=(int16_t)(*(data_buf+6)<<8)|*(data_buf+7);
		par[2]=(int16_t)(*(data_buf+8)<<8)|*(data_buf+9);
		par[3]=(int16_t)(*(data_buf+10)<<8)|*(data_buf+11);//ip
		par[4]=(int16_t)(*(data_buf+12)<<8)|*(data_buf+13);
		par[5]=(int16_t)(*(data_buf+14)<<8)|*(data_buf+15);
		par[6]=(int16_t)(*(data_buf+16)<<8)|*(data_buf+17);//yp
		par[7]=(int16_t)(*(data_buf+18)<<8)|*(data_buf+19);
		par[8]=(int16_t)(*(data_buf+20)<<8)|*(data_buf+21);
		par[9]=((int16_t)(*(data_buf+22)<<8)|*(data_buf+23));//hp
		par[10]=((int16_t)(*(data_buf+24)<<8)|*(data_buf+25));
		par[11]=((int16_t)(*(data_buf+26)<<8)|*(data_buf+27));
		//kf_data_sel=*(data_buf+28);
		if(!lis3mdl.Acc_CALIBRATE&&*(data_buf+29)==1)
		lis3mdl.Acc_CALIBRATE=1;
		if(!lis3mdl.Gyro_CALIBRATE&&*(data_buf+30)==1)
		lis3mdl.Gyro_CALIBRATE=1;
		static u8 mag_flag;
		if(!lis3mdl.Mag_CALIBRATED&&*(data_buf+31)==1&&mag_flag==0)
		mag_flag=lis3mdl.Mag_CALIBRATED=1;
	  
		if(!lis3mdl.Mag_CALIBRATED&&*(data_buf+31)==0)
			mag_flag=0;
	}	
		else if(*(data_buf+2)==0x83)//
  {
	 k_flow_devide=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/1000.;
   flow_module_offset_x=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/1000.;
	 flow_module_offset_y=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/1000.;
	 flow_module_set_yaw=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/10.;
	 en_px4_mapper=*(data_buf+12);
   imu_feed_dog=*(data_buf+13);		
	}
  	
}
 


u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0; 

u8 Rx_Buf[256];	//���ڽ��ջ���
u8 RxBuffer[50];
u8 RxState = 0;
u8 RxBufferNum = 0;
u8 RxBufferCnt = 0;
u8 RxLen = 0;
u8 com_data ;
static u8 _data_len = 0,_data_cnt = 0;
void USART2_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(USART2->SR & USART_SR_ORE)//ORE�ж�
	{
		com_data = USART2->DR;
	}

  //�����ж�
	if( USART_GetITStatus(USART2,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//����жϱ�־

		com_data = USART2->DR;
				if(RxState==0&&com_data==0xAA)
		{
			RxState=1;
			RxBuffer[0]=com_data;
		}
		else if(RxState==1&&com_data==0xAF)
		{
			RxState=2;
			RxBuffer[1]=com_data;
		}
		else if(RxState==2&&com_data>0&&com_data<0XF1)
		{
			RxState=3;
			RxBuffer[2]=com_data;
		}
		else if(RxState==3&&com_data<50)
		{
			RxState = 4;
			RxBuffer[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(RxState==4&&_data_len>0)
		{
			_data_len--;
			RxBuffer[4+_data_cnt++]=com_data;
			if(_data_len==0)
				RxState = 5;
		}
		else if(RxState==5)
		{
			RxState = 0;
			RxBuffer[4+_data_cnt]=com_data;
			Data_Receive_Anl(RxBuffer,_data_cnt+5);
		}
		else
			RxState = 0;
	
	}
	//���ͣ�������λ���ж�
	if( USART_GetITStatus(USART2,USART_IT_TXE ) )
	{
				
		USART2->DR = TxBuffer[TxCounter++]; //дDR����жϱ�־          
		if(TxCounter == count)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�������жϣ��ж�
		}


		USART_ClearITPendingBit(USART2,USART_IT_TXE);
	}

   OSIntExit(); 

}

void UsartSend_GOL_LINK(uint8_t ch)
{


	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
USART_SendData(USART2, ch); 
}

static void Send_Data_GOL_LINK(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GOL_LINK(dataToSend[i]);
}


//------------------------------------------------------GOL_LINK----------------------------------------------------
#include "ekf_ins.h"
#include "ukf_task.h"
#include "ukf_baro.h"
void Send_UKF1(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x12;//������
	data_to_send[_cnt++]=0;//������
	
	_temp = (vs16) (flow.rate);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( imu_nav.flow.speed.x*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  (vs16)(imu_nav.flow.speed.y*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16) (flow_ground_temp[0]*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16) (flow_ground_temp[1]*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	_temp = (vs16)( FLOW_POS_X*100);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16) (FLOW_POS_Y*100);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	//-------------------
	_temp = (vs16)(ALT_VEL_SONAR*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(ALT_POS_SONAR2*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ALT_VEL_BMP*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32) (ALT_POS_BMP*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
  _temp = (vs16)( ALT_VEL_BMP_UNION*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32) (ALT_POS_BMP_EKF*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	//-------------oldx_ukf flow qr fushion
	_temp = (vs16)( X_ukf_Pos[0]*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16) (X_ukf_Pos[1]*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	//spd
	_temp = (vs16)( X_ukf[1]*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16) (X_ukf[4]*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16) (X_ukf_baro[3]*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16) (X_ukf_baro[4]*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

  _temp = pi_flow.sensor.x;//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = pi_flow.sensor.y;//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = pi_flow.sensor.z;//navUkfata.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}

void Send_CAL(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x02;//������
	data_to_send[_cnt++]=0;//������
	
	_temp = (vs16)(mpu6050.Acc_Offset.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(mpu6050.Acc_Offset.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(mpu6050.Acc_Offset.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( mpu6050.Gyro_Offset.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( mpu6050.Gyro_Offset.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( mpu6050.Gyro_Offset.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( ak8975.Mag_Offset.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( ak8975.Mag_Offset.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ak8975.Mag_Offset.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp=  (vs16)( ak8975.Mag_Gain.x*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( ak8975.Mag_Gain.y*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ak8975.Mag_Gain.z*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( 0);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
  
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}  

void Send_MEMS(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x10;//������
	data_to_send[_cnt++]=0;//������

	_temp = (vs16)(imu_fushion.Acc.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(imu_fushion.Acc.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(imu_fushion.Acc.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( imu_fushion.Gyro.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( imu_fushion.Gyro.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( imu_fushion.Gyro.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( imu_fushion.Mag_Val.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( imu_fushion.Mag_Val.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( imu_fushion.Mag_Val.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)(baroAlt);//ultra_distance;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);

	
	_temp=  (vs16)( flow_matlab_data[0]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=  (vs16)( flow_matlab_data[1]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=  (vs16)( flow_matlab_data[2]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=  (vs16)( flow_matlab_data[3]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp=  (vs16)( baro_matlab_data[1]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}  

void Send_ATT(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x11;//������
	data_to_send[_cnt++]=0;//������

	_temp = (vs16)(Pitch*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(Roll*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( ref_q[0]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( ref_q[1]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q[2]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q[3]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(Pitch_mid_down*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(Roll_mid_down*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw_mid_down*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( ref_q_imd_down[0]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( ref_q_imd_down[1]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q_imd_down[2]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q_imd_down[3]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);


	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}  

void Send_BLE_DEBUG(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//������
	data_to_send[_cnt++]=0;//������
  
	if(debug_pi_flow[0])
	_temp =  1;	
	else	
	_temp =  en_ble_debug;
	data_to_send[_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[1];	
	else	
	_temp =  ax;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[2];	
	else	
	_temp =  ay;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[3];	
	else	
	_temp =  az;//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[4];	
	else	
	_temp =  gx;//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[5];	
	else	
	_temp =  gy;//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[6];	
	else	
	_temp =  gz;//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[7];	
	else	
	_temp =  hx;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[8];	
	else	
	_temp =  hy;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[9];	
	else	
	_temp =  hz;//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp=  ( pi_flow.sensor.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=  ( pi_flow.sensor.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=  ( pi_flow.sensor.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	Send_Data_GOL_LINK(data_to_send, _cnt);
}



void Send_Laser(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x13;//������
	data_to_send[_cnt++]=0;//������
	_temp = (vs16)(Laser_avoid[0]);//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
 for(i=1;i<=20;i++){
	_temp = (vs16)(Laser_avoid[i]);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 }

	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}  

void Send_GPS(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp1;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x14;//������
	data_to_send[_cnt++]=0;//������
  //origin
	_temp1 = (vs32)( gpsx.longitude*10000000);//ultra_distance;
	data_to_send[_cnt++]=BYTE3(_temp1);
	data_to_send[_cnt++]=BYTE2(_temp1);
	data_to_send[_cnt++]=BYTE1(_temp1);
	data_to_send[_cnt++]=BYTE0(_temp1);
	_temp1 = (vs32)( gpsx.latitude*10000000);//ultra_distance;
	data_to_send[_cnt++]=BYTE3(_temp1);
	data_to_send[_cnt++]=BYTE2(_temp1);
	data_to_send[_cnt++]=BYTE1(_temp1);
	data_to_send[_cnt++]=BYTE0(_temp1);
	_temp = (vs16)( gpsx.spd*100);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( gpsx.angle*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(gpsx.gpssta);// gpsx.gpssta);//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( gpsx.posslnum);//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( gpsx.svnum);//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	
	//KF
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}  
void GOL_LINK_TASK(void)//5ms
{
static u8 cnt[10];
static u8 flag[10];
	
//������ֵ
Send_MEMS();//2.5ms
	
if(cnt[1]++>=2-1){//5ms
//��̬����
	cnt[1]=0;
Send_ATT();
//goto end_gol_link;
}

if(cnt[2]++>=4-1)//10ms
{cnt[2]=0;
Send_UKF1();
//goto end_gol_link;
}

if(cnt[3]++>=10-1)//100ms
{cnt[3]=0;
Send_GPS();	
goto end_gol_link;
}


//if(cnt[4]++>=20-1)//50ms
//{cnt[4]=0;
//Send_Laser();
//goto end_gol_link;
//}



//if(cnt[5]++>60)
//{cnt[5]=0;
//Send_CAL();
//goto end_gol_link;
//}



end_gol_link:;
}

u16 nrf_uart_cnt;
void data_per_uart4(u8 sel)
{
	u8 i;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;
	vs16 _temp;
  vs32 _temp32;

switch(sel){
	case SEND_IMU_MEMS:
	cnt_reg=nrf_uart_cnt;
  SendBuff2[nrf_uart_cnt++]=0xAA;
	SendBuff2[nrf_uart_cnt++]=0xAF;
	SendBuff2[nrf_uart_cnt++]=0x10;//������
	SendBuff2[nrf_uart_cnt++]=0;//������

	_temp = (vs16)(imu_fushion.Acc.x);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(imu_fushion.Acc.y);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(imu_fushion.Acc.z);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( imu_fushion.Gyro.x);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( imu_fushion.Gyro.y);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)( imu_fushion.Gyro.z);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( imu_fushion.Mag_Val.x);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( imu_fushion.Mag_Val.y);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)( imu_fushion.Mag_Val.z);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp = ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	#if USE_M100_IMU
	_temp32 = (vs32)(m100.H*1000);//ultra_distance;
	#else
	_temp32 = (vs32)(baroAlt);//ultra_distance;
	#endif
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);

	
	_temp=  (vs16)( flow_matlab_data[0]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=  (vs16)( flow_matlab_data[1]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=  (vs16)( flow_matlab_data[2]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=  (vs16)( flow_matlab_data[3]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);

	_temp=  (vs16)( baro_matlab_data[1]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
  _temp=  m100.connect&&m100.m100_data_refresh&&(m100.GPS_STATUS>=3);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);

	SendBuff2[cnt_reg+3] = nrf_uart_cnt-cnt_reg-4;
	for( i=cnt_reg;i< nrf_uart_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[nrf_uart_cnt++] = sum;
	break;//------------
	
	case SEND_IMU_ATT:
	cnt_reg=nrf_uart_cnt;	
	SendBuff2[nrf_uart_cnt++]=0xAA;
	SendBuff2[nrf_uart_cnt++]=0xAF;
	SendBuff2[nrf_uart_cnt++]=0x11;//������
	SendBuff2[nrf_uart_cnt++]=0;//������

	_temp = (vs16)(Pitch*10);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(Roll*10);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( ref_q[0]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( ref_q[1]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q[2]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q[3]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(Pitch_mid_down*10);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(Roll_mid_down*10);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw_mid_down*10);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( ref_q_imd_down[0]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( ref_q_imd_down[1]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q_imd_down[2]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q_imd_down[3]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);

	
	SendBuff2[cnt_reg+3] = nrf_uart_cnt-cnt_reg-4;
	for( i=cnt_reg;i< nrf_uart_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[nrf_uart_cnt++] = sum;
	break;//---------
	
	case SEND_IMU_FLOW:
	cnt_reg=nrf_uart_cnt;
  SendBuff2[nrf_uart_cnt++]=0xAA;
	SendBuff2[nrf_uart_cnt++]=0xAF;
	SendBuff2[nrf_uart_cnt++]=0x12;//������
	SendBuff2[nrf_uart_cnt++]=0;//������
	
	_temp = (vs16) (flow.rate);//ukf autoquad
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
//	if(pi_flow.insert)
//	_temp = (vs16)(	pi_flow.spdx*1000);
//	else
	_temp = (vs16)( imu_nav.flow.speed.x*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
//	if(pi_flow.insert)
//	_temp = (vs16)(	pi_flow.spdy*1000);
//	else
	_temp =  (vs16)(imu_nav.flow.speed.y*1000);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16) (flow_ground_temp[0]*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16) (flow_ground_temp[1]*1000);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
//	if(pi_flow.insert)
//	_temp = (vs16)(	pi_flow.x*100);
//	else
	_temp = (vs16)( FLOW_POS_X*100);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
//	if(pi_flow.insert)
//	_temp = (vs16)(	pi_flow.y*100);
//	else
	_temp = (vs16) (FLOW_POS_Y*100);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	//-------------------
	if(pi_flow.insert&&module.sonar==0)
	_temp = (vs16)(	pi_flow.spdz*1000);
	else
	_temp = (vs16)(ALT_VEL_SONAR*1000);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	#if SONAR_USE_FLOW
	sys.sonar=1;
	#endif
	#if SENSOR_FORM_PI_FLOW_SONAR_NOT
	if(sys.sonar)
	_temp = (vs16)(ALT_POS_SONAR3*1000);//navUkfData.posE[0]*1000;
	else
	_temp= 9000;
	#else
	if(pi_flow.insert&&module.sonar==0)
	_temp = (vs16)(pi_flow.z_o*1000);	
	else if(sys.sonar)
	_temp = (vs16)(ALT_POS_SONAR3*1000);//navUkfData.posE[0]*1000;
	else
	_temp= 9000;
	#endif
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ALT_VEL_BMP_EKF*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp32 = (vs32) (ALT_POS_BMP_EKF*1000);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
  _temp =  (vs16)  (ALT_VEL_BMP_UNION*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp32 = (vs32) (ALT_POS_BMP_EKF*1000);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
	//-------------oldx_ukf flow qr fushion
	if(pi_flow.insert&&gpsx.pvt.PVT_fixtype==0&&module.flow==0)
	_temp = (vs16)(	pi_flow.x*1000);
	else
	_temp = (vs16)( X_ukf_Pos[0]*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	if(pi_flow.insert&&gpsx.pvt.PVT_fixtype==0&&module.flow==0)
	_temp = (vs16)(	pi_flow.y*1000);
	else
	_temp = (vs16) (X_ukf_Pos[1]*1000);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	//spd
	if(pi_flow.insert&&gpsx.pvt.PVT_fixtype==0&&module.flow==0)
	_temp = (vs16)(	pi_flow.spdx*1000);
	else
	_temp = (vs16)( X_ukf[1]*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	if(pi_flow.insert&&gpsx.pvt.PVT_fixtype==0&&module.flow==0)
	_temp = (vs16)(	pi_flow.spdy*1000);
	else
	_temp = (vs16) (X_ukf[4]*1000);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp = (vs16) (X_ukf_baro[3]*1000);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16) (X_ukf_baro[4]*1000);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
 	SendBuff2[cnt_reg+3] = nrf_uart_cnt-cnt_reg-4;
	for( i=cnt_reg;i< nrf_uart_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[nrf_uart_cnt++] = sum;
	break;//---------

	
	case SEND_IMU_GPS:
	cnt_reg=nrf_uart_cnt;
	SendBuff2[nrf_uart_cnt++]=0xAA;
	SendBuff2[nrf_uart_cnt++]=0xAF;
	SendBuff2[nrf_uart_cnt++]=0x14;//������
	SendBuff2[nrf_uart_cnt++]=0;//������
 
	_temp = (vs32)(local_Lon);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((local_Lon-(int)(local_Lon))*1000000000);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
	
	_temp = (vs32)(local_Lat);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((local_Lat-(int)(local_Lat))*1000000000);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);

  _temp32 = (vs32)(r1);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
	_temp32 = (vs32)(r2);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
  
	 //origin
	_temp = (gpsx.pvt.PVT_fixtype);// gpsx.gpssta);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (gpsx.pvt.PVT_numsv);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)( gpsx.pvt.PVT_longitude*10000000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
	_temp32 = (vs32)( gpsx.pvt.PVT_latitude*10000000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
	_temp = (vs16)( gpsx.pvt.PVT_speed*100);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)( gpsx.angle*10);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);

	
	SendBuff2[cnt_reg+3] = nrf_uart_cnt-cnt_reg-4;
	for( i=cnt_reg;i< nrf_uart_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[nrf_uart_cnt++] = sum;
	break;//---------
	default:break;
}
}

void GOL_LINK_TASK_DMA(void)//5ms
{
static u8 cnt[10];
static u8 flag[10];
	
//������ֵ
data_per_uart4(SEND_IMU_MEMS);
	
if(cnt[1]++>=2-1){//5ms
//��̬����
	cnt[1]=0;
data_per_uart4(SEND_IMU_ATT);
goto end_gol_link1;
}

if(cnt[2]++>=2-1)//10ms
{cnt[2]=0;
data_per_uart4(SEND_IMU_FLOW);
goto end_gol_link1;
}

if(cnt[3]++>=5-1)//100ms
{cnt[3]=0;
data_per_uart4(SEND_IMU_GPS);
 goto end_gol_link1;
}


if(cnt[4]++>=10-1)//50ms
{cnt[4]=0;
//Send_Laser();
//goto end_gol_link1;
}



if(cnt[5]++>30)
{cnt[5]=0;
//Send_CAL();
//goto end_gol_link1;
}



end_gol_link1:;
}

void clear_nrf_uart(void)
{u16 i;
nrf_uart_cnt=0;
for(i=0;i<SEND_BUF_SIZE2;i++)
SendBuff2[i]=0;

}

void Uart5_Init(u32 br_num)//-----video sonar Flow
{
	USART_InitTypeDef USART_InitStructure;
	//USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //����USART2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
	
	//����PC12��ΪUART5��Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//����PD2��ΪUART5��Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//����UART5
	//�жϱ�������
	USART_InitStructure.USART_BaudRate = br_num;       //�����ʿ���ͨ������վ����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8λ����
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //��֡��β����1��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;    //������żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Ӳ��������ʧ��
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //���͡�����ʹ��
	USART_Init(UART5, &USART_InitStructure);
	


	//ʹ��UART5�����ж�
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	//ʹ��USART5
	USART_Cmd(UART5, ENABLE); 
//	//ʹ�ܷ��ͣ�������λ���ж�
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}

}



//���ݽ���
void Data_Receive_Anl_FLOW(u8 *data_buf,u8 num)
{
	u8 i;
	u8 sum = 0;
	for(i=0;i<(num-1);i++)
	{
		sum += *(data_buf+i);
	}
	
	if(!(sum==*(data_buf+num-1)))		
	{	
		return;		//��֤sum
	}
	
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		
	{	
		return;		//��֤֡ͷ
	}
	
	//����
	if(*(data_buf+2)==0X01)
	{ module.flow_iic=1;
		//flow_rad.frame_count = (*(data_buf+4)<<8)|*(data_buf+5);
		flow_rad.integrated_x = (int16_t)((*(data_buf+6)<<8)|*(data_buf+7));
		flow_rad.integrated_y = (int16_t)((*(data_buf+8)<<8)|*(data_buf+9));
		flow_rad.integrated_xgyro = (int16_t)((*(data_buf+10)<<8)|*(data_buf+11));
		flow_rad.integrated_ygyro = (int16_t)((*(data_buf+12)<<8)|*(data_buf+13));
		flow_rad.integrated_zgyro = (int16_t)((*(data_buf+14)<<8)|*(data_buf+15));
		flow_rad.integration_time_us = (*(data_buf+16)<<24)|(*(data_buf+17)<<16)|(*(data_buf+18)<<8)|*(data_buf+19);
		//flow_rad. = (*(data_buf+20)<<24)|(*(data_buf+21)<<16)|(*(data_buf+22)<<8)|*(data_buf+23);
		flow_rad.distance = (*(data_buf+24)<<8)|*(data_buf+25);
		flow_rad.temperature = (*(data_buf+26)<<8)|*(data_buf+27);
		flow_rad.quality = *(data_buf+28);
	}

}


u8 Tx5Buffer[256];
u8 Tx5Counter=0;
u8 count5=0; 

void UART5_IRQHandler(void)
{
static u8 RxBuffer[50];
static u8 _data_len = 0,_data_cnt = 0;
static u8 state = 0;
	
	u8 com_data;
 OSIntEnter();    
  //�����ж�
		if(UART5->SR & USART_SR_ORE)//ORE�ж�
	{
		com_data = UART5->DR;
	}
	
	if( USART_GetITStatus(UART5,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);//����жϱ�־

		com_data = UART5->DR;
		
	
		if(state==0&&com_data==0xAA)
		{
			state=1;
			RxBuffer[0]=com_data;
		}
		else if(state==1&&com_data==0xAF)
		{
			state=2;
			RxBuffer[1]=com_data;
		}
		else if(state==2&&com_data<0XF1)
		{
			state=3;
			RxBuffer[2]=com_data;
		}
		else if(state==3&&com_data<50)
		{
			state = 4;
			RxBuffer[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(state==4&&_data_len>0)
		{
			_data_len--;
			RxBuffer[4+_data_cnt++]=com_data;
			if(_data_len==0)
				state = 5;
		}
		else if(state==5)
		{
			state = 0;
			RxBuffer[4+_data_cnt]=com_data;
			Data_Receive_Anl_FLOW(RxBuffer,_data_cnt+5);
		}
		else
			state = 0;
	
		flow_uart_rx_oldx(com_data,&flow,&flow_rad);//Ultra_Get(com_data);
		module.flow=1;
	}

	//���ͣ�������λ���ж�
	if( USART_GetITStatus(UART5,USART_IT_TXE ) )
	{
				
		UART5->DR = Tx5Buffer[Tx5Counter++]; //дDR����жϱ�־
          
		if(Tx5Counter == count5)
		{
			UART5->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�������жϣ��ж�
		}


		USART_ClearITPendingBit(UART5,USART_IT_TXE);
	}
     OSIntExit();    
}

void Uart5_Send(unsigned char *DataToSend ,u8 data_num)
{

while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, data_num); ;//USART1, ch); 

}



void Usart1_Init(u32 br_num)//-------GPS
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //����USART2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//����PD5��ΪUSART2��Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//����PD6��ΪUSART2��Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

//	//����PA3  WK  BLE���ƶ�
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; 
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//  GPIO_Init(GPIOA, &GPIO_InitStructure); 
//	
//	GPIO_ResetBits(GPIOA,GPIO_Pin_3);//͸��ģʽ

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = br_num;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	

	//ʹ��USART2�����ж�
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//ʹ��USART2
	USART_Cmd(USART1, ENABLE); 
//	//ʹ�ܷ��ͣ�������λ���ж�
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}
   delay_ms(100);
   Ublox_PVT_Mode();

}

float dlf_odroid=0.5;
 void Data_Receive_Anl_odroid(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//�ж�֡ͷ
  if(*(data_buf+2)==0x01)//FLOW_MINE_frame
  {
	 imu_nav.flow.speed.x_f= dlf_odroid*(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))+(1-dlf_odroid)*imu_nav.flow.speed.x_f;///10.;//UKF
	 imu_nav.flow.speed.y_f= dlf_odroid*(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))+(1-dlf_odroid)*imu_nav.flow.speed.y_f;///10.; 
	}
}
u8 TxBuffer_u1[256];
u8 TxCounter_u1=0;
u8 count_u1=0; 

u8 Rx_Buf_u1[256];	//���ڽ��ջ���
u8 RxBuffer_u1[50];
u8 RxState_u1 = 0;
u8 RxBufferNum_u1 = 0;
u8 RxBufferCnt_u1 = 0;
u8 RxLen_u1 = 0;
static u8 _data_len_u1 = 0,_data_cnt_u1 = 0;

//GPS
#define TEST_GPS 0//<----------------------------------------------
u16 USART3_RX_STA=0;
//���ڷ��ͻ����� 	
__align(8) u8 USART3_TX_BUF[USART3_MAX_SEND_LEN]; 	//���ͻ���,���USART3_MAX_SEND_LEN�ֽ�
u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//���ջ���,���USART3_MAX_RECV_LEN���ֽ�.
u8 USART1_TX_BUF[USART3_MAX_RECV_LEN]; 					//����1,���ͻ�����

nmea_msg gpsx; 		
u8  buf_GPRMC[100]={'G','P','R','M','C',','};//GPS��Ϣ
u8  buf_GPRMCt[100]={"GPRMC,144326.00,A,5107.0017737,N,11402.3291611,W,0.080,323.3,210307,0.0,E,A*20"};//GPS��Ϣ
u8  buf_GPGGA[100]={'G','P','G','G','A',','};//GPS��Ϣ
u8  buf_GPGGAt[100]={"GPGGA,134658.00,5106.9792,N,11402.3003,W,2,09,1.0,1048.47,M,-16.27,M,08,AAAA*60"};//GPS��Ϣ
u8  buf_imu_dj[20];
float angle_imu_dj[3];
u8 gps_good=0;
u16 gps_loss_cnt=0;
void IMU_DJ_ANGLE(u8 data)
{
//2.1.2 ����GPRMC
//˵�����������ʱ�䡢���ڡ���λ�����溽��͵����ٶ�
	static u8 state0,cnt_buf;
switch(state0){
			case 0:if(data=='R')
			state0=1;
			break;
			case 1:if(data=='M')
			{state0=2;cnt_buf=0;}
			else
			state0=0;
			break;
			case 2:if(data=='C')
			{state0=3;cnt_buf=0;}
			else
			state0=0;
			break;
			case 3:if(data==',')
			{state0=4;cnt_buf=0;}
			else
			state0=0;
			break;
			case 4:if(data=='*')
			{buf_GPRMC[6+cnt_buf++]=data;state0=5;}
			else if(cnt_buf>90)
			{cnt_buf=0;state0=0;}	
			else
			buf_GPRMC[6+cnt_buf++]=data;
			break;
			case 5:
				#if TEST_GPS
			NMEA_GPRMC_Analysis(&gpsx,buf_GPRMCt);	
			#else
			NMEA_GPRMC_Analysis(&gpsx,buf_GPRMC);	//GPRMC����	
			#endif
//			for(cnt_buf=6;cnt_buf<sizeof(buf_GPRMC);cnt_buf++)
//			buf_GPRMC[cnt_buf]=0;
			cnt_buf=0;state0=0;
			break;
		}
		
//$GPGGA,134658.00,5106.9792,N,11402.3003,W,2,09,1.0,1048.47,M,-16.27,M,08,AAAA*60
		static u8 state1,cnt_buf1;
switch(state1){
			case 0:if(data=='G')
			state1=1;
			break;
			case 1:if(data=='G')
			{state1=2;cnt_buf1=0;}
			else
			state1=0;
			break;
			case 2:if(data=='A')
			{state1=3;cnt_buf1=0;}
			else
			state1=0;
			break;
			case 3:if(data==',')
			{state1=4;cnt_buf1=0;}
			else
			state1=0;
			break;
			case 4:if(data=='*')
			{buf_GPRMC[6+cnt_buf1++]=data;state1=5;}
			else if(cnt_buf1>90)
			{cnt_buf1=0;state1=0;}	
			else
			buf_GPGGA[6+cnt_buf1++]=data;
			break;
			case 5:
			gps_good=1;
			gps_loss_cnt=0;
			#if TEST_GPS
			NMEA_GPGGA_Analysis(&gpsx,buf_GPGGAt);	
			#else
			NMEA_GPGGA_Analysis(&gpsx,buf_GPGGA);	//GPRMC����	
			#endif
//			for(cnt_buf=6;cnt_buf<sizeof(buf_GPRMC);cnt_buf++)
//			buf_GPRMC[cnt_buf]=0;
			cnt_buf1=0;state1=0;
			break;
		}
}

//GPS PVT
void Ublox_PVT_Mode(void)
{	 u8 TX_BUF[11],i;
			TX_BUF[0]=0xB5;
			TX_BUF[1]=0x62;
			TX_BUF[2]=0x06;
			TX_BUF[3]=0x01;
			TX_BUF[4]=0x03;
			TX_BUF[5]=0x00;
			TX_BUF[6]=0x01;
			TX_BUF[7]=0x07;
			TX_BUF[8]=0x01;
			TX_BUF[9]=0x13;
			TX_BUF[10]=0x51;
for(i=0;i<11;i++){	
while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, TX_BUF[i]); 
}
}

u8 Gps_data_get(u8 in)
{
	static u8 buf[100],state;	
	static u16 cnt;
	u16 j;
	u8 i=0,sum_a=0,sum_b=0;
	switch(state)
	{
		case 0:
			if(in==0xB5)
			{
			 for(j=0;j<100;j++)
			 buf[j]=0;
			cnt=0;buf[cnt++]=in;state++;}
		break;
	  case 1:
			if(in==0x62)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;
	  case 2:
			if(in==0x01)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;
		case 3:
			if(in==0x07)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;
		case 4:
			if(in==0x5c)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;	
		case 5:
			if(in==0x00)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;	
		case 6:
     if(cnt<100)
			 buf[cnt++]=in;
		 else 
			 {  
					for(i=2;i<98;i++)
					{
					sum_a+=buf[i];
					sum_b+=sum_a;
					}
			   if(sum_a==buf[98]&&sum_b==buf[99])//0.1~0.2delay
					{
					 module.gps=1;
					 gpsx.pvt.rx_cnt++;	
					 gpsx.pvt.rx_dt=Get_Cycle_T(GET_T_PVT); 	
					 gpsx.pvt.PVT_fixtype=buf[20+6];
					 gpsx.pvt.PVT_numsv=buf[23+6];
					 gpsx.pvt.PVT_longitude=(double)(long)((((u32)buf[27+6])<<24)|((u32)buf[26+6])<<16|((u32)buf[25+6])<<8|((u32)buf[24+6]))/10000000.;
					 gpsx.pvt.PVT_latitude=(double)(long)((((u32)buf[31+6])<<24)|((u32)buf[30+6])<<16|((u32)buf[29+6])<<8|((u32)buf[28+6]))/10000000.;
					 static float off;
					 static u8 init_flag;
           float temp;					 
					 temp=(float)(int)((((u32)buf[39+6])<<24)|((u32)buf[38+6])<<16|((u32)buf[37+6])<<8|((u32)buf[36+6]))/1000.;						 
					 if(temp>0&&init_flag==0&&gpsx.pvt.PVT_fixtype>=1&&gpsx.pvt.PVT_numsv>=4)
					 {off=temp;init_flag=1;}	 
					 gpsx.pvt.PVT_height=temp-off;
					 gpsx.pvt.PVT_Hacc=(int)((((u32)buf[43+6])<<24)|((u32)buf[42+6])<<16|((u32)buf[41+6])<<8|((u32)buf[40+6]));				 
					 gpsx.pvt.PVT_North_speed=(float)(int)((((u32)buf[51+6])<<24)|((u32)buf[50+6])<<16|((u32)buf[49+6])<<8|((u32)buf[48+6]))/1000.;
					 gpsx.pvt.PVT_East_speed=(float)(int)((((u32)buf[55+6])<<24)|((u32)buf[54+6])<<16|((u32)buf[53+6])<<8|((u32)buf[52+6]))/1000.;
					 gpsx.pvt.PVT_Down_speed=-(float)(int)((((u32)buf[59+6])<<24)|((u32)buf[58+6])<<16|((u32)buf[57+6])<<8|((u32)buf[56+6]))/1000.;
					 gpsx.pvt.PVT_speed=(float)(int)((((u32)buf[63+6])<<24)|((u32)buf[62+6])<<16|((u32)buf[61+6])<<8|((u32)buf[60+6]))/1000.;	
					}
					state=0;
			 }
    break;		
	}
}





///--
 void Data_Receive_Anl11(u8 *data_buf,u8 num)
{ static u8 flag;
	double zen,xiao;
	static float m100_h_off;
	static float m100_hr,m100_attr[3];
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//�ж�֡ͷ
  	if(*(data_buf+2)==0x01)//
  { m100.rx_dt=Get_Cycle_T(GET_T_M100); 	
		m100.loss_cnt=0;
		m100.connect=1;
	  m100.Pit=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/10.;
		m100.Rol=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/10.;
		m100.Yaw=-1*(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/10.;
		float temp;
		temp=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/1000.;
		if(temp!=0&&m100_h_off==0)
			m100_h_off=temp;
		m100.H=temp-m100_h_off;
		if(m100.H!=m100_hr||m100_attr[0]!=m100.Pit||m100_attr[1]!=m100.Rol||m100_attr[2]!=m100.Yaw)
		{m100.cnt_m100_data_refresh=0;
		 m100.m100_data_refresh=1;
		}
		m100_hr=m100.H;
		m100_attr[0]=m100.Pit;
		m100_attr[1]=m100.Rol;
		m100_attr[2]=m100.Yaw;
		
		//m100.H_Spd=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/1000.;
		zen=(*(data_buf+14)<<8)|*(data_buf+15);
		xiao=(double)((u32)(*(data_buf+16)<<24)|(*(data_buf+17)<<16)|(*(data_buf+18)<<8)|*(data_buf+19))/1000000000.;
		m100.Lat=zen+xiao;
		zen=(*(data_buf+20)<<8)|*(data_buf+21);
		xiao=(double)((u32)(*(data_buf+22)<<24)|(*(data_buf+23)<<16)|(*(data_buf+24)<<8)|*(data_buf+25))/1000000000.;
		m100.Lon=zen+xiao;
		
		m100.Bat=(float)((int16_t)(*(data_buf+26)<<8)|*(data_buf+27))/100.;
		m100.Rc_pit=(float)((int16_t)(*(data_buf+28)<<8)|*(data_buf+29));
		m100.Rc_rol=(float)((int16_t)(*(data_buf+30)<<8)|*(data_buf+31));
		m100.Rc_yaw=(float)((int16_t)(*(data_buf+32)<<8)|*(data_buf+33));
		m100.Rc_thr=(float)((int16_t)(*(data_buf+34)<<8)|*(data_buf+45));
		m100.Rc_mode=(float)((int16_t)(*(data_buf+36)<<8)|*(data_buf+37));
		m100.Rc_gear=(float)((int16_t)(*(data_buf+38)<<8)|*(data_buf+39));
		m100.STATUS=*(data_buf+40);		
		m100.GPS_STATUS=*(data_buf+41);
		m100.spd[0]=(float)((int16_t)(*(data_buf+42)<<8)|*(data_buf+43))/1000.;
		m100.spd[1]=(float)((int16_t)(*(data_buf+44)<<8)|*(data_buf+45))/1000.;
		m100.spd[2]=m100.H_Spd=(float)((int16_t)(*(data_buf+46)<<8)|*(data_buf+47))/1000.;
	}	
  
}
u8 RxBuffer11[50];
u8 RxState11 = 0;
u8 RxBufferNum11 = 0;
u8 RxBufferCnt11 = 0;
u8 RxLen11 = 0;
u16 _data_cnt11=0;
u16 _data_len11=0;
u8  buf_GNVTG[65];//GPS��Ϣ
void USART1_IRQHandler(void)//-------------------GPS
{ OSIntEnter(); 
	u8 com_data;
	static u16 cnt;
	static u8 state0,cnt_buf;
	if(USART1->SR & USART_SR_ORE)//ORE�ж�
	{
		com_data = USART1->DR;
	}

  //�����ж�
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//����жϱ�־

		com_data = USART1->DR;
		#if !USE_M100_IMU
		IMU_DJ_ANGLE(com_data);
		//ubloxCharIn(com_data);
		Gps_data_get(com_data);
		#endif
		if(cnt++>50)
			cnt=0;
		else
		USART3_RX_BUF[cnt]=com_data;

		
		if(RxState11==0&&com_data==0xAA)
		{
			RxState11=1;
			RxBuffer11[0]=com_data;
		}
		else if(RxState11==1&&com_data==0xAF)
		{
			RxState11=2;
			RxBuffer11[1]=com_data;
		}
		else if(RxState11==2&&com_data>0&&com_data<0XF1)
		{
			RxState11=3;
			RxBuffer11[2]=com_data;
		}
		else if(RxState11==3&&com_data<50)
		{
			RxState11= 4;
			RxBuffer11[3]=com_data;
			_data_len11 = com_data;
			_data_cnt11= 0;
		}
		else if(RxState11==4&&_data_len11>0)
		{
			_data_len11--;
			RxBuffer11[4+_data_cnt11++]=com_data;
			if(_data_len11==0)
				RxState11 = 5;
		}
		else if(RxState11==5)
		{
			RxState11 = 0;
			RxBuffer11[4+_data_cnt11]=com_data;
			//Data_Receive_Anl11(RxBuffer11,_data_cnt11+5);
		}
		else
			RxState11 = 0;
		
	}
	//���ͣ�������λ���ж�
	if( USART_GetITStatus(USART1,USART_IT_TXE ) )
	{
				
		USART1->DR = TxBuffer_u1[TxCounter_u1++]; //дDR����жϱ�־          
		if(TxCounter_u1 == count_u1)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�������жϣ��ж�
		}


		USART_ClearITPendingBit(USART1,USART_IT_TXE);
	}

   OSIntExit(); 

}


void Usart4_Init(u32 br_num)//-------SD_board
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //����USART2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	
	//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
	
	//����PD5��ΪUSART2��Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//����PD6��ΪUSART2��Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

	

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = br_num;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(UART4, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(UART4, ENABLE);  //ʹ�ܴ���1 
	
	USART_ClearFlag(UART4, USART_FLAG_TC);
	

	//ʹ��USART2�����ж�
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	//ʹ��USART2
	USART_Cmd(UART4, ENABLE); 
//	//ʹ�ܷ��ͣ�������λ���ж�
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}


}


u16 laser_buf[360];
void Laser_Get(u8 com_data)
{
	static u8 ultra_tmp,state;
	float temp1,temp2,temp;
	float dt;
  u16 angle;
	u16 dis;
  static u8 cnt,buf[9];
	u8 check,i;
  u16 Strength;
	
	switch(state)
	{
		case 0:
			if(com_data==0x59)
			{state=1;cnt=0;}
		break;
		case 1:
			if(com_data==0x59)
			{state=2;cnt=0;}
			else 
			 state=0;
		break	;
		case 2:
			 if(cnt<=9)
			  buf[cnt++]=com_data;
			 else
			 {state=0;
				
				 check=0x59+0x59+buf[0]+buf[1]+buf[2]+buf[3]+buf[4]+buf[5]+buf[6]+buf[7];
				 if(check==buf[8])
				 {
				
				 dis=(buf[1]<<8|buf[0]); 
				 Strength=buf[3]<<8|buf[2];
				 angle= buf[5]<<8|buf[4];
					 if(Strength>10)
					 { 
						 
						 if(angle%10==0)
						 {
						 laser_buf[angle/10]=dis;//cm					 
						 }
						
					 }
					 else  if(angle%10==0)
					 {
					 laser_buf[angle/10]=8888;//cm					 
					 }
						 
				 }
				 
			 }
			 break;
	
	}
}

M100 m100;
int debug_pi_flow[20];
struct _FLOW_PI pi_flow;
void Data_Receive_Anl4(u8 *data_buf,u8 num)
{ static u8 flag;
	double zen,xiao;
	float temp;
	static float m100_hr,m100_attr[3];
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//�ж�֡ͷ
  if(*(data_buf+2)==0x66)//PI_FLOW FUSION OUT
  { 
		module.pi_flow=1;
		pi_flow.loss_cnt=0;
		pi_flow.insert=1;
		pi_flow.x=(float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100.;
		pi_flow.y=(float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/100.;
		pi_flow.z=(float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/1000.;
		
		temp=(float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/1000.;
		if(fabs(temp)<8)
		pi_flow.spdx=temp;
		temp=(float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000.;
		if(fabs(temp)<8)
		pi_flow.spdy=temp;
		temp=(float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/1000.;
		if(fabs(temp)<8)
		pi_flow.spdz=temp;
		
		pi_flow.pit=(float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100.;
		pi_flow.rol=(float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/100.;
		pi_flow.yaw=(float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100.;

    pi_flow.connect=*(data_buf+22);
		pi_flow.check=*(data_buf+23);
		
		pi_flow.z_o=(float)((vs16)(*(data_buf+24)<<8)|*(data_buf+25))/1000.;
	}
	else  if(*(data_buf+2)==0x77)//PI_FLOW FUSION OUT
  { 
		pi_flow.sensor.spdx=(float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000.;
		pi_flow.sensor.spdy=(float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000.;
	
	}	
		else  if(*(data_buf+2)==0x88)//PI_FLOW FUSION OUT
  { 
		pi_flow.sensor.x=((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
		pi_flow.sensor.y=((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
		pi_flow.sensor.z=((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
	}	
	else  if(*(data_buf+2)==0x11)//PI_FLOW FUSION OUT
  { 
		debug_pi_flow[0]=((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
		debug_pi_flow[1]=((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
		debug_pi_flow[2]=((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
		debug_pi_flow[3]=((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
		debug_pi_flow[4]=((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
		debug_pi_flow[5]=((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
		debug_pi_flow[6]=((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
		debug_pi_flow[7]=((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
		debug_pi_flow[8]=((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
		debug_pi_flow[9]=((vs16)(*(data_buf+22)<<8)|*(data_buf+23));
		debug_pi_flow[10]=((vs16)(*(data_buf+24)<<8)|*(data_buf+25));
		debug_pi_flow[11]=((vs16)(*(data_buf+26)<<8)|*(data_buf+27));
		debug_pi_flow[12]=((vs16)(*(data_buf+28)<<8)|*(data_buf+29));
		debug_pi_flow[13]=((vs16)(*(data_buf+30)<<8)|*(data_buf+31));
		debug_pi_flow[14]=((vs16)(*(data_buf+32)<<8)|*(data_buf+33));
		debug_pi_flow[15]=((vs16)(*(data_buf+34)<<8)|*(data_buf+35));
		debug_pi_flow[16]=((vs16)(*(data_buf+36)<<8)|*(data_buf+37));
		debug_pi_flow[17]=((vs16)(*(data_buf+38)<<8)|*(data_buf+39));
		debug_pi_flow[18]=((vs16)(*(data_buf+40)<<8)|*(data_buf+41));
		debug_pi_flow[19]=((vs16)(*(data_buf+42)<<8)|*(data_buf+43));
	}
 	
}
 



u8 RxBuffer4_test[25],RxBuffer4_test_cnt;
u8 RxBuffer4[50];
u8 RxState4 = 0;
u8 RxBufferNum4 = 0;
u8 RxBufferCnt4 = 0;
u8 RxLen4 = 0;
static u8 _data_len4 = 0,_data_cnt4 = 0;
void UART4_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(UART4->SR & USART_SR_ORE)//ORE�ж�
	{
		com_data = UART4->DR;
	}

  //�����ж�
	if( USART_GetITStatus(UART4,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);//����жϱ�־

		com_data = UART4->DR;
		RxBuffer4_test[RxBuffer4_test_cnt++]=com_data;
		if(RxBuffer4_test_cnt>24)
			RxBuffer4_test_cnt=0;
		 Laser_Get(com_data);
		if(RxState4==0&&com_data==0xAA)
		{
			RxState4=1;
			RxBuffer4[0]=com_data;
		}
		else if(RxState4==1&&com_data==0xAF)
		{
			RxState4=2;
			RxBuffer4[1]=com_data;
		}
		else if(RxState4==2&&com_data>0&&com_data<0XF1)
		{
			RxState4=3;
			RxBuffer4[2]=com_data;
		}
		else if(RxState4==3&&com_data<50)
		{
			RxState4 = 4;
			RxBuffer4[3]=com_data;
			_data_len4 = com_data;
			_data_cnt4 = 0;
		}
		else if(RxState4==4&&_data_len4>0)
		{
			_data_len4--;
			RxBuffer4[4+_data_cnt4++]=com_data;
			if(_data_len4==0)
				RxState4 = 5;
		}
		else if(RxState4==5)
		{
			RxState4 = 0;
			RxBuffer4[4+_data_cnt4]=com_data;
			Data_Receive_Anl4(RxBuffer4,_data_cnt4+5);
			Data_Receive_Anl11(RxBuffer4,_data_cnt4+5);
		}
		else
			RxState4 = 0;
	
	}
	
   OSIntExit(); 
}

void UsartSend_M100(uint8_t ch)
{

while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
USART_SendData(UART4, ch);  
}


void UsartSend_SD(uint8_t ch)
{

while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
USART_SendData(UART4, ch); ;//USART1, ch); 
}

static void Send_Data_SD(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_SD(dataToSend[i]);
}


void Laser_start(void)
{

UsartSend_SD( 0x42); //USART1, ch); 
UsartSend_SD( 0x57); //USART1, ch); 
UsartSend_SD( 0x02); //USART1, ch); 
UsartSend_SD( 0x00); //USART1, ch); 
UsartSend_SD( 0x00); //USART1, ch); UsartSend_SD(UART4, 0x42); ;//USART1, ch); 
UsartSend_SD( 0x00); //USART1, ch); 
UsartSend_SD( 0x02); //USART1, ch); 
UsartSend_SD( 0x02); //USART1, ch); 
}

void Laser_stop(void)
{

UsartSend_SD( 0x42); //USART1, ch); 
UsartSend_SD( 0x57); //USART1, ch); 
UsartSend_SD( 0x02); //USART1, ch); 
UsartSend_SD( 0x00); //USART1, ch); 
UsartSend_SD( 0x00); //USART1, ch); UsartSend_SD(UART4, 0x42); ;//USART1, ch); 
UsartSend_SD( 0x00); //USART1, ch); 
UsartSend_SD( 0x01); //USART1, ch); 
UsartSend_SD( 0x02); //USART1, ch); 
}



void Send_TO_FLOW_PI(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x66;//������
	data_to_send[_cnt++]=0;//������
	_temp = (int)(k_flow_devide*1000.);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(flow_module_offset_x*1000.);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(flow_module_offset_y*1000.);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(flow_module_set_yaw*10.);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	if(sys.sonar&&ALT_POS_SONAR3>0)
	_temp = (int)(ALT_POS_SONAR3*1000);	
	else
	_temp = (int)(0*1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (int)((Yaw)*10);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

u8 en_px4_mapper;
void Send_TO_FLOW_NAV_GPS(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x99;//������
	data_to_send[_cnt++]=0;//������
	_temp = (vs16)((Pitch)*10);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)((Roll)*10);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)((Yaw)*10);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs32)(GPS_J_F);//latitude;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((GPS_J_F-(int)(GPS_J_F))*1000000000);//latitude;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	
	_temp = (vs32)(GPS_W_F);//latitude;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((GPS_W_F-(int)(GPS_W_F))*1000000000);//latitude;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	
  _temp = (vs16) ((X_kf_baro[0])*10);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
  //_temp = en_px4_mapper;
  _temp = en_px4_mapper&&module.gps&&gpsx.pvt.PVT_numsv>=1&&gpsx.pvt.PVT_fixtype>=3;
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

void UART2_Put_Char(unsigned char DataToSend)
{

while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, DataToSend); 

}

void UART2_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
 	unsigned int temp=0xaF+9;
	char ctemp;
	UART2_Put_Char(0xa5);
	UART2_Put_Char(0x5a);
	UART2_Put_Char(14+8);
	UART2_Put_Char(0xA2);

	if(ax<0)ax=32768-ax;
	ctemp=ax>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ax;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(ay<0)ay=32768-ay;
	ctemp=ay>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ay;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(az<0)az=32768-az;
	ctemp=az>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=az;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(gx<0)gx=32768-gx;
	ctemp=gx>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gx;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(gy<0)gy=32768-gy;
	ctemp=gy>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gy;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
//-------------------------
	if(gz<0)gz=32768-gz;
	ctemp=gz>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gz;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(hx<0)hx=32768-hx;
	ctemp=hx>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hx;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(hy<0)hy=32768-hy;
	ctemp=hy>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hy;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(hz<0)hz=32768-hz;
	ctemp=hz>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hz;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	UART2_Put_Char(temp%256);
	UART2_Put_Char(0xaa);
}


void UART2_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
 	unsigned int temp=0xaF+2+2;
	char ctemp;
	UART2_Put_Char(0xa5);
	UART2_Put_Char(0x5a);
	UART2_Put_Char(14+4);
	UART2_Put_Char(0xA1);

	if(yaw<0)yaw=32768-yaw;
	ctemp=yaw>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=yaw;							
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(pitch<0)pitch=32768-pitch;
	ctemp=pitch>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=pitch;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
								 
	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=roll;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(alt<0)alt=32768-alt;
	ctemp=alt>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;				
	ctemp=alt;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(tempr<0)tempr=32768-tempr;
	ctemp=tempr>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=tempr;
	UART2_Put_Char(ctemp);	   
	temp+=ctemp;

	if(press<0)press=32768-press;
	ctemp=press>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=press;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	ctemp=IMUpersec>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=IMUpersec;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	UART2_Put_Char(temp%256);
	UART2_Put_Char(0xaa);
}



//------------------------------------------------------GOL_LINK_SD----------------------------------------------------
void Send_IMU_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x05;//������
	data_to_send[_cnt++]=0;//������
	_temp = Roll*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Pitch*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Yaw*10	;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[1]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[2]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[3]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	

data_to_send[3] = _cnt-4;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

void Send_ATT_PID_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//������
	data_to_send[_cnt++]=0;//������

	
	
	
	
	
	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

void Send_FLOW_PID_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x02;//������
	data_to_send[_cnt++]=0;//������

	
	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}


void Send_ALT_PID_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x03;//������
	data_to_send[_cnt++]=0;//������

	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}


void Send_MODE_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x04;//������
	data_to_send[_cnt++]=0;//������
	_temp = mode.en_sd_save;
	data_to_send[_cnt++]=BYTE0(_temp);

	
	
  data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

void Send_FLOW_USE_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x06;//������
	data_to_send[_cnt++]=0;//������
	_temp = imu_nav.flow.speed.east ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = imu_nav.flow.speed.west;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  imu_nav.flow.speed.x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = imu_nav.flow.speed.y ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = imu_nav.flow.position.east	;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = imu_nav.flow.position.west	;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}


void Send_SLAM_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x07;//������
	data_to_send[_cnt++]=0;//������
	_temp = slam.dis[2];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = slam.dis[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = slam.dis[3];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = slam.dis[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = slam.dis[4];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	

	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}


void Send_GPS_SD(void)
{u8 i;	u8 sum = 0;u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x08;//������
	data_to_send[_cnt++]=0;//������
	_temp32 =  imu_nav.gps.J;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.W;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp =imu_nav.gps.gps_mode;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =imu_nav.gps.star_num;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 =  imu_nav.gps.X_O;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.Y_O;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.X_UKF;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.Y_UKF;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	
	

	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

//void SD_LINK_TASK(void)
//{
//static u8 cnt[4];
//static u8 flag;
//if(cnt[0]++>1)
//{cnt[0]=0;
//Send_ATT_PID_SD();
//}
//if(cnt[1]++>0)
//{cnt[1]=0;
////	if(flag)
////	{flag=0;
//	Send_IMU_SD();
////}
////	else{flag=1;
////	Send_MODE_SD();
////	}
//}
//if(cnt[2]++>2)
//{cnt[2]=0;
//	//Send_FLOW_PID_SD();
//	Send_FLOW_USE_SD();
//}
//if(cnt[3]++>2)
//{cnt[3]=0;
//	//Send_ALT_PID_SD();
//	Send_GPS_SD();//Send_SLAM_SD();
//}

//}

/*
#define SEND_IMU 0
#define SEND_FLOW 1
#define SEND_GPS 2
#define SEND_ALT 3
*/
void SD_LINK_TASK2(u8 sel)
{
static u8 cnt[4];
static u8 flag;
	
	switch(sel)
	{
		case SEND_IMU:
				Send_IMU_SD();
		break;
		case SEND_FLOW:
				Send_FLOW_USE_SD();
		break;
		case SEND_GPS:
				Send_GPS_SD();
		break;
		case SEND_ALT:
				Send_ALT_PID_SD();
		break;
	}

 Send_MODE_SD();
}

void Usart3_Init(u32 br_num)//-------SONAR
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //����USART2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);	
	
	//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	
	//����PD5��ΪUSART2��Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
	//����PD6��ΪUSART2��Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

	
   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = br_num;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���1 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	

	//ʹ��USART2�����ж�
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//ʹ��USART2
	USART_Cmd(USART3, ENABLE); 
//	//ʹ�ܷ��ͣ�������λ���ж�
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}


}
#include "sonar_avoid.h"
float rate_gps_board;
 void Data_Receive_Anl3(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//�ж�֡ͷ
	if(*(data_buf+2)==0x01)//SONAR
  {
//	//rate_gps_board=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));///10.;
//	sonar_avoid[0]=((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));
//	sonar_avoid[1]=((int16_t)(*(data_buf+6)<<8)|*(data_buf+7));
//	sonar_avoid[2]=((int16_t)(*(data_buf+8)<<8)|*(data_buf+9));
//	sonar_avoid[3]=((int16_t)(*(data_buf+10)<<8)|*(data_buf+11));
//	sonar_avoid[4]=((int16_t)(*(data_buf+12)<<8)|*(data_buf+13));
//	sonar_avoid[5]=((int16_t)(*(data_buf+14)<<8)|*(data_buf+15));
//	sonar_avoid[6]=((int16_t)(*(data_buf+16)<<8)|*(data_buf+17));
//	sonar_avoid[7]=((int16_t)(*(data_buf+18)<<8)|*(data_buf+19));

	sys.avoid=1;
	}	
	else if(*(data_buf+2)==0x02)//SLAM_frame
  {
  imu_nav.gps.Y_UKF=((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));
	imu_nav.gps.X_UKF=((int16_t)(*(data_buf+6)<<8)|*(data_buf+7));
	imu_nav.gps.Y_O=  ((int16_t)(*(data_buf+8)<<8)|*(data_buf+9));
	imu_nav.gps.X_O=  ((int16_t)(*(data_buf+10)<<8)|*(data_buf+11));
	imu_nav.gps.J=  ((int16_t)(*(data_buf+12)<<8)|*(data_buf+13));
	imu_nav.gps.W=  ((int16_t)(*(data_buf+14)<<8)|*(data_buf+15));	
	sys.gps=1;	
	}			
}
 



u8 Rx_Buf3[256];	//���ڽ��ջ���
u8 RxBuffer3[50];
u8 RxState3 = 0;
u8 RxBufferNum3 = 0;
u8 RxBufferCnt3 = 0;
u8 RxLen3 = 0;
static u8 _data_len3 = 0,_data_cnt3 = 0;
void USART3_IRQHandler(void)
{  OSIntEnter();  
	u8 com_data;
	
	if(USART3->SR & USART_SR_ORE)//ORE�ж�
	{
		com_data = USART3->DR;
	}

  //�����ж�
	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//����жϱ�־

		com_data = USART3->DR;
		 Ultra_Get( com_data);
				if(RxState3==0&&com_data==0xAA)
		{
			RxState3=1;
			RxBuffer3[0]=com_data;
		}
		else if(RxState3==1&&com_data==0xAF)
		{
			RxState3=2;
			RxBuffer3[1]=com_data;
		}
		else if(RxState3==2&&com_data>0&&com_data<0XF1)
		{
			RxState3=3;
			RxBuffer3[2]=com_data;
		}
		else if(RxState3==3&&com_data<50)
		{
			RxState3 = 4;
			RxBuffer3[3]=com_data;
			_data_len3 = com_data;
			_data_cnt3 = 0;
		}
		else if(RxState3==4&&_data_len3>0)
		{
			_data_len3--;
			RxBuffer3[4+_data_cnt3++]=com_data;
			if(_data_len3==0)
				RxState3 = 5;
		}
		else if(RxState3==5)
		{
			RxState3 = 0;
			RxBuffer3[4+_data_cnt3]=com_data;
			Data_Receive_Anl3(RxBuffer3,_data_cnt3+5);
		}
		else
			RxState3 = 0;
	
	}
	//���ͣ�������λ���ж�
	if( USART_GetITStatus(USART3,USART_IT_TXE ) )
	{
				
		USART3->DR = TxBuffer[TxCounter++]; //дDR����жϱ�־          
		if(TxCounter == count)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�������жϣ��ж�
		}

		USART_ClearITPendingBit(USART3,USART_IT_TXE);
	}
 OSIntExit();        
}

void UsartSend_GPS(uint8_t ch)
{

while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, ch); ;//USART1, ch); 
}

static void Send_Data_GPS(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GPS(dataToSend[i]);
}

#define MAX_Yun_Angle 60
float Angle_Yun[2]={0,0};

void Send_IMU_TO_GPS(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//������
	data_to_send[_cnt++]=0;//������
	_temp =  mode.en_sonar_avoid;//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	

	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GPS(data_to_send, _cnt);
}

void CPU_LINK_TASK(void)
{
static u8 cnt[4];
static u8 flag;
if(cnt[0]++>0)
{cnt[0]=0;
 Send_IMU_TO_GPS();
}
}



//-------------------------NAV_BOARD_LINK

//------------------------------------------------------GOL_LINK----------------------------------------------------

void UsartSend_GOL_LINK_NAV(uint8_t ch)
{


	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, ch); 
}

static void Send_Data_GOL_LINK_NAV(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GOL_LINK_NAV(dataToSend[i]);
}

void Send_IMU_NAV(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//������
	data_to_send[_cnt++]=0;//������
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = mpu6050.Acc.x;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = mpu6050.Acc.y;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = mpu6050.Acc.z;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);


	
	_temp = q_nav[0]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = q_nav[1]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[2]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[3]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK_NAV(data_to_send, _cnt);
}


u8 SendBuff1[SEND_BUF_SIZE1];	//�������ݻ�����
void data_per_uart1(int16_t ax,int16_t ay, int16_t az, int16_t gx,int16_t  gy, int16_t gz,int16_t hx, int16_t hy, int16_t hz,
	int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
u16 i=0; 	
unsigned int temp=0xaF+9;

char ctemp;	
	
SendBuff1[i++]=0xa5;
SendBuff1[i++]=0x5a;
SendBuff1[i++]=14+8;
SendBuff1[i++]=0xA2;

if(ax<0)ax=32768-ax;
ctemp=ax>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=ax;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(ay<0)ay=32768-ay;
ctemp=ay>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=ay;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(az<0)az=32768-az;
ctemp=az>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=az;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(gx<0)gx=32768-gx;
ctemp=gx>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=gx;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(gy<0)gy=32768-gy;
ctemp=gy>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=gy;
SendBuff1[i++]=ctemp;
temp+=ctemp;
//-------------------------
if(gz<0)gz=32768-gz;
ctemp=gz>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=gz;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(hx<0)hx=32768-hx;
ctemp=hx>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=hx;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(hy<0)hy=32768-hy;
ctemp=hy>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=hy;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(hz<0)hz=32768-hz;
ctemp=hz>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=hz;
SendBuff1[i++]=ctemp;
temp+=ctemp;

SendBuff1[i++]=temp%256;
SendBuff1[i++]=(0xaa);
//
 temp=0xaF+2+2;
SendBuff1[i++]=(0xa5);
SendBuff1[i++]=(0x5a);
SendBuff1[i++]=(14+4);
SendBuff1[i++]=(0xA1);


if(yaw<0)yaw=32768-yaw;
ctemp=yaw>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=yaw;							
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(pitch<0)pitch=32768-pitch;
ctemp=pitch>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=pitch;
SendBuff1[i++]=ctemp;
temp+=ctemp;
							 
if(roll<0)roll=32768-roll;
ctemp=roll>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=roll;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(alt<0)alt=32768-alt;
ctemp=alt>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;				
ctemp=alt;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(tempr<0)tempr=32768-tempr;
ctemp=tempr>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=tempr;
SendBuff1[i++]=ctemp;	   
temp+=ctemp;

if(press<0)press=32768-press;
ctemp=press>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=press;
SendBuff1[i++]=ctemp;
temp+=ctemp;

ctemp=IMUpersec>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=IMUpersec;
SendBuff1[i++]=ctemp;
temp+=ctemp;

SendBuff1[i++]=(temp%256);
SendBuff1[i++]=(0xaa);
}


u8 SendBuff2[SEND_BUF_SIZE2];	//�������ݻ�����
void data_per_uart2(void)
{
	u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
  SendBuff2[_cnt++]=0xAA;
	SendBuff2[_cnt++]=0xAF;
	SendBuff2[_cnt++]=0x01;//������
	SendBuff2[_cnt++]=0;//������
	
	_temp = ultra_distance;//
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);

	
	
	
	SendBuff2[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[_cnt++] = sum;
}

u8 en_ble_debug=0;
void GOL_LINK_BLE_DEBUG(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
 u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	SendBuff2[_cnt++]=0xAA;
	SendBuff2[_cnt++]=0xAF;
	SendBuff2[_cnt++]=0x12;//������
	SendBuff2[_cnt++]=0;//������

	_temp =  en_ble_debug;
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  ax;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  ay;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  az;//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  gx;//navUkfData.posE[0]*1000;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  gy;//navUkfData.posE[0]*1000;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  gz;//navUkfData.posE[0]*1000;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  hx;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  hy;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  hz;//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	
	
	SendBuff2[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[_cnt++] = sum;
}

u8 SendBuff3[SEND_BUF_SIZE3];	//�������ݻ�����
void data_per_uart3(void)
{
	u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
  SendBuff3[_cnt++]=0xAA;
	SendBuff3[_cnt++]=0xAF;
	SendBuff3[_cnt++]=0x01;//������
	SendBuff3[_cnt++]=0;//������
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	
	_temp = mpu6050.Acc.x;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
 	_temp = mpu6050.Acc.y;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp = mpu6050.Acc.z;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);

	_temp = q_nav[0]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
 	_temp = q_nav[1]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[2]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[3]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	
	SendBuff3[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += SendBuff3[i];
	SendBuff3[_cnt++] = sum;
}


u8 SendBuff4[SEND_BUF_SIZE4];	//�������ݻ�����

 FLOW flow;//flow ����
 FLOW_RAD flow_rad;//����������


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

