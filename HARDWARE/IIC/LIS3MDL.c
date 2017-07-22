#include "LIS3MDL.h"
#include "iic_soft.h"
#include "iic.h"
#include "iic_imu1.h"
#include "iic2.h"
#include "hml5833l.h"
#include "filter.h"
#include  <math.h> 
#include "cycle_cal_oldx.h"
u8 IMU1_Fast;
LIS3MDL_S lis3mdl;
Cal_Cycle_OLDX acc_lsq;
#define LIS3MDL_SA1_HIGH_ADDRESS  0x1E
#define LIS3MDL_SA1_LOW_ADDRESS   0x1C
#define LIS3MDL_ADDRESS1  (LIS3MDL_SA1_HIGH_ADDRESS << 1)
#define LIS3MDL_ADDRESS2  (LIS3MDL_SA1_LOW_ADDRESS << 1)
#define TEST_REG_ERROR -1

#define LIS3MDL_WHO_ID  0x3D

#define LIS3MDL_IIC_ID LIS3MDL_ADDRESS1

#define DS33_SA0_HIGH_ADDRESS 0x6b
#define DS33_SA0_LOW_ADDRESS  0x6a
// Reads the 3 mag channels and stores them in vector m
#define DS33_ADDRESS1  (DS33_SA0_HIGH_ADDRESS << 1)
#define DS33_ADDRESS2  (DS33_SA0_LOW_ADDRESS << 1)
#define DS33_IIC_ID DS33_ADDRESS1


#define LPSSA0_HIGH_ADDRESS 0x5d<<	1
#define LPSSA0_LOW_ADDRESS  0x5c<<	1
#define LPS_IIC_ID LPSSA0_HIGH_ADDRESS


uint8_t id[3] ;
void LIS3MDL_enableDefault(void)
{
 //------------init hml
   // 0x70 = 0b01110000
    // OM = 11 (ultra-high-performance mode for X and Y); DO = 100 (10 Hz ODR)
    IIC_IMU1writeByte(LIS3MDL_IIC_ID,CTRL_REG1, 0x74);//20hz

    // 0x00 = 0b00000000
    // FS = 00 (+/- 4 gauss full scale)
    IIC_IMU1writeByte(LIS3MDL_IIC_ID,CTRL_REG2, 0x60);

    // 0x00 = 0b00000000
    // MD = 00 (continuous-conversion mode)
    IIC_IMU1writeByte(LIS3MDL_IIC_ID,CTRL_REG3, 0x00);

    // 0x0C = 0b00001100
    // OMZ = 11 (ultra-high-performance mode for Z)
    IIC_IMU1writeByte(LIS3MDL_IIC_ID,CTRL_REG4, 0x0C);
  
    id[0] = I2C_IMU1_ReadOneByte(LIS3MDL_IIC_ID,WHO_AM_I);
		
//---------------init acc & gro		
		// 0x80 = 0b10000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (+/-2 g full scale)
    IIC_IMU1writeByte(DS33_IIC_ID,CTRL1_XL, 0x4f);//50hz 8g  0x88);

    // Gyro

    // 0x80 = 0b010000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (245 dps)
    IIC_IMU1writeByte(DS33_IIC_ID,CTRL2_G, 0x4c);//2000 50hz

    // Common

    // 0x04 = 0b00000100
    // IF_INC = 1 (automatically increment register address)
    IIC_IMU1writeByte(DS33_IIC_ID,CTRL3_C, 0x04);
		
		
		id[1] = I2C_IMU1_ReadOneByte(DS33_IIC_ID,WHO_AM_I);
//------------------------init bmp
		// 0xB0 = 0b10110000
    // PD = 1 (active mode);  ODR = 011 (12.5 Hz pressure & temperature output data rate)
		IIC_IMU1writeByte(LPS_IIC_ID,CTRL_REG1, 0xB4);//0xB4);
		id[2] = I2C_IMU1_ReadOneByte(LPS_IIC_ID,WHO_AM_I);
}


void LIS3MDL_read(u8 fast)
{ u8 buffer[6];

  IMU1_Fast=fast;
  IIC_IMU1readBytes(LIS3MDL_IIC_ID, OUT_X_H, 1,buffer);
	IIC_IMU1readBytes(LIS3MDL_IIC_ID, OUT_X_L, 1,buffer+1);
	IIC_IMU1readBytes(LIS3MDL_IIC_ID, OUT_Y_H, 1,buffer+2);
	IIC_IMU1readBytes(LIS3MDL_IIC_ID, OUT_Y_L, 1,buffer+3);
	IIC_IMU1readBytes(LIS3MDL_IIC_ID, OUT_Z_H, 1,buffer+4);
	IIC_IMU1readBytes(LIS3MDL_IIC_ID, OUT_Z_L, 1,buffer+5);
  // combine high and low bytes
  lis3mdl.Mag_Adc.x = (int16_t)(buffer[0] << 8 | buffer[1]);
  lis3mdl.Mag_Adc.y = (int16_t)(buffer[2] << 8 | buffer[3]);
  lis3mdl.Mag_Adc.z = (int16_t)(buffer[4] << 8 | buffer[5]);
}



// Reads the 3 accelerometer channels and stores them in vector a
void LSM6_readAcc(u8 fast)
{u8 buffer[6];
	IMU1_Fast=fast;
  IIC_IMU1readBytes(DS33_IIC_ID, OUTX_H_XL, 1, buffer);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTX_L_XL, 1,buffer+1);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTY_H_XL, 1,buffer+2);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTY_L_XL, 1,buffer+3);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTZ_H_XL, 1,buffer+4);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTZ_L_XL, 1,buffer+5);
  // combine high and low bytes
  lis3mdl.Acc_I16.x = (int16_t)(buffer[0] << 8 | buffer[1]);
  lis3mdl.Acc_I16.y = (int16_t)(buffer[2] << 8 | buffer[3]);
  lis3mdl.Acc_I16.z = (int16_t)(buffer[4] << 8 | buffer[5]);
	
}

// Reads the 3 gyro channels and stores them in vector g
void LSM6_readGyro(u8 fast)
{u8 buffer[6];
	IMU1_Fast=fast;
  IIC_IMU1readBytes(DS33_IIC_ID, OUTX_H_G, 1,buffer);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTX_L_G, 1,buffer+1);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTY_H_G, 1,buffer+2);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTY_L_G, 1,buffer+3);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTZ_H_G, 1,buffer+4);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTZ_L_G, 1,buffer+5);
  // combine high and low bytes
  lis3mdl.Gyro_I16.x = (int16_t)(buffer[0] << 8 | buffer[1]);
  lis3mdl.Gyro_I16.y = (int16_t)(buffer[2] << 8 | buffer[3]);
  lis3mdl.Gyro_I16.z = (int16_t)(buffer[4] << 8 | buffer[5]);
	
}


// reads pressure and returns raw 24-bit sensor output
int32_t LPS_readPressureRaw(void)
{
  u8 pxl ;
  u8 pl ;
  u8 ph ;
  IIC_IMU1readBytes(LPS_IIC_ID, PRESS_OUT_XL, 1,&pxl);
	IIC_IMU1readBytes(LPS_IIC_ID, PRESS_OUT_L, 1,&pl);
	IIC_IMU1readBytes(LPS_IIC_ID, PRESS_OUT_H, 1,&ph);
  // combine bytes
  return (int32_t)(int8_t)ph << 16 | (uint16_t)pl << 8 | pxl;
}

// reads temperature and returns raw 16-bit sensor output
int16_t LPS_readTemperatureRaw(void)
{
  u8 tl ;
  u8 th ;
  IIC_IMU1readBytes(LPS_IIC_ID, TEMP_OUT_L_BMP, 1,&tl);
	IIC_IMU1readBytes(LPS_IIC_ID, TEMP_OUT_H_BMP, 1,&th);
  // combine bytes
  return (int16_t)(th << 8 | tl);
}
// reads temperature in degrees C
float LPS_readTemperatureC(void)
{
  return 42.5 + (float)LPS_readTemperatureRaw() / 480;
}

// reads temperature in degrees F
float LPS_readTemperatureF(void)
{
  return 108.5 + (float)LPS_readTemperatureRaw() / 480 * 1.8;
}


// converts pressure in mbar to altitude in meters, using 1976 US
// Standard Atmosphere model (note that this formula only applies to a
// height of 11 km, or about 36000 ft)
//  If altimeter setting (QNH, barometric pressure adjusted to sea
//  level) is given, this function returns an indicated altitude
//  compensated for actual regional pressure; otherwise, it returns
//  the pressure altitude above the standard pressure level of 1013.25
//  mbar or 29.9213 inHg
float LPS_pressureToAltitudeMeters(float pressure_mbar, float altimeter_setting_mbar)
{
  return (1 - pow(pressure_mbar / altimeter_setting_mbar, 0.190263)) * 44330.8;
}

// converts pressure in inHg to altitude in feet; see notes above
float LPS_pressureToAltitudeFeet(float pressure_inHg, float altimeter_setting_inHg)
{
  return (1 - pow(pressure_inHg / altimeter_setting_inHg, 0.190263)) * 145442;
}



// Reads the 3 gyro channels and stores them in vector g
void LP_readbmp(u8 fast)
{ static float pre_off;
	float pre_temp;
  static u8 init;
	static u8 cnt;
	IMU1_Fast=fast;
	if(cnt++>20){cnt=0;
	lis3mdl.Tem_bmp=LPS_readTemperatureC();}
	pre_temp=(float)LPS_readPressureRaw()/4096;
	if(!init&&pre_temp!=0)
	{pre_off=pre_temp;init=1;}
	else{
	lis3mdl.Pressure=pre_temp;
	lis3mdl.Alt=LPS_pressureToAltitudeMeters(pre_temp,pre_off);
	}
}
#define CALIBRATING_MAG_CYCLES              1000  //У׼ʱ�����20s
void LIS_CalOffset_Mag(void)
{ static xyz_f_t	Mag_Reg;
	static xyz_f_t	MagMAX = { -100 , -100 , -100 }, MagMIN = { 100 , 100 , 100 }, MagSum;
	static uint16_t cnt_m=0;
	static u8 hml_cal_temp=0;
  static u8 state_cal_hml;
	static u8 init;
	
	if(lis3mdl.Mag_CALIBRATED)
	{	
		if(!init){init=1;
		cycle_init_oldx(&hml_lsq)	;
		MagMAX.x=MagMAX.y=MagMAX.z=-100;MagMIN.x=MagMIN.y=MagMIN.z=100;		
		}
		

		if(ABS(lis3mdl.Mag_Adc.x)<1500&&ABS(lis3mdl.Mag_Adc.y)<1500&&ABS(lis3mdl.Mag_Adc.z)<1500)
		{ if(hml_lsq.size<350&&(fabs(Mag_Reg.x-lis3mdl.Mag_Adc.x)>25||fabs(Mag_Reg.y-lis3mdl.Mag_Adc.y)>25||fabs(Mag_Reg.z-lis3mdl.Mag_Adc.z)>25))
			cycle_data_add_oldx(&hml_lsq, (float)lis3mdl.Mag_Adc.x/1000.,(float)lis3mdl.Mag_Adc.y/1000.,(float)lis3mdl.Mag_Adc.z/1000.);
			
			MagMAX.x = MAX(lis3mdl.Mag_Adc.x, MagMAX.x);
			MagMAX.y = MAX(lis3mdl.Mag_Adc.y, MagMAX.y);
			MagMAX.z = MAX(lis3mdl.Mag_Adc.z, MagMAX.z);
			
			MagMIN.x = MIN(lis3mdl.Mag_Adc.x, MagMIN.x);
			MagMIN.y = MIN(lis3mdl.Mag_Adc.y, MagMIN.y);
			MagMIN.z = MIN(lis3mdl.Mag_Adc.z, MagMIN.z);		
			
			if(cnt_m >= CALIBRATING_MAG_CYCLES*3)
			{ float sphere_x,sphere_y,sphere_z,sphere_r;
				init=0;
				cycle_cal_oldx(&hml_lsq, 666,0.001,  &sphere_x, &sphere_y, &sphere_z, &sphere_r);
				if(fabs(sphere_r)>0){
				lis3mdl.Mag_Offset_c.x=(hml_lsq.Off[0]*1000);
				lis3mdl.Mag_Offset_c.y=(hml_lsq.Off[1]*1000);
				lis3mdl.Mag_Offset_c.z=(hml_lsq.Off[2]*1000);
				lis3mdl.Mag_Gain_c.x =  (hml_lsq.Gain[0]);
				lis3mdl.Mag_Gain_c.y =  (hml_lsq.Gain[1]);
				lis3mdl.Mag_Gain_c.z =  (hml_lsq.Gain[2]);	
				}
				lis3mdl.Mag_Offset.x = (int16_t)((MagMAX.x + MagMIN.x) * 0.5f);
				lis3mdl.Mag_Offset.y = (int16_t)((MagMAX.y + MagMIN.y) * 0.5f);
				lis3mdl.Mag_Offset.z = (int16_t)((MagMAX.z + MagMIN.z) * 0.5f);
	      
				MagSum.x = MagMAX.x - MagMIN.x;
				MagSum.y = MagMAX.y - MagMIN.y;
				MagSum.z = MagMAX.z - MagMIN.z;

				float temp_max=MagSum.x ;
				if( MagSum.y>temp_max)
					temp_max=MagSum.y;
			  if( MagSum.z>temp_max)
					temp_max=MagSum.z;
				
				lis3mdl.Mag_Gain.x =  temp_max/MagSum.x ;
				lis3mdl.Mag_Gain.y =  temp_max/MagSum.y ;
				lis3mdl.Mag_Gain.z =  temp_max/MagSum.z ;
				#if USE_CYCLE_HML_CAL
				if(lis3mdl.Mag_Gain_c.x<1.5&&lis3mdl.Mag_Gain_c.y<1.5&&lis3mdl.Mag_Gain_c.z<1.5&&
					fabs(lis3mdl.Mag_Offset_c.x)<1500&&fabs(lis3mdl.Mag_Offset_c.y)<1500&&fabs(lis3mdl.Mag_Offset_c.z)<1500){
				lis3mdl.Mag_Gain.x =  lis3mdl.Mag_Gain_c.x;
				lis3mdl.Mag_Gain.y =  lis3mdl.Mag_Gain_c.y;
				lis3mdl.Mag_Gain.z =  lis3mdl.Mag_Gain_c.z;
				lis3mdl.Mag_Offset.x =  lis3mdl.Mag_Offset_c.x;
				lis3mdl.Mag_Offset.y =  lis3mdl.Mag_Offset_c.y;
				lis3mdl.Mag_Offset.z =  lis3mdl.Mag_Offset_c.z;
					}
				#endif
			  WRITE_PARM();
				cnt_m = 0;
				lis3mdl.Mag_CALIBRATED = 0;
			}
			Mag_Reg.x=lis3mdl.Mag_Adc.x;
			Mag_Reg.y=lis3mdl.Mag_Adc.y;
			Mag_Reg.z=lis3mdl.Mag_Adc.z;
			
		}

		cnt_m++;
		
	}
	else
	{

	}
}
#define OFFSET_AV_NUM_ACC 18
#define OFFSET_AV_NUM 50
s32 sum_temp[7]= {0,0,0,0,0,0,0};
float sum_temp_att[2]={0};
s32 sum_temp_3d[7]= {0,0,0,0,0,0,0};
u16 acc_sum_cnt = 0,acc_sum_cnt_3d=0,acc_smple_cnt_3d=0,gyro_sum_cnt = 0;
void LIS_Data_Offset(void)
{static u8 acc_cal_temp=0,gro_cal_temp=0;
static u8 state_cal_acc,state_cal_gro;
static u8 init;
	
	if(lis3mdl.Acc_CALIBRATE == 1)
	{
    acc_sum_cnt++;
		sum_temp[A_X] += lis3mdl.Acc_I16.x;
		sum_temp[A_Y] += lis3mdl.Acc_I16.y;
		sum_temp[A_Z] += lis3mdl.Acc_I16.z - 65536/16;   // +-8G
		sum_temp[TEM] += lis3mdl.Tempreature;

    if( acc_sum_cnt >= OFFSET_AV_NUM )
		{
			lis3mdl.Acc_Offset.x = sum_temp[A_X]/OFFSET_AV_NUM;
			lis3mdl.Acc_Offset.y = sum_temp[A_Y]/OFFSET_AV_NUM;
			lis3mdl.Acc_Offset.z = sum_temp[A_Z]/OFFSET_AV_NUM;
			lis3mdl.Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
			acc_sum_cnt =0;
			lis3mdl.Acc_CALIBRATE = 0;
			sum_temp[A_X] = sum_temp[A_Y] = sum_temp[A_Z] = sum_temp[TEM] = 0;
		}	
	}

// 3d cal
		static xyz_f_t ACC_Reg;
		static u8 acc_3d_step_reg,acc_3d_step_imu;
		float sphere_x,sphere_y,sphere_z,sphere_r;
	
	  if(acc_3d_step==6)
		  acc_3d_step_imu=6;
    else if(acc_3d_step_imu!=6)
			acc_3d_step_imu=acc_3d_step;
		switch(acc_3d_step_imu)
			{ 
			case 0:
				acc_smple_cnt_3d=acc_sum_cnt_3d=sum_temp_3d[A_X] = sum_temp_3d[A_Y] = sum_temp_3d[A_Z] = sum_temp_3d[TEM] = 0;
				cycle_init_oldx(&acc_lsq);
			break;
			default:
			if(acc_lsq.size<360&&(fabs(ACC_Reg.x-lis3mdl.Acc_I16.x)>0||fabs(ACC_Reg.y-lis3mdl.Acc_I16.y)>0||fabs(ACC_Reg.z-lis3mdl.Acc_I16.z)>0))
			{
			if(acc_3d_step_imu>acc_3d_step_reg)
			acc_smple_cnt_3d=acc_sum_cnt_3d=sum_temp_3d[A_X] = sum_temp_3d[A_Y] = sum_temp_3d[A_Z] = sum_temp_3d[TEM] = 0;		
			acc_sum_cnt_3d++;
      sum_temp_3d[A_X] += lis3mdl.Acc_I16.x;
      sum_temp_3d[A_Y] += lis3mdl.Acc_I16.y;
      sum_temp_3d[A_Z] += lis3mdl.Acc_I16.z;   
				if(acc_sum_cnt_3d>OFFSET_AV_NUM_ACC){
					if(acc_smple_cnt_3d<12){
					acc_smple_cnt_3d++;	
					xyz_f_t data;	
					data.x = sum_temp_3d[A_X]/OFFSET_AV_NUM_ACC;
					data.y = sum_temp_3d[A_Y]/OFFSET_AV_NUM_ACC;
					data.z = sum_temp_3d[A_Z]/OFFSET_AV_NUM_ACC;	
					acc_sum_cnt_3d=sum_temp_3d[A_X] = sum_temp_3d[A_Y] = sum_temp_3d[A_Z] = sum_temp_3d[TEM] = 0;	
					cycle_data_add_oldx(&acc_lsq, (float)data.x/1000.,(float)data.y/1000.,(float)data.z/1000.);}
					else if(acc_3d_step_imu==6){
					acc_3d_step_imu=0;	
					cycle_cal_oldx(&acc_lsq, 666,0.001,  &sphere_x, &sphere_y, &sphere_z, &sphere_r);	
					lis3mdl.Off_3d.x=(acc_lsq.Off[0]*1000);
					lis3mdl.Off_3d.y=(acc_lsq.Off[1]*1000);
					lis3mdl.Off_3d.z=(acc_lsq.Off[2]*1000);
					lis3mdl.Gain_3d.x =  (acc_lsq.Gain[0]);
					lis3mdl.Gain_3d.y =  (acc_lsq.Gain[1]);
					lis3mdl.Gain_3d.z =  (acc_lsq.Gain[2]);	
          WRITE_PARM();						
					}		 		
				} 
					acc_3d_step_reg=acc_3d_step_imu;	
			}
			break;
		}
		ACC_Reg.x=lis3mdl.Acc_I16.x;
	  ACC_Reg.y=lis3mdl.Acc_I16.y;
		ACC_Reg.z=lis3mdl.Acc_I16.z;

//
	if(lis3mdl.Gyro_CALIBRATE)
	{
		gyro_sum_cnt++;
		sum_temp[G_X] += lis3mdl.Gyro_I16.x;
		sum_temp[G_Y] += lis3mdl.Gyro_I16.y;
		sum_temp[G_Z] += lis3mdl.Gyro_I16.z;
		sum_temp[TEM] += lis3mdl.Tempreature;

    if( gyro_sum_cnt >= OFFSET_AV_NUM )
		{
			lis3mdl.Gyro_Offset.x = (float)sum_temp[G_X]/OFFSET_AV_NUM;
			lis3mdl.Gyro_Offset.y = (float)sum_temp[G_Y]/OFFSET_AV_NUM;
			lis3mdl.Gyro_Offset.z = (float)sum_temp[G_Z]/OFFSET_AV_NUM;
			lis3mdl.Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
			gyro_sum_cnt =0;
			if(lis3mdl.Gyro_CALIBRATE == 1)
					WRITE_PARM();
			lis3mdl.Gyro_CALIBRATE = 0;
			sum_temp[G_X] = sum_temp[G_Y] = sum_temp[G_Z] = sum_temp[TEM] = 0;
		}
	}
}


static void Transform(float itx,float ity,float itz,float *it_x,float *it_y,float *it_z)
{
	*it_x = itx;
	*it_y = ity;
	*it_z = itz;

}

/********************************************/
static float Data_conversion(float *AccBuffer,float *MagBuffer)
{
  unsigned char i;
	float HeadingValue = 0.0f;
	float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch = 0.0f;
  float fTiltedX,fTiltedY = 0.0f;
	float fcosf=0;
	      for(i=0;i<3;i++)
        AccBuffer[i] /= 100.0f;
      
      fNormAcc = sqrt((AccBuffer[0]*AccBuffer[0])+(AccBuffer[1]*AccBuffer[1])+(AccBuffer[2]*AccBuffer[2]));
      
      fSinRoll = AccBuffer[1]/fNormAcc;
      fCosRoll = sqrt(1.0-(fSinRoll * fSinRoll));
      fSinPitch = AccBuffer[0]/fNormAcc;
      fCosPitch = sqrt(1.0-(fSinPitch * fSinPitch));

      
      fTiltedX = MagBuffer[0]*fCosPitch + MagBuffer[2]*fSinPitch;
      fTiltedY = MagBuffer[0]*fSinRoll*fSinPitch + MagBuffer[1]*fCosRoll - MagBuffer[2]*fSinRoll*fCosPitch;
			
      fcosf=fTiltedX /sqrt(fTiltedX*fTiltedX+fTiltedY*fTiltedY);
	
			if(fTiltedY>0)
			  HeadingValue = (float)(acos(fcosf)*180/PI);
			else
				HeadingValue =360-(float)(acos(fcosf)*180/PI);
			
      HeadingValue = (float) ((atan2f((float)fTiltedY,(float)fTiltedX))*180)/PI;
      HeadingValue+=11;//�شŵı����͵���ı������11�����ҡ�
			if(HeadingValue>360)
				HeadingValue=HeadingValue-360;
   
	    return HeadingValue ;
}

#define  IIR_ORDER     4      //ʹ��IIR�˲����Ľ���
static double b_IIR[IIR_ORDER+1] ={ 0.0004  ,  0.0017  ,  0.0025  ,  0.0017 ,   0.0004};  //ϵ��b
static double a_IIR[IIR_ORDER+1] ={ 1.0000   ,-3.1806   , 3.8612  , -2.1122  ,  0.4383};//ϵ��a
static double InPut_IIR[3][IIR_ORDER+1] = {0};
static double OutPut_IIR[3][IIR_ORDER+1] = {0};

static double b_IIR_gro[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //ϵ��b
static double a_IIR_gro[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//ϵ��a
static double InPut_IIR_gro[3][IIR_ORDER+1] = {0};
static double OutPut_IIR_gro[3][IIR_ORDER+1] = {0};

static float lis3mdl_tmp[7],mpu_fil_tmp[7];
static s16 FILT_BUF[7][(FILTER_NUM + 1)];
static uint8_t filter_cnt = 0,filter_cnt_old = 0;
void LIS_Data_Prepare(float T)
{	
	u8 i;
	s32 FILT_TMP[7] = {0,0,0,0,0,0,0};
  float Gyro_tmp[3];
  LIS_Data_Offset();
	LIS_CalOffset_Mag();
	Gyro_tmp[0] = lis3mdl.Gyro_I16.x ;//
  Gyro_tmp[1] = lis3mdl.Gyro_I16.y ;//
	Gyro_tmp[2] = lis3mdl.Gyro_I16.z ;//


	lis3mdl.TEM_LPF += 2 *3.14f *T *(lis3mdl.Tempreature - lis3mdl.TEM_LPF);
	lis3mdl.Ftempreature = lis3mdl.TEM_LPF/340.0f + 36.5f;

//======================================================================
	if( ++filter_cnt > FILTER_NUM )	
	{
		filter_cnt = 0;
		filter_cnt_old = 1;
	}
	else
	{
		filter_cnt_old = (filter_cnt == FILTER_NUM)? 0 : (filter_cnt + 1);
	}
//10 170 4056
	/* �ó�У׼������� */
#define EN_ODR_MINE 0	
#if EN_ODR_MINE
	lis3mdl_tmp[A_X] = IIR_I_Filter(lis3mdl.Acc_I16.x - lis3mdl.Acc_Offset.x, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	lis3mdl_tmp[A_Y] = IIR_I_Filter(lis3mdl.Acc_I16.y - lis3mdl.Acc_Offset.y, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	lis3mdl_tmp[A_Z] = IIR_I_Filter(lis3mdl.Acc_I16.z - lis3mdl.Acc_Offset.z, InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	
	lis3mdl_tmp[G_X] = IIR_I_Filter(Gyro_tmp[0] - lis3mdl.Gyro_Offset.x , InPut_IIR_gro[0], OutPut_IIR_gro[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);;//
	lis3mdl_tmp[G_Y] = IIR_I_Filter(Gyro_tmp[1] - lis3mdl.Gyro_Offset.y , InPut_IIR_gro[1], OutPut_IIR_gro[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);;//
	lis3mdl_tmp[G_Z] = IIR_I_Filter(Gyro_tmp[2] - lis3mdl.Gyro_Offset.z , InPut_IIR_gro[2], OutPut_IIR_gro[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);;//
#else
		if((fabs(lis3mdl.Off_3d.x)>10||fabs(lis3mdl.Off_3d.y)>10||fabs(lis3mdl.Off_3d.z)>10)
			&&(fabs(lis3mdl.Off_3d.x)<600&&fabs(lis3mdl.Off_3d.y)<600&&fabs(lis3mdl.Off_3d.z)<600))
		lis3mdl.Cali_3d=1;
	if(lis3mdl.Cali_3d){
	lis3mdl_tmp[A_X] = (lis3mdl.Acc_I16.x - lis3mdl.Off_3d.x)*lis3mdl.Gain_3d.x;// - lis3mdl.Acc_Offset.x*en_off_3d_off;
	lis3mdl_tmp[A_Y] = (lis3mdl.Acc_I16.y - lis3mdl.Off_3d.y)*lis3mdl.Gain_3d.y;// - lis3mdl.Acc_Offset.y*en_off_3d_off;
	lis3mdl_tmp[A_Z] = (lis3mdl.Acc_I16.z - lis3mdl.Off_3d.z)*lis3mdl.Gain_3d.z;// - lis3mdl.Acc_Offset.z*en_off_3d_off;
	}
   else{			
  lis3mdl_tmp[A_X] = (lis3mdl.Acc_I16.x - lis3mdl.Acc_Offset.x);//, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	lis3mdl_tmp[A_Y] = (lis3mdl.Acc_I16.y - lis3mdl.Acc_Offset.y);//, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	lis3mdl_tmp[A_Z] = (lis3mdl.Acc_I16.z - lis3mdl.Acc_Offset.z);//, InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	 }
	lis3mdl_tmp[G_X] = (Gyro_tmp[0] - lis3mdl.Gyro_Offset.x );//, InPut_IIR_gro[0], OutPut_IIR_gro[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);;//
	lis3mdl_tmp[G_Y] = (Gyro_tmp[1] - lis3mdl.Gyro_Offset.y );//, InPut_IIR_gro[1], OutPut_IIR_gro[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);;//
	lis3mdl_tmp[G_Z] = (Gyro_tmp[2] - lis3mdl.Gyro_Offset.z );//, InPut_IIR_gro[2], OutPut_IIR_gro[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);;//
#endif	
	

	/* �����˲������������� */
	FILT_BUF[A_X][filter_cnt] = lis3mdl_tmp[A_X];
	FILT_BUF[A_Y][filter_cnt] = lis3mdl_tmp[A_Y];
	FILT_BUF[A_Z][filter_cnt] = lis3mdl_tmp[A_Z];
	FILT_BUF[G_X][filter_cnt] = lis3mdl_tmp[G_X]; 
	FILT_BUF[G_Y][filter_cnt] = lis3mdl_tmp[G_Y];
	FILT_BUF[G_Z][filter_cnt] = lis3mdl_tmp[G_Z];

	for(i=0;i<FILTER_NUM;i++)
	{
		FILT_TMP[A_X] += FILT_BUF[A_X][i];
		FILT_TMP[A_Y] += FILT_BUF[A_Y][i];
		FILT_TMP[A_Z] += FILT_BUF[A_Z][i];
		FILT_TMP[G_X] += FILT_BUF[G_X][i];
		FILT_TMP[G_Y] += FILT_BUF[G_Y][i];
		FILT_TMP[G_Z] += FILT_BUF[G_Z][i];
	}

mpu_fil_tmp[A_X] = (float)( FILT_TMP[A_X] )/(float)FILTER_NUM;
mpu_fil_tmp[A_Y] = (float)( FILT_TMP[A_Y] )/(float)FILTER_NUM;
mpu_fil_tmp[A_Z] = (float)( FILT_TMP[A_Z] )/(float)FILTER_NUM;
mpu_fil_tmp[G_X] = (float)( FILT_TMP[G_X] )/(float)FILTER_NUM;
mpu_fil_tmp[G_Y] = (float)( FILT_TMP[G_Y] )/(float)FILTER_NUM;
mpu_fil_tmp[G_Z] = (float)( FILT_TMP[G_Z] )/(float)FILTER_NUM;

	
float AccBuffer[3],MagBuffer[3];
	AccBuffer[0]=lis3mdl.Acc_I16.x;
	AccBuffer[1]=lis3mdl.Acc_I16.y;
	AccBuffer[2]=lis3mdl.Acc_I16.z;
	MagBuffer[0]=lis3mdl.Mag_Val.x;
	MagBuffer[1]=lis3mdl.Mag_Val.y;
	MagBuffer[2]=lis3mdl.Mag_Val.z;
	
	
	
	/*����ת��*/
	Transform(mpu_fil_tmp[A_X],mpu_fil_tmp[A_Y],mpu_fil_tmp[A_Z],&lis3mdl.Acc.x,&lis3mdl.Acc.y,&lis3mdl.Acc.z);
	Transform(mpu_fil_tmp[G_X],mpu_fil_tmp[G_Y],mpu_fil_tmp[G_Z],&lis3mdl.Gyro.x,&lis3mdl.Gyro.y,&lis3mdl.Gyro.z);

	lis3mdl.Gyro_deg.x = lis3mdl.Gyro.x *TO_ANGLE;
	lis3mdl.Gyro_deg.y = lis3mdl.Gyro.y *TO_ANGLE;
	lis3mdl.Gyro_deg.z = lis3mdl.Gyro.z *TO_ANGLE;
	
	
	lis3mdl.Acc_t.x=lis3mdl.Acc.y;
	lis3mdl.Acc_t.y=-lis3mdl.Acc.x;
  lis3mdl.Acc_t.z=lis3mdl.Acc.z;
	
	lis3mdl.Gyro_t.x=lis3mdl.Gyro.y;
	lis3mdl.Gyro_t.y=-lis3mdl.Gyro.x;
  lis3mdl.Gyro_t.z=lis3mdl.Gyro.z;
		
	lis3mdl.Gyro_deg_t.x = lis3mdl.Gyro_t.x *TO_ANGLE;
	lis3mdl.Gyro_deg_t.y = lis3mdl.Gyro_t.y *TO_ANGLE;
	lis3mdl.Gyro_deg_t.z = lis3mdl.Gyro_t.z *TO_ANGLE;

  lis3mdl.Mag_Val.x = (lis3mdl.Mag_Adc.x - lis3mdl.Mag_Offset.x);//*lis3mdl.Mag_Gain.x ;
	lis3mdl.Mag_Val.y = (lis3mdl.Mag_Adc.y - lis3mdl.Mag_Offset.y);//*lis3mdl.Mag_Gain.y ;
	lis3mdl.Mag_Val.z = (lis3mdl.Mag_Adc.z - lis3mdl.Mag_Offset.z);//*lis3mdl.Mag_Gain.z ;
	
	lis3mdl.Mag_Val_t.x=firstOrderFilter(lis3mdl.Mag_Val.y,&firstOrderFilters[HML_LOWPASS_X],T);
	lis3mdl.Mag_Val_t.y=firstOrderFilter(-lis3mdl.Mag_Val.x,&firstOrderFilters[HML_LOWPASS_Y],T);
	lis3mdl.Mag_Val_t.z=firstOrderFilter(lis3mdl.Mag_Val.z,&firstOrderFilters[HML_LOWPASS_Z],T);
	
	lis3mdl.yaw=Data_conversion(AccBuffer,MagBuffer);
	static u8 state[3];
	switch (state[0]){
		case 0:
		if(imu_fushion.Acc_CALIBRATE)
	{lis3mdl.Acc_CALIBRATE=mpu6050.Acc_CALIBRATE=1;
	 state[0]=1;}
	  break;
		case 1:
			if(lis3mdl.Acc_CALIBRATE==0&&mpu6050.Acc_CALIBRATE==0)
	    { state[0]=0; imu_fushion.Acc_CALIBRATE=0;}
			break;
  }
	switch (state[1]){
		case 0:
		if(imu_fushion.Gyro_CALIBRATE)
	{lis3mdl.Gyro_CALIBRATE=mpu6050.Gyro_CALIBRATE=1;
	 state[1]=1;}
	  break;
		case 1:
			if(lis3mdl.Gyro_CALIBRATE==0&&mpu6050.Gyro_CALIBRATE==0)
	    { state[1]=0; imu_fushion.Gyro_CALIBRATE=0;}
			break;
  }
	switch (state[2]){
		case 0:
		if(imu_fushion.Mag_CALIBRATED)
	{lis3mdl.Mag_CALIBRATED=ak8975.Mag_CALIBRATED=1;
	 state[2]=1;}
	  break;
		case 1:
			if(lis3mdl.Mag_CALIBRATED==0&&ak8975.Mag_CALIBRATED==0)
	    { state[2]=0; imu_fushion.Mag_CALIBRATED=0;}
			break;
  }
}
