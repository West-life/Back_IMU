
#include "imu.h"
#include "include.h"
#include "hml5833l.h"
#include "my_math.h"
#include "filter.h"
#include "kf_oldx_yaw.h"
float Kp =0.625f;//2.25f;//0.6f   ;             	// proportional gain governs rate of convergence to accelerometer/magnetometer
float Ki =0.001f    ;            	// 0.001  integral gain governs rate of convergence of gyroscope biases

#define IMU_INTEGRAL_LIM  ( 2.0f *ANGLE_TO_RADIAN )
#define NORM_ACC_LPF_HZ 10  		//(Hz)
#define REF_ERR_LPF_HZ  1				//(Hz)

float q_nav[4];
xyz_f_t reference_v;
ref_t 	ref;
float reference_vr[3];
//xyz_f_t Gravity_Vec;  				//解算的重力向量
	
float Roll,Pitch,Yaw;    				//姿态角
float Roll_mid_down,Pitch_mid_down,Yaw_mid_down;    				//姿态角
float ref_q[4] = {1,0,0,0};
float norm_acc,norm_q;
float norm_acc_lpf;
xyz_f_t mag_sim_3d;
extern u8 fly_ready;

int test_flag[3]={1,-1,1};
xyz_f_t mag_sim_3d,acc_3d_hg,acc_ng,acc_ng_offset;
//use
float mag_norm ,mag_norm_xyz, yaw_mag_view[5];
//------------------KF  parameter------------------
float gh_yaw=0.15;
float ga_yaw=0.1;//<---use
float gw_yaw=0.1;
float yaw_kf;
double P_kf_yaw[4]={1,0,0,1}; 
double X_kf_yaw[2]={0,0};
float k_kf_z=1.1;//1.428;
u8 yaw_cross;
float yaw_qr_off;
u8 dis_angle_lock;
u8 yaw_cal_by_qr;
float yaw_qr_off_local;
void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw) 
{		
	float ref_err_lpf_hz;
	static float yaw_correct;
	float mag_norm_tmp;
	static xyz_f_t mag_tmp;
	static float yaw_mag;
	static u16 cnt;
	static u8 init;
	if(cnt++>256&&init==0)
	{
	init=1;
	ref.err.x=ref.err.y=ref.err.z=0;
	ref.err_Int.x=ref.err_Int.y=ref.err_Int.z=0;
	ref.err_lpf.x=ref.err_lpf.y=ref.err_lpf.z=0;
	ref.err_tmp.x=ref.err_tmp.y=ref.err_tmp.z=0;
	ref.g.x=ref.g.y=ref.g.z=0;
	}else{
	X_kf_yaw[0]=yaw_mag_view[4];
	X_kf_yaw[1]=0;
	}
	
	mag_norm_tmp = 20 *(6.28f *half_T);	
	
	mag_norm_xyz = my_sqrt(imu_fushion.Mag_Val.x * imu_fushion.Mag_Val.x + imu_fushion.Mag_Val.y * imu_fushion.Mag_Val.y + imu_fushion.Mag_Val.z * imu_fushion.Mag_Val.z);
	if(mag_norm_xyz==0)mag_norm_xyz=0.0001;
		if( mag_norm_xyz != 0)
	{
		mag_tmp.x += mag_norm_tmp *( (float)imu_fushion.Mag_Val.x /( mag_norm_xyz ) - mag_tmp.x);
		mag_tmp.y += mag_norm_tmp *( (float)imu_fushion.Mag_Val.y /( mag_norm_xyz ) - mag_tmp.y);	
		mag_tmp.z += mag_norm_tmp *( (float)imu_fushion.Mag_Val.z /( mag_norm_xyz ) - mag_tmp.z);	
	}

	simple_3d_trans(&reference_v,&mag_tmp,&mag_sim_3d);
	
	mag_norm = my_sqrt(mag_sim_3d.x * mag_sim_3d.x + mag_sim_3d.y *mag_sim_3d.y);
	if(mag_norm==0)mag_norm=0.0001;
	if( mag_sim_3d.x != 0 && mag_sim_3d.y != 0 && mag_sim_3d.z != 0 && mag_norm != 0)
	{
		yaw_mag_view[1] = fast_atan2( ( mag_sim_3d.y/mag_norm ) , ( mag_sim_3d.x/mag_norm) ) *57.3f;
		
	}
		float calMagY,calMagX,magTmp2[3],euler[3];
		magTmp2[0]=test_flag[0]*imu_fushion.Mag_Val.x;
		magTmp2[1]=test_flag[1]*imu_fushion.Mag_Val.y;
		magTmp2[2]=test_flag[2]*imu_fushion.Mag_Val.z;
		euler[1]=Pitch/RAD_DEG  ;
		euler[0]=Roll/RAD_DEG   ;
	
		calMagY = magTmp2[2] * sin(euler[0]) - magTmp2[1] * cos(euler[0]); //倾斜补偿磁力计的Y轴分量
		calMagX = magTmp2[0] * cos(euler[1]) + magTmp2[1] * sin(euler[1]) * sin(euler[0]) + magTmp2[2] * sin(euler[1]) * cos(euler[0]); //倾斜补偿磁力计的X轴分量

		yaw_mag_view[0] = fast_atan2(calMagY, calMagX) * RAD_DEG; //计算Yaw (-PI < Roll < PI) 并将弧度转化成角度
	  yaw_mag_view[3]=yaw_mag_view[0]/2+yaw_mag_view[1]/2;
  	yaw_mag=yaw_mag_view[1] ;
	
		magTmp2[0]=imu_fushion.Mag_Val.x;
		magTmp2[1]=imu_fushion.Mag_Val.y;
		magTmp2[2]=imu_fushion.Mag_Val.z;
		euler[0]=Pitch_mid_down/RAD_DEG  ;
		euler[1]=Roll_mid_down/RAD_DEG  ;
    calMagY = magTmp2[0] * cos(euler[1]) + magTmp2[1] * sin(euler[1])* sin(euler[0])+magTmp2[2] * sin(euler[1]) * cos(euler[0]); 
    calMagX = magTmp2[1] * cos(euler[0]) + magTmp2[2] * sin(euler[0]);
		float tempy;
		if( dis_angle_lock||(fabs(Roll_mid_down)<12 && fabs(Pitch_mid_down)<12))
		tempy=To_180_degrees(fast_atan2(calMagX,calMagY)* RAD_DEG );
		else
		tempy=X_kf_yaw[0];
	  
		if(qr.check)
			yaw_cal_by_qr=1;
	
		static u8 init_check_qr;
		if(yaw_cal_by_qr){
		if(!init_check_qr&&qr.check)
		{init_check_qr=1;
			
		X_kf_yaw[0]=qr.yaw;
		yaw_qr_off_local=	To_180_degrees(fast_atan2(calMagX,calMagY)* RAD_DEG -qr.yaw);
		}
					
		if(fabs(Roll_mid_down)<5 && fabs(Pitch_mid_down)<5&&qr.check)	
		yaw_qr_off_local=	To_180_degrees(fast_atan2(calMagX,calMagY)* RAD_DEG -qr.yaw);//yaw_qr_off_local=	To_180_degrees(fast_atan2(calMagX,calMagY)* RAD_DEG );
		if(qr.check)	
		tempy=qr.yaw;	
		else{
			if( dis_angle_lock||(fabs(Roll_mid_down)<12 && fabs(Pitch_mid_down)<12))
			tempy=To_180_degrees(fast_atan2(calMagX,calMagY)* RAD_DEG -yaw_qr_off_local);
			else
			tempy=X_kf_yaw[0];
	  }
		}
		yaw_mag_view[4]=Moving_Median(14,5,tempy);	
		
		double Z_yaw[2]={ yaw_mag_view[4] , 0 };
		if(yaw_mag_view[4]*X_kf_yaw[0]<0&&!(fabs(yaw_mag_view[4])<90))
		{Z_yaw[0]=X_kf_yaw[0];yaw_cross=1;}
		else
		yaw_cross=0;
		
		kf_oldx_yaw( X_kf_yaw,  P_kf_yaw,  Z_yaw,  -gz*k_kf_z, gh_yaw,  ga_yaw,  gw_yaw,  half_T*2);
	
	//=============================================================================
	// 计算等效重力向量//十分重要
	if(mode.en_imu_ekf==0){
	reference_vr[0]=reference_v.x = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
	reference_vr[1]=reference_v.y = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
	reference_vr[2]=reference_v.z = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);}//ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3]

	//这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
	//根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
	//所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。       
	//=============================================================================
	  acc_ng.x = 10 *TO_M_S2 *(ax - 4096*reference_v.x) - acc_ng_offset.x;
		acc_ng.y = 10 *TO_M_S2 *(ay - 4096*reference_v.y) - acc_ng_offset.y;
		acc_ng.z = 10 *TO_M_S2 *(az - 4096*reference_v.z) - acc_ng_offset.z;
		
		acc_3d_hg.z = acc_ng.x *reference_v.x + acc_ng.y *reference_v.y + acc_ng.z *reference_v.z;

	// 计算加速度向量的模
	norm_acc = my_sqrt(ax*ax + ay*ay + az*az);   
	norm_acc_lpf +=  NORM_ACC_LPF_HZ *(6.28f *half_T) *(norm_acc - norm_acc_lpf);  //10hz *3.14 * 2*0.001
  yaw_mag=yaw_mag_view[4];
  	if(norm_acc==0)norm_acc=0.0001;
	if(ABS(ax)<4400 && ABS(ay)<4400 && ABS(az)<4400 )
	{	
		//把加计的三维向量转成单位向量。
		ax = ax / norm_acc;//4096.0f;
		ay = ay / norm_acc;//4096.0f;
		az = az / norm_acc;//4096.0f; 
		
		if( 3800 < norm_acc && norm_acc < 4400 )
		{
			/* 叉乘得到误差 */
			ref.err_tmp.x = ay*reference_v.z - az*reference_v.y;
			ref.err_tmp.y = az*reference_v.x - ax*reference_v.z;
	    //ref.err_tmp.z = ax*reference_v.y - ay*reference_v.x;
			
			/* 误差低通 */
			ref_err_lpf_hz = REF_ERR_LPF_HZ *(6.28f *half_T);
			ref.err_lpf.x += ref_err_lpf_hz *( ref.err_tmp.x  - ref.err_lpf.x );
			ref.err_lpf.y += ref_err_lpf_hz *( ref.err_tmp.y  - ref.err_lpf.y );
	//			 ref.err_lpf.z += ref_err_lpf_hz *( ref.err_tmp.z  - ref.err_lpf.z );
			
			ref.err.x = ref.err_lpf.x;//
			ref.err.y = ref.err_lpf.y;//
//				ref.err.z = ref.err_lpf.z ;
		}
	}
	else
	{
		ref.err.x = 0; 
		ref.err.y = 0  ;
//		ref.err.z = 0 ;
	}
	/* 误差积分 */
	ref.err_Int.x += ref.err.x *Ki *2 *half_T ;
	ref.err_Int.y += ref.err.y *Ki *2 *half_T ;
	ref.err_Int.z += ref.err.z *Ki *2 *half_T ;
	
	/* 积分限幅 */
	ref.err_Int.x = LIMIT(ref.err_Int.x, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.y = LIMIT(ref.err_Int.y, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.z = LIMIT(ref.err_Int.z, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	static u16 init_mag_cnt;
	if( reference_v.z > 0.0f )
	{
		if(( fly_ready||(fabs(Pitch)>10)||(fabs(Roll)>10))&&init_mag_cnt++>400  )
		{ init_mag_cnt=401;
	//	yaw_correct = Kp *0.2f *To_180_degrees(yaw_mag - YAW_R);
			yaw_correct = Kp *0.1f *LIMIT( my_deathzoom( To_180_degrees(yaw_mag - Yaw), 10),-20,20 );
			//已经解锁，只需要低速纠正。
		}
		else
		{
			yaw_correct = Kp *1.5f *To_180_degrees(yaw_mag - Yaw);
			//没有解锁，视作开机时刻，快速纠正
		}
	}

	ref.g.x = (gx - reference_v.x *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.x + ref.err_Int.x) ) ;     //IN RADIAN
	ref.g.y = (gy - reference_v.y *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.y + ref.err_Int.y) ) ;		  //IN RADIAN
	ref.g.z = (gz - reference_v.z *yaw_correct) *ANGLE_TO_RADIAN;
	
	/* 用叉积误差来做PI修正陀螺零偏 */
	// integrate quaternion rate and normalise
	ref_q[0] = ref_q[0] +(-ref_q[1]*ref.g.x - ref_q[2]*ref.g.y - ref_q[3]*ref.g.z)*half_T;
	ref_q[1] = ref_q[1] + (ref_q[0]*ref.g.x + ref_q[2]*ref.g.z - ref_q[3]*ref.g.y)*half_T;
	ref_q[2] = ref_q[2] + (ref_q[0]*ref.g.y - ref_q[1]*ref.g.z + ref_q[3]*ref.g.x)*half_T;
	ref_q[3] = ref_q[3] + (ref_q[0]*ref.g.z + ref_q[1]*ref.g.y - ref_q[2]*ref.g.x)*half_T;  

	/* 四元数规一化 normalise quaternion */
	norm_q = my_sqrt(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3]);
	if(norm_q==0)norm_q=1;
	q_nav[0]=ref_q[0] = ref_q[0] / norm_q;
	q_nav[1]=ref_q[1] = ref_q[1] / norm_q;
	q_nav[2]=ref_q[2] = ref_q[2] / norm_q;
	q_nav[3]=ref_q[3] = ref_q[3] / norm_q;
	
	*rol = fast_atan2(2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]),1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2])) *57.3f;
	*pit = asin(2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2])) *57.3f;
	*yaw = fast_atan2(2*(-ref_q[1]*ref_q[2] - ref_q[0]*ref_q[3]), 2*(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1]) - 1) *57.3f  ;// 
	//*yaw =X_kf_yaw[0];// yaw_mag;
}




//------------------------------------------????----------------------------------------------------
// Variable definitions
float Mag_P,Mag_R,Mag_Y;
float exAcc    = 0.0f,    eyAcc = 0.0f,    ezAcc = 0.0f; // accel error
float exAccInt = 0.0f, eyAccInt = 0.0f, ezAccInt = 0.0f; // accel integral error

float exMag    = 0.0f, eyMag    = 0.0f, ezMag    = 0.0f; // mag error
float exMagInt = 0.0f, eyMagInt = 0.0f, ezMagInt = 0.0f; // mag integral error

float kpAcc, kiAcc;

float q0_m1 = 1.0f, q1_m1 = 0.0f, q2_m1 = 0.0f, q3_m1 = 0.0f;
float ref_q_m1[4];
// auxiliary variables to reduce number of repeated operations
float q0q0, q0q1, q0q2, q0q3;
float q1q1, q1q2, q1q3;
float q2q2, q2q3;
float q3q3;

float halfT;

uint8_t MargAHRSinitialized = 0;

//----------------------------------------------------------------------------------------------------
float KpAcc = 1.0f;    // proportional gain governs rate of convergence to accelerometer
float KiAcc = 0.0f;    // integral gain governs rate of convergence of gyroscope biases
float KpMag = 5.0f;    // proportional gain governs rate of convergence to magnetometer
float KiMag = 0.0f;    // integral gain governs rate of convergence of gyroscope biases
float accConfidenceDecay = 0.0f;
float accConfidence      = 1.0f;

#define HardFilter(O,N)  ((O)*0.9f+(N)*0.1f)
#define accelOneG 9.8
void calculateAccConfidence(float accMag)
{
	// G.K. Egan (C) computes confidence in accelerometers when
	// aircraft is being accelerated over and above that due to gravity

	static float accMagP = 1.0f;

	accMag /= accelOneG;  // HJI Added to convert MPS^2 to G's

	accMag  = HardFilter(accMagP, accMag );
	accMagP = accMag;

	accConfidence
			= LIMIT(1.0 - (accConfidenceDecay * sqrt(fabs(accMag - 1.0f))), 0.0f, 1.0f);

}
float To_180_degrees_imu(float x)
{
	return (x>180?(x-360):(x<-180?(x+360):x));
}
//----------------------------------------------------------------------------------------------------

//====================================================================================================
// Initialization
//====================================================================================================
void euler_to_q_m1(float angle[3],float q[4]) {
	float roll=angle[0];
	float pitch=angle[1];
	float yaw=angle[2];	
	double cosPhi_2 = cos((roll) / 2.0);
	double sinPhi_2 = sin((roll) / 2.0);
	double cosTheta_2 = cos((pitch) / 2.0);
	double sinTheta_2 = sin((pitch) / 2.0);
	double cosPsi_2 = cos((yaw) / 2.0);
	double sinPsi_2 = sin((yaw) / 2.0);

	/* operations executed in double to avoid loss of precision through
	 * consecutive multiplications. Result stored as float.
	 */
	q[0] = (cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2);
	q[1] = (sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2);
	q[2] = (cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2);
	q[3] = (cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2);
}

void MargAHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{

    float Pit,Rol;
    Pit=atan(ax/az)*57.3;
		Rol=atan(ay/az)*57.3;
			
		float magTmp2 [3];	
		magTmp2[0]=mx;
		magTmp2[1]=my;
		magTmp2[2]=mz;
		float euler[2]; 	
		euler[1]=Pit*0.0173  ;
		euler[0]=Rol*0.0173  ;
		float calMagY = magTmp2[0] * cos(euler[1]) + magTmp2[1] * sin(euler[1])* sin(euler[0])+magTmp2[2] * sin(euler[1]) * cos(euler[0]); 
		float calMagX = magTmp2[1] * cos(euler[0]) + magTmp2[2] * sin(euler[0]);
		float yaw_mag=To_180_degrees(fast_atan2(calMagX,calMagY)* 57.3 );
		float angle_cal[3];
		angle_cal[0]=euler[0];
		angle_cal[1]=euler[1];
		angle_cal[2]=yaw_mag;
		float q_temp[4];
		euler_to_q_m1(angle_cal,q_temp);
	  q0_m1=-q_temp[1];
	  q1_m1=q_temp[0];
	  q2_m1=-q_temp[3];
	  q3_m1=q_temp[2];	

    // auxillary variables to reduce number of repeated operations, for 1st pass
    q0q0 = q0_m1 * q0_m1;
    q0q1 = q0_m1 * q1_m1;
    q0q2 = q0_m1 * q2_m1;
    q0q3 = q0_m1 * q3_m1;
    q1q1 = q1_m1 * q1_m1;
    q1q2 = q1_m1 * q2_m1;
    q1q3 = q1_m1 * q3_m1;
    q2q2 = q2_m1 * q2_m1;
    q2q3 = q2_m1 * q3_m1;
    q3q3 = q3_m1 * q3_m1;
}

//====================================================================================================
// Function
//====================================================================================================

void MargAHRSupdate(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    float accelCutoff, uint8_t magDataUpdate, float dt,float *rol,float *pit,float *yaw)
{   static u16 init;
    float norm, normR;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float q0i, q1i, q2i, q3i;
  
    //-------------------------------------------

    if ((MargAHRSinitialized == 0) && (magDataUpdate == 1) && init++>200)
    {
        MargAHRSinit(ax, ay, az, mx, my, mz);

        MargAHRSinitialized = 1;
    }

    //-------------------------------------------

    if (MargAHRSinitialized == 1)
    {
        halfT = dt * 0.5f;

        norm = sqrt(ax*(ax) + ay*(ay) + az*az);

        if (norm != 0.0f)
        {
			calculateAccConfidence(norm);
            kpAcc = KpAcc * accConfidence;
            kiAcc = KiAcc * accConfidence;

            normR = 1.0f / norm;
            ax *= normR;
            ay *= normR;
            az *= normR;

            // estimated direction of gravity (v)
            vx = 2.0f * (q1q3 - q0q2);
            vy = 2.0f * (q0q1 + q2q3);
            vz = q0q0 - q1q1 - q2q2 + q3q3;

            // error is sum of cross product between reference direction
		    // of fields and direction measured by sensors
		    exAcc = vy * az - vz * ay;
            eyAcc = vz * ax - vx * az;
            ezAcc = vx * ay - vy * ax;

            gx += exAcc * kpAcc;
            gy += eyAcc * kpAcc;
            gz += ezAcc * kpAcc;

            if (kiAcc > 0.0f)
            {
		    	exAccInt += exAcc * kiAcc;
                eyAccInt += eyAcc * kiAcc;
                ezAccInt += ezAcc * kiAcc;

                gx += exAccInt;
                gy += eyAccInt;
                gz += ezAccInt;
		    }
	    }

        //-------------------------------------------

        norm = sqrt(mx*(mx) + my*(my) + mz*(mz));

        if (( magDataUpdate == 1) && (norm != 0.0f))
        {
            normR = 1.0f / norm;
            mx *= normR;
            my *= normR;
            mz *= normR;

            // compute reference direction of flux
            hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));

            hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));

            hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

            bx = sqrt((hx * hx) + (hy * hy));

            bz = hz;

            // estimated direction of flux (w)
            wx = 2.0f * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));

            wy = 2.0f * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));

            wz = 2.0f * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));

            exMag = my * wz - mz * wy;
            eyMag = mz * wx - mx * wz;
            ezMag = mx * wy - my * wx;

			// use un-extrapolated old values between magnetometer updates
			// dubious as dT does not apply to the magnetometer calculation so
			// time scaling is embedded in KpMag and KiMag
			gx += exMag * KpMag;
			gy += eyMag * KpMag;
			gz += ezMag * KpMag;

			if (KiMag > 0.0f)
			{
				exMagInt += exMag * KiMag;
				eyMagInt += eyMag * KiMag;
				ezMagInt += ezMag * KiMag;

				gx += exMagInt;
				gy += eyMagInt;
				gz += ezMagInt;
			}
        }

        //-------------------------------------------

        // integrate quaternion rate
        q0i = (-q1_m1 * gx - q2_m1 * gy - q3_m1 * gz) * halfT;
        q1i = ( q0_m1 * gx + q2_m1 * gz - q3_m1 * gy) * halfT;
        q2i = ( q0_m1 * gy - q1_m1 * gz + q3_m1 * gx) * halfT;
        q3i = ( q0_m1 * gz + q1_m1 * gy - q2_m1 * gx) * halfT;
        q0_m1 += q0i;
        q1_m1 += q1i;
        q2_m1 += q2i;
        q3_m1 += q3i;

        // normalise quaternion
        normR = 1.0f / sqrt(q0_m1 * q0_m1 + q1_m1 * q1_m1 + q2_m1 * q2_m1 + q3_m1 * q3_m1);
        q0_m1 *= normR;
        q1_m1 *= normR;
        q2_m1 *= normR;
        q3_m1 *= normR;

        // auxiliary variables to reduce number of repeated operations
        q0q0 = q0_m1 * q0_m1;
        q0q1 = q0_m1 * q1_m1;
        q0q2 = q0_m1 * q2_m1;
        q0q3 = q0_m1 * q3_m1;
        q1q1 = q1_m1 * q1_m1;
        q1q2 = q1_m1 * q2_m1;
        q1q3 = q1_m1 * q3_m1;
        q2q2 = q2_m1 * q2_m1;
        q2q3 = q2_m1 * q3_m1;
        q3q3 = q3_m1 * q3_m1;
				
       ref_q[0]=q_nav[0]=ref_q_m1[0]=q1_m1;
			 ref_q[1]=q_nav[1]=ref_q_m1[1]=-q0_m1;
			 ref_q[2]=q_nav[2]=ref_q_m1[2]=q3_m1;
			 ref_q[3]=q_nav[3]=ref_q_m1[3]=-q2_m1;

			reference_vr[0]=reference_v.x = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
			reference_vr[1]=reference_v.y = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
			reference_vr[2]=reference_v.z = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);
			*rol = fast_atan2(2*(ref_q_m1[0]*ref_q_m1[1] + ref_q_m1[2]*ref_q_m1[3]),1 - 2*(ref_q_m1[1]*ref_q_m1[1] + ref_q_m1[2]*ref_q_m1[2])) *57.3f;
			*pit = asin(2*(ref_q_m1[1]*ref_q_m1[3] - ref_q_m1[0]*ref_q_m1[2])) *57.3f;
			*yaw = fast_atan2(2*(-ref_q_m1[1]*ref_q_m1[2] - ref_q_m1[0]*ref_q_m1[3]), 2*(ref_q_m1[0]*ref_q_m1[0] + ref_q_m1[1]*ref_q_m1[1]) - 1) *57.3f  ;// 
	
    }
}

