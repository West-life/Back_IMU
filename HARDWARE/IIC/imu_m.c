//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files
#include "imu.h"
#include "include.h"
#include "hml5833l.h"
#include "my_math.h"
#include "filter.h"
#include "MadgwickAHRS.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	400.0f		// sample frequency in Hz


//---------------------------------------------------------------------------------------------------
// Variable definitions
// AHRS algorithm update
	float ref_q_imd_down[4] = {1,0,0,0};
	float reference_vr_imd_down[3];
volatile float beta = 0.01f;								// 2 * proportional gain (Kp)
volatile float q0_m = 1.0f, q1_m = 0.0f, q2_m = 0.0f, q3_m = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
//---------------------------------------------------------------------------------------------------
volatile float q0_fc = 1.0f, q1_fc = 0.0f, q2_fc = 0.0f, q3_fc = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//====================================================================================================
// Functions
void euler_to_q(float angle[3],float q[4]) {
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
//---------------------------------------------------------------------------------------------------
// AHRS algorithm update
//	float ref_q_imd_down[4] = {1,0,0,0};
	float reference_vr_imd_down_fc[3];
	u8 init_q=1;
void MadgwickAHRSupdate(float dt,float gx, float gy, float gz, float ax, float ay, float az, 
	float mx, float my, float mz,float *rol,float *pit,float *yaw){
  float T;
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  static u16 init_cnt;
		if(init_cnt++>100){init_cnt=100+1;T=dt;
		}
		else 
		{		
	  if(init_q){
    float Pit,Rol;
    Pit=-atan(imu_fushion.Acc.x/imu_fushion.Acc.z)*57.3;
		Rol=atan(imu_fushion.Acc.y/imu_fushion.Acc.z)*57.3;
			
	//#if IMU_HML_ADD_500
		float magTmp2 [3];	
		magTmp2[0]=imu_fushion.Mag_Val.x;
		magTmp2[1]=imu_fushion.Mag_Val.y;
		magTmp2[2]=imu_fushion.Mag_Val.z;
		float euler[2]; 	
		euler[1]=Pit*0.0173  ;
		euler[0]=Rol*0.0173  ;
		float calMagY = magTmp2[0] * cos(euler[1]) + magTmp2[1] * sin(euler[1])* sin(euler[0])+magTmp2[2] * sin(euler[1]) * cos(euler[0]); 
		float calMagX = magTmp2[1] * cos(euler[0]) + magTmp2[2] * sin(euler[0]);
		float yaw_mag=To_180_degrees(fast_atan2(calMagX,calMagY)* RAD_DEG +180);
	//	#endif	
		float angle_cal[3];
		angle_cal[0]=euler[0];
		angle_cal[1]=euler[1];
		angle_cal[2]=yaw_mag;
		euler_to_q(angle_cal,ref_q_imd_down);
	  q0=ref_q_imd_down[0];
	  q1=ref_q_imd_down[1];
	  q2=ref_q_imd_down[2];
	  q3=ref_q_imd_down[3];	
	  }T=2*dt;
		
	}
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {

	MadgwickAHRSupdateIMU(T,gx, gy, gz, ax, ay, az);
	ref_q_imd_down[0]=q0;
	ref_q_imd_down[1]=q1;
	ref_q_imd_down[2]=q2;
	ref_q_imd_down[3]=q3;
	reference_vr_imd_down[0] = 2*(ref_q_imd_down[1]*ref_q_imd_down[3] - ref_q_imd_down[0]*ref_q_imd_down[2]);
	reference_vr_imd_down[1] = 2*(ref_q_imd_down[0]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[3]);
	reference_vr_imd_down[2] = 1 - 2*(ref_q_imd_down[1]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[2]);
	*rol = atan2(2*(ref_q_imd_down[0]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[3]),1 - 2*(ref_q_imd_down[1]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[2])) *57.3f;
	*pit = asin(2*(ref_q_imd_down[1]*ref_q_imd_down[3] - ref_q_imd_down[0]*ref_q_imd_down[2])) *57.3f;
	*yaw = fast_atan2(2*(-ref_q_imd_down[1]*ref_q_imd_down[2] - ref_q_imd_down[0]*ref_q_imd_down[3]), 2*(ref_q_imd_down[0]*ref_q_imd_down[0] + ref_q_imd_down[1]*ref_q_imd_down[1]) - 1) *57.3f  ;// 

		return;
	}
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1_fc * gx - q2_fc * gy - q3_fc * gz);
	qDot2 = 0.5f * (q0_fc * gx + q2_fc * gz - q3_fc * gy);
	qDot3 = 0.5f * (q0_fc * gy - q1_fc * gz + q3_fc * gx);
	qDot4 = 0.5f * (q0_fc * gz + q1_fc * gy - q2_fc * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0_fc * mx;
		_2q0my = 2.0f * q0_fc * my;
		_2q0mz = 2.0f * q0_fc * mz;
		_2q1mx = 2.0f * q1_fc * mx;
		_2q0 = 2.0f * q0_fc;
		_2q1 = 2.0f * q1_fc;
		_2q2 = 2.0f * q2_fc;
		_2q3 = 2.0f * q3_fc;
		_2q0q2 = 2.0f * q0_fc * q2_fc;
		_2q2q3 = 2.0f * q2_fc * q3_fc;
		q0q0 = q0_fc * q0_fc;
		q0q1 = q0_fc * q1_fc;
		q0q2 = q0_fc * q2_fc;
		q0q3 = q0_fc * q3_fc;
		q1q1 = q1_fc * q1_fc;
		q1q2 = q1_fc * q2_fc;
		q1q3 = q1_fc * q3_fc;
		q2q2 = q2_fc * q2_fc;
		q2q3 = q2_fc * q3_fc;
		q3q3 = q3_fc * q3_fc;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3_fc + _2q0mz * q2_fc + mx * q1q1 + _2q1 * my * q2_fc + _2q1 * mz * q3_fc - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3_fc + my * q0q0 - _2q0mz * q1_fc + _2q1mx * q2_fc - my * q1q1 + my * q2q2 + _2q2 * mz * q3_fc - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2_fc + _2q0my * q1_fc + mz * q0q0 + _2q1mx * q3_fc - mz * q1q1 + _2q2 * my * q3_fc - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2_fc * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3_fc + _2bz * q1_fc) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) 
		+ _2bx * q2_fc * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1_fc * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3_fc * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) 
		+ (_2bx * q2_fc + _2bz * q0_fc) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3_fc - _4bz * q1_fc) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2_fc * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
		+ (-_4bx * q2_fc - _2bz * q0_fc) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1_fc + _2bz * q3_fc) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0_fc - _4bz * q2_fc) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3_fc + _2bz * q1_fc) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + 
		(-_2bx * q0_fc + _2bz * q2_fc) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1_fc * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0_fc += qDot1 * (1.0f / sampleFreq);
	q1_fc += qDot2 * (1.0f / sampleFreq);
	q2_fc += qDot3 * (1.0f / sampleFreq);
	q3_fc += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0_fc * q0_fc + q1_fc * q1_fc + q2_fc * q2_fc + q3_fc * q3_fc);
	q0_fc *= recipNorm;
	q1_fc *= recipNorm;
	q2_fc *= recipNorm;
	q3_fc *= recipNorm;

	ref_q_imd_down[0]=q0_fc;
	ref_q_imd_down[1]=q1_fc;
	ref_q_imd_down[2]=q2_fc;
	ref_q_imd_down[3]=q3_fc;
	reference_vr_imd_down[0] = 2*(ref_q_imd_down[1]*ref_q_imd_down[3] - ref_q_imd_down[0]*ref_q_imd_down[2]);
	reference_vr_imd_down[1] = 2*(ref_q_imd_down[0]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[3]);
	reference_vr_imd_down[2] = 1 - 2*(ref_q_imd_down[1]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[2]);
	*rol = fast_atan2(2*(ref_q_imd_down[0]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[3]),1 - 2*(ref_q_imd_down[1]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[2])) *57.3f;
	*pit = asin(2*(ref_q[1]*ref_q_imd_down[3] - ref_q_imd_down[0]*ref_q_imd_down[2])) *57.3f;
	*yaw = fast_atan2(2*(-ref_q_imd_down[1]*ref_q_imd_down[2] - ref_q_imd_down[0]*ref_q_imd_down[3]), 2*(ref_q_imd_down[0]*ref_q_imd_down[0] + ref_q_imd_down[1]*ref_q_imd_down[1]) - 1) *57.3f  ;// 

}


//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float dt,float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	
	

}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


//---------------------------------------------------------------------------------------------------
// Definitions



//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = 0.8;//(2.0f * 0.5f);											// 2 * proportional gain (Kp)
volatile float twoKi = 0.2;//(2.0f * 0.0f);											// 2 * integral gain (Ki)
float twoKp_s=1.6;
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki


//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MahonyAHRSupdate(float dt,float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz
	,float *rol,float *pit,float *yaw) {
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
 static u16 init_cnt;
		if(init_cnt++>500){init_cnt=500+1;twoKp=twoKp_s;
		}
			else twoKp=2;
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(dt,gx, gy, gz, ax, ay, az);
	ref_q_imd_down[0]=q0;
	ref_q_imd_down[1]=q1;
	ref_q_imd_down[2]=q2;
	ref_q_imd_down[3]=q3;
	reference_vr_imd_down[0] = 2*(ref_q_imd_down[1]*ref_q_imd_down[3] - ref_q_imd_down[0]*ref_q_imd_down[2]);
	reference_vr_imd_down[1] = 2*(ref_q_imd_down[0]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[3]);
	reference_vr_imd_down[2] = 1 - 2*(ref_q_imd_down[1]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[2]);
	*rol = fast_atan2(2*(ref_q_imd_down[0]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[3]),1 - 2*(ref_q_imd_down[1]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[2])) *57.3f;
	*pit = asin(2*(ref_q[1]*ref_q_imd_down[3] - ref_q_imd_down[0]*ref_q_imd_down[2])) *57.3f;
	*yaw = fast_atan2(2*(-ref_q_imd_down[1]*ref_q_imd_down[2] - ref_q_imd_down[0]*ref_q_imd_down[3]), 2*(ref_q_imd_down[0]*ref_q_imd_down[0] + ref_q_imd_down[1]*ref_q_imd_down[1]) - 1) *57.3f  ;// 

		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(float dt,float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * dt;//(1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * dt;//(1.0f / sampleFreq);
			integralFBz += twoKi * halfez * dt;//(1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);//(1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * dt);////(1.0f / sampleFreq));
	gz *= (0.5f * dt);//(1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}
//5.0f, 0.01f, 0.01f
//5.0f, 0.25f, 0.01f
volatile float beta_start=5;
volatile float beta_end=0.25;
volatile float beta_step=0.01;

volatile float beta;
// *  smpl_frq    sampling frequency of AHRS data
// *  b_start     algorithm gain starting value
// *  b_end       algorithm gain end value
// *  b_step      algorithm gain decrement step size
int madgwick_update_new(float ax,float ay,float az, float wx,float wy,float wz, float mx,float my ,float mz,float *rol,float *pit,float *yaw,float T)								
{
static u8 init;
    float recip_norm;
    float s0, s1, s2, s3;
    float dq1, dq2, dq3, dq4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

if(!init){init=1;
    beta = beta_start;
}
    /* Check for valid magnetometer data. */
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        return 0;
    }
    /* Check if beta has reached its specified end value or if it has to be
     * decremented. */
    if (beta > beta_end)
    {
        /* Decrement beta only if it does not fall below the specified end
         * value. */
        if ((beta - beta_step) > beta_end)
        {
            beta -= beta_step;
        }
        else
        {
            beta = beta_end;
        }
    }

    /* Calculate quaternion rate of change of from angular velocity. */
    dq1 = 0.5f * (-q1 * wx - q2 * wy - q3 * wz);
    dq2 = 0.5f * (q0 * wx + q2 * wz - q3 * wy);
    dq3 = 0.5f * (q0 * wy - q1 * wz + q3 * wx);
    dq4 = 0.5f * (q0 * wz + q1 * wy - q2 * wx);

    /* Calculate feedback only if accelerometer measurement is valid. This
     * prevents NaNs in acceleration normalization. */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        /* Normalize accelerometer measurement. */
        recip_norm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        /* Normalize magnetometer measurement. */
        recip_norm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recip_norm;
        my *= recip_norm;
        mz *= recip_norm;

        /* Auxiliary variables to avoid repeated arithmetic and therefore
         * improve performance. */
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        /* Reference direction of earth magnetic field. */
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        /* Gradient decent algorithm corrective step. */
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        /* Normalize step magnitude. */
        recip_norm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recip_norm;
        s1 *= recip_norm;
        s2 *= recip_norm;
        s3 *= recip_norm;

        /* Apply feedback step. */
        dq1 -= beta * s0;
        dq2 -= beta * s1;
        dq3 -= beta * s2;
        dq4 -= beta * s3;
    }

    /* Integrate quaternion rate of change to get quaternion describing the
     * current orientation. */
    q0 += dq1 * T;
    q1 += dq2 * T;
    q2 += dq3 * T;
    q3 += dq4 * T;

    /* Normalize quaternion. */
    recip_norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recip_norm;
    q1 *= recip_norm;
    q2 *= recip_norm;
    q3 *= recip_norm;

	ref_q_imd_down[0]=q0;
	ref_q_imd_down[1]=q1;
	ref_q_imd_down[2]=q2;
	ref_q_imd_down[3]=q3;
	reference_vr_imd_down[0] = 2*(ref_q_imd_down[1]*ref_q_imd_down[3] - ref_q_imd_down[0]*ref_q_imd_down[2]);
	reference_vr_imd_down[1] = 2*(ref_q_imd_down[0]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[3]);
	reference_vr_imd_down[2] = 1 - 2*(ref_q_imd_down[1]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[2]);
	*rol = fast_atan2(2*(ref_q_imd_down[0]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[3]),1 - 2*(ref_q_imd_down[1]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[2])) *57.3f;
	*pit = asin(2*(ref_q[1]*ref_q_imd_down[3] - ref_q_imd_down[0]*ref_q_imd_down[2])) *57.3f;
	*yaw = fast_atan2(2*(-ref_q_imd_down[1]*ref_q_imd_down[2] - ref_q_imd_down[0]*ref_q_imd_down[3]), 2*(ref_q_imd_down[0]*ref_q_imd_down[0] + ref_q_imd_down[1]*ref_q_imd_down[1]) - 1) *57.3f  ;// 

    return 1;
}
//====================================================================================================
// END OF CODE
//====================================================================================================
