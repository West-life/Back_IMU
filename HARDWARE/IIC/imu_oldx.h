#ifndef _IMU_OLDX_H_
#define	_IMU_OLDX_H_

#include "stm32f4xx.h"
#include "math.h"
void OLDX_AHRS(     float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    float acc_weight,float *rol,float *pit,float *yaw,float dt);
										
extern float ref_q_m1[4];
extern float reference_v_m1[3];

#define KpAcc  1.0f    
#define KiAcc  0.1f 
#define KpMag  5.0f    
#define KiMag  0.1f    									
#endif

