#ifndef __FILTER_H
#define __FILTER_H

#include "include.h"
//float Moving_Average(u8 item,u8 width_num,float in);
void Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out);
float Moving_Median(u8 item,u8 width_num,float in);
extern double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na);
extern fp32 LPF_1st(fp32 oldData, fp32 newData, fp32 lpf_factor);
#define _xyz_f_t xyz_f_t
void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out);

#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))

#define LPF_1(A,B,C,D) LPF_1_((A),(B),(C),*(D));

typedef struct
{
	float a;
	float b;
	float e_nr;
	float out;
} _filter_1_st;

void anotc_filter_1(float base_hz,float gain_hz,float dT,float in,_filter_1_st *f1);
void Moving_Average1(float moavarray[],u16 len ,u16 *fil_cnt,float in,float *out);
float my_deathzoom1(float x,float ref,float zoom);//my_deadzone;




#define NUMBER_OF_FIRST_ORDER_FILTERS 20
#define ACC_LOWPASS_X 0
#define ACC_LOWPASS_Y 1
#define ACC_LOWPASS_Z 2
#define HML_LOWPASS_X 3
#define HML_LOWPASS_Y 4
#define HML_LOWPASS_Z 5
#define ACC_UKF_LOWPASS_X 6
#define ACC_UKF_LOWPASS_Y 7
#define ACC_UKF_LOWPASS_Z 8
typedef struct firstOrderFilterData {
  float   gx1;
  float   gx2;
  float   gx3;
  float   previousInput;
  float   previousOutput;
} firstOrderFilterData_t;

extern firstOrderFilterData_t firstOrderFilters[NUMBER_OF_FIRST_ORDER_FILTERS];

void initFirstOrderFilter(float T);
float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters,float T);

typedef struct
{
 //volatile 
   float Input_Butter[3];
 //volatile 
   float Output_Butter[3];
}Butter_BufferData;


typedef struct
{
  float a[3];
  float b[3];
}Butter_Parameter;

extern Butter_Parameter LF_20Hz;
extern Butter_BufferData Acc_20Hz[3];

// first order filter
typedef struct {
    float tc;
    float z1;
} utilFilter_t;

typedef struct {
    const float *window;
    float *data;
    uint8_t n;
    uint8_t i;
} utilFirFilter_t;


void utilFilterReset(utilFilter_t *f, float setpoint);
void utilFilterReset3(utilFilter_t *f, float setpoint) ;
// larger tau, smoother filter
void utilFilterInit(utilFilter_t *f, float dt, float tau, float setpoint) ;
void utilFilterInit3(utilFilter_t *f, float dt, float tau, float setpoint);
float utilFilter(utilFilter_t *f, float signal) ;
#endif
