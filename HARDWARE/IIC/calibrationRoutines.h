#include "stm32f4xx.h"

typedef struct  
 {
	 float x_sumplain ;
	 float x_sumsq ;
	 float x_sumcube ;

	 float y_sumplain ;
	 float y_sumsq ;
	 float y_sumcube ;

	 float z_sumplain ;
	 float z_sumsq ;
	 float z_sumcube ;

	 float xy_sum ;
	 float xz_sum ;
	 float yz_sum ;

	 float x2y_sum ;
	 float x2z_sum ;
	 float y2x_sum ;
	 float y2z_sum ;
	 float z2x_sum ;
	 float z2y_sum ;

	 unsigned int size;
 }LSQIntermediate;

extern LSQIntermediate hml_lsq;

 int sphere_fit_least_squares(const float x[], const float y[], const float z[],
                              unsigned int size, unsigned int max_iterations, float delta, float *sphere_x, float *sphere_y, float *sphere_z, float *sphere_radius);


 void lsq_init(LSQIntermediate * pLSQ);
 unsigned int lsq_accumulate(LSQIntermediate * pLSQ, float x, float y, float z);
 void lsq_calculate_oldx(LSQIntermediate * pLSQ, unsigned int max_iterations, float delta, float *sphere_x, float *sphere_y, float *sphere_z, float *sphere_radius);
