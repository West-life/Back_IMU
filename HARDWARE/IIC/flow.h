#ifndef __FLOW_H
#define __FLOW_H	 
#include "stm32f4xx.h" 
#include "include.h" 
	
u8 flow_task(void);


extern	double pixel_flow_x,pixel_flow_x_gro_fix;
extern float pixel_flow_x_pix,pixel_flow_y_pix;
extern	double pixel_flow_y;
extern	double pixel_flow_x2;
extern	double pixel_flow_y2;
extern double flow_compx ;
extern double flow_compy ;
extern float get_time_between_images;
extern float wz_speed_flow[2];

#endif











