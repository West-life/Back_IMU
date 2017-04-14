#include "usart_fc.h"
#include "include.h"
void BubbleSort(u16 *R,u16 n)
   { //R（l..n)是待排序的文件，采用自下向上扫描，对R做冒泡排序
		
     int i,j;
     u8 exchange;//交换标志
     for(i=1;i<n;i++){ //最多做n-1趟排序
       exchange=0; //本趟排序开始前，交换标志应为假
       for(j=n-1;j>=i;j--) //对当前无序区R[i..n]自下向上扫描
        if(R[j+1]<R[j]){//交换记录
          R[0]=R[j+1]; //R[0]不是哨兵，仅做暂存单元
          R[j+1]=R[j];
          R[j]=R[0];
          exchange=1;//发生了交换，故将交换标志置为真
         }
       if(!exchange) //本趟排序未发生交换，提前终止算法
             return;
     } //endfor(外循环)
    } //BubbleSort
#define DIV_NUM 8 //MAX==20
u16 Laser_avoid[60];
float flt_laser=0.8;
float rate_use[2]={0.16,0.66};
void Laser_cal(void)
{
u32 temp;
u16 i;
u8 j;	
u16 k;
u8 cnt;	
Laser_avoid[0]=DIV_NUM;
u16 laser_buf_temp[360];
u16 bufs[360/DIV_NUM];
for (i=0;i<360;i++)
	{
		laser_buf_temp[i]=laser_buf[(i-360/DIV_NUM/2)%360];
	}	
	
for(j=0;j<DIV_NUM;j++){
			temp=0;
	    cnt=0;
	for(i=j*360/DIV_NUM;i<j*360/DIV_NUM+360/DIV_NUM;i++)
      {  
				
				 //temp+=laser_buf_temp[i];
				if(laser_buf_temp[i]!=0)
				bufs[cnt++]=laser_buf_temp[i];

			}
			BubbleSort(bufs,cnt);
			cnt=0;
			for(i=360/DIV_NUM*rate_use[0];i<360/DIV_NUM*(1-rate_use[1]);i++)
      {  
				
				 temp+=bufs[i];
				 cnt++;

			}	
				Laser_avoid[j+1]=flt_laser*temp/(cnt+1)+(1-flt_laser)*Laser_avoid[j+1];

}


}