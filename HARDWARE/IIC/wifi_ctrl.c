#include "string.h"
#include <math.h>
#include "wifi_ctrl.h"

typedef enum
{
	waitForStart,
	waitForData,
	waitForChksum,
	waitForEnd
} WifilinkRxState;

static u8 rawWifiData[8];
ctrlVal_t wifiCtrl;/*���͵�commander��̬��������*/

/*wifi��Դ����*/
void wifiPowerControl(u8 state)
{
//	if(state == 1)
//		WIFI_POWER_ENABLE = 1;
//	else
//		WIFI_POWER_ENABLE = 0;
}

/*wifiģ���ʼ��*/
void wifiModuleInit(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
//	
//	/* ����wifi��Դ���ƽ���� */
//	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	
//	wifiPowerControl(true);	
	
//	uart1Init(19200);	/*����1��ʼ���������ʹ̶�19200*/
}

static u8 wifiDataCrc(u8 *data)
{
	u8 temp=(data[1]^data[2]^data[3]^data[4]^data[5])&0xff;
	if(temp==data[6])
		return 1;
	return 0;
}

///*�������*/
static void wifiCmdProcess(u8 data)
{
	wifiCmd_t wifiCmd = *(wifiCmd_t*)&data;
	
	if(wifiCtrl.h_mode==1)/*��ǰ�������Ϊ����ģʽ*/
	{
		if(wifiCmd.keyFlight) /*һ�����*/
		{
//			setCommanderKeyFlight(1);
//			setCommanderKeyland(0);	
		}
		if(wifiCmd.keyLand) /*һ������*/
		{
//			setCommanderKeyFlight(0);
//			setCommanderKeyland(1);
		}
		if(wifiCmd.emerStop) /*����ͣ��*/
		{
//			setCommanderKeyFlight(0);
//			setCommanderKeyland(0);
//			setCommanderEmerStop(1);
		}else
		{
			//setCommanderEmerStop(0);
		}
	}
	else/*��ǰ�������Ϊ�ֶ���ģʽ*/
	{
		wifiCtrl.h_mode=0;
//		setCommanderAltholdMode(0);
//		setCommanderKeyFlight(0);
//		setCommanderKeyland(0);
	}

	//setCommanderFlightmode(wifiCmd.flightMode);
	
	if(wifiCmd.flipOne) /*�̶����򷭹�*/
	{
	}
	if(wifiCmd.flipFour) /*4D����*/
	{
	}
	if(wifiCmd.ledControl) /*�ƹ����*/
	{		
	}
	if(wifiCmd.gyroCalib) /*����У׼*/
	{
	}
}

static void wifiDataHandle(u8 *data)
{
	static u16 lastThrust;
	wifiCtrl.connect=1;
	wifiCtrl.loss_cnt=0;
	wifiCtrl.roll   = ((float)data[1]-(float)0x80)*0.25f/19.2*500+1500;	/*roll: ��9.5 ��19.2 ��31.7*/
	wifiCtrl.pitch  = ((float)data[2]-(float)0x80)*0.25f/19.2*500+1500;	/*pitch:��9.5 ��19.2 ��31.7*/
	wifiCtrl.yaw    = ((float)data[4]-(float)0x80)*1.6f/203*500+1500;	/*yaw : ��203.2*/				
	wifiCtrl.thrust = (float)((u16)data[3] << 8)/65535*1000+1000;					/*thrust :0~63356*/
	
	if(wifiCtrl.thrust==32768 && lastThrust<10000)/*�ֶ����л�������*/
	{
		
//		setCommanderAltholdMode(1);
//		setCommanderKeyFlight(0);
//		setCommanderKeyland(0);
	}
	else if(wifiCtrl.thrust==0 && lastThrust>256)/*�����л����ֶ���*/
	{
		//setCommanderAltholdMode(0);
	}
	lastThrust = wifiCtrl.thrust;

//	wifiCmdProcess(data[5]);/*λ��־�������*/
//	flightCtrldataCache(WIFI, wifiCtrl);
}

void wifiLinkTask(u8 c)
{
 static u8 dataIndex;	
 static u8 rxState;
			switch(rxState)
			{
				case waitForStart:
					if(c == 0x66)					/*��ʼ����ȷ*/
					{
						dataIndex=1;
						rawWifiData[0] = c;
						rxState = waitForData;
					} else							/*��ʼ������*/
					{
						rxState = waitForStart;
					}
					break;				
				case waitForData:
					rawWifiData[dataIndex] = c;
					dataIndex++;
					if (dataIndex == 6)				/*���ݽ�����ɣ�У��*/
					{
						rxState = waitForChksum;
					}
					break;
				case waitForChksum:
					rawWifiData[6] = c;
					if (wifiDataCrc(rawWifiData))	/*У����ȷ���жϽ�����*/
					{
						rxState = waitForEnd;
					} else
					{
						rxState = waitForStart;		/*У�����*/
					}
					break;
				case waitForEnd:
					if (c == 0x99)					/*��������ȷ*/
					{
						rawWifiData[7] = c;
						wifiDataHandle(rawWifiData);/*������յ�������*/
						
						
					} else
					{
						rxState = waitForStart;		/*����������*/
						//IF_DEBUG_ASSERT(1);
					}
					rxState = waitForStart;
					break;
				default:
					;//ASSERT(0);
					break;
			}
		
}






