
#include "Soft_I2C_PX4.h"
#include "delay.h"


/**************************I2C GPIO����*********************************/
void Soft_I2C_Init_PX4(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	GPIO_InitStructure.GPIO_Pin   = I2C_Pin_SCL_PX4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(I2C_GPIO_PX4_S, &GPIO_InitStructure);		

	GPIO_InitStructure.GPIO_Pin   = I2C_Pin_SDA_PX4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(I2C_GPIO_PX4_D, &GPIO_InitStructure);		
}
u8 iic_px4_delay=1;
/***************************I2C ��ʱ����*******************************/
static void I2C_delay_PX4(void)
{

	
	
	delay_us(iic_px4_delay);
//	volatile int i = 10;
//	while(i)
//		i--;	
	
}

 uint8_t I2C_Start_PX4(void)
{
	SDA_H_PX4;
	SCL_H_PX4;
	I2C_delay_PX4();
	//SDA��Ϊ�͵�ƽ������æ,�˳�
	if(!SDA_Read_PX4) return 0;
	SDA_L_PX4;
	I2C_delay_PX4();
	//SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
	if(SDA_Read_PX4) return 0;
	SDA_L_PX4;
	I2C_delay_PX4();
	return 1;	
}

 void I2C_Stop_PX4(void)
{
	SCL_L_PX4;
	I2C_delay_PX4();
	SDA_L_PX4;
	I2C_delay_PX4();
	SCL_H_PX4;
	I2C_delay_PX4();
	SDA_H_PX4;
	I2C_delay_PX4();
}

 void I2C_Ack_PX4(void)
{	
	SCL_L_PX4;
	I2C_delay_PX4();
	SDA_L_PX4;
	I2C_delay_PX4();
	SCL_H_PX4;
	I2C_delay_PX4();
	SCL_L_PX4;
	I2C_delay_PX4();
}   

 void I2C_NoAck_PX4(void)
{	
	SCL_L_PX4;
	I2C_delay_PX4();
	SDA_H_PX4;
	I2C_delay_PX4();
	SCL_H_PX4;
	I2C_delay_PX4();
	SCL_L_PX4;
	I2C_delay_PX4();
} 

/************************Wait I2C Ӧ��********************************/
//����Ϊ:=1��ACK	=0��ACK
 uint8_t I2C_WaitAck_PX4(void)
{
	SCL_L_PX4;
	I2C_delay_PX4();
	SDA_H_PX4;			
	I2C_delay_PX4();
	SCL_H_PX4;
	I2C_delay_PX4();
	if(SDA_Read_PX4)
	{
		SCL_L_PX4;
		I2C_delay_PX4();
		return 0;
	}
	SCL_L_PX4;
	I2C_delay_PX4();
	return 1;
}

//���ݴӸ�λ����λ
 void I2C_SendByte_PX4(uint8_t byte) 
{
    u8 i=8;
    while(i--)
    {
        SCL_L_PX4;
        I2C_delay_PX4();
		if(byte&0x80)
			SDA_H_PX4;  
		else 
			SDA_L_PX4;   
        byte <<= 1;
        I2C_delay_PX4();
		SCL_H_PX4;
		I2C_delay_PX4();
    }
    SCL_L_PX4;
}  

//���ݴӸ�λ����λ
 uint8_t I2C_ReceiveByte_PX4(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H_PX4;
    while (i--)
	{
        byte <<= 1;
        SCL_L_PX4;
        I2C_delay_PX4();
        SCL_H_PX4;
        I2C_delay_PX4();
        if (SDA_Read_PX4) 
		{
            byte |= 0x01;
        }
    }
    SCL_L_PX4;
    return byte;
}


/*******************************************************************************
* ������  : I2C_Single_Write
* ����    : ��I2C�豸д��һ�ֽ�����
* ����    : �豸��ַ	�Ĵ�����ַ		��д����ֽ�
* ���    : ��
* ����    : 0 or 1
****************************************************************************** */
uint8_t I2C_Single_Write_PX4(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t data)
{
    if(!I2C_Start_PX4()) return 0;
	//�����豸��ַ+д�ź� 
    I2C_SendByte_PX4(SlaveAddress << 1 | I2C_Direction_Transmitter);   
    if(!I2C_WaitAck_PX4()){
		I2C_Stop_PX4(); 
		return 0;
	}  
    I2C_SendByte_PX4(REG_Address );   
    I2C_WaitAck_PX4();	
    I2C_SendByte_PX4(data);
    I2C_WaitAck_PX4();   
    I2C_Stop_PX4(); 
    return 1;
}


/*******************************************************************************
* ������  : I2C_Single_Read
* ����    : ��I2C���ȡһ�ֽ�����
* ����    : �豸��ַ	�Ĵ�����ַ
* ���    : ��
* ����    : 0 or ����
****************************************************************************** */
uint8_t I2C_Single_Read_PX4(uint8_t SlaveAddress, uint8_t REG_Address)
{
	uint8_t data;
    if (!I2C_Start_PX4()) return 0;
	//�����豸��ַ+д�ź�
    I2C_SendByte_PX4(SlaveAddress << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck_PX4()) {
        I2C_Stop_PX4();
        return 0;
    }
    I2C_SendByte_PX4(REG_Address);
    I2C_WaitAck_PX4();
    I2C_Start_PX4();
    I2C_SendByte_PX4(SlaveAddress << 1 | I2C_Direction_Receiver);
    I2C_WaitAck_PX4();
	data = I2C_ReceiveByte_PX4();
    I2C_NoAck_PX4();
	I2C_Stop_PX4();
	return data;
}

/*******************************************************************************
* ������  : I2C_WriteBuffer
* ����    : ��I2C�豸д����ֽ�����
* ����    : �豸��ַ	�Ĵ�����ַ		���ݳ���		��д������
* ���    : ��
* ����    : 0 or 1
****************************************************************************** */
uint8_t I2C_WriteBuffer_PX4(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    int i;
    if (!I2C_Start_PX4())
        return 0;
    I2C_SendByte_PX4(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck_PX4()) {
        I2C_Stop_PX4();
        return 0;
    }
    I2C_SendByte_PX4(reg);
    I2C_WaitAck_PX4();
    for (i = 0; i < len; i++) {
        I2C_SendByte_PX4(data[i]);
        if (!I2C_WaitAck_PX4()) {
            I2C_Stop_PX4();
            return 0;
        }
    }
    I2C_Stop_PX4();
    return 1;
}

/*******************************************************************************
* ������  : I2C_ReadBuffer
* ����    : ��I2C��������ֽ�����
* ����    : �豸��ַ	�Ĵ�����ַ		�������ݳ���	�������ݵ�ַ	
* ���    : ����������
* ����    : 0 or 1
****************************************************************************** */
uint8_t I2C_ReadBuffer_PX4(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!I2C_Start_PX4())
        return 0;
    I2C_SendByte_PX4(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck_PX4()) {
        I2C_Stop_PX4();
        return 0;
    }
    I2C_SendByte_PX4(reg);
    I2C_WaitAck_PX4();
    I2C_Start_PX4();
    I2C_SendByte_PX4(addr << 1 | I2C_Direction_Receiver);
    I2C_WaitAck_PX4();
    while (len) {
        *buf = I2C_ReceiveByte_PX4();
        if (len == 1)
            I2C_NoAck_PX4();
        else
            I2C_Ack_PX4();
        buf++;
        len--;
    }
    I2C_Stop_PX4();
    return 1;
}
//----------------------------------------------------------------------------

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Start(void)
*��������:		����IIC��ʼ�ź�
*******************************************************************************/
void IIC_Start_PX4(void)
{
	SDA_OUT_PX4();     //sda�����
	IIC_SDA_PX4=1;	  	  
	IIC_SCL_PX4=1;
	delay_us(4);
 	IIC_SDA_PX4=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL_PX4=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Stop(void)
*��������:	    //����IICֹͣ�ź�
*******************************************************************************/	  
void IIC_Stop_PX4(void)
{
	SDA_OUT_PX4();//sda�����
	IIC_SCL_PX4=0;
	IIC_SDA_PX4=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL_PX4=1; 
	IIC_SDA_PX4=1;//����I2C���߽����ź�
	delay_us(4);		
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Wait_Ack(void)
*��������:	    �ȴ�Ӧ���źŵ��� 
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
*******************************************************************************/
u8 IIC_Wait_Ack_PX4(void)
{
	u8 ucErrTime=0;
	SDA_IN_PX4();      //SDA����Ϊ����  
	IIC_SDA_PX4=1;delay_us(1);	   
	IIC_SCL_PX4=1;delay_us(1);	 
	while(READ_SDA_PX4)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop_PX4();
			return 1;
		}
	  delay_us(1);
	}
	IIC_SCL_PX4=0;//ʱ�����0 	   
	return 0;  	
} 

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Ack(void)
*��������:	    ����ACKӦ��
*******************************************************************************/
void IIC_Ack_PX4(void)
{
	IIC_SCL_PX4=0;
	SDA_OUT_PX4();
	IIC_SDA_PX4=0;
	delay_us(2);
	IIC_SCL_PX4=1;
	delay_us(2);
	IIC_SCL_PX4=0;
	
}
	
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_NAck(void)
*��������:	    ����NACKӦ��
*******************************************************************************/	    
void IIC_NAck_PX4(void)
{
	IIC_SCL_PX4=0;
	SDA_OUT_PX4();
	IIC_SDA_PX4=1;
	delay_us(2);
	IIC_SCL_PX4=1;
	delay_us(2);
	IIC_SCL_PX4=0;
}					 				     

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Send_Byte(u8 txd)
*��������:	    IIC����һ���ֽ�
*******************************************************************************/		  
void IIC_Send_Byte_PX4(u8 txd)
{ 
    u8 t;   
	SDA_OUT_PX4(); 	    
    IIC_SCL_PX4=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA_PX4=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   
		IIC_SCL_PX4=1;
		delay_us(2); 
		IIC_SCL_PX4=0;	
		delay_us(2);
    }		
} 	 
   
/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Read_Byte(unsigned char ack)
*��������:	    //��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
*******************************************************************************/  
u8 IIC_Read_Byte_PX4(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN_PX4();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL_PX4=0; 
        delay_us(2);
		IIC_SCL_PX4=1;
        receive<<=1;
        if(READ_SDA_PX4)receive++;   
		delay_us(2); 
    }					 
    if (ack)
        IIC_Ack_PX4(); //����ACK 
    else
        IIC_NAck_PX4();//����nACK  
    return receive;		
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����	I2C_Addr  Ŀ���豸��ַ
		addr	   �Ĵ�����ַ
����   ��������ֵ
*******************************************************************************/ 
unsigned char I2C_ReadOneByte_PX4(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;
	
	IIC_Start_PX4();	
	IIC_Send_Byte_PX4(I2C_Addr);	   //����д����
	res++;
	IIC_Wait_Ack_PX4();
	IIC_Send_Byte_PX4(addr); res++;  //���͵�ַ
	IIC_Wait_Ack_PX4();	  
	//IIC_Stop();//����һ��ֹͣ����	
	IIC_Start_PX4();
	IIC_Send_Byte_PX4(I2C_Addr+1); res++;          //�������ģʽ			   
	IIC_Wait_Ack_PX4();
	res=IIC_Read_Byte_PX4(0);	   
    IIC_Stop_PX4();//����һ��ֹͣ����

	return res;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ����� length��ֵ
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫ�����ֽ���
		*data  ���������ݽ�Ҫ��ŵ�ָ��
����   ���������ֽ�����
*******************************************************************************/ 
u8 IICreadBytes_PX4(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	
	IIC_Start_PX4();
	IIC_Send_Byte_PX4(dev);	   //����д����
	IIC_Wait_Ack_PX4();
	IIC_Send_Byte_PX4(reg);   //���͵�ַ
    IIC_Wait_Ack_PX4();	  
	IIC_Start_PX4();
	IIC_Send_Byte_PX4(dev+1);  //�������ģʽ	
	IIC_Wait_Ack_PX4();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=IIC_Read_Byte_PX4(1);  //��ACK�Ķ�����
		 	else  data[count]=IIC_Read_Byte_PX4(0);	 //���һ���ֽ�NACK
	}
    IIC_Stop_PX4();//����һ��ֹͣ����
    return count;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*��������:	    ������ֽ�д��ָ���豸 ָ���Ĵ���
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫд���ֽ���
		*data  ��Ҫд�����ݵ��׵�ַ
����   �����Ƿ�ɹ�
*******************************************************************************/ 
u8 IICwriteBytes_PX4(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	IIC_Start_PX4();
	IIC_Send_Byte_PX4(dev);	   //����д����
	IIC_Wait_Ack_PX4();
	IIC_Send_Byte_PX4(reg);   //���͵�ַ
    IIC_Wait_Ack_PX4();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte_PX4(data[count]); 
		IIC_Wait_Ack_PX4(); 
	 }
	IIC_Stop_PX4();//����һ��ֹͣ����

    return 1; //status == 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICreadByte(u8 dev, u8 reg, u8 *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		*data  ���������ݽ�Ҫ��ŵĵ�ַ
����   1
*******************************************************************************/ 
u8 IICreadByte_PX4(u8 dev, u8 reg, u8 *data){
	*data=I2C_ReadOneByte_PX4(dev, reg);
    return 1;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*��������:	    д��ָ���豸 ָ���Ĵ���һ���ֽ�
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		data  ��Ҫд����ֽ�
����   1
*******************************************************************************/ 
unsigned char IICwriteByte_PX4(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes_PX4(dev, reg, 1, &data);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitStart  Ŀ���ֽڵ���ʼλ
		length   λ����
		data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   �ɹ� Ϊ1 
 		ʧ��Ϊ0
*******************************************************************************/ 
u8 IICwriteBitsm_PX4(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b;
    if (IICreadByte_PX4(dev, reg, &b) != 0) {
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte_PX4(dev, reg, b);
    } else {
        return 0;
    }
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
		data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
����   �ɹ� Ϊ1 
 		ʧ��Ϊ0
*******************************************************************************/ 
u8 IICwriteBitm_PX4(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    IICreadByte_PX4(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte_PX4(dev, reg, b);
}

//------------------End of File----------------------------

