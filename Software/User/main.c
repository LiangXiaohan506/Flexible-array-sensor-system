/**
******************************************************************************
* @file    			main.c
* @author  			WIZnet Software Team
* @version 			V1.0
* @date    			2015-02-14
* @brief   			��3.5.0�汾�⽨�Ĺ���ģ��
* @attention  	ʵ��ƽ̨��Ұ�� iSO-V3 STM32 ������ + Ұ��W5500���������
*
*               Ĭ��ʹ��Ұ�𿪷����SPI2�ӿ�
*							
*               �������ԣ��뱣֤W5500��IP�����PC����ͬһ�����ڣ��Ҳ���ͻ
*               ����������߸�PC��ֱ����������PC���������ӵ�ַIPΪ��̬IP
*
* ʵ��ƽ̨:Ұ�� iSO-V3 STM32 ������ 
* ��̳    :http://www.firebbs.cn
* �Ա�    :https://fire-stm32.taobao.com
******************************************************************************
*/ 
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"
#include "sys.h"
#include "bsp_usart1.h"
#include "bsp_i2c_ee.h"
#include "bsp_i2c_gpio.h"
#include "bsp_led.h"

#include "w5500.h"
#include "W5500_conf.h"
#include "socket.h"
#include "utility.h"
#include "tcp_demo.h"

#define RUN PCout(5)// PC5
#define Tongxin PCout(4)// PC4

#include "DMA_ADC.h"

/////////////////////////���Ŷ���///////////////////
#define L_A PCout(12)
#define L_B PCout(11)
#define L_C PCout(10)
#define L_D PAout(15)

#define L_G1 PBout(8)
#define L_G2 PBout(9)
#define L_G3 PBout(10)
#define L_G4 PBout(11)

#define R_S1 PBout(5)
#define R_S2 PBout(6)
#define R_S3 PBout(7)

/////////////////////////��������///////////////////
u8 gc=0;//Ƭѡ��־
u8 lc=0;//��ѡ��־
u8 rc=0;//��ѡ��־
u8 r=0;//����ͳ��
u8 l=0;//����ͳ��
u8 Flag_over = 0;//�ɼ���ɱ�־λ
	
u16 times=0;//����DMA�жϵĴ���

extern __IO u16 ADC_ConvertedValue[11];//DMA�洢���ݵ�����

extern u8 Res;

extern uint8 Voltage[8192];//��ѹֵ��λ����Ϣ����

u16 VoltageS[44][52];//���д�����������
 
u8 List_A[13] = {0,1,0,1,0,1,0,1,0,1,0,1,0};
u8 List_B[13] = {0,1,1,0,0,1,1,0,0,1,1,0,0};
u8 List_C[13] = {1,0,0,0,0,1,1,1,1,0,0,0,0};
u8 List_D[13] = {1,1,1,1,1,0,0,0,0,0,0,0,0};//0~12

u8 List_G1[4] = {0,1,1,1};
u8 List_G2[4] = {1,0,1,1};
u8 List_G3[4] = {1,1,0,1};
u8 List_G4[4] = {1,1,1,0};//0~3

u8 Row_S1[4] = {0,1,0,1};
u8 Row_S2[4] = {1,0,0,1};
u8 Row_S3[4] = {0,0,0,0};//0~3

u8 Lists[44] = {43,41,39,37,
								35,33,31,29,
								27,25,23,21,
								19,17,15,13,
								11,9,7,5,
								3,1,0,2,
								4,6,8,10,
								12,14,16,18,
								20,22,24,26,
								28,30,32,34,
								36,38,40,42};

u16 Voltage_max = 0;
u8 Row,List = 0;
								
/////////////////////////��������///////////////////
void LED_Init(void);//LED��ʼ��
void GPIO_Configuration(void);//�˿ڳ�ʼ��
void DMA_Adc_collect();//DMA_adc�ɼ�����
void Uart1_send();//���ڷ��ͺ���
void W5500_send();//w5500���ͺ���
								
int main(void)
{ 	
	systick_init(72);										/*��ʼ��Systick����ʱ��*/
	USART1_Config(921600);							/*��ʼ������ͨ��:115200@8-n-1*/
	i2c_CfgGpio();											/*��ʼ��eeprom*/

//	printf("  Ұ����������� �����ʼ�� Demo V1.0 \r\n");		

	gpio_for_w5500_config();						/*��ʼ��MCU�������*/
	reset_w5500();											/*Ӳ��λW5500*/
	set_w5500_mac();										/*����MAC��ַ*/
	set_w5500_ip();											/*����IP��ַ*/
	
	socket_buf_init(txsize, rxsize);		/*��ʼ��8��Socket�ķ��ͽ��ջ����С*/
	
	LED_Init();

	ADC_Initializes();//DMA+ADC
	
	GPIO_Configuration();
	
//  printf(" ������ΪTCP������,��W5500��Ϊ TCP�ͻ���ȥ���� \r\n");
//	printf(" ������IP:%d.%d.%d.%d\r\n",remote_ip[0],remote_ip[1],remote_ip[2],remote_ip[3]);
//	printf(" �����˿�:%d \r\n",remote_port);
//	printf(" ���ӳɹ��󣬷������������ݸ�W5500��W5500�����ض�Ӧ���� \r\n");
//	printf(" Ӧ�ó���ִ���С���\r\n"); 
	
	while(1) 														/*ѭ��ִ�еĺ���*/ 
	{
		DMA_Adc_collect();
		if(Flag_over == 1)
		{ 
			W5500_send();
//			Uart1_send();
			Flag_over = 0;
		}
		RUN=!RUN;
	}
} 

//��ʼ��PB5��PE5Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //ʹ��PC�˿�ʱ��
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				 //RUN-->PC5 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOC, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOC5
 GPIO_SetBits(GPIOC,GPIO_Pin_5);						 //PC5 �����

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	    		 //Tongxin-->PC4 �˿�����, �������
 GPIO_Init(GPIOC, &GPIO_InitStructure);	  				 //������� ��IO���ٶ�Ϊ50MHz
 GPIO_SetBits(GPIOC,GPIO_Pin_4); 						 //PC4 ����� 
}

//=============================================================================
//�ļ����ƣ�GPIO_Configuration
//���ܸ�Ҫ��GPIO��ʼ��
//����˵������
//�������أ���
//=============================================================================
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE); //ʹ��PB�˿�ʱ��	
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); /*ʹ��SWD ����JTAG*/
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;//��ѡ�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO���ٶ�Ϊ50MHz
  GPIO_Init(GPIOC, &GPIO_InitStructure);//�����趨������ʼ��GPIOB
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//��ѡ�˿�����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//�����趨������ʼ��GPIOA
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;//Ƭѡ�˿�����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//�����趨������ʼ��GPIOB
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;//��ѡ�˿�����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//�����趨������ʼ��GPIOB
}

//=============================================================================
//�ļ����ƣ�DMA_Adc_collect()
//���ܸ�Ҫ����ȡDMA_ADֵ
//����˵������
//�������أ���
//=============================================================================
void DMA_Adc_collect()
{
	u8 i,m,n,k;

	for(r=0; r<4; r++)//��
	{
		for(l=0; l<52; l++)//��
		{
			for(i=0;i<11;i++)
			{
				m = Lists[r+i*4];//��
				VoltageS[m][l]=(float)ADC_ConvertedValue[i]/4096*3.300*1000;
				if(VoltageS[m][l] > Voltage_max)
				{
					Voltage_max = VoltageS[m][l];
					Row = m;
					List = l;
				}
			}
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);//ʹ��ָ����ADC1�����ת����������
		}
	}
	Flag_over = 1;
	times = 0;
}

//=============================================================================
//�ļ����ƣ�Uart1_send()
//���ܸ�Ҫ�����ڷ���
//����˵������
//�������أ���
//=============================================================================
void Uart1_send()
{
	u8 i,j;
	u8 num = 0;
	
	USART_SendData(USART1, 0x55);//����֡ͷ
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	USART_SendData(USART1, 0x11);//����֡ͷ
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	USART_SendData(USART1, 0x11);//����֡ͷ
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	USART_SendData(USART1, 0xE1);//����֡ͷ
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���

	for(i=0; i<52; i++)//��
	{
		for(j=0; j<44; j++)//��
		{
			USART_SendData(USART1, VoltageS[j][i]>>8);//���͸�λ
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			USART_SendData(USART1, VoltageS[j][i]&0xff);//���͵�λ
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
		}
	}
	
	USART_SendData(USART1, 0x00);//����֡β
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	USART_SendData(USART1, 0xAA);//����֡β
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	USART_SendData(USART1, 0x00);//����֡β
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	USART_SendData(USART1, 0xAA);//����֡β
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	
	Tongxin=!Tongxin;
}

//=============================================================================
//�ļ����ƣ�W5500_send()
//���ܸ�Ҫ��w5500���ڷ���
//����˵������
//�������أ���
//=============================================================================
void W5500_send()
{
	u8 i,j;
	u16 num = 0;

	for(i=0; i<52; i++)//��
	{
		for(j=0; j<44; j++)//��
		{
			if(VoltageS[j][i]>Voltage_max-100 && VoltageS[j][i]>100)
			{
				Voltage[0+num*4] = VoltageS[j][i]>>8;//��λ
				Voltage[1+num*4] = VoltageS[j][i]&0xff;//��λ
				Voltage[2+num*4] = j;//��
				Voltage[3+num*4] = i;//��

				num++;
			}
		}
	}
	
//	for(i=0; i<2; i++)//��
//	{
//		for(j=0; j<2; j++)//��
//		{
//				Voltage[0+num*4] = 0;//��λ
//				Voltage[1+num*4] = i*2+j;//��λ
//				Voltage[2+num*4] = 0;//��
//				Voltage[3+num*4] = i*2+j;//��

//				num++;
//		}
//	}
	
	
	Voltage[0+num*4] = num>>8;//������
	Voltage[1+num*4] = num&0xff;//������
	
	do_tcp_client(2+num*4);/*TCP_Client ���ݻػ����Գ���*/  
	
	Tongxin=!Tongxin;
	Voltage_max = 0;
}

/************************************************
�������� �� DMA1_Channel1_IRQHandler(void)
��    �� �� DMA�ж�
��    �� �� ��
�� �� ֵ �� ��
��    �� �� goldking
*************************************************/
void DMA1_Channel1_IRQHandler(void)		//DMA�жϴ��䣬����ͨ����
{
	if(DMA_GetITStatus(DMA1_IT_TC1) != RESET) 
	{
		gc = (times%52)/13;//Ƭ
		lc = times%13;//��
		rc = (times/52)%4;//��
		
		
		L_G1 = List_G1[gc];//Ƭѡ
		L_G2 = List_G2[gc];
		L_G3 = List_G3[gc];
		L_G4 = List_G4[gc];
		
		L_A = List_A[lc];//��ѡ
		L_B = List_B[lc];
		L_C = List_C[lc];
		L_D = List_D[lc];
		
		R_S1 = Row_S1[rc];//��ѡ
		R_S2 = Row_S2[rc];
		R_S3 = Row_S3[rc];
		
		times += 1;
		
		DMA_ClearITPendingBit(DMA1_IT_TC1); 
	}
}

