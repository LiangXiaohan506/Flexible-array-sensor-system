/**
******************************************************************************
* @file    			main.c
* @author  			WIZnet Software Team
* @version 			V1.0
* @date    			2015-02-14
* @brief   			用3.5.0版本库建的工程模板
* @attention  	实验平台：野火 iSO-V3 STM32 开发板 + 野火W5500网络适配板
*
*               默认使用野火开发板的SPI2接口
*							
*               内网测试，请保证W5500的IP与测试PC机在同一网段内，且不冲突
*               如果是用网线跟PC机直连，请设置PC机本地连接地址IP为静态IP
*
* 实验平台:野火 iSO-V3 STM32 开发板 
* 论坛    :http://www.firebbs.cn
* 淘宝    :https://fire-stm32.taobao.com
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

/////////////////////////引脚定义///////////////////
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

/////////////////////////变量定义///////////////////
u8 gc=0;//片选标志
u8 lc=0;//列选标志
u8 rc=0;//行选标志
u8 r=0;//行数统计
u8 l=0;//列数统计
u8 Flag_over = 0;//采集完成标志位
	
u16 times=0;//进入DMA中断的次数

extern __IO u16 ADC_ConvertedValue[11];//DMA存储数据的容器

extern u8 Res;

extern uint8 Voltage[8192];//电压值及位置信息缓存

u16 VoltageS[44][52];//阵列传感器的数据
 
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
								
/////////////////////////函数定义///////////////////
void LED_Init(void);//LED初始化
void GPIO_Configuration(void);//端口初始化
void DMA_Adc_collect();//DMA_adc采集函数
void Uart1_send();//串口发送函数
void W5500_send();//w5500发送函数
								
int main(void)
{ 	
	systick_init(72);										/*初始化Systick工作时钟*/
	USART1_Config(921600);							/*初始化串口通信:115200@8-n-1*/
	i2c_CfgGpio();											/*初始化eeprom*/

//	printf("  野火网络适配版 网络初始化 Demo V1.0 \r\n");		

	gpio_for_w5500_config();						/*初始化MCU相关引脚*/
	reset_w5500();											/*硬复位W5500*/
	set_w5500_mac();										/*配置MAC地址*/
	set_w5500_ip();											/*配置IP地址*/
	
	socket_buf_init(txsize, rxsize);		/*初始化8个Socket的发送接收缓存大小*/
	
	LED_Init();

	ADC_Initializes();//DMA+ADC
	
	GPIO_Configuration();
	
//  printf(" 电脑作为TCP服务器,让W5500作为 TCP客户端去连接 \r\n");
//	printf(" 服务器IP:%d.%d.%d.%d\r\n",remote_ip[0],remote_ip[1],remote_ip[2],remote_ip[3]);
//	printf(" 监听端口:%d \r\n",remote_port);
//	printf(" 连接成功后，服务器发送数据给W5500，W5500将返回对应数据 \r\n");
//	printf(" 应用程序执行中……\r\n"); 
	
	while(1) 														/*循环执行的函数*/ 
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

//初始化PB5和PE5为输出口.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //使能PC端口时钟
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				 //RUN-->PC5 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化GPIOC5
 GPIO_SetBits(GPIOC,GPIO_Pin_5);						 //PC5 输出高

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	    		 //Tongxin-->PC4 端口配置, 推挽输出
 GPIO_Init(GPIOC, &GPIO_InitStructure);	  				 //推挽输出 ，IO口速度为50MHz
 GPIO_SetBits(GPIOC,GPIO_Pin_4); 						 //PC4 输出高 
}

//=============================================================================
//文件名称：GPIO_Configuration
//功能概要：GPIO初始化
//参数说明：无
//函数返回：无
//=============================================================================
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE); //使能PB端口时钟	
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); /*使能SWD 禁用JTAG*/
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;//列选端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO口速度为50MHz
  GPIO_Init(GPIOC, &GPIO_InitStructure);//根据设定参数初始化GPIOB
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//列选端口配置
  GPIO_Init(GPIOA, &GPIO_InitStructure);//根据设定参数初始化GPIOA
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;//片选端口配置
  GPIO_Init(GPIOB, &GPIO_InitStructure);//根据设定参数初始化GPIOB
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;//行选端口配置
  GPIO_Init(GPIOB, &GPIO_InitStructure);//根据设定参数初始化GPIOB
}

//=============================================================================
//文件名称：DMA_Adc_collect()
//功能概要：读取DMA_AD值
//参数说明：无
//函数返回：无
//=============================================================================
void DMA_Adc_collect()
{
	u8 i,m,n,k;

	for(r=0; r<4; r++)//行
	{
		for(l=0; l<52; l++)//列
		{
			for(i=0;i<11;i++)
			{
				m = Lists[r+i*4];//行
				VoltageS[m][l]=(float)ADC_ConvertedValue[i]/4096*3.300*1000;
				if(VoltageS[m][l] > Voltage_max)
				{
					Voltage_max = VoltageS[m][l];
					Row = m;
					List = l;
				}
			}
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);//使能指定的ADC1的软件转换启动功能
		}
	}
	Flag_over = 1;
	times = 0;
}

//=============================================================================
//文件名称：Uart1_send()
//功能概要：串口发送
//参数说明：无
//函数返回：无
//=============================================================================
void Uart1_send()
{
	u8 i,j;
	u8 num = 0;
	
	USART_SendData(USART1, 0x55);//发送帧头
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	USART_SendData(USART1, 0x11);//发送帧头
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	USART_SendData(USART1, 0x11);//发送帧头
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	USART_SendData(USART1, 0xE1);//发送帧头
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束

	for(i=0; i<52; i++)//列
	{
		for(j=0; j<44; j++)//行
		{
			USART_SendData(USART1, VoltageS[j][i]>>8);//发送高位
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
			USART_SendData(USART1, VoltageS[j][i]&0xff);//发送低位
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
		}
	}
	
	USART_SendData(USART1, 0x00);//发送帧尾
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	USART_SendData(USART1, 0xAA);//发送帧尾
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	USART_SendData(USART1, 0x00);//发送帧尾
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	USART_SendData(USART1, 0xAA);//发送帧尾
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	
	Tongxin=!Tongxin;
}

//=============================================================================
//文件名称：W5500_send()
//功能概要：w5500网口发送
//参数说明：无
//函数返回：无
//=============================================================================
void W5500_send()
{
	u8 i,j;
	u16 num = 0;

	for(i=0; i<52; i++)//列
	{
		for(j=0; j<44; j++)//行
		{
			if(VoltageS[j][i]>Voltage_max-100 && VoltageS[j][i]>100)
			{
				Voltage[0+num*4] = VoltageS[j][i]>>8;//高位
				Voltage[1+num*4] = VoltageS[j][i]&0xff;//低位
				Voltage[2+num*4] = j;//行
				Voltage[3+num*4] = i;//列

				num++;
			}
		}
	}
	
//	for(i=0; i<2; i++)//列
//	{
//		for(j=0; j<2; j++)//行
//		{
//				Voltage[0+num*4] = 0;//高位
//				Voltage[1+num*4] = i*2+j;//低位
//				Voltage[2+num*4] = 0;//行
//				Voltage[3+num*4] = i*2+j;//列

//				num++;
//		}
//	}
	
	
	Voltage[0+num*4] = num>>8;//数据量
	Voltage[1+num*4] = num&0xff;//数据量
	
	do_tcp_client(2+num*4);/*TCP_Client 数据回环测试程序*/  
	
	Tongxin=!Tongxin;
	Voltage_max = 0;
}

/************************************************
函数名称 ： DMA1_Channel1_IRQHandler(void)
功    能 ： DMA中断
参    数 ： 无
返 回 值 ： 无
作    者 ： goldking
*************************************************/
void DMA1_Channel1_IRQHandler(void)		//DMA中断传输，更改通道。
{
	if(DMA_GetITStatus(DMA1_IT_TC1) != RESET) 
	{
		gc = (times%52)/13;//片
		lc = times%13;//列
		rc = (times/52)%4;//行
		
		
		L_G1 = List_G1[gc];//片选
		L_G2 = List_G2[gc];
		L_G3 = List_G3[gc];
		L_G4 = List_G4[gc];
		
		L_A = List_A[lc];//列选
		L_B = List_B[lc];
		L_C = List_C[lc];
		L_D = List_D[lc];
		
		R_S1 = Row_S1[rc];//行选
		R_S2 = Row_S2[rc];
		R_S3 = Row_S3[rc];
		
		times += 1;
		
		DMA_ClearITPendingBit(DMA1_IT_TC1); 
	}
}

