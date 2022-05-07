#include "DMA_ADC.h"
#include "sys.h"

#define ADC1_DR_Address    ((u32)0x4001244C)//DMA 

__IO u16 ADC_ConvertedValue; 


/************************************************
函数名称 ： ADC_GPIO_Configuration
功    能 ： ADC引脚配置
参    数 ： 无
返 回 值 ： 无
作    者 ： goldking
*************************************************/
void ADC_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;    
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		    //GPIO设置为模拟输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);  

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
}

/************************************************
函数名称 ： ADC_DMA_Configuration
功    能 ： ADC DMA配置
参    数 ： 无
返 回 值 ： 无
作    者 ： goldking
*************************************************/
void ADC_DMA_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	/* 配置DMA中断源*/
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;//使能DMA1_Channel1中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//子优先级0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能DMA1_Channel1中断
  NVIC_Init(&NVIC_InitStructure);//初始化NVIC寄存器  

	/* 配置DMA*/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //使能MDA1时钟
	/* DMA channel1 configuration */
  DMA_DeInit(DMA1_Channel1);  //指定DMA通道
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;////ADC1地址---代表ADC1保存转换值的寄存器
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;	//设置DMA内存地址，ADC转换结果直接放入该地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //外设为设置为数据传输的来源
  DMA_InitStructure.DMA_BufferSize = 11;	//DMA通道的DMA缓存的大小  
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //内存地址自增---总体表示始终从外设ADC1地址处取值---依次保存到连续的两个内存变量中---
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//表示外设数据单元的大小//内存传输数据单元---半字16位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//内存数据大小也为半字
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//循环模式---2个数据依次循环接收从外设ADC1传输过来的ADC值---
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//优先级高
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//不使用内存到内存的传输
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  /* Enable DMA channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);  //使能DMA通道
	
	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
} 

/************************************************
函数名称 ： ADC_Configuration
功    能 ： ADC参数配置
参    数 ： 无
返 回 值 ： 无
作    者 ： goldking
*************************************************/												   
void ADC_Configuration(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE );	  //使能ADC1通道时钟
 
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

	ADC_DeInit(ADC1);  //复位ADC1 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//模数转换工作在单通道模式（还是扫描模式，多通道）
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在连续模式/单次模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 11;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

  /* ADC1 regular channel11 configuration */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5); //通道1采样周期55.5个时钟周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_28Cycles5); //通道2采样周期55.5个时钟周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_28Cycles5); //通道3采样周期55.5个时钟周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_28Cycles5); //通道4采样周期55.5个时钟周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_28Cycles5); //通道4采样周期55.5个时钟周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_28Cycles5); //通道4采样周期55.5个时钟周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_28Cycles5); //通道4采样周期55.5个时钟周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_28Cycles5); //通道4采样周期55.5个时钟周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 9, ADC_SampleTime_28Cycles5); //通道4采样周期55.5个时钟周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 10, ADC_SampleTime_28Cycles5); //通道4采样周期55.5个时钟周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 11, ADC_SampleTime_28Cycles5); //通道4采样周期55.5个时钟周期


  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);	 //ADC向DMA发出请求，请求DMA传输数据 
	
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	
	ADC_ResetCalibration(ADC1);	//使能复位校准  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	
	ADC_StartCalibration(ADC1);	 //开启AD校准
 
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能
}		

/************************************************
函数名称 ： ADC_Initializes
功    能 ： ADC初始化
参    数 ： 无
返 回 值 ： 无
作    者 ： goldking
*************************************************/
void ADC_Initializes(void)
{
  ADC_GPIO_Configuration();
  ADC_DMA_Configuration();
  ADC_Configuration();
}



