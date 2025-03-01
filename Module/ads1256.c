#include "ads1256.h"
#include "spi.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include <stdint.h>
#include "main.h"



//***************************
//		Pin assign	   	
//	  STM32			ADS1256
//		PA3	<--- DRDY
//		PA4 ---> CS
//		PA5	---> SCK
//		PA6(MISO)  <--- DOUT
//		PA7(MOSI)  ---> DIN
//***************************	

#define CS_0() HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, 0)
#define CS_1() HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, 1)

#define ADS1256_DRDY HAL_GPIO_ReadPin(DRDY_GPIO_Port, DRDY_Pin)

uint8_t SPI_WriteByte(uint8_t TxData)
{
  uint8_t RxData;
  // while(!__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE));
  HAL_SPI_Transmit(&hspi1, &TxData, 1, 1000);
  // while(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE));
  HAL_SPI_Receive(&hspi1, &RxData, 1, 1000);
  return RxData;
} 

//-----------------------------------------------------------------//
//	功    能：ADS1256 写数据
//	入口参数: /
//	出口参数: /
//	全局变量: /
//	备    注: 向ADS1256中地址为regaddr的寄存器写入一个字节databyte
//-----------------------------------------------------------------//
void ADS1256WREG(uint8_t regaddr,uint8_t databyte)
{
    
	CS_0();
  while(ADS1256_DRDY);
	//向寄存器写入数据地址
  SPI_WriteByte(ADS1256_CMD_WREG | (regaddr & 0x0F));
  //写入数据的个数n-1
  SPI_WriteByte(0x00);
  //向regaddr地址指向的寄存器写入数据databyte
  SPI_WriteByte(databyte);
	CS_1();
}


//初始化ADS1256
void ADS1256_Init(void)
{
	CS_1();
	HAL_Delay(10);
	//*************自校准****************
  while(ADS1256_DRDY);
	CS_0();
	SPI_WriteByte(ADS1256_CMD_SELFCAL);
	while(ADS1256_DRDY);
	CS_1();
	//**********************************

	ADS1256WREG(ADS1256_STATUS,0x06);               // 高位在前、使用缓冲
//	ADS1256WREG(ADS1256_STATUS,0x04);               // 高位在前、不使用缓冲

//	ADS1256WREG(ADS1256_MUX,0x08);                  // 初始化端口A0为‘+’，AINCOM位‘-’
	ADS1256WREG(ADS1256_ADCON,ADS1256_GAIN_1);                // 放大倍数1
	ADS1256WREG(ADS1256_DRATE,ADS1256_DRATE_2_5SPS);  // 数据10sps
	ADS1256WREG(ADS1256_IO,0x00);              

	//*************自校准****************
	while(ADS1256_DRDY);
	CS_0();
	SPI_WriteByte(ADS1256_CMD_SELFCAL);
	while(ADS1256_DRDY);
	CS_1(); 
	//**********************************
}

//读取AD值
int32_t ADS1256ReadData(uint8_t channel)  
{

  uint32_t sum=0;
	
	while(ADS1256_DRDY);//当ADS1256_DRDY为低时才能写寄存器 
	ADS1256WREG(ADS1256_MUX,channel);		//设置通道
	CS_0();
	SPI_WriteByte(ADS1256_CMD_SYNC);
	SPI_WriteByte(ADS1256_CMD_WAKEUP);	               
	SPI_WriteByte(ADS1256_CMD_RDATA);
  sum |= (SPI_WriteByte(0xff) << 16);
	sum |= (SPI_WriteByte(0xff) << 8);
	sum |= SPI_WriteByte(0xff);
	CS_1();

	if (sum>0x7FFFFF)           // if MSB=1, 
	{
		sum -= 0x1000000;       // do 2's complement

	}
    return sum;
}
