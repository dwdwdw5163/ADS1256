#include "ads1256.h"
#include "main.h"
#include "spi.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include <stdint.h>
#include <stdio.h>

//***************************
//		Pin assign
//	  STM32			ADS1256
//		PA3	<--- DRDY
//		PA4 ---> CS
//		PA5	---> SCK
//		PA6(MISO)  <--- DOUT
//		PA7(MOSI)  ---> DIN
//***************************

// Define a macro to set a GPIO pin to low
#define LOW(x) HAL_GPIO_WritePin(x##_GPIO_Port, x##_Pin, GPIO_PIN_RESET)

// Define a macro to set a GPIO pin to high
#define HIGH(x) HAL_GPIO_WritePin(x##_GPIO_Port, x##_Pin, GPIO_PIN_SET)

#define ADS1256_DRDY (HAL_GPIO_ReadPin(DRDY_GPIO_Port, DRDY_Pin))

void delay_us(uint32_t n) {
  uint32_t ticks;
  uint32_t told;
  uint32_t tnow;
  uint32_t tcnt = 0;
  uint32_t reload;

  reload = SysTick->LOAD;
  ticks = n * (SystemCoreClock / 1000000); /* 需要的节拍数 */

  tcnt = 0;
  told = SysTick->VAL; /* 刚进入时的计数器值 */

  while (1) {
    tnow = SysTick->VAL;
    if (tnow != told) {
      /* SYSTICK是一个递减的计数器 */
      if (tnow < told) {
        tcnt += told - tnow;
      }
      /* 重新装载递减 */
      else {
        tcnt += reload - tnow + told;
      }
      told = tnow;

      /* 时间超过/等于要延迟的时间,则退出 */
      if (tcnt >= ticks) {
        break;
      }
    }
  }
}

void ADS1256_WaitDRDY() {
  while (ADS1256_DRDY)
    ;
}

uint8_t ADS1256_ReadReg(uint8_t addr) {
  uint8_t read;
  uint8_t tx_buf[2] = {ADS1256_CMD_RREG | addr, 0x00};
  // LOW(SPI1_NSS);
  HAL_SPI_Transmit(&hspi1, tx_buf, 2, 1000);
  delay_us(10); /* 必须延迟才能读取芯片返回数据 */

  HAL_SPI_Receive(&hspi1, &read, 1, 1000);
  // HIGH(SPI1_NSS);

  return read;
}

void ADS1256_WriteReg(uint8_t addr, uint8_t data) {
  uint8_t tx_buf[3] = {ADS1256_CMD_WREG | addr, 0x00, data};
  // LOW(SPI1_NSS);
  HAL_SPI_Transmit(&hspi1, tx_buf, 3, 1000);
  // HIGH(SPI1_NSS);
}

void ADS1256_SendCommand(uint8_t cmd) {
  // LOW(SPI1_NSS);
  HAL_SPI_Transmit(&hspi1, &cmd, 1, 1000);
  // HIGH(SPI1_NSS);
}

uint8_t ADS1256_ReadChipID(void) {
  uint8_t id;
  ADS1256_WaitDRDY();
  id = ADS1256_ReadReg(ADS1256_STATUS);
  return (id >> 4);
}

void ADS1256_Init() {
  HIGH(SPI1_NSS);
  delay_us(10);
  LOW(SPI1_NSS);

  ADS1256_WaitDRDY();
  ADS1256_SendCommand(ADS1256_CMD_REST);

  ADS1256_WaitDRDY();
  ADS1256_WriteReg(ADS1256_STATUS, 0x06);
  ADS1256_WriteReg(ADS1256_ADCON, ADS1256_GAIN_1);
  ADS1256_WriteReg(ADS1256_DRATE, ADS1256_DRATE_2_5SPS);
  ADS1256_WriteReg(ADS1256_MUX, ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM);

  ADS1256_SendCommand(ADS1256_CMD_SELFCAL);
  
  HIGH(SPI1_NSS);
  HAL_Delay(10);
}


uint32_t ADS1256_ReadAdcData(void) {
  uint8_t rx_buf[3] = {0};
  uint32_t read = 0;

  ADS1256_SendCommand(ADS1256_CMD_RDATA); /* 读数据的命令 */

  delay_us(20); /* 必须延迟才能读取芯片返回数据 */

  HAL_SPI_Receive(&hspi1, rx_buf, 3, 1000);

  read = ((uint32_t)rx_buf[0] << 16) | ((uint32_t)rx_buf[1] << 8) | (uint32_t)rx_buf[2];
  return read;
}

int32_t ADS1256_GetAdc(void) {
  uint32_t code = 0;
  LOW(SPI1_NSS);
  ADS1256_WaitDRDY();
  ADS1256_SendCommand(ADS1256_CMD_SYNC);
  ADS1256_SendCommand(ADS1256_CMD_WAKEUP);
  code = ADS1256_ReadAdcData();
  delay_us(20);
  HIGH(SPI1_NSS);
  int32_t data=0;
  if (code&0x800000) {
    data = 0-(code&0x7fffff);
  } else {
    data = code;
  }
  return data;
}
