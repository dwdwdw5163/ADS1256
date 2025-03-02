/**
 * @file ads1256.h
 * @brief Header file for TI ADS1256 ADC driver library for STM32 using HAL
 */

#ifndef ADS1256_H
#define ADS1256_H

#include "stm32f4xx_hal.h" // Change to your specific STM32 family

/* ADS1256 Commands */
#define ADS1256_CMD_WAKEUP    0x00
#define ADS1256_CMD_RDATA     0x01
#define ADS1256_CMD_RDATAC    0x03
#define ADS1256_CMD_SDATAC    0x0F
#define ADS1256_CMD_RREG      0x10
#define ADS1256_CMD_WREG      0x50
#define ADS1256_CMD_SELFCAL   0xF0
#define ADS1256_CMD_SELFOCAL  0xF1
#define ADS1256_CMD_SELFGCAL  0xF2
#define ADS1256_CMD_SYSOCAL   0xF3
#define ADS1256_CMD_SYSGCAL   0xF4
#define ADS1256_CMD_SYNC      0xFC
#define ADS1256_CMD_STANDBY   0xFD
#define ADS1256_CMD_RESET     0xFE

/* ADS1256 Register Addresses */
#define ADS1256_REG_STATUS    0x00
#define ADS1256_REG_MUX       0x01
#define ADS1256_REG_ADCON     0x02
#define ADS1256_REG_DRATE     0x03
#define ADS1256_REG_IO        0x04
#define ADS1256_REG_OFC0      0x05
#define ADS1256_REG_OFC1      0x06
#define ADS1256_REG_OFC2      0x07
#define ADS1256_REG_FSC0      0x08
#define ADS1256_REG_FSC1      0x09
#define ADS1256_REG_FSC2      0x0A

/* Data Rates */
#define ADS1256_DRATE_30000SPS   0xF0
#define ADS1256_DRATE_15000SPS   0xE0
#define ADS1256_DRATE_7500SPS    0xD0
#define ADS1256_DRATE_3750SPS    0xC0
#define ADS1256_DRATE_2000SPS    0xB0
#define ADS1256_DRATE_1000SPS    0xA1
#define ADS1256_DRATE_500SPS     0x92
#define ADS1256_DRATE_100SPS     0x82
#define ADS1256_DRATE_60SPS      0x72
#define ADS1256_DRATE_50SPS      0x63
#define ADS1256_DRATE_30SPS      0x53
#define ADS1256_DRATE_25SPS      0x43
#define ADS1256_DRATE_15SPS      0x33
#define ADS1256_DRATE_10SPS      0x23
#define ADS1256_DRATE_5SPS       0x13
#define ADS1256_DRATE_2_5SPS     0x03

/* PGA Gain Settings */
#define ADS1256_GAIN_1        0x00
#define ADS1256_GAIN_2        0x01
#define ADS1256_GAIN_4        0x02
#define ADS1256_GAIN_8        0x03
#define ADS1256_GAIN_16       0x04
#define ADS1256_GAIN_32       0x05
#define ADS1256_GAIN_64       0x06

/* Input Channel Selection for MUX register */
#define ADS1256_MUXP_AIN0     0x00
#define ADS1256_MUXP_AIN1     0x10
#define ADS1256_MUXP_AIN2     0x20
#define ADS1256_MUXP_AIN3     0x30
#define ADS1256_MUXP_AIN4     0x40
#define ADS1256_MUXP_AIN5     0x50
#define ADS1256_MUXP_AIN6     0x60
#define ADS1256_MUXP_AIN7     0x70
#define ADS1256_MUXP_AINCOM   0x80

#define ADS1256_MUXN_AIN0     0x00
#define ADS1256_MUXN_AIN1     0x01
#define ADS1256_MUXN_AIN2     0x02
#define ADS1256_MUXN_AIN3     0x03
#define ADS1256_MUXN_AIN4     0x04
#define ADS1256_MUXN_AIN5     0x05
#define ADS1256_MUXN_AIN6     0x06
#define ADS1256_MUXN_AIN7     0x07
#define ADS1256_MUXN_AINCOM   0x08

/* ADS1256 Status Register Bits */
#define ADS1256_STATUS_ORDER  (1 << 3)
#define ADS1256_STATUS_ACAL   (1 << 2)
#define ADS1256_STATUS_BUFEN  (1 << 1)
#define ADS1256_STATUS_DRDY   (1 << 0)

/* ADS1256 ADCON Register Bits */
#define ADS1256_ADCON_CLKOUT  (1 << 5)
#define ADS1256_ADCON_SDCS    (1 << 3)

typedef struct {
    SPI_HandleTypeDef* hspi;     // SPI handle
    GPIO_TypeDef* CS_PORT;       // Chip Select port
    uint16_t CS_PIN;             // Chip Select pin
    GPIO_TypeDef* DRDY_PORT;     // Data Ready port
    uint16_t DRDY_PIN;           // Data Ready pin
    GPIO_TypeDef* RESET_PORT;    // Reset port (optional)
    uint16_t RESET_PIN;          // Reset pin (optional)
    uint8_t gain;                // Current gain setting
    uint8_t data_rate;           // Current data rate
    uint8_t buffer_enabled;      // Buffer state
} ADS1256_HandleTypeDef;

/* Function Prototypes */
HAL_StatusTypeDef ADS1256_Init(ADS1256_HandleTypeDef* hadc, SPI_HandleTypeDef* hspi,
                              GPIO_TypeDef* cs_port, uint16_t cs_pin,
                              GPIO_TypeDef* drdy_port, uint16_t drdy_pin,
                              GPIO_TypeDef* reset_port, uint16_t reset_pin);

HAL_StatusTypeDef ADS1256_Reset(ADS1256_HandleTypeDef* hadc);
HAL_StatusTypeDef ADS1256_SelfCal(ADS1256_HandleTypeDef* hadc);
HAL_StatusTypeDef ADS1256_SetGain(ADS1256_HandleTypeDef* hadc, uint8_t gain);
HAL_StatusTypeDef ADS1256_SetDataRate(ADS1256_HandleTypeDef* hadc, uint8_t data_rate);
HAL_StatusTypeDef ADS1256_SetBuffer(ADS1256_HandleTypeDef* hadc, uint8_t enable);
HAL_StatusTypeDef ADS1256_SetChannel(ADS1256_HandleTypeDef* hadc, uint8_t channel_p, uint8_t channel_n);
HAL_StatusTypeDef ADS1256_SetDifferentialChannel(ADS1256_HandleTypeDef* hadc, uint8_t channel);
HAL_StatusTypeDef ADS1256_WaitForDataReady(ADS1256_HandleTypeDef* hadc, uint32_t timeout);
int32_t ADS1256_ReadData(ADS1256_HandleTypeDef* hadc);
HAL_StatusTypeDef ADS1256_ReadChannel(ADS1256_HandleTypeDef* hadc, uint8_t channel, int32_t* data);
HAL_StatusTypeDef ADS1256_ReadDifferentialChannel(ADS1256_HandleTypeDef* hadc, uint8_t channel, int32_t* data);

/* Low-level functions */
HAL_StatusTypeDef ADS1256_WriteRegister(ADS1256_HandleTypeDef* hadc, uint8_t reg, uint8_t value);
uint8_t ADS1256_ReadRegister(ADS1256_HandleTypeDef* hadc, uint8_t reg);
HAL_StatusTypeDef ADS1256_SendCommand(ADS1256_HandleTypeDef* hadc, uint8_t cmd);

#endif /* ADS1256_H */

