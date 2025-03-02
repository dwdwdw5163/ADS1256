/**
 * @file ads1256.c
 * @brief Implementation file for TI ADS1256 ADC driver library for STM32 using HAL
 */

#include "ads1256.h"

/* Private function prototypes */
static void ADS1256_CS_Low(ADS1256_HandleTypeDef* hadc);
static void ADS1256_CS_High(ADS1256_HandleTypeDef* hadc);
static uint8_t ADS1256_IsDRDY(ADS1256_HandleTypeDef* hadc);

/**
 * @brief Initialize the ADS1256 ADC
 * @param hadc: ADS1256 handle structure
 * @param hspi: STM32 HAL SPI handle
 * @param cs_port: Chip select GPIO port
 * @param cs_pin: Chip select GPIO pin
 * @param drdy_port: Data ready GPIO port
 * @param drdy_pin: Data ready GPIO pin
 * @param reset_port: Reset GPIO port (optional, can be NULL)
 * @param reset_pin: Reset GPIO pin (optional, ignored if reset_port is NULL)
 * @return HAL status
 */
HAL_StatusTypeDef ADS1256_Init(ADS1256_HandleTypeDef* hadc, SPI_HandleTypeDef* hspi,
                              GPIO_TypeDef* cs_port, uint16_t cs_pin,
                              GPIO_TypeDef* drdy_port, uint16_t drdy_pin,
                              GPIO_TypeDef* reset_port, uint16_t reset_pin)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    /* Store GPIO information */
    hadc->hspi = hspi;
    hadc->CS_PORT = cs_port;
    hadc->CS_PIN = cs_pin;
    hadc->DRDY_PORT = drdy_port;
    hadc->DRDY_PIN = drdy_pin;
    hadc->RESET_PORT = reset_port;
    hadc->RESET_PIN = reset_pin;
    
    /* Default settings */
    hadc->gain = ADS1256_GAIN_1;
    hadc->data_rate = ADS1256_DRATE_10SPS;
    hadc->buffer_enabled = 0;
    
    /* Set CS high initially */
    ADS1256_CS_High(hadc);
    
    // /* Optional hardware reset */
    // if (hadc->RESET_PORT != NULL) {
    //     HAL_GPIO_WritePin(hadc->RESET_PORT, hadc->RESET_PIN, GPIO_PIN_RESET);
    //     HAL_Delay(10);
    //     HAL_GPIO_WritePin(hadc->RESET_PORT, hadc->RESET_PIN, GPIO_PIN_SET);
    //     HAL_Delay(10);
    // }
    
    /* Software reset */
    status = ADS1256_Reset(hadc);
    if (status != HAL_OK) return status;
    
    /* Wait for DRDY to be low (ready) */
    status = ADS1256_WaitForDataReady(hadc, 1000);
    if (status != HAL_OK) return status;
    
    /* Configure registers */
    
    /* STATUS: Order (MSB first), no ACAL, no BUFEN */
    status = ADS1256_WriteRegister(hadc, ADS1256_REG_STATUS, 0);
    if (status != HAL_OK) return status;
    
    /* ADCON: Set PGA gain */
    status = ADS1256_WriteRegister(hadc, ADS1256_REG_ADCON, hadc->gain);
    if (status != HAL_OK) return status;
    
    /* DRATE: Set data rate */
    status = ADS1256_WriteRegister(hadc, ADS1256_REG_DRATE, hadc->data_rate);
    if (status != HAL_OK) return status;
    
    /* Perform self-calibration */
    status = ADS1256_SelfCal(hadc);
    if (status != HAL_OK) return status;
    
    ADS1256_SendCommand(hadc, ADS1256_CMD_SYNC);
    ADS1256_SendCommand(hadc, ADS1256_CMD_WAKEUP);
    
    return HAL_OK;
}

/**
 * @brief Reset the ADS1256
 * @param hadc: ADS1256 handle structure
 * @return HAL status
 */
HAL_StatusTypeDef ADS1256_Reset(ADS1256_HandleTypeDef* hadc)
{
    HAL_StatusTypeDef status = ADS1256_SendCommand(hadc, ADS1256_CMD_RESET);
    HAL_Delay(10); /* Wait for reset to complete */
    return status;
}

/**
 * @brief Perform self-calibration
 * @param hadc: ADS1256 handle structure
 * @return HAL status
 */
HAL_StatusTypeDef ADS1256_SelfCal(ADS1256_HandleTypeDef* hadc)
{
    HAL_StatusTypeDef status = ADS1256_SendCommand(hadc, ADS1256_CMD_SELFCAL);
    
    /* Wait for calibration to complete (DRDY goes low) */
    return ADS1256_WaitForDataReady(hadc, 1000);
}

/**
 * @brief Set the PGA gain
 * @param hadc: ADS1256 handle structure
 * @param gain: Gain value (ADS1256_GAIN_x)
 * @return HAL status
 */
HAL_StatusTypeDef ADS1256_SetGain(ADS1256_HandleTypeDef* hadc, uint8_t gain)
{
    uint8_t adcon;
    
    /* Read current ADCON register */
    adcon = ADS1256_ReadRegister(hadc, ADS1256_REG_ADCON);
    
    /* Clear gain bits and set new gain */
    adcon &= 0xF8; /* Clear bits 0-2 */
    adcon |= (gain & 0x07); /* Set new gain bits */
    
    /* Write updated ADCON register */
    HAL_StatusTypeDef status = ADS1256_WriteRegister(hadc, ADS1256_REG_ADCON, adcon);
    
    if (status == HAL_OK) {
        hadc->gain = gain;
    }
    
    return status;
}

/**
 * @brief Set the data rate
 * @param hadc: ADS1256 handle structure
 * @param data_rate: Data rate value (ADS1256_DRATE_x)
 * @return HAL status
 */
HAL_StatusTypeDef ADS1256_SetDataRate(ADS1256_HandleTypeDef* hadc, uint8_t data_rate)
{
    HAL_StatusTypeDef status = ADS1256_WriteRegister(hadc, ADS1256_REG_DRATE, data_rate);
    
    if (status == HAL_OK) {
        hadc->data_rate = data_rate;
    }
    
    return status;
}

/**
 * @brief Enable or disable the input buffer
 * @param hadc: ADS1256 handle structure
 * @param enable: 1 to enable, 0 to disable
 * @return HAL status
 */
HAL_StatusTypeDef ADS1256_SetBuffer(ADS1256_HandleTypeDef* hadc, uint8_t enable)
{
    uint8_t status_reg;
    
    /* Read current STATUS register */
    status_reg = ADS1256_ReadRegister(hadc, ADS1256_REG_STATUS);
    
    /* Set or clear BUFEN bit */
    if (enable) {
        status_reg |= ADS1256_STATUS_BUFEN;
    } else {
        status_reg &= ~ADS1256_STATUS_BUFEN;
    }
    
    /* Write updated STATUS register */
    HAL_StatusTypeDef hal_status = ADS1256_WriteRegister(hadc, ADS1256_REG_STATUS, status_reg);
    
    if (hal_status == HAL_OK) {
        hadc->buffer_enabled = enable;
    }
    
    return hal_status;
}

/**
 * @brief Set input channel (single-ended or differential)
 * @param hadc: ADS1256 handle structure
 * @param channel_p: Positive input channel (ADS1256_MUXP_x)
 * @param channel_n: Negative input channel (ADS1256_MUXN_x)
 * @return HAL status
 */
HAL_StatusTypeDef ADS1256_SetChannel(ADS1256_HandleTypeDef* hadc, uint8_t channel_p, uint8_t channel_n)
{
    uint8_t mux_val = channel_p | channel_n;
    return ADS1256_WriteRegister(hadc, ADS1256_REG_MUX, mux_val);
}

/**
 * @brief Set differential input channel pair
 * @param hadc: ADS1256 handle structure
 * @param channel: Channel pair (0-3)
 *                 0: AIN0-AIN1, 1: AIN2-AIN3, 2: AIN4-AIN5, 3: AIN6-AIN7
 * @return HAL status
 */
HAL_StatusTypeDef ADS1256_SetDifferentialChannel(ADS1256_HandleTypeDef* hadc, uint8_t channel)
{
    uint8_t channel_p, channel_n;
    
    switch (channel) {
        case 0:
            channel_p = ADS1256_MUXP_AIN0;
            channel_n = ADS1256_MUXN_AIN1;
            break;
        case 1:
            channel_p = ADS1256_MUXP_AIN2;
            channel_n = ADS1256_MUXN_AIN3;
            break;
        case 2:
            channel_p = ADS1256_MUXP_AIN4;
            channel_n = ADS1256_MUXN_AIN5;
            break;
        case 3:
            channel_p = ADS1256_MUXP_AIN6;
            channel_n = ADS1256_MUXN_AIN7;
            break;
        default:
            return HAL_ERROR;
    }
    
    return ADS1256_SetChannel(hadc, channel_p, channel_n);
}

/**
 * @brief Wait for DRDY signal to go low (data ready)
 * @param hadc: ADS1256 handle structure
 * @param timeout: Timeout in milliseconds
 * @return HAL status
 */
HAL_StatusTypeDef ADS1256_WaitForDataReady(ADS1256_HandleTypeDef* hadc, uint32_t timeout)
{
    uint32_t start_tick = HAL_GetTick();
    
    while (ADS1256_IsDRDY(hadc) != 0) {
        if (HAL_GetTick() - start_tick > timeout) {
            return HAL_TIMEOUT;
        }
    }
    
    return HAL_OK;
}

/**
 * @brief Read ADC data
 * @param hadc: ADS1256 handle structure
 * @return 24-bit signed data
 */
int32_t ADS1256_ReadData(ADS1256_HandleTypeDef* hadc)
{
    int32_t data = 0;
    uint8_t buffer[3];
    
    /* Send RDATA command */
    // ADS1256_SendCommand(hadc, ADS1256_CMD_SYNC);
    // ADS1256_SendCommand(hadc, ADS1256_CMD_WAKEUP);
    ADS1256_SendCommand(hadc, ADS1256_CMD_RDATA);
    
    /* Wait for t6 delay (50*tCLKIN) */
    HAL_Delay(1);
    
    /* Read 3 bytes of data */
    ADS1256_CS_Low(hadc);
    HAL_SPI_Receive(hadc->hspi, buffer, 3, 100);
    ADS1256_CS_High(hadc);
    
    /* Convert to 24-bit signed value */
    data = ((int32_t)buffer[0] << 16) | ((int32_t)buffer[1] << 8) | buffer[2];
    
    /* Sign extend to 32 bits if negative */
    if (data & 0x800000) {
        data |= 0xFF000000;
    }
    
    return data;
}

/**
 * @brief Read a single channel
 * @param hadc: ADS1256 handle structure
 * @param channel: Channel number (0-7)
 * @param data: Pointer to store the read data
 * @return HAL status
 */
HAL_StatusTypeDef ADS1256_ReadChannel(ADS1256_HandleTypeDef* hadc, uint8_t channel, int32_t* data)
{
    uint8_t channel_p, channel_n;
    HAL_StatusTypeDef status;
    
    if (channel > 7) return HAL_ERROR;
    
    /* Set channel_p to selected channel, channel_n to AINCOM */
    channel_p = channel << 4;
    channel_n = ADS1256_MUXN_AINCOM;
    
    /* Set mux channel */
    status = ADS1256_SetChannel(hadc, channel_p, channel_n);
    if (status != HAL_OK) return status;
    
    /* Wait for settling time and DRDY */
    HAL_Delay(5);
    status = ADS1256_WaitForDataReady(hadc, 1000);
    if (status != HAL_OK) return status;
    
    /* Read data */
    *data = ADS1256_ReadData(hadc);
    
    return HAL_OK;
}

/**
 * @brief Read a differential channel pair
 * @param hadc: ADS1256 handle structure
 * @param channel: Differential channel pair (0-3)
 *                 0: AIN0-AIN1, 1: AIN2-AIN3, 2: AIN4-AIN5, 3: AIN6-AIN7
 * @param data: Pointer to store the read data
 * @return HAL status
 */
HAL_StatusTypeDef ADS1256_ReadDifferentialChannel(ADS1256_HandleTypeDef* hadc, uint8_t channel, int32_t* data)
{
    HAL_StatusTypeDef status;
    
    /* Set differential channel */
    status = ADS1256_SetDifferentialChannel(hadc, channel);
    if (status != HAL_OK) return status;
    
    /* Wait for settling time and DRDY */
    HAL_Delay(5);
    status = ADS1256_WaitForDataReady(hadc, 1000);
    if (status != HAL_OK) return status;
    
    /* Read data */
    *data = ADS1256_ReadData(hadc);
    
    return HAL_OK;
}

/**
 * @brief Write to ADS1256 register
 * @param hadc: ADS1256 handle structure
 * @param reg: Register address
 * @param value: Value to write
 * @return HAL status
 */
HAL_StatusTypeDef ADS1256_WriteRegister(ADS1256_HandleTypeDef* hadc, uint8_t reg, uint8_t value)
{
    uint8_t buffer[3];
    HAL_StatusTypeDef status;
    
    /* Prepare command byte: 0101 rrrr where rrrr is the register address */
    buffer[0] = ADS1256_CMD_WREG | (reg & 0x0F);
    /* Number of registers to write - 1 */
    buffer[1] = 0x00;
    /* Register value */
    buffer[2] = value;
    
    /* Send command and data */
    ADS1256_CS_Low(hadc);
    status = HAL_SPI_Transmit(hadc->hspi, buffer, 3, 100);
    ADS1256_CS_High(hadc);
    
    /* Wait for the operation to complete (tSCCS) */
    HAL_Delay(1);
    
    return status;
}

/**
 * @brief Read from ADS1256 register
 * @param hadc: ADS1256 handle structure
 * @param reg: Register address
 * @return Register value
 */
uint8_t ADS1256_ReadRegister(ADS1256_HandleTypeDef* hadc, uint8_t reg)
{
    uint8_t cmd[2];
    uint8_t value;
    
    /* Prepare command byte: 0001 rrrr where rrrr is the register address */
    cmd[0] = ADS1256_CMD_RREG | (reg & 0x0F);
    /* Number of registers to read - 1 */
    cmd[1] = 0x00;
    
    /* Send command */
    ADS1256_CS_Low(hadc);
    HAL_SPI_Transmit(hadc->hspi, cmd, 2, 100);
    
    /* Wait for t6 delay (50*tCLKIN) */
    HAL_Delay(1);
    
    /* Read register value */
    HAL_SPI_Receive(hadc->hspi, &value, 1, 100);
    ADS1256_CS_High(hadc);
    
    /* Wait for the operation to complete (tSCCS) */
    HAL_Delay(1);
    
    return value;
}

/**
 * @brief Send command to ADS1256
 * @param hadc: ADS1256 handle structure
 * @param cmd: Command byte
 * @return HAL status
 */
HAL_StatusTypeDef ADS1256_SendCommand(ADS1256_HandleTypeDef* hadc, uint8_t cmd)
{
    HAL_StatusTypeDef status;
    
    ADS1256_CS_Low(hadc);
    status = HAL_SPI_Transmit(hadc->hspi, &cmd, 1, 100);
    ADS1256_CS_High(hadc);
    
    /* Wait for the operation to complete (tSCCS) */
    HAL_Delay(1);
    
    return status;
}

/* Private helper functions */

/**
 * @brief Set CS pin low (select device)
 * @param hadc: ADS1256 handle structure
 */
static void ADS1256_CS_Low(ADS1256_HandleTypeDef* hadc)
{
    HAL_GPIO_WritePin(hadc->CS_PORT, hadc->CS_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Set CS pin high (deselect device)
 * @param hadc: ADS1256 handle structure
 */
static void ADS1256_CS_High(ADS1256_HandleTypeDef* hadc)
{
    HAL_GPIO_WritePin(hadc->CS_PORT, hadc->CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief Check if DRDY pin is low (data ready)
 * @param hadc: ADS1256 handle structure
 * @return 0 if DRDY is low (data ready), non-zero otherwise
 */
static uint8_t ADS1256_IsDRDY(ADS1256_HandleTypeDef* hadc)
{
    return HAL_GPIO_ReadPin(hadc->DRDY_PORT, hadc->DRDY_PIN);
}

