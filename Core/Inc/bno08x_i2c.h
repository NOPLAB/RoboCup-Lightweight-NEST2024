#if !defined(BNO08X_I2C)
#define BNO08X_I2C

#include "bno08x_hal.h"

#include <sh2_hal.h>
#include <stdbool.h>

#define RESET_DELAY_US (10000)
#define START_DELAY_US (4000000)
#define HDR_READ_LEN (4)
#define I2C_WRITE_DELAY_US (10)
#define MAX_RETRIES (5)

typedef enum bno08x_i2c_addr_e
{
    BNO08X_I2C_ADDR1 = 0x4A,
    BNO08X_I2C_ADDR2 = 0x4B,
} bno08x_i2c_addr_t;

typedef struct i2c_config_s
{
    TIM_HandleTypeDef *tim;
    I2C_HandleTypeDef *i2c;
    bno08x_i2c_addr_t i2c_addr;
    void (*set_rst)(bool);
    void (*set_boot)(bool);
    void (*set_ints)(bool);
    void (*set_i2c_ints)(bool);
} i2c_config_t;

typedef enum bus_state_e
{
    BUS_INIT,
    BUS_IDLE,
    BUS_READING_LEN,
    BUS_GOT_LEN,
    BUS_READING_TRANSFER,
    BUS_WRITING,
    BUS_READING_DFU,
    BUS_WRITING_DFU,
} bus_state_t;

typedef struct i2c_handle_s
{
    i2c_config_t *config;

    bool is_open;

    bool in_reset;

    bus_state_t bus_state;

    uint8_t rx_buf[SH2_HAL_MAX_TRANSFER_IN];
    uint32_t rx_buf_len;

    uint8_t hdr_buf[HDR_READ_LEN];
    uint32_t hdr_buf_len;

    uint8_t tx_buf[SH2_HAL_MAX_TRANSFER_OUT];
    uint32_t tx_buf_len;

    bool rx_data_ready;

    uint32_t int_last_timestamp_us;

    uint16_t read_retries;

    uint32_t payload_len;
} i2c_handle_t;

typedef struct i2c_hal_s
{
    sh2_Hal_t sh2_hal;
    i2c_handle_t *i2c_handle;

} i2c_hal_t;

i2c_hal_t *shtp_i2c_hal_init(i2c_config_t *config);

void IMPL_HAL_GPIO_EXTI_Callback(uint16_t n, i2c_hal_t *i2c_hal);
void IMPL_HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *pI2c, i2c_hal_t *i2c_hal);
void IMPL_HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *i2c, i2c_hal_t *i2c_hal);
void IMPL_HAL_I2C_ErrorCallback(I2C_HandleTypeDef *i2c, i2c_hal_t *i2c_hal);

#endif // BNO08X_I2C
