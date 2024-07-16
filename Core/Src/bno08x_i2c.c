#include "bno08x_i2c.h"

#include <stdio.h>
#include <stdlib.h>
#include <sh2_err.h>
#include <memory.h>

static uint32_t timeNowUs(TIM_HandleTypeDef *tim) {
	return __HAL_TIM_GET_COUNTER(tim);
}

static void delay_us(TIM_HandleTypeDef *tim, uint32_t t) {
	uint32_t now = timeNowUs(tim);
	uint32_t start = now;
	while ((now - start) < t) {
		now = timeNowUs(tim);
	}
}

static void wait_int_delay_us(TIM_HandleTypeDef *tim, uint32_t t,
bool *in_reset) {
	uint32_t now = timeNowUs(tim);
	uint32_t start = now;
	while (((now - start) < t) && (in_reset)) {
		now = timeNowUs(tim);
	}
}

static int shtp_i2c_hal_open(sh2_Hal_t *self_) {
	i2c_hal_t *self = (i2c_hal_t*) self_;
	i2c_handle_t *handle = self->i2c_handle;
	i2c_config_t *config = handle->config;

	if (handle->is_open)
		return SH2_ERR;

	handle->bus_state = BUS_INIT;

	handle->is_open = true;

	config->set_rst(false);

	handle->in_reset = true;

	handle->bus_state = BUS_IDLE;

	handle->rx_buf_len = 0;
	handle->hdr_buf_len = 0;
	handle->rx_data_ready = false;

	config->set_boot(true);

	delay_us(config->tim, RESET_DELAY_US);

	config->set_ints(true);

	config->set_rst(true);

	// reset_delay_us
	wait_int_delay_us(config->tim, START_DELAY_US, &(handle->in_reset));

	config->set_boot(true);

	printf("shtp opened\n");

	return SH2_OK;
}

static void shtp_i2c_hal_close(sh2_Hal_t *self_) {
	i2c_hal_t *self = (i2c_hal_t*) self_;
	i2c_handle_t *handle = self->i2c_handle;
	i2c_config_t *config = handle->config;

	handle->bus_state = BUS_INIT;
	delay_us(config->tim, 1000); // Give any in-flight I2C operations a chance to finish.

	// Hold sensor hub in reset
	config->set_rst(false);
	config->set_boot(true);

	// Deinit I2C peripheral
	HAL_I2C_DeInit(config->i2c);

	// Disable interrupts
	config->set_ints(false);

	// Deinit timer
	__HAL_TIM_DISABLE(config->tim);

	handle->is_open = false;
}

static int shtp_i2c_hal_read(sh2_Hal_t *self_, uint8_t *pBuffer, unsigned len,
		uint32_t *t) {
	i2c_hal_t *self = (i2c_hal_t*) self_;
	i2c_handle_t *handle = self->i2c_handle;
	i2c_config_t *config = handle->config;

	int retval = 0;

	config->set_ints(false);

	// read bus state just once for consistency.
	bus_state_t bus_state = handle->bus_state;

	if (handle->hdr_buf_len > 0) {
		// There is data in handle->hdr_buf to return to SHTP layer
		if (len < handle->hdr_buf_len) {
			// Client buffer too small!
			// Discard what was read
			handle->hdr_buf_len = 0;
			retval = SH2_ERR_BAD_PARAM;
		} else {
			// Copy data to the client buffer
			memcpy(pBuffer, handle->hdr_buf, handle->hdr_buf_len);
			retval = handle->hdr_buf_len;
			handle->hdr_buf_len = 0;
			*t = handle->int_last_timestamp_us;
		}
	} else if (handle->rx_buf_len > 0) {
		// There is data in handle->rx_buf to return to SHTP layer
		if (len < handle->rx_buf_len) {
			// Client buffer too small!
			// Discard what was read
			handle->rx_buf_len = 0;
			retval = SH2_ERR_BAD_PARAM;
		} else {
			// Copy data to the client buffer
			memcpy(pBuffer, handle->rx_buf, handle->rx_buf_len);
			retval = handle->rx_buf_len;
			handle->rx_buf_len = 0;
			*t = handle->int_last_timestamp_us;
		}
	}
	config->set_ints(true);

	// if sensor hub asserted INTN, data is ready
	if (handle->rx_data_ready) {
		if ((bus_state == BUS_IDLE) && (handle->hdr_buf_len == 0)) {
			handle->read_retries = 0;
			handle->rx_data_ready = false;
			handle->bus_state = BUS_READING_LEN;
			HAL_I2C_Master_Receive_IT(config->i2c, config->i2c_addr,
					handle->hdr_buf, HDR_READ_LEN);
		} else if ((bus_state == BUS_GOT_LEN) && (handle->rx_buf_len == 0)) {
			// Copy the header from handle->rx_buf to pBuffer.  retval = READ_LEN
			memcpy(pBuffer, handle->hdr_buf, HDR_READ_LEN);
			retval = HDR_READ_LEN;
			handle->hdr_buf_len = 0;
			*t = handle->int_last_timestamp_us;

			handle->read_retries = 0;
			handle->rx_data_ready = false;
			handle->bus_state = BUS_READING_TRANSFER;
			HAL_I2C_Master_Receive_IT(config->i2c, config->i2c_addr,
					handle->rx_buf, handle->payload_len);
		}
	}

	return retval;
}

static int shtp_i2c_hal_write(sh2_Hal_t *self_, uint8_t *pBuffer, unsigned len) {
	i2c_hal_t *self = (i2c_hal_t*) self_;
	i2c_handle_t *handle = self->i2c_handle;
	i2c_config_t *config = handle->config;

	int retval = 0;

	// Validate parameters
	if ((pBuffer == 0) || (len == 0) || (len > sizeof(handle->tx_buf))) {
		return SH2_ERR_BAD_PARAM;
	}

	// Disable I2C Interrupt for a moment so busState can't change
	config->set_i2c_ints(false);

	if (handle->bus_state == BUS_IDLE) {
		handle->bus_state = BUS_WRITING;

		// Set up write operation
		handle->read_retries = 0;
		handle->tx_buf_len = len;
		memcpy(handle->tx_buf, pBuffer, len);
		delay_us(config->tim, I2C_WRITE_DELAY_US);
		HAL_I2C_Master_Transmit_IT(config->i2c, config->i2c_addr,
				handle->tx_buf, handle->tx_buf_len);

		retval = len;
	}

	// re-enable interrupts
	config->set_i2c_ints(true);

	return retval;
}

static uint32_t shtp_i2c_getTimeUs(sh2_Hal_t *self_) {
	i2c_hal_t *self = (i2c_hal_t*) self_;

	return timeNowUs(self->i2c_handle->config->tim);
}

i2c_hal_t* shtp_i2c_hal_init(i2c_config_t *config) {
	i2c_hal_t *i2c_hal = (i2c_hal_t*) malloc(sizeof(i2c_hal_t));
	if (i2c_hal == NULL) {
		return NULL;
	}

	i2c_handle_t *i2c_handle = (i2c_handle_t*) malloc(sizeof(i2c_handle_t));
	if (i2c_handle == NULL) {
		free(i2c_hal);
		return NULL;
	}

//	for (size_t i = 0; i < sizeof(i2c_hal_t); i++) {
//		((uint8_t *)i2c_hal)[i] = 0;
//	}
//
//	for (size_t i = 0; i < sizeof(i2c_handle_t); i++) {
//		((uint8_t *)i2c_handle)[i] = 0;
//	}

	i2c_handle->config = config;
	i2c_handle->is_open = false;
	i2c_handle->in_reset = false;
	i2c_handle->bus_state = BUS_INIT;

	i2c_hal->i2c_handle = i2c_handle;
	i2c_hal->sh2_hal.open = shtp_i2c_hal_open;
	i2c_hal->sh2_hal.close = shtp_i2c_hal_close;
	i2c_hal->sh2_hal.read = shtp_i2c_hal_read;
	i2c_hal->sh2_hal.write = shtp_i2c_hal_write;
	i2c_hal->sh2_hal.getTimeUs = shtp_i2c_getTimeUs;

	return i2c_hal;
}

void IMPL_HAL_GPIO_EXTI_Callback(uint16_t n, i2c_hal_t *i2c_hal) {
	i2c_handle_t *handle = i2c_hal->i2c_handle;
	i2c_config_t *config = handle->config;

	// read bus state just once for consistency.
	bus_state_t busState = handle->bus_state;

	if (busState == BUS_INIT) {
		// No active hal, ignore this call, don't crash.
		return;
	}

	handle->in_reset = false;

	// Start read, if possible
	if ((busState == BUS_IDLE) && (handle->hdr_buf_len == 0)
			&& (handle->rx_buf_len == 0)) {
		handle->int_last_timestamp_us = timeNowUs(config->tim);

		// Read header to get payload length
		handle->read_retries = 0;
		handle->bus_state = BUS_READING_LEN;
		HAL_I2C_Master_Receive_IT(config->i2c, config->i2c_addr,
				handle->hdr_buf, HDR_READ_LEN);
	} else if ((busState == BUS_GOT_LEN) && (handle->rx_buf_len == 0)) {
		// Read payload
		handle->read_retries = 0;
		handle->bus_state = BUS_READING_TRANSFER;
		HAL_I2C_Master_Receive_IT(config->i2c, config->i2c_addr, handle->rx_buf,
				handle->payload_len);
	} else {
		// We can't start read immediately, set flag so it gets done later.
		handle->rx_data_ready = true;
	}
}

void IMPL_HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *pI2c,
		i2c_hal_t *i2c_hal) {
	i2c_handle_t *handle = i2c_hal->i2c_handle;

	// read bus state just once for consistency.
	bus_state_t busState = handle->bus_state;

	// Read completed
	if (busState == BUS_READING_LEN) {
		// Len of payload is available, decide how long to do next read
		uint16_t len = (handle->hdr_buf[0] + (handle->hdr_buf[1] << 8))
				& ~0x8000;
		if (len > sizeof(handle->rx_buf)) {
			// read only what will fit in handle->rx_buf
			handle->payload_len = sizeof(handle->rx_buf);
		} else {
			handle->payload_len = len;
		}

		handle->hdr_buf_len = HDR_READ_LEN;
		handle->bus_state = BUS_GOT_LEN;
	} else if (busState == BUS_READING_TRANSFER) {
		// handle->rx_buf is now ready for client.
		handle->rx_buf_len = handle->payload_len;

		// Nothing left to do
		handle->bus_state = BUS_IDLE;
	} else if (busState == BUS_READING_DFU) {
		// Transition back to idle state
		handle->hdr_buf_len = 0;
		handle->rx_buf_len = handle->payload_len;
		handle->bus_state = BUS_IDLE;
	}
}

void IMPL_HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *i2c,
		i2c_hal_t *i2c_hal) {
	i2c_handle_t *handle = i2c_hal->i2c_handle;

	// read bus state just once for consistency.
	bus_state_t busState = handle->bus_state;

	if (busState == BUS_WRITING) {
		// Switch back to bus idle
		handle->bus_state = BUS_IDLE;
	} else if (busState == BUS_WRITING_DFU) {
		// Switch back to bus idle
		handle->bus_state = BUS_IDLE;
	}
}

void IMPL_HAL_I2C_ErrorCallback(I2C_HandleTypeDef *i2c, i2c_hal_t *i2c_hal) {
	printf("I2CError\n");
	i2c_handle_t *handle = i2c_hal->i2c_handle;
	i2c_config_t *config = handle->config;

	// Assume we will abort this operation.
	// (Gets reset if we determine we will retry.)
	bool abort = true;

	if (handle->read_retries < MAX_RETRIES) {
		// Re-issue the I2C operation
		handle->read_retries++;

		switch (handle->bus_state) {
		case BUS_WRITING:
		case BUS_WRITING_DFU:
			// Set up write operation
			delay_us(config->tim, I2C_WRITE_DELAY_US);
			HAL_I2C_Master_Transmit_IT(i2c, config->i2c_addr, handle->tx_buf,
					handle->tx_buf_len);
			abort = false;
			break;
		case BUS_READING_LEN:
			// Restart Read operation for header
			HAL_I2C_Master_Receive_IT(i2c, config->i2c_addr, handle->hdr_buf,
			HDR_READ_LEN);
			abort = false;
			break;
		case BUS_READING_TRANSFER:
		case BUS_READING_DFU:
			// Restart read operation for transfer
			HAL_I2C_Master_Receive_IT(i2c, config->i2c_addr, handle->rx_buf,
					handle->payload_len);
			abort = false;
			break;
		default:
			// No operation in progress from other states.
			break;
		}
	}

	// If we didn't retry above, we should abort now.
	if (abort) {
		handle->hdr_buf_len = 0;
		handle->rx_buf_len = 0;
		handle->tx_buf_len = 0;
		handle->bus_state = BUS_IDLE;
	}
}
