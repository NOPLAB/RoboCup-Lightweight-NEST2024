/*
 * bno08x_i2c_f446re.c
 *
 *  Created on: Jul 6, 2024
 *      Author: nop
 */

#include "bno08x_i2c_f446.h"
#include <stdio.h>

#define RSTN_PORT GPIOB
#define RSTN_PIN GPIO_PIN_13

#define BOOTN_PORT GPIOB
#define BOOTN_PIN GPIO_PIN_12

#define INTN_PORT GPIOA
#define INTN_PIN GPIO_PIN_0

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim5;

i2c_hal_t* i2c_hal;
i2c_config_t i2c_config;

static void init_gpio(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure RSTN*/
    HAL_GPIO_WritePin(RSTN_PORT, RSTN_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = RSTN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RSTN_PORT, &GPIO_InitStruct);

    /* Configure BOOTN */
    HAL_GPIO_WritePin(BOOTN_PORT, BOOTN_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = BOOTN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOOTN_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : INTN */
    GPIO_InitStruct.Pin = INTN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(INTN_PORT, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
}

static void init_i2c(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // Configure GPIO Pins for use with I2C
    // PB8 : I2C1_SCL
    // PB9 : I2C1_SDA
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Peripheral clock enable
    __HAL_RCC_I2C1_CLK_ENABLE();

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
    	printf("HAL_I2C_Init Error\n");
    }

    // Set Priority for I2C IRQ and enable
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
}

static void init_timer(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();

    // Prescale to get 1 count per uS
    uint32_t prescaler = (uint32_t)((HAL_RCC_GetPCLK2Freq() / 1000000) - 1);

    htim5.Instance = TIM5;
    htim5.Init.Period = 0xFFFFFFFF;
    htim5.Init.Prescaler = prescaler;
    htim5.Init.ClockDivision = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;

    HAL_TIM_Base_Init(&htim5);
    HAL_TIM_Base_Start(&htim5);
}

static void init_hw(void)
{
	init_timer();
    init_gpio();
    init_i2c();
}

static void set_rst(bool state)
{
    HAL_GPIO_WritePin(RSTN_PORT, RSTN_PIN,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void set_boot(bool state)
{
    HAL_GPIO_WritePin(BOOTN_PORT, BOOTN_PIN,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void set_ints(bool state)
{
    if (state)
    {
        // Enable INTN interrupt
        HAL_NVIC_EnableIRQ(EXTI0_IRQn);

        // Enable I2C interrupts
        HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
        HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    }
    else
    {
        // Disable I2C interrupts
        HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
        HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);

        // Disable INTN interrupt line
        HAL_NVIC_DisableIRQ(EXTI0_IRQn);
    }
}

static void set_i2c_ints(bool state)
{
    if (state)
    {
        // Enable I2C interrupts
        HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
        HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    }
    else
    {
        // Disable I2C interrupts
        HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
        HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    }
}

i2c_hal_t *bno08x_init_i2c(void)
{
    init_hw();

    i2c_config.tim = &htim5;
    i2c_config.i2c = &hi2c1;
    i2c_config.i2c_addr = BNO08X_I2C_ADDR2 << 1;
    i2c_config.set_boot = set_boot;
    i2c_config.set_rst = set_rst;
    i2c_config.set_ints = set_ints;
    i2c_config.set_i2c_ints = &set_i2c_ints;

    i2c_hal = shtp_i2c_hal_init(&i2c_config);

    return i2c_hal;
}

void HAL_GPIO_EXTI_Callback(uint16_t n)
{
    if (n == INTN_PIN)
    {
        IMPL_HAL_GPIO_EXTI_Callback(n, i2c_hal);
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *pI2c)
{
    if (pI2c == &hi2c1)
    {
        IMPL_HAL_I2C_MasterRxCpltCallback(pI2c, i2c_hal);
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *pI2c)
{
	if (pI2c == &hi2c1)
    {
        IMPL_HAL_I2C_MasterTxCpltCallback(pI2c, i2c_hal);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *pI2c){
	IMPL_HAL_I2C_ErrorCallback(pI2c, i2c_hal);
}

// Handle I2C1 EV IRQ, passing it to STM32 HAL library
//void I2C1_EV_IRQHandler(void)
//{
//	printf("I2C1_EV_IRQHandler\n");
//    HAL_I2C_EV_IRQHandler(&hi2c1);
//}
//
//// Handle I2C1 ER IRQ, passing it to STM32 HAL library
//void I2C1_ER_IRQHandler(void)
//{
//	printf("I2C1_ER_IRQHandler\n");
//    HAL_I2C_ER_IRQHandler(&hi2c1);
//}

void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
