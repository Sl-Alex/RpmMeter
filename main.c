/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

//#include <stdlib.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include "lcd.h"

/* HW dependent settings */
#define MOTOR_PPR           12    // Pulses per revolution
#define MOTOR_RPM_MIN       60*MOTOR_PPR  // Minimum is 1 revolution per second
#define LCD_UPDATE_RATE     10 // Updates per second

static volatile uint32_t    time_cntr = 0;
static volatile uint32_t    rpm_filtered = 0;
static volatile uint32_t    rpm_raw;
static volatile uint8_t     update_all = 1;

/* A very simple IIR filter
* m must be bigger than n
*/
static uint32_t filter(uint32_t val_prev, uint32_t val, uint32_t m, uint32_t n)
{
    return val_prev - ((val_prev * m) / n) + ((val  * m) / n);
}

int main(void)
{
    /* Initialize syystem clock */
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    rcc_periph_clock_enable(RCC_GPIOC);     // LED
    rcc_periph_clock_enable(RCC_GPIOB);     // LCD
    rcc_periph_clock_enable(RCC_AFIO);      // LCD SPI
    rcc_periph_clock_enable(RCC_SPI2);      // LCD SPI
    rcc_periph_clock_enable(RCC_TIM1);      // RPM counter
    rcc_periph_clock_enable(RCC_TIM2);      // Time counter

    /* Initialize LED, toggles each second if the motor is running */
    gpio_set(GPIOC, GPIO13);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
              GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    /* Initialize LCD_CS as normal GPIO */
    gpio_set(GPIOB, GPIO14);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
              GPIO_CNF_OUTPUT_PUSHPULL, GPIO14);

    /* Initialize LCD SPI clock and data pins */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO13 |
                                            GPIO15 );

    /* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
    spi_reset(SPI2);

    /* SPI Master for LCD with 1/256 of peripheral clock */
    spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_256, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

    /* Enable SPI2 periph. */
    spi_enable(SPI2);
    
    /* Enable TIM2 global interrupt. */
    nvic_enable_irq(NVIC_TIM2_IRQ);


    /* Set the prescaler to get 50kHz timer clock */
    timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 50000) - 1);
    /* Set the period according to the desired LCD update rate */
    timer_set_period(TIM2, (50000/LCD_UPDATE_RATE) - 1);

    /* Enable the timer and update interrupt */
    timer_enable_counter(TIM2);
    timer_enable_irq(TIM2, TIM_DIER_UIE);

    /* Enable external clock (ETR2 input) */
    TIM_SMCR(TIM1) |= TIM_SMCR_ECE;
    timer_enable_counter(TIM1);

    lcd_init(SPI2, GPIOB, GPIO14);

    while(1)
    {
        /* Wait here until it's time to update LCD and filter */
        while (!update_all)	{};
        update_all = 0;

        /* Filter RPM value and update the LCD */
        rpm_filtered = filter(rpm_filtered, rpm_raw, 1, 2);
        if (rpm_filtered < MOTOR_RPM_MIN)
        {
            lcd_show_time(time_cntr);
        }
        else
        {
            lcd_show_rpm(rpm_filtered);
        }
    }
}

void tim2_isr(void)
{
    if (timer_get_flag(TIM2, TIM_SR_UIF)) {

        /* Clear compare interrupt flag. */
        timer_clear_flag(TIM2, TIM_SR_UIF);

        static uint8_t frames = 0;
        frames++;

        /* Check if 1 second is elapsed */
        if (frames >= LCD_UPDATE_RATE)
        {
            frames = 0;
            if (rpm_filtered > MOTOR_RPM_MIN)
            {
                time_cntr++;
                gpio_toggle(GPIOC, GPIO13);
            }
        }

        rpm_raw = timer_get_counter(TIM1)*LCD_UPDATE_RATE*(60/MOTOR_PPR);
        timer_set_counter(TIM1, 0);
        update_all = 1;
    }
}