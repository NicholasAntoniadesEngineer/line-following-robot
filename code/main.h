/******************************************************************************
 * @file    main.h
 * @brief   Header file for main.c in line-following-robot
 * @version 1.0
 ******************************************************************************/

#ifndef MAIN_H_
#define MAIN_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "lcd_stm32f0.h"
#include <stdint.h>
#include "stm32_lib.h"
#include "stm32_pwm.h"

/* Function prototypes ------------------------------------------------------*/
static void init_stm32(void);
static void lcd_display(const char* line1, const char* line2);
static void robot_state_machine(void);

/* Timer interrupt handler -------------------------------------------------*/
void TIM14_IRQHandler(void);

/* Constants --------------------------------------------------------------*/
#define CENTI_SEC_MAX       25U    /* Maximum value for centisecond counter */

/* GPIO Pin definitions --------------------------------------------------*/
#define BRAKE_PIN           GPIO_IDR_0
#define DRIVE_PIN           GPIO_IDR_1
#define SOFTSTART_PIN       GPIO_IDR_2
#define REVERSE_PIN         GPIO_IDR_3

#endif /* MAIN_H_ */ 