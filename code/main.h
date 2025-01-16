/******************************************************************************
 * @file    main.h
 * @brief   Main program header file
 ******************************************************************************/

#ifndef MAIN_H_
#define MAIN_H_

#include "stm32f0xx.h"

/* Pin definitions */
#define BRAKE_PIN     GPIO_IDR_0
#define DRIVE_PIN     GPIO_IDR_1
#define SOFTSTART_PIN GPIO_IDR_2
#define REVERSE_PIN   GPIO_IDR_3

#endif /* MAIN_H_ */ 