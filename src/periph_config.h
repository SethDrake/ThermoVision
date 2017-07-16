
#pragma once
#ifndef __PERIPH_CONFIG_H_
#define __PERIPH_CONFIG_H_

/* PA13, PA14 - SWD - DON'T USE!!! */
/* PA11, PA12 - USB */

#define OV7670_DATA_PORT		GPIOA /* Pin_0 - Pin_7 */

#define SD_PORT					GPIOA
#define SD_PIN_CS				GPIO_Pin_8

#define OV7670_CLCK_PORT		GPIOB
#define OV7670_VSYNC_PIN        GPIO_Pin_0
#define OV7670_RCLK_PIN         GPIO_Pin_1
#define OV7670_WREN_PIN         GPIO_Pin_2
#define OV7670_WRRST_PIN        GPIO_Pin_3

#define OV7670_SCCB_PORT		GPIOB
#define OV7670_SCCB_CLK_PIN     GPIO_Pin_4
#define OV7670_SCCB_DATA_PIN    GPIO_Pin_5

#define I2C_PORT		GPIOB
#define I2C_SCL			GPIO_Pin_6
#define I2C_SDA			GPIO_Pin_7

#define SERVO_PORT		GPIOB
#define SERVO_PWM1		GPIO_Pin_8
#define SERVO_PWM2		GPIO_Pin_9

#define ILI9341_PORT            GPIOB
#define ILI9341_PIN_RS          GPIO_Pin_10
#define ILI9341_PIN_RESET       GPIO_Pin_11
#define ILI9341_PIN_CS          GPIO_Pin_12

#define SPI_PORT		GPIOB
#define SPI_SCK			GPIO_Pin_13
#define SPI_MISO		GPIO_Pin_14
#define SPI_MOSI		GPIO_Pin_15

#define SERVO_TIM		TIM4
#define HORIZ_SERVO_CH	3
#define VERT_SERVO_CH	4

#endif //__PERIPH_CONFIG_H_
