/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h> // Includes for uint*_t types

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum IO { IN, OUT };

typedef struct Bio_Data
    {
    uint32_t irLed;
    uint32_t redLed;
    uint16_t heartRate; // LSB = 0.1bpm
    uint8_t  confidence; // 0-100% LSB = 1%
    uint16_t oxygen; // 0-100% LSB = 1%
    uint8_t  status; // 0: Success, 1: Not Ready, 2: Object Detectected, 3: Finger Detected
    float    rValue;      // -- Algorithm Mode 2 vv
    int8_t   extStatus;   // --
    uint8_t  reserveOne;  // --
    uint8_t  resserveTwo; // -- Algorithm Mode 2 ^^
    } Bio_Data_t;

  // Status bytes for when I2C transmission and indicates what the status is for these.
  enum READ_STATUS_BYTE_VALUE
      {
      O2_SUCCESS                  = 0x00,
      ERR_UNAVAIL_CMD,
      ERR_UNAVAIL_FUNC,
      ERR_DATA_FORMAT,
      ERR_INPUT_VALUE,
      ERR_TRY_AGAIN,
      ERR_BTLDR_GENERAL        = 0x80,
      ERR_BTLDR_CHECKSUM,
      ERR_BTLDR_AUTH,
      ERR_BTLDR_INVALID_APP,
      ERR_UNKNOWN              = 0xFF
      };

typedef struct GPIO
    {
    enum IO io_state;
    uint8_t  pin_num;
    char        port;

    } GPIO_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define WRITE_FIFO_INPUT_BYTE  0x04
#define DISABLE                0x00
#define ENABLE                 0x01
#define MODE_ONE               0x01
#define MODE_TWO               0x02
#define APP_MODE               0x00
#define BOOTLOADER_MODE        0x08
#define NO_WRITE               0x00
#define INCORR_PARAM           0xEE

#define CONFIGURATION_REGISTER 0x0A
#define PULSE_MASK             0xFC
#define READ_PULSE_MASK        0x03
#define SAMP_MASK              0xE3
#define READ_SAMP_MASK         0x1C
#define ADC_MASK               0x9F
#define READ_ADC_MASK          0x60

#define ENABLE_CMD_DELAY          45 // Milliseconds
#define CMD_DELAY                 6  // Milliseconds
#define MAXFAST_ARRAY_SIZE        6  // Number of bytes....
#define MAXFAST_EXTENDED_DATA     5
#define MAX30101_LED_ARRAY        12 // 4 values of 24 bit (3 byte) LED values

#define SET_FORMAT             0x00
#define READ_FORMAT            0x01 // Index Byte under Family Byte: READ_OUTPUT_MODE (0x11)
#define WRITE_SET_THRESHOLD    0x01 //Index Byte for WRITE_INPUT(0x14)
#define WRITE_EXTERNAL_TO_FIFO 0x00

extern const uint8_t bio_addr_c;

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void config_gpio( const char port, const int pin_num, const enum IO direction );
void set_pin_mode( struct GPIO * const gpio, const enum IO direction );

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
