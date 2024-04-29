/**
 ******************************************************************************
 * @file    veml6030.h
 * @author  MCD Application Team
 * @brief   VEML6030 header driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef AMBIENT_LIGHT_H
#define AMBIENT_LIGHT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "drivers/veml6030_reg.h"
#include <stdbool.h>


//static int32_t integration_time;


void enable_I2C(void);
void setup_I2C_Read_Write(void);
void init_VEML6030(void);
void start_VEML6030(void);
void configure_VEML6030(int32_t integration_time, uint32_t gain, bool tr, bool fal);
void get_VEML6030_Values(int32_t integration_time, uint32_t gain);
void shutdown_VEML6030(void);
void Stop_VEML6030(void);
void DeInit_VEML6030(void);


#ifdef __cplusplus
}
#endif

#endif /* AMBIENT_LIGHT_H */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
