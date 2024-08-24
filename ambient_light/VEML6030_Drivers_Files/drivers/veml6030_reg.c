/*
 ******************************************************************************
 * @file    veml6030_reg.c
 * @author  MCD Application Team
 * @brief   VEML6030 driver file
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

#include "veml6030_reg.h"
#include <stdio.h>
#include "sl_udelay.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @addtogroup VEML6030
  * @brief     This file provides a set of functions needed to drive the
  *            VEML6030 Light sensor.
  * @{
  */

/** @defgroup VEML6030_Private_Types_Definitions Private Types Definitions
  * @{
  */
/**
  * @}
  */
/** @defgroup VEML6030_Private_Variables
  * @{
  */

VEML6030_Drv_t VEML6030_Driver =
{
  VEML6030_Init,
  VEML6030_DeInit,
  VEML6030_ReadID,
  VEML6030_GetCapabilities,
  VEML6030_SetExposureTime,
  VEML6030_GetExposureTime,
  VEML6030_SetGain,
  VEML6030_GetGain,
  VEML6030_SetInterMeasurementTime,
  VEML6030_GetInterMeasurementTime,
  VEML6030_Start,
  VEML6030_Stop,
  VEML6030_StartFlicker,
  VEML6030_StopFlicker,
  VEML6030_GetValues_ALS,
  VEML6030_GetValues_WHITE,
  //VEML6030_GetValues,
  VEML6030_SetControlMode
};

/**
  * @}
  */

/** @defgroup VEML6030_Private_Functions_Prototypes Private Functions Prototypes
  * @{
  */
static int32_t VEML6030_ReadRegWrap(void *handle, uint8_t Reg, uint8_t *Data, uint16_t Length);
static int32_t VEML6030_WriteRegWrap(void *handle, uint8_t Reg, const uint8_t *Data, uint16_t Length);
static int32_t VEML6030_Pwr_On(VEML6030_Object_t *pObj);
static int32_t VEML6030_Shutdown(VEML6030_Object_t *pObj);
//static int32_t VEML6030_Delay(VEML6030_Object_t *pObj, uint32_t Delay);

/**
  * @}
  */

/**
  * @brief  Read VEML6030 component registers
  * @param  ctx component contex
  * @param  reg Register to read from
  * @param  pdata Pointer to data buffer
  * @param  length Number of data to read
  * @retval Component status
  */
int32_t veml6030_read_reg(veml6030_ctx_t *ctx, uint8_t reg,
                          uint16_t *pdata, uint16_t length)
{
  int32_t ret;
  uint8_t buff[2];

  ret = ctx->ReadReg(ctx->handle, reg, &buff[0], length);

  if(ret >= 0)
  {
    *pdata = buff[1];
    *pdata = (*pdata * 256U) +  buff[0];
  }
  return ret;
}

/**
  * @brief  Write VEML6030 component registers
  * @param  ctx component contex
  * @param  reg Register to write to
  * @param  pdata Pointer to data buffer
  * @param  length Number of data to write
  * @retval Component status
  */
int32_t veml6030_write_reg(veml6030_ctx_t *ctx, uint8_t reg,
                           uint16_t data, uint16_t length)
{
  uint8_t buff[2];

  buff[1] = (uint8_t)(data / 256U);
  buff[0] = (uint8_t)(data - (buff[1] * 256U));

  return ctx->WriteReg(ctx->handle, reg, &buff[0], length);
}

/**
  * @brief Initializes the veml6030 context object.
  * @param pObj    veml6030 context object.
  * @param pIO     BSP IO struct.
  * @retval VEML6030 status
  */
int32_t VEML6030_RegisterBusIO(VEML6030_Object_t *pObj, VEML6030_IO_t *pIO)
{
  int32_t ret;

  if (pObj == NULL)
  {
    ret = VEML6030_ERROR;
  }
  else
  {
    pObj->IO.Init          = pIO->Init;
    pObj->IO.DeInit        = pIO->DeInit;
    pObj->IO.ReadAddress   = pIO->ReadAddress;
    pObj->IO.WriteAddress  = pIO->WriteAddress;
    pObj->IO.WriteReg      = pIO->WriteReg;
    pObj->IO.ReadReg       = pIO->ReadReg;
    pObj->IO.GetTick       = pIO->GetTick;

    pObj->Ctx.ReadReg      = VEML6030_ReadRegWrap;
    pObj->Ctx.WriteReg     = VEML6030_WriteRegWrap;
    pObj->Ctx.handle       = pObj;

    if (pObj->IO.ReadReg != NULL)
    {
      ret = VEML6030_OK;
    }
    else
    {
      ret = VEML6030_ERROR;
    }
  }

  return ret;

}

/**
  * @brief Initializes the veml6030 light sensor.
  * @param pObj    veml6030 context object.
  * @retval VEML6030 status
  */
int32_t VEML6030_Init(VEML6030_Object_t *pObj)
{
  int32_t ret = VEML6030_OK;
  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else if (pObj->IsInitialized == 0U)
  {
    if (VEML6030_Pwr_On(pObj) < 0)
    {
      ret = VEML6030_ERROR;
    }
    else
    {
      pObj->IsInitialized = 1;
      pObj->IsContinuous = 1;
      pObj->IsStarted= 1;
    }
  }
  return ret;
}

/**
  * @brief Deinitializes the veml6030 light sensor.
  * @param pObj    veml6030 context object.
  * @retval VEML6030 status
  */
int32_t VEML6030_DeInit(VEML6030_Object_t *pObj)
{
  int32_t ret = VEML6030_OK;
  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else if (pObj->IsInitialized == 1U)
  {
    if (VEML6030_Shutdown(pObj) < 0)
    {
      ret = VEML6030_ERROR;
    }
    else
    {
      pObj->IsInitialized = 0;
    }
  }
  return ret;
}

/**
  * @brief Read the veml6030 device ID.
  * @param pObj    veml6030 context object.
  * @param pId    Pointer to the device ID.
  * @retval veml6030 status
  */
int32_t VEML6030_ReadID(VEML6030_Object_t *pObj, uint16_t *pId)
{
  int32_t ret;
  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else
  {
    veml6030_read_reg(&pObj->Ctx, VEML6030_REG_DEVICE_ID, pId, 2);
    ret = VEML6030_OK;
  }
  return ret;
}

/**
  * @brief Get the veml6030 capabilities.
  * @param pObj    veml6030 context object.
  * @param pCapabilities    Pointer to the veml6030 capabilities.
  * @retval VEML6030 status
  */
int32_t VEML6030_GetCapabilities(VEML6030_Object_t *pObj, VEML6030_Capabilities_t *pCapabilities)
{
  int32_t ret;

  if ((pObj != NULL) && (pCapabilities != NULL))
  {
    pCapabilities->NumberOfChannels = VEML6030_MAX_CHANNELS;
    pCapabilities->FlickerDetection = 0U;
    pCapabilities->Autogain = 0U;

    ret = VEML6030_OK;
  }
  else
  {
    ret = VEML6030_INVALID_PARAM;
  }

  return ret;
}

/**
  * @brief Set the exposure time.
  * @param pObj    veml6030 context object.
  * @param ExposureTime    New exposure time to be applied.
  * @warning This function must not be called when a capture is ongoing.
  * @retval VEML6030 status
  */
int32_t VEML6030_SetExposureTime(VEML6030_Object_t *pObj, uint32_t ExposureTime)
{
 int32_t ret;
  uint16_t config = 0;
  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else if(veml6030_read_reg(&pObj->Ctx, VEML6030_REG_ALS_CONF, &config, 2) != 0)
  {
    ret = VEML6030_ERROR;
  }
  else
  {
    config &= ~VEML6030_CONF_IT_MASK;
    config |= (uint16_t)ExposureTime;
    ret = VEML6030_OK;
    if(veml6030_write_reg(&pObj->Ctx, VEML6030_REG_ALS_CONF, config, 2)!= 0)
    {
      ret = VEML6030_ERROR;
    }
  }
  return ret;
}
/**
  * @brief Get the exposure time.
  * @param pObj    veml6030 context object.
  * @param pExposureTime    Pointer to the current exposure time value.
  * @retval VEML6030 status
  */
int32_t VEML6030_GetExposureTime(VEML6030_Object_t *pObj, uint32_t *pExposureTime)
{
  int32_t ret;
  uint16_t tmp = 0;
  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else if(veml6030_read_reg(&pObj->Ctx, VEML6030_REG_ALS_CONF, &tmp, 2)!=0)
  {
    ret = VEML6030_ERROR ;
  }
  else
  {
    *pExposureTime=(tmp & VEML6030_CONF_IT_MASK );
    ret = VEML6030_OK;
  }
  return ret;
}

/**
  * @brief Set the current gain of a channel.
  * @param pObj       veml6030 context object.
  * @param Channel    Device channel.
  * @param pGain    Pointer to the current gain value.
  * @retval VEML6030 status
  */
int32_t VEML6030_SetGain(VEML6030_Object_t *pObj, uint8_t Channel, uint32_t Gain)
{
  int32_t ret;
  uint16_t config = 0;
  if (pObj == NULL || Channel >= VEML6030_MAX_CHANNELS)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else if(veml6030_read_reg(&pObj->Ctx, VEML6030_REG_ALS_CONF, &config, 2)!=0)
  {
    ret = VEML6030_ERROR;
  }
  else
  {
    config&=(uint16_t) ~VEML6030_CONF_GAIN_1_4;

    config |= Gain;

    ret = VEML6030_OK;

      if(veml6030_write_reg(&pObj->Ctx, VEML6030_REG_ALS_CONF, config, 2)!=0)
      {
        ret = VEML6030_ERROR ;
      }
  }
  return ret;
}

/**
  * @brief Get the current gain of a channel.
  * @param pObj    veml6030 context object.
  * @param Channel    Device channel.
  * @param pGain    Pointer to the current gain value.
  * @retval VEML6030 status
  */
int32_t VEML6030_GetGain(VEML6030_Object_t *pObj, uint8_t Channel, uint32_t *Gain)
{
  int32_t ret;
  uint16_t tmp = 0;

  if (pObj == NULL || Channel >= VEML6030_MAX_CHANNELS)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else if(veml6030_read_reg(&pObj->Ctx, VEML6030_REG_ALS_CONF, &tmp, 2)!=0)
  {
    ret = VEML6030_ERROR ;
  }
  else
  {
    *Gain = (tmp & VEML6030_GAIN_MASK);
    ret = VEML6030_OK;
  }
  return ret;
}

 /**
  * @brief Set the inter-measurement time.
  * @param pObj                    veml6030 context object.
  * @param InterMeasurementTime    Inter-measurement to be applied expressed in microseconds.
  * @note This should be configured only when using the device in continuous mode.
  * @warning This function must not be called when a capture is ongoing.
  * @retval VEML6030 status
  */
int32_t VEML6030_SetInterMeasurementTime(VEML6030_Object_t * pObj, uint32_t InterMeasurementTime)
{
  int32_t ret;
  uint16_t psm = 0 ;
  uint16_t als_it = 0 ;
  uint16_t config;
  if (pObj == NULL )
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else
  {
    psm = (InterMeasurementTime & 0x0CU)>>1;  // currently PSM will be set to 00 as a default mode. @Dhana
    //als_it = (InterMeasurementTime & 0x03U);
    als_it = InterMeasurementTime;

    /* set the integration Time */
    if(veml6030_read_reg(&pObj->Ctx, VEML6030_REG_ALS_CONF, &config, 2) != 0)
    {
      ret = VEML6030_ERROR;
    }
    else
    {
      config &= ~VEML6030_CONF_IT_MASK;
      config |= (uint16_t)als_it;
      ret = VEML6030_OK;
      if(veml6030_write_reg(&pObj->Ctx, VEML6030_REG_ALS_CONF, config, 2)!= 0)
      {
        ret = VEML6030_ERROR;
      }
      else
      {
        ret = VEML6030_OK;
      }
    }
    if(ret == VEML6030_OK)
    {
      /* Set the Power Saving Mode */
      config  = (psm | VEML6030_POWER_SAVING_ENABLE) & 0x000F;;
      ret = VEML6030_OK;
      if(veml6030_write_reg(&pObj->Ctx, VEML6030_REG_POWER_SAVING, config, 2)!=0)
      {
        ret = VEML6030_ERROR ;
      }
      else
      {
        ret = VEML6030_OK;
      }
    }
  }
  return ret;
}
/**
  * @brief Get the inter-measurement time.
  * @param pObj                     veml6030 context object.
  * @param pInterMeasurementTime    Pointer to the current inter-measurement time expressed in microseconds.
  * @retval VEML6030 status
  */
int32_t VEML6030_GetInterMeasurementTime(VEML6030_Object_t *pObj, uint32_t *pInterMeasurementTime)
{
  int32_t ret;
  uint16_t tmp = 0;
  uint16_t als_it = 0;
  uint16_t psm = 0;
  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  if(veml6030_read_reg(&pObj->Ctx, VEML6030_REG_ALS_CONF, &tmp, 2)!=0)
  {
    ret = VEML6030_ERROR ;
  }
  else
  {
    als_it = (tmp & VEML6030_CONF_IT_MASK );
    ret = VEML6030_OK ;
  }
  if (ret == VEML6030_OK)
  {
    if(veml6030_read_reg(&pObj->Ctx, VEML6030_REG_POWER_SAVING, &tmp, 2)!=0)
    {
      ret = VEML6030_ERROR ;
    }
    else
    {
      psm = ((tmp & 0x06U)<<1);
      ret = VEML6030_OK ;
    }
  }

  *pInterMeasurementTime = (psm|als_it);
  return ret;
}
/**
  * @brief Start the light measurement on all channels.
  * @param pObj    veml6030 context object.
  * @param Mode    Measurement mode (continuous)
  * @retval VEML6030 status
  */
int32_t VEML6030_Start(VEML6030_Object_t *pObj, uint32_t Mode)
{
  //printf("Enter VEML6030_Start() \r\n");

  int32_t ret;
  if (pObj == NULL || Mode != VEML6030_MODE_CONTINUOUS )
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else
  {

    ret = VEML6030_Pwr_On(pObj);

    if(ret == VEML6030_OK)
    {
      pObj->IsStarted= 1;
    }

  }
  //printf("Exit VEML6030_Start() \r\n");
  return ret;
}
/**
  * @brief Stop the measurement on all channels.
  * @param pObj    veml6030 context object.
  * @retval VEML6030 status
  */
int32_t VEML6030_Stop(VEML6030_Object_t *pObj)
{
  //printf("Enter VEML6030_Stop() \r\n");
  int32_t ret;
  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else
  {
    ret = VEML6030_Shutdown(pObj);
    if(ret == VEML6030_OK)
    {
      pObj->IsStarted= 0;
    }
  }
  //printf("Exit VEML6030_Stop() \r\n");

  return ret;
}

/**
  * @brief Stop flicker capture.
  * @param pObj     veml6030 context object.
  * @retval BSP status
  */
int32_t VEML6030_StartFlicker(VEML6030_Object_t *pObj, uint8_t Channel, uint8_t OutputMode)
{
  (void)(pObj);
  (void)(Channel);
  (void)(OutputMode);

  /* Feature not supported */
  return VEML6030_ERROR;
}
/**
  * @brief Stop flicker capture.
  * @param pObj     veml6030 context object.
  * @retval BSP status
  */
int32_t VEML6030_StopFlicker(VEML6030_Object_t *pObj)
{
  (void)(pObj);

  /* Feature not supported */
  return VEML6030_ERROR;
}

/**
  * @brief Returns the measurement values for all the channels.
  * @param pObj        veml6030 context object.
  * @param pResult    Pointer to an array which will be filled with the values of each channel.
  * @note The array size must match the number of channels of the device.
  * @retval VEML6030 status
  */
int32_t VEML6030_GetValues_ALS(VEML6030_Object_t *pObj, uint16_t *Values)
{
  //printf("VEML6030_GetValues_ALS() is enter\r\n");
  int32_t ret = VEML6030_OK;

  if (pObj == NULL)
  {
    printf("Invalid\r\n");
    ret = VEML6030_INVALID_PARAM;
  }
  else if(veml6030_read_reg(&pObj->Ctx, VEML6030_REG_ALS, Values, 2)!=0)
  {
    printf("Read_Reg ALS Error \r\n");
    ret = VEML6030_ERROR ;
  }
  return ret;
}

int32_t VEML6030_GetValues_WHITE(VEML6030_Object_t *pObj, uint16_t *Values)
{
  //printf("VEML6030_GetValues_WHITE() is enter\r\n");
  int32_t ret = VEML6030_OK;

  if (pObj == NULL)
  {
    printf("Invalid\r\n");
    ret = VEML6030_INVALID_PARAM;
  }
  else if(veml6030_read_reg(&pObj->Ctx, VEML6030_REG_WHITE, Values, 2)!=0)
  {
    printf("Read_Reg White Error\r\n");
    ret = VEML6030_ERROR ;
  }
  return ret;
}
/*
int32_t VEML6030_GetValues(VEML6030_Object_t *pObj, uint32_t *Values)
{
  int32_t ret = VEML6030_OK;

  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else if(veml6030_read_reg(&pObj->Ctx, VEML6030_REG_ALS, (uint16_t *)Values, 2)!=0)
  {
    ret = VEML6030_ERROR ;
  }
  else if(veml6030_read_reg(&pObj->Ctx, VEML6030_REG_WHITE, (uint16_t *)Values, 2)!=0)
  {
    ret = VEML6030_ERROR ;
  }

  return ret;
}
*/
/**
  * @brief Enable and disable control features.
  * @param pObj    veml6030 context object.
  * @param ControlMode    Feature to be be enabled or disabled.
  * @param Value    Value to be applied.
  * @warning This function must not be called when a capture is ongoing.
  * @retval VEML6030 status
  */
int32_t VEML6030_SetControlMode(VEML6030_Object_t *pObj, uint32_t ControlMode, uint32_t Value)
{
  (void)(pObj);
  (void)(ControlMode);
  (void)(Value);

  /* Feature not supported */
  return VEML6030_ERROR;
}

/**
  * @brief Set the persistence protect number setting.
  * @param pObj           veml6030 context object.
  * @param Persistence    perssistance value to be applied .
  * @retval VEML6030 status
  */
int32_t VEML6030_SetPersistence(VEML6030_Object_t *pObj, uint32_t Persistence)
{
  int32_t ret;
  uint16_t config=0;
  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else if(veml6030_read_reg(&pObj->Ctx, VEML6030_REG_ALS_CONF, &config, 2)!=0)
  {
    ret = VEML6030_ERROR;
  }
  else
  {
    config&=~VEML6030_PERS_MASK;
    config |= Persistence;
    ret = VEML6030_OK;
    if(veml6030_write_reg(&pObj->Ctx, VEML6030_REG_ALS_CONF, config, 2)!=0)
    {
      ret = VEML6030_ERROR ;
    }
  }
  return ret;
}

/**
  * @brief Get the persistence protect number setting.
  * @param pObj           veml6030 context object.
  * @param PPersistence   Pointer to the current persistence value.
  * @retval VEML6030 status
  */
int32_t VEML6030_GetPersistence(VEML6030_Object_t *pObj, uint32_t *PPersistence)
{
  int32_t ret;
  uint16_t tmp = 0;
  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else if(veml6030_read_reg(&pObj->Ctx, VEML6030_REG_ALS_CONF, &tmp, 2)!=0)
  {
    ret = VEML6030_ERROR ;
  }
  else
  {
    *PPersistence = (tmp & VEML6030_PERS_MASK);
    ret = VEML6030_OK;
  }
  return ret;
}
/** @brief Set Power Saving Mode.
  * @param pObj         veml6030 context object.
  * @param PowerMode    New PowerMode Value to be applied.
  * @warning This function must not be called when a capture is ongoing.
  * @retval VEML6030 status
  */
int32_t VEML6030_SetPowerSavingMode(VEML6030_Object_t *pObj, uint32_t PowerMode)
{
  int32_t ret;
  uint16_t config = 0;
  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else
  {
    config  = (PowerMode | VEML6030_POWER_SAVING_ENABLE) & 0x000F;;
    ret = VEML6030_OK;
    if(veml6030_write_reg(&pObj->Ctx, VEML6030_REG_POWER_SAVING, config, 2)!=0)
    {
      ret = VEML6030_ERROR ;
    }
  }
  return ret;
}

/** @brief Get Power Saving Mode.
  * @param pObj          veml6030 context object.
  * @param pPowerMode    Pointer to the current PowerMode value.
  * @retval VEML6030 status
  */
 int32_t VEML6030_GetPowerSavingMode(VEML6030_Object_t *pObj, uint32_t *pPowerMode)
 {
   int32_t ret;
   uint16_t tmp = 0;
  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else if(veml6030_read_reg(&pObj->Ctx, VEML6030_REG_POWER_SAVING, &tmp, 2)!=0)
  {
    ret = VEML6030_ERROR ;
  }
  else
  {
    *pPowerMode = (tmp & 0x000E);
    ret = VEML6030_OK;
  }
  return ret;
 }
/**
  * @brief Set the High Threshold.
  * @param pObj         veml6030 context object.
  * @param Threshold    New Threshold Value to be applied.
  * @warning This function must not be called when a capture is ongoing.
  * @retval VEML6030 status
  */
int32_t VEML6030_SetHighThreshold(VEML6030_Object_t *pObj , uint16_t Threshold)
{
  int32_t ret = VEML6030_OK;

  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else if(veml6030_write_reg(&pObj->Ctx, VEML6030_REG_ALS_WH, Threshold, 2) != 0)
  {
    ret = VEML6030_ERROR;
  }
  else
  { /* avoid Misra Wranning*/
  }
  return ret;
}
/**
  * @brief Get the High Threshold.
  * @param pObj         veml6030 context object.
  * @param Threshold    New Threshold Value  time to be applied.
  * @warning This function must not be called when a capture is ongoing.
  * @retval VEML6030 status
  */
int32_t VEML6030_GetHighThreshold(VEML6030_Object_t *pObj , uint32_t *Threshold)
{
  int32_t ret;
  uint16_t data = 0;
  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else if(veml6030_read_reg(&pObj->Ctx, VEML6030_REG_ALS_WH, &data, 2)!=0)
  {
    ret = VEML6030_ERROR ;
  }
  else
  {
    *Threshold = data;
    ret = VEML6030_OK;
  }
  return ret;
}
/**
  * @brief Set the Low Threshold.
  * @param pObj         veml6030 context object.
  * @param Threshold    New Threshold Value to be applied.
  * @warning This function must not be called when a capture is ongoing.
  * @retval VEML6030 status
  */
int32_t VEML6030_SetLowThreshold(VEML6030_Object_t *pObj , uint16_t Threshold)
{
  int32_t ret = VEML6030_OK;

  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else if(veml6030_write_reg(&pObj->Ctx, VEML6030_REG_ALS_WL, Threshold, 2) != 0)
  {
    ret = VEML6030_ERROR;
  }
  else
  {
    /* avoid Misra Warning */
  }
  return ret;
}

/**
  * @brief Get the Low Threshold.
  * @param pObj         veml6030 context object.
  * @param Threshold    New Threshold Value  time to be applied.
  * @warning This function must not be called when a capture is ongoing.
  * @retval VEML6030 status
  */
int32_t VEML6030_GetLowThreshold(VEML6030_Object_t *pObj , uint32_t *Threshold)
{
  int32_t ret;
  uint16_t data = 0;
  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else if(veml6030_read_reg(&pObj->Ctx, VEML6030_REG_ALS_WL, &data, 2)!=0)
  {
    ret = VEML6030_ERROR ;
  }
  else
  {
    *Threshold = data;
    ret = VEML6030_OK;
  }
  return ret;
}

/**
  * @brief Disable the VEML6030  Interupt.
  * @param pObj    veml6030 context object.
  * @warning This function must not be called when a capture is ongoing.
  * @retval VEML6030 status
  */
int32_t VEML6030_Enable_IT(VEML6030_Object_t *pObj)
{
  int32_t ret;
  uint16_t config = 0;
  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else if(veml6030_read_reg(&pObj->Ctx, VEML6030_REG_ALS_CONF, &config, 2)!=0)
  {
    ret = VEML6030_ERROR;
  }
  else
  {
    config |= VEML6030_CONF_IT_ENABLE;
    ret = VEML6030_OK;
    if (VEML6030_Shutdown(pObj) != VEML6030_OK)
    {
      ret = VEML6030_ERROR;
    }
    else if(veml6030_write_reg(&pObj->Ctx,VEML6030_REG_ALS_CONF, config, 2)!=0)
    {
      ret = VEML6030_ERROR ;
    }
    else
    {
      if (VEML6030_Pwr_On(pObj) != VEML6030_OK)
      {
        ret = VEML6030_ERROR;
      }
    }
  }
  return ret;
}
/**
  * @brief Disable the VEML6030  Interupt.
  * @param pObj    veml6030 context object.
  * @warning This function must not be called when a capture is ongoing.
  * @retval VEML6030 status
  */
int32_t VEML6030_Disable_IT(VEML6030_Object_t *pObj)
{
  int32_t ret;
  uint16_t config = 0;
  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else if(veml6030_read_reg(&pObj->Ctx, VEML6030_REG_ALS_CONF, &config, 2)!=0)
  {
    ret = VEML6030_ERROR;
  }
  else
  {
    config &= ~VEML6030_CONF_IT_ENABLE;
    ret = VEML6030_OK;

    if (VEML6030_Shutdown(pObj) != VEML6030_OK)
    {
      ret = VEML6030_ERROR;
    }
    else if(veml6030_write_reg(&pObj->Ctx, VEML6030_REG_ALS_CONF, config, 2)!=0)
    {
      ret = VEML6030_ERROR ;
    }
    else
    {
      if (VEML6030_Pwr_On(pObj) != VEML6030_OK)
      {
        ret = VEML6030_ERROR;
      }
    }
  }
  return ret;
}
/**
  * @brief Get the Interrupt status (High Threshold or Low Threshold Interrupt).
  * @param pObj      veml6030 context object.
  * @param status    Pointer to the interrupt status.
  * @retval VEML6030 status
  */
int32_t VEML6030_GetIntStatus(VEML6030_Object_t *pObj,uint32_t *status)
{
  int32_t ret = 0;
  uint16_t data = 0;
  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else if(veml6030_read_reg(&pObj->Ctx, VEML6030_REG_ALS_INT, &data, 2)!=0)
  {
    ret = VEML6030_ERROR ;
  }
  else
  {
    if((data &0x8000) == 0x8000)
    {
      *status = VEML6030_INT_TH_HIGH;
    }
    else if((data &0x4000) == 0x4000)
    {
      *status = VEML6030_INT_TH_LOW;
    }
    else
    {
      *status = VEML6030_INT_TH_NONE;
    }
  }
  return ret;
}

/**
  * @brief  Wrap component ReadReg to Bus Read function
  * @param  handle  Component object handle
  * @param  Reg  The target register address to write
  * @param  pData  The target register value to be written
  * @param  Length  buffer size to be written
  * @retval error status
  */
static int32_t VEML6030_ReadRegWrap(void *handle, uint8_t Reg,
                                    uint8_t *pData, uint16_t Length)
{
  VEML6030_Object_t *pObj = (VEML6030_Object_t *)handle;

  return pObj->IO.ReadReg(pObj->IO.ReadAddress, Reg, pData, Length);
}

/**
  * @brief  Wrap component WriteReg to Bus Write function
  * @param  handle  Component object handle
  * @param  Reg  The target register address to write
  * @param  pData  The target register value to be written
  * @param  Length  buffer size to be written
  * @retval error status
  */
static int32_t VEML6030_WriteRegWrap(void *handle, uint8_t Reg,
                                     const uint8_t *pData, uint16_t Length)
{
  VEML6030_Object_t *pObj = (VEML6030_Object_t *)handle;

  return pObj->IO.WriteReg(pObj->IO.WriteAddress, Reg, pData, Length);
}

/**
  * @brief Shutdown the VEML6030.
  * @param pObj  veml6030 context object.
  * @retval      VEML6030 status
  */
static int32_t VEML6030_Shutdown(VEML6030_Object_t *pObj)
{
  int32_t ret;
  uint16_t config=0;
  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else if ( veml6030_read_reg(&pObj->Ctx, VEML6030_REG_ALS_CONF, &config, 2) != 0 )
  {
    ret = VEML6030_ERROR;
  }
  else
  {
    config |= VEML6030_CONF_SHUTDOWN;
    ret = VEML6030_OK ;
    if( veml6030_write_reg(&pObj->Ctx, VEML6030_REG_ALS_CONF, config, 2) != 0)
    {
      ret = VEML6030_ERROR;
    }
  }
  return ret;
}

/**
  * @brief Set Power ON the VEML6030.
  * @param pObj  veml6030 context object.
  * @retval      VEML6030 status
  */
static int32_t VEML6030_Pwr_On(VEML6030_Object_t *pObj)
{
  int32_t ret;
  uint16_t config=0;
  if (pObj == NULL)
  {
    ret = VEML6030_INVALID_PARAM;
  }
  else if (veml6030_read_reg(&pObj->Ctx, VEML6030_REG_ALS_CONF, &config, 2) != 0)
  {
    ret = VEML6030_ERROR;
  }
  else
  {
    config &= ~VEML6030_CONF_SHUTDOWN;
    ret = VEML6030_OK ;
    if(veml6030_write_reg(&pObj->Ctx, VEML6030_REG_ALS_CONF, config, 2) != 0)
    {
      ret = VEML6030_ERROR;
    }
    sl_udelay_wait(9000); // 9000 us = 9 ms
    //VEML6030_Delay(pObj,4);
  }
  return ret;
}

/**
  * @brief This function provides accurate delay (in milliseconds)
  * @param pObj pointer to component object
  * @param Delay: specifies the delay time length, in milliseconds
  * @retval Component status
  */
/*
 static int32_t VEML6030_Delay(VEML6030_Object_t *pObj, uint32_t Delay)
{
  uint32_t tickstart;

  tickstart = pObj->IO.GetTick();
  while((pObj->IO.GetTick() - tickstart) < Delay)
  {
  }
  return VEML6030_OK;
}
*/
/**
  * @}
 */

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
