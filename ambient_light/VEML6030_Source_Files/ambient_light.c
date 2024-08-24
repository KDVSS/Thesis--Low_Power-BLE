/**
 ******************************************************************************
 * @file    veml6030.c
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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "drivers/veml6030_reg.h"
#include "ambient_light.h"

#include "em_device.h"
#include "em_chip.h"
#include "em_i2c.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "sl_udelay.h"


// Defines
/** I2C Device Address 7 bit format **/
#define VEML6030_I2C_SLAVE_ADD          0x48U //1001000

/** I2C Device Address 8 bit format **/
#define VEML6030_I2C_ADDRESS (VEML6030_I2C_SLAVE_ADD << 1)

#define I2C_TXBUFFER_SIZE                 1
#define I2C_RXBUFFER_SIZE                 1

#define SL_I2C_RECOVER_NUM_CLOCKS         10

// Buffers
uint8_t i2c_txBuffer[I2C_TXBUFFER_SIZE] = {0};
uint8_t i2c_rxBuffer[I2C_RXBUFFER_SIZE] = {0};

// I2C pins (SCL = PD2; SDA = PD3)
#define I2C_SCL_PORT            gpioPortD
#define I2C_SCL_PIN             2
#define I2C_SDA_PORT            gpioPortD
#define I2C_SDA_PIN             3

static veml6030_ctx_t dev_ctx;
static VEML6030_IO_t dev_io;
static VEML6030_Object_t dev_obj;

static float lux_resolution = 0;

static uint32_t set_exposure_time;
static uint32_t set_gain_setting;
static uint32_t get_exposure_Time;
static uint32_t get_gain_setting;
static uint32_t get_ppersistence;
static uint32_t set_ppersistence;


//static int32_t integration_time;
//static uint32_t gain;


/***************************************************************************//**
 * @brief I2C write numBytes to VEML6030 device starting at target address
 ******************************************************************************/

int32_t ctx_veml6030_I2C_write_to_reg(void *handle,
                                  uint8_t registerAddress,
                                  const uint8_t *txBuff,
                                  uint16_t numBytes)
{
  (void)handle;
  //printf("veml6030_I2C_write_to_reg() is enter\r\n");

  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef result;

  uint8_t i2c_write_data[numBytes + 1];
  uint8_t i2c_read_data[1];

  seq.addr  = VEML6030_I2C_ADDRESS;
  seq.flags = I2C_FLAG_WRITE;

  i2c_write_data[0] = registerAddress;
  for(int i = 0; i < numBytes; i++)
    {
      i2c_write_data[i + 1] = txBuff[i];
    }

  /* Select register and data to write */
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = numBytes + 1;

  /* Select length of data to be read */
  seq.buf[1].data   = i2c_read_data;
  seq.buf[1].len    = 0;

  result = I2C_TransferInit(I2C0, &seq);
  while (result == i2cTransferInProgress) {
      result = I2C_Transfer(I2C0);
    }
    //app_log("Read: result for I2C_Transfer  %d \n", result);
  if (result != i2cTransferDone) {
      // Indicate I2C transmission problem
      return -1;
    }
  //printf("veml6030_I2C_write_to_reg() is exit\r\n");
  return 0;
}

/***************************************************************************//**
 * @brief I2C read numBytes from VEML6030 device starting at target address
 ******************************************************************************/
int32_t ctx_veml6030_I2C_read_from_reg(void *handle,
                                   uint8_t registerAddress,
                                   uint8_t *rxBuff,
                                   uint16_t numBytes)
{
  (void)handle;
  //printf("veml6030_I2C_read_from_reg() is enter\r\n");

  // Transfer structure
  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef result;
  uint8_t i2c_write_data[1];

  seq.addr          = VEML6030_I2C_WRITE_ADD;
  seq.flags         = I2C_FLAG_WRITE_READ; // must write target address before reading

  i2c_write_data[0] = registerAddress;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;

  /* Select length of data to be read */
  seq.buf[1].data   = rxBuff;
  seq.buf[1].len    = numBytes;

  result = I2C_TransferInit(I2C0, &seq);
  while (result == i2cTransferInProgress) {
      result = I2C_Transfer(I2C0);
    }
  //printf("Read: result for I2C_Transfer successful =  %d \r\n", result);
  if (result != i2cTransferDone) {
      // Indicate I2C transmission problem
      printf("Read: result != i2cTransferDone \r\n");
      return -1;
    }
  //printf("veml6030_I2C_read_from_reg() is exit \r\n");

  return 0;
}

/***************************************************************************//**
 * @brief I2C write numBytes to VEML6030 device starting at target address
 ******************************************************************************/

int32_t veml6030_I2C_write_to_reg(uint16_t writeAddress,
                                  uint8_t registerAddress,
                                  const uint8_t *txBuff,
                                  uint16_t numBytes)
{
  //printf("veml6030_I2C_write_to_reg() is enter\r\n");

  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef result;

  uint8_t i2c_write_data[numBytes + 1];
  uint8_t i2c_read_data[1];

  seq.addr  = writeAddress;
  seq.flags = I2C_FLAG_WRITE;

  i2c_write_data[0] = registerAddress;
  for(int i = 0; i < numBytes; i++)
    {
      i2c_write_data[i + 1] = txBuff[i];
    }

  /* Select register and data to write */
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = numBytes + 1;

  /* Select length of data to be read */
  seq.buf[1].data   = i2c_read_data;
  seq.buf[1].len    = 0;

  result = I2C_TransferInit(I2C0, &seq);
  while (result == i2cTransferInProgress) {
      result = I2C_Transfer(I2C0);
    }
    //app_log("Read: result for I2C_Transfer  %d \n", result);
  if (result != i2cTransferDone) {
      // Indicate I2C transmission problem
      return -1;
    }
  //printf("veml6030_I2C_write_to_reg() is exit\r\n");
  return 0;
}

/***************************************************************************//**
 * @brief I2C read numBytes from VEML6030 device starting at target address
 ******************************************************************************/
int32_t veml6030_I2C_read_from_reg(uint16_t readAddress,
                                   uint8_t registerAddress,
                                   uint8_t *rxBuff,
                                   uint16_t numBytes)
{
  //printf("veml6030_I2C_read_from_reg() is enter\r\n");

  // Transfer structure
  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef result;
  uint8_t i2c_write_data[1];

  seq.addr          = readAddress;
  seq.flags         = I2C_FLAG_WRITE_READ; // must write target address before reading

  i2c_write_data[0] = registerAddress;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;

  /* Select length of data to be read */
  seq.buf[1].data   = rxBuff;
  seq.buf[1].len    = numBytes;

  result = I2C_TransferInit(I2C0, &seq);
  while (result == i2cTransferInProgress) {
      result = I2C_Transfer(I2C0);
    }
  //printf("Read: result for I2C_Transfer successful =  %d \r\n", result);
  if (result != i2cTransferDone) {
      // Indicate I2C transmission problem
      printf("result != i2cTransferDone \r\n");
      return -1;
    }
  //printf("veml6030_I2C_read_from_reg() is exit \r\n");

  return 0;
}

void setup_I2C_Read_Write(void)
{
  dev_ctx.ReadReg  = ctx_veml6030_I2C_read_from_reg;
  dev_ctx.WriteReg = ctx_veml6030_I2C_write_to_reg;
  //dev_obj.Ctx.ReadReg  = veml6030_I2C_read_from_reg;
  //dev_obj.Ctx.WriteReg = veml6030_I2C_write_to_reg;

  dev_io.ReadAddress   = VEML6030_I2C_READ_ADD;
  dev_io.WriteAddress  = VEML6030_I2C_WRITE_ADD;
  dev_io.ReadReg       = veml6030_I2C_read_from_reg;
  dev_io.WriteReg      = veml6030_I2C_write_to_reg;

  VEML6030_RegisterBusIO(&dev_obj, &dev_io);

}

/***************************************************************************//**
 * @brief Enable clocks
 ******************************************************************************/
void initCMU(void)
{
  //app_log("initCMU called \n");
  // Enable clocks to the I2C
  CMU_ClockEnable(cmuClock_I2C0, true);
}

/***************************************************************************//**
 * @brief Setup I2C
 ******************************************************************************/
void initI2C(void)
{
  //app_log("initI2C Called \n");
  // Use default settings
  I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

  // Configure SCL (P0D2) and SDA (P0D3) pins
  GPIO_PinModeSet(I2C_SDA_PORT, I2C_SDA_PIN, gpioModeWiredAndPullUp, 1);
  GPIO_PinModeSet(I2C_SCL_PORT, I2C_SCL_PIN, gpioModeWiredAndPullUp, 1);

  /* In some situations, after a reset during an I2C transfer, the slave
      device may be left in an unknown state. Send 9 clock pulses to
      set slave in a defined state. */
   for (int i = 0; i < SL_I2C_RECOVER_NUM_CLOCKS; i++) {
     GPIO_PinOutClear(I2C_SCL_PORT, I2C_SCL_PIN);
     sl_udelay_wait(100);
     GPIO_PinOutSet(I2C_SCL_PORT, I2C_SCL_PIN);
     sl_udelay_wait(100);
   }

  // Route I2C pins to GPIO
  GPIO->I2CROUTE[0].ROUTEEN = GPIO_I2C_ROUTEEN_SDAPEN | GPIO_I2C_ROUTEEN_SCLPEN;
  GPIO->I2CROUTE[0].SCLROUTE = (uint32_t)((I2C_SCL_PIN << _GPIO_I2C_SCLROUTE_PIN_SHIFT)
                                              | (I2C_SCL_PORT << _GPIO_I2C_SCLROUTE_PORT_SHIFT));
  GPIO->I2CROUTE[0].SDAROUTE = (uint32_t)((I2C_SDA_PIN << _GPIO_I2C_SDAROUTE_PIN_SHIFT)
                                              | (I2C_SDA_PORT << _GPIO_I2C_SDAROUTE_PORT_SHIFT));

  // Initialize the I2C
  I2C_Init(I2C0, &i2cInit);

  I2C_IntClear(I2C0, _I2C_IF_MASK);
  I2C_IntEnable(I2C0, I2C_IEN_ADDR | I2C_IEN_RXDATAV | I2C_IEN_ACK | I2C_IEN_SSTOP | I2C_IEN_BUSERR | I2C_IEN_ARBLOST);

  // Enable automatic STOP on NACK
  I2C0->CTRL = I2C_CTRL_AUTOSN;
}

void init_VEML6030(void)
{
  VEML6030_Init(&dev_obj);
  sl_udelay_wait(3000);

  uint16_t register_07 = 0;
  uint8_t buffer[2];

  VEML6030_ReadID(&dev_obj, &register_07);

  buffer[0] = (uint8_t)(register_07 & 0xFF);         // Store the lower 8 bits
  buffer[1] = (uint8_t)((register_07 >> 8) & 0xFF);  // Store the upper 8 bits

  if((buffer[0] == VEML6030_ID) && (buffer[1] == 0xd4)) //0xd4 is code for slave address 0x90
    {
      printf("Register_07 -> Device ID: 0x%x \r\n", register_07);
    }
  else
    {
      printf("Error in Register_07: 0x%x \r\n", register_07);
    }
}

void configure_VEML6030(int32_t integration_time, uint32_t gain, bool IT, bool GAIN)
{
  //printf("configure_VEML6030() enter \r\n");

  if(IT)
   {
      switch (integration_time)
      {
          case -2:
              set_exposure_time = VEML6030_CONF_IT25;
              lux_resolution = (gain == 4) ? 0.1344 : ((gain == 3) ? 0.2688 : ((gain == 2) ? 1.0752 : 2.1504));
              break;
          case -1:
              set_exposure_time = VEML6030_CONF_IT50;
              lux_resolution = (gain == 4) ? 0.0672 : ((gain == 3) ? 0.1344 : ((gain == 2) ? 0.5376 : 1.0752));
              break;
          case 0:
              set_exposure_time = VEML6030_CONF_IT100;
              lux_resolution = (gain == 4) ? 0.0336 : ((gain == 3) ? 0.0672 : ((gain == 2) ? 0.2688 : 0.5376));
              break;
          case 1:
              set_exposure_time = VEML6030_CONF_IT200;
              lux_resolution = (gain == 4) ? 0.0168 : ((gain == 3) ? 0.0336 : ((gain == 2) ? 0.1344 : 0.2688));
              break;
          case 2:
              set_exposure_time = VEML6030_CONF_IT400;
              lux_resolution = (gain == 4) ? 0.0084 : ((gain == 3) ? 0.0168 : ((gain == 2) ? 0.0672f : 0.1344));
              break;
          case 3:
              set_exposure_time = VEML6030_CONF_IT800;
              lux_resolution = (gain == 4) ? 0.0042 : ((gain == 3) ? 0.0084 : ((gain == 2) ? 0.0336 : 0.0672));
              break;
          default:
              printf("Invalid integration time.\r\n");
              return;
      }

   }
  else if(GAIN)
    {
      switch (gain)
      {
        case 1:
            set_gain_setting = VEML6030_CONF_GAIN_1_8;
            break;
        case 2:
            set_gain_setting = VEML6030_CONF_GAIN_1_4;
            break;
        case 3:
              set_gain_setting = VEML6030_CONF_GAIN_1;
              break;
        case 4:
              set_gain_setting = VEML6030_CONF_GAIN_2;
              break;
          default:
              printf("Invalid gain setting.\r\n");
              return;
      }
    }

  // Get exposure time and gain
  //VEML6030_GetExposureTime(&dev_obj, &get_exposure_Time);
  VEML6030_GetInterMeasurementTime(&dev_obj, &get_exposure_Time);
  VEML6030_GetGain(&dev_obj, 0, &get_gain_setting);
  VEML6030_GetPersistence(&dev_obj, &get_ppersistence);

  /*printf("Get Integration Time: %ld ms, Gain: %lu, Get Exposure Time: %lu, "
      "Get Gain Mode: %lu, Get Persistence: %lu, Lux_Resolution: %.4f\r\n", integration_time, gain,
      (get_exposure_Time>>6), (get_gain_setting>>11), (get_ppersistence>>4), lux_resolution);*/

  // Set exposure time and gain
  //VEML6030_SetExposureTime(&dev_obj, set_exposure_time);
  VEML6030_SetInterMeasurementTime(&dev_obj, set_exposure_time);
  VEML6030_SetGain(&dev_obj, 0, set_gain_setting);
  VEML6030_SetPersistence(&dev_obj, set_ppersistence);

  /*printf("Set Integration Time: %ld ms, Gain: %lu, Set Exposure Time: %lu, "
      "Set Gain Mode: %lu, Set Persistence: %lu, Lux_Resolution: %.4f\r\n", integration_time, gain,
      (set_exposure_time>>6), (set_gain_setting>>11), (set_ppersistence>>4), lux_resolution);*/

  // Get exposure time and gain
  //VEML6030_GetExposureTime(&dev_obj, &get_exposure_Time);
  VEML6030_GetInterMeasurementTime(&dev_obj, &get_exposure_Time);
  VEML6030_GetGain(&dev_obj, 0, &get_gain_setting);
  VEML6030_GetPersistence(&dev_obj, &get_ppersistence);


   /*printf("Get Integration Time: %ld ms, Gain: %lu, Get Exposure Time: %lu, "
       "Get Gain Mode: %lu, Get Persistence: %lu, Lux_Resolution: %.4f\r\n", integration_time, gain,
       (get_exposure_Time>>6), (get_gain_setting>>11), (get_ppersistence>>4), lux_resolution);*/

  //printf("configure_VEML6030() exit \r\n");
}

void get_VEML6030_Values(int32_t integration_time, uint32_t gain)
{
  //printf("get_VEML6030_Values() enter \r\n");
  uint16_t lux_VEML = 0;
  float lux_VEML_float = 0;
  uint16_t counts = 0 ;
  bool isLuxVEML = false;
  bool isLuxCalculated = false;
  while(!isLuxVEML)
  {
    VEML6030_GetValues_ALS(&dev_obj, &counts);
    //printf("\r\n");
    //printf("ALS Count is: %u\r\n", counts);
    //printf("%u\r\n", counts);
    if(counts <= 100)
    {
      VEML6030_Stop(&dev_obj);
      gain = (gain == 4) ? (gain = 4) : (gain + 1);
      configure_VEML6030(integration_time, gain, false, true);
      if(gain == 4)
      {
        integration_time = (integration_time == 3) ? (integration_time = 3) : (integration_time + 1);
        configure_VEML6030(integration_time, gain, true, true);
        if(integration_time == 3)
        {
          VEML6030_Start(&dev_obj, 1);
          sl_udelay_wait(100000);
          VEML6030_GetValues_ALS(&dev_obj, &counts);
          //printf("For IT[%ld]ms, Gain[%lu] -> ALS Counts[%u] lx\r\n", integration_time, gain, counts);
          lux_VEML       = (counts * lux_resolution);
          lux_VEML_float = (counts * lux_resolution);
          printf("**Lux_VEML**: %u lx, %.4f lx\r\n", lux_VEML, lux_VEML_float);
          //printf("%f\r\n", lux_VEML_float);
          isLuxVEML = true;
        }
        else
        {
          VEML6030_Start(&dev_obj, 1);
          sl_udelay_wait(100000);
          isLuxVEML = false;
          //printf("For IT[%ld]ms, \r\n", integration_time);
        }
      }
      else
      {
        VEML6030_Start(&dev_obj, 1);
        sl_udelay_wait(100000);
        isLuxVEML = false;
      }
    }
    else if((counts >= 100) && (counts < 65535))
    {
      isLuxVEML = true; //to turn off the first while loop
      gain = 1;
      while(!isLuxCalculated)
      {
        if(counts < 10000)
        {
          configure_VEML6030(integration_time, gain, true, false);
          VEML6030_GetValues_ALS(&dev_obj, &counts);
          //printf("No : For IT[%ld]ms, Gain[%lu] -> ALS Counts[%u] lx\r\n", integration_time, gain, counts);
          isLuxCalculated = true;
        }
        else if(counts > 10000)
        {
          integration_time = (integration_time == -2) ? (integration_time = -2) : (integration_time - 1);
          if(integration_time == -2)
          {
            configure_VEML6030(integration_time, gain, true, false);
            VEML6030_GetValues_ALS(&dev_obj, &counts);
            //printf("Yes: For IT[%ld]ms, Gain[%lu] -> ALS Counts[%u] lx\r\n", integration_time, gain, counts);
            isLuxCalculated = true;
          }
          else
          {
            //printf("Current Integration time is %ld \r\n", integration_time);
            configure_VEML6030(integration_time, gain, true, false);
            VEML6030_GetValues_ALS(&dev_obj, &counts);
            //printf("ALS Count in else case: %u\r\n", counts);

          }
        }
      }
    }
    else
    {
       //printf("##### ALS Count is greater than 65535: #####\r\n");
       isLuxVEML = true;
    }
  }

  /*
  VEML6030_Stop(&dev_obj);
  uint16_t white_values;

  VEML6030_GetValues_WHITE(&dev_obj, &white_values);
  printf("Count White Value is: %u\n", white_values);
  */

  if(isLuxCalculated)
  {
    lux_VEML = (counts * lux_resolution);

    if(lux_VEML>1000)
      {
        //Correcting the lux value

        uint16_t poly_coeff = pow(6.0135, -13)*pow(lux_VEML, 4) + pow(-9.3924, -9)*pow(lux_VEML, 3)
                                + pow(8.1488, -5)*pow(lux_VEML, 2) + pow(1.0023, 1)*pow(lux_VEML, 1);

        //uint32_t poly_coeff = pow(6.0135, -13)*pow(lux, 4) + pow(-9.3924, -9)*pow(lux, 3)
                              //  + pow(8.1488, -5)*pow(lux, 2) + pow(1.0023, 1)*pow(lux, 1);

        printf("***Final Lux_Intensity(Corrected)***: %u lx\r\n", poly_coeff);
        //printf("%u\r\n", poly_coeff);
      }
    else
      {
        printf("***Lux_VEML***: %u lx\r\n", lux_VEML);
        //printf("%u\r\n", lux_VEML);
      }
  }

//printf("get_VEML6030_Values() exit \r\n");
}

void enable_I2C(void)
{
  initCMU();
  initI2C();
  I2C_Enable(I2C0, true);
}

/*void shutdown_VEML6030(void)
{
  VEML6030_Shutdown(&dev_obj);
}*/

void start_VEML6030(void)
{
  uint32_t mode = 1;
  VEML6030_Start(&dev_obj, mode);
}


/*void Stop_VEML6030(void)
{
  VEML6030_Stop(&dev_obj);
}*/

void DeInit_VEML6030(void)
{
  //printf("DeInit_VEML6030 enter\r\n");
  VEML6030_DeInit(&dev_obj);
  //printf("DeInit_VEML6030 exit\r\n");
}
