#include <configure_presence.h>
#include <stdio.h>
#include <string.h>

#include "em_device.h"
#include "em_chip.h"
#include "em_i2c.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "sl_udelay.h"

#include "sl_status.h"

#include "drivers/sths34pf80_reg.h"

// Defines
/** I2C Device Address 7 bit format **/
#define STHS34PF80_I2C_SLAVE_ADD          0x5AU    // 1011010

/** I2C Device Address 8 bit format **/
#define STHS34PF80_I2C_ADDRESS (STHS34PF80_I2C_SLAVE_ADD << 1) // 10110101 (0xB4)

/** Device Identification (Who am I) **/
#define STHS34PF80_ID                     0xD3U

/** Register address for Device Identification (Who am I) **/
#define STHS34PF80_WHO_AM_I               0x0FU

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

//extern sl_iostream_t *sl_iostream_vcom_handle; // need to extend the structure at the top of the file to use the sl_iostream_write()

static stmdev_ctx_t dev_ctx;

/***************************************************************************//**
 * @brief I2C write numBytes to STHS34PF80 device starting at target address
 ******************************************************************************/
int32_t sths34pf80_I2C_write_to_reg(void *handle,
                                    uint8_t targetAddress,
                                    const uint8_t *txBuff,
                                    uint16_t numBytes)
{
  (void)handle;

  // Transfer structure
  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef result;
  uint8_t i2c_write_data[numBytes + 1];
  uint8_t i2c_read_data[1];

  seq.addr          = STHS34PF80_I2C_ADDRESS;
  seq.flags         = I2C_FLAG_WRITE;

  i2c_write_data[0] = targetAddress;
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

  /*
      uint8_t i2c_read_data_dummy[1];
  for (int i = 0; i < 1; i++) {
      i2c_read_data_dummy[i] = i2c_read_data[i];
  }
   */

  return 0;
}

/***************************************************************************//**
 * @brief I2C read numBytes from STHS34PF80 device starting at target address
 ******************************************************************************/
int32_t sths34pf80_I2C_read_from_reg(void *handle,
                                     uint8_t targetAddress,
                                     uint8_t *rxBuff,
                                     uint16_t numBytes)
{
  (void)handle;

  // Transfer structure
  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef result;
  uint8_t i2c_write_data[1];

  seq.addr          = STHS34PF80_I2C_ADDRESS;
  seq.flags         = I2C_FLAG_WRITE_READ; // must write target address before reading

  i2c_write_data[0] = targetAddress;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;

  /* Select length of data to be read */
  seq.buf[1].data   = rxBuff;
  seq.buf[1].len    = numBytes;

  result = I2C_TransferInit(I2C0, &seq);
  while (result == i2cTransferInProgress) {
      result = I2C_Transfer(I2C0);
    }
  //printf("Read: result for I2C_Transfer  %d \r\n", result);
  if (result != i2cTransferDone) {
      // Indicate I2C transmission problem
      printf("result != i2cTransferDone \r\n");
      return -1;
    }
  return 0;
}

void setup_I2C_Read_Write(void)
{
  dev_ctx.write_reg = sths34pf80_I2C_write_to_reg;
  dev_ctx.read_reg = sths34pf80_I2C_read_from_reg;
  //dev_ctx.handle = STHS34PF80_I2C_ADDRESS;
}

/***************************************************************************//**
 * @brief I2C Read/Verify
 ******************************************************************************/
void verify_sths34pf80_ID(void)
{
  //printf("verify_sths34pf80_ID() is enter \r\n");

  /* Prevent buffering of output/input.*/
#if !defined(__CROSSWORKS_ARM) && defined(__GNUC__)
  setvbuf(stdout, NULL, _IONBF, 0);   /*Set unbuffered mode for stdout (newlib)*/
  setvbuf(stdin, NULL, _IONBF, 0);   /*Set unbuffered mode for stdin (newlib)*/
#endif

  /*
  const char str2[] = "IOStream Example 1 \r\n\r\n";
  for(int i=0; i< strlen(str2); i++)
  {
        USART_Tx(USART1, str2[i]);
  }
  */

  /*extern sl_iostream_t *sl_iostream_vcom_handle; // need to extend the
  structure at the top of the file to use the sl_iostream_write()*/
  //char buffer[] = "whoami: 0xd3";
  //sl_iostream_write(sl_iostream_vcom_handle, buffer, strlen(buffer));

  uint8_t whoami;

  /* Check device ID */
  sths34pf80_device_id_get(&dev_ctx, &whoami);

  if (whoami != STHS34PF80_ID)
  {
    printf("Who_Am_I != STHS34PF80_ID \r\n");
  }
  printf("Who_Am_I: 0x%x \r\n", whoami);
}

void configure_sths34pf80(void)
{
  sths34pf80_lpf_bandwidth_t lpf_m, lpf_p, lpf_p_m, lpf_a_t;

  /* Set averages (AVG_TAMB = 8, AVG_TMOS = 32) */
  sths34pf80_avg_tobject_num_set(&dev_ctx, STHS34PF80_AVG_TMOS_32);
  sths34pf80_avg_tambient_num_set(&dev_ctx, STHS34PF80_AVG_T_8);

  sths34pf80_lpf_m_bandwidth_get(&dev_ctx, &lpf_m);
  sths34pf80_lpf_p_bandwidth_get(&dev_ctx, &lpf_p);
  sths34pf80_lpf_p_m_bandwidth_get(&dev_ctx, &lpf_p_m);
  sths34pf80_lpf_a_t_bandwidth_get(&dev_ctx, &lpf_a_t);

  /*
  char bandwidth_lpf[60];
  sprintf(bandwidth_lpf, "Bandwidth of lpf_m: %d, lpf_p: %d, lpf_p_m: %d, lpf_a_t: %d: \r\n\r\n",
                          lpf_m, lpf_p, lpf_p_m, lpf_a_t);
  printf("%s", bandwidth_lpf);
  memset(bandwidth_lpf, '\0', sizeof(bandwidth_lpf)); */

  printf("Bandwidth of lpf_m: %d, lpf_p: %d, lpf_p_m: %d, lpf_a_t: %d: \r\n", lpf_m, lpf_p, lpf_p_m, lpf_a_t);

  uint8_t set_bdu = 1;
  sths34pf80_block_data_update_set(&dev_ctx, set_bdu);

  sths34pf80_presence_threshold_set(&dev_ctx, 100);
  sths34pf80_presence_hysteresis_set(&dev_ctx, 20);
  sths34pf80_motion_threshold_set(&dev_ctx, 150);
  sths34pf80_motion_hysteresis_set(&dev_ctx, 30);

  sths34pf80_algo_reset(&dev_ctx);

  /* Set interrupt */
  sths34pf80_int_or_set(&dev_ctx, STHS34PF80_INT_PRESENCE);
  sths34pf80_route_int_set(&dev_ctx, STHS34PF80_INT_OR);

  /* Set ODR */
  sths34pf80_odr_set(&dev_ctx, STHS34PF80_ODR_AT_30Hz);
  //printf("configure_sths34pf80() EXIT \r\n");
}

void reConfigure_sths34pf80(void)
{
  sths34pf80_lpf_bandwidth_t lpf_m, lpf_p, lpf_p_m, lpf_a_t;

  /* Set averages (AVG_TAMB = 8, AVG_TMOS = 32) */
  sths34pf80_avg_tobject_num_set(&dev_ctx, STHS34PF80_AVG_TMOS_32);
  sths34pf80_avg_tambient_num_set(&dev_ctx, STHS34PF80_AVG_T_8);

  sths34pf80_lpf_m_bandwidth_get(&dev_ctx, &lpf_m);
  sths34pf80_lpf_p_bandwidth_get(&dev_ctx, &lpf_p);
  sths34pf80_lpf_p_m_bandwidth_get(&dev_ctx, &lpf_p_m);
  sths34pf80_lpf_a_t_bandwidth_get(&dev_ctx, &lpf_a_t);

  /*
  char bandwidth_lpf[60];
  sprintf(bandwidth_lpf, "Bandwidth of lpf_m: %d, lpf_p: %d, lpf_p_m: %d, lpf_a_t: %d: \r\n\r\n",
                          lpf_m, lpf_p, lpf_p_m, lpf_a_t);
  printf("%s", bandwidth_lpf);
  memset(bandwidth_lpf, '\0', sizeof(bandwidth_lpf)); */

  printf("Bandwidth of lpf_m: %d, lpf_p: %d, lpf_p_m: %d, lpf_a_t: %d: \r\n\r\n", lpf_m, lpf_p, lpf_p_m, lpf_a_t);

  uint8_t set_bdu = 1;
  sths34pf80_block_data_update_set(&dev_ctx, set_bdu);
/*
  sths34pf80_presence_threshold_set(&dev_ctx, 200);
  sths34pf80_presence_hysteresis_set(&dev_ctx, 20);
  sths34pf80_motion_threshold_set(&dev_ctx, 300);
  sths34pf80_motion_hysteresis_set(&dev_ctx, 30);

  sths34pf80_algo_reset(&dev_ctx);
 */

  /* Set interrupt */
  sths34pf80_int_or_set(&dev_ctx, STHS34PF80_INT_PRESENCE);
  sths34pf80_route_int_set(&dev_ctx, STHS34PF80_INT_OR);

  /* Set ODR */
  sths34pf80_odr_set(&dev_ctx, STHS34PF80_ODR_AT_30Hz);
  printf("reConfigure_sths34pf80() EXIT \r\n");
}

void get_presence(int number_of_samples, int16_t *t_presence_raw)
{
  (void)number_of_samples;
  uint8_t presence = 0;
  sths34pf80_func_status_t func_status;

  sths34pf80_func_status_get(&dev_ctx, &func_status);
  presence = func_status.pres_flag;

  if (presence)
   {
     printf("PRESENCE DETECTED (PRES_FLAG-> %d): \r\n", func_status.pres_flag);
     sths34pf80_tpresence_raw_get(&dev_ctx, t_presence_raw);
   }
  else
    {
      printf("No PRESENCE DETECTED (PRES_FLAG-> %d): \r\n", func_status.pres_flag);
    }
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

/***************************************************************************//**
 * @brief GPIO Interrupt handler
 ******************************************************************************/
sl_status_t reInitialise_I2C()
{
  initCMU();
  initI2C();

  // Re-enable I2C
  I2C_Enable(I2C0, true);
  return SL_STATUS_OK;
}

void disable_I2C(void)
{
  //printf("disable_I2C() Enter\r\n");
  //I2C_Reset(I2C0);
  I2C_Enable(I2C0, false);

  GPIO->I2CROUTE[0].SDAROUTE = 0;
  GPIO->I2CROUTE[0].SCLROUTE = 0;
  GPIO->I2CROUTE[0].ROUTEEN =  0;

  // Disabled SCL (P0D2) and SDA (P0D3) pins
  // gpioModeDisabled = 1 then presence is 225uA, sleep 139 uA
  // gpioModeDisabled = 0 then presence is 445uA, sleep 59 uA
  // gpioModePushPull = 0 then presence is 390uA, sleep 59 uA

  GPIO_PinModeSet(I2C_SCL_PORT, I2C_SCL_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(I2C_SDA_PORT, I2C_SDA_PIN, gpioModePushPull, 0);


  GPIO->I2CROUTE[0].SDAROUTE = 0;
  GPIO->I2CROUTE[0].SCLROUTE = 0;
  GPIO->I2CROUTE[0].ROUTEEN =  0;
/*
  GPIO->P[gpioPortD].MODEH = 0x0;
  GPIO->P[gpioPortD].MODEL = 0x0;
*/
  CMU_ClockEnable(cmuClock_I2C0, false);
  //printf("disable_I2C() Exit\r\n");
}