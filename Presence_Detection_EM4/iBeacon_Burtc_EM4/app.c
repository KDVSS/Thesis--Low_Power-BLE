/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include <stdio.h>
#include <configure_presence.h>
//#include <burtc_i2c_EM4.h>

#include "sl_bluetooth.h"
#include "app_assert.h"
#include "app.h"

#include "em_emu.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_rmu.h"
#include "em_burtc.h"

// Number of 1 KHz ULFRCO clocks between BURTC interrupts
#define BURTC_IRQ_PERIOD  30000
// Macros.
#define UINT16_TO_BYTES(n)            ((uint8_t) (n)), ((uint8_t)((n) >> 8))
#define UINT16_TO_BYTE0(n)            ((uint8_t) (n))
#define UINT16_TO_BYTE1(n)            ((uint8_t) ((n) >> 8))

#define SIGNAL_LOSS_AT_1_M_IN_DBM     41 // The beacon's measured RSSI at 1 m

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

volatile bool enter_EM4 = false;
volatile bool adv_presence = false;


PACKSTRUCT(static struct {
    uint8_t flags_len;     // Length of the Flags field.
    uint8_t flags_type;    // Type of the Flags field.
    uint8_t flags;         // Flags field.
    uint8_t mandata_len;   // Length of the Manufacturer Data field.
    uint8_t mandata_type;  // Type of the Manufacturer Data field.
    uint8_t comp_id[2];    // Company ID field.
    uint8_t beac_type[2];  // Beacon Type field.
    uint8_t uuid[16];      // 128-bit Universally Unique Identifier (UUID). The UUID is an identifier for the company using the beacon.
    uint8_t maj_num[2];    // Beacon major number. Used to group related beacons.
    uint8_t min_num[2];    // Beacon minor number. Used to specify individual beacons within a group.
    int8_t tx_power;       // The Beacon's measured RSSI at 1 meter distance in dBm. See the iBeacon specification for measurement guidelines.
  })
  bcn_beacon_adv_data
    = {
    // Flag bits - See Bluetooth 4.0 Core Specification , Volume 3, Appendix C, 18.1 for more details on flags.
    2,            // Length of field.
    0x01,         // Type of field.
    0x04 | 0x02,  // Flags: LE General Discoverable Mode, BR/EDR is disabled.

    // Manufacturer specific data.
    26,   // Length of field.
    0xFF, // Type of field.

    // The first two data octets shall contain a company identifier code from
    // the Assigned Numbers - Company Identifiers document.
    // 0x004C = Apple
    { UINT16_TO_BYTES(0x004C) },

    // Beacon type.
    // 0x0215 is iBeacon.
    { UINT16_TO_BYTE1(0x0215), UINT16_TO_BYTE0(0x0215) },

    // 128 bit / 16 byte UUID
    /*{ 0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48, 0xD2, \
      0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0 },*/
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },

    // Beacon major number.
    // Set to 34987 and converted to correct format.
    { UINT16_TO_BYTE1(34987), UINT16_TO_BYTE0(34987) },

    // Beacon minor number.
    // Set as 1025 and converted to correct format.
    { UINT16_TO_BYTE1(1025), UINT16_TO_BYTE0(1025) },

    // A dummy value which will be eventually overwritten
    0
    };


/**************************************************************************//**
 * Set up a custom advertisement package according to iBeacon specifications.
 * The advertisement package is 30 bytes long.
 * See the iBeacon specification for further details.
 *****************************************************************************/
static void bcn_setup_adv_beaconing(void);

/***************************************************************************//**
 * @brief Enable clocks
 ******************************************************************************/
void initCMUClocks(void)
{
  printf("initCMUClocks() \r\n");
  // Disable clocks to the GPIO
  CMU_ClockEnable(cmuClock_GPIO, true);
}

/***************************************************************************//**
 * @brief Disable clocks
 ******************************************************************************/
void deInitCMUClocks(void)
{
  printf("deInitCMUClocks() \r\n");
  // Disable clocks to the GPIO
  CMU_ClockEnable(cmuClock_GPIO, false);
}

/**************************************************************************//**
 * @brief  Turn ON Presence Detector
 *****************************************************************************/
void presenceDetectorPowerON(void)
{
  printf("presenceDetectorPowerON()\r\n");
  GPIO_PinModeSet(gpioPortC, 3, gpioModePushPull, 1);
}

/**************************************************************************//**
 * @brief  Turn OFF Presence Detector
 *****************************************************************************/
void presenceDetectorPowerOFF(void)
{
  printf("presenceDetectorPowerOFF()\r\n");
  GPIO_PinModeSet(gpioPortC, 3, gpioModePushPull, 0);
}

/**************************************************************************//**
 * @brief  BURTC Handler
 *****************************************************************************/
void BURTC_IRQHandler(void)
{
  printf("BURTC_IRQHandler() ");
  BURTC_IntClear(BURTC_IF_COMP); // compare match
  BURTC_CounterReset(); // reset BURTC counter to wait full ~5 sec before EM4 wakeup
}

/**************************************************************************//**
 * @brief  Initialize GPIOs for push button and LED
 *****************************************************************************/
void initGPIO(void)
{
  //GPIO_PinModeSet(gpioPortC, 3, gpioModePushPull, 1);
}

/**************************************************************************//**
 * @brief  Configure BURTC to interrupt every BURTC_IRQ_PERIOD and
 *         wake from EM4
 *****************************************************************************/
void initBURTC(void)
{
  CMU_ClockSelectSet(cmuClock_EM4GRPACLK, cmuSelect_ULFRCO);
  CMU_ClockEnable(cmuClock_BURTC, true);
  CMU_ClockEnable(cmuClock_BURAM, true);

  BURTC_Init_TypeDef burtcInit = BURTC_INIT_DEFAULT;
  burtcInit.compare0Top = true; // reset counter when counter reaches compare value
  burtcInit.em4comp = true;     // BURTC compare interrupt wakes from EM4 (causes reset)
  BURTC_Init(&burtcInit);

  BURTC_CounterReset();
  BURTC_CompareSet(0, BURTC_IRQ_PERIOD);

  BURTC_IntEnable(BURTC_IEN_COMP);    // compare match
  NVIC_EnableIRQ(BURTC_IRQn);
  BURTC_Enable(true);
}

/**************************************************************************//**
 * @brief Check RSTCAUSE for EM4 wakeups (reset) and save wakeup count
 *          to BURAM
 *****************************************************************************/
void checkResetCause (void)
{
  uint32_t cause = RMU_ResetCauseGet();

  RMU_ResetCauseClear();

  // Print reset cause
  if (cause & EMU_RSTCAUSE_PIN)
  {
    printf("-- RSTCAUSE = PIN \n");
    BURAM->RET[0].REG = 0; // reset EM4 wakeup counter
  }
  else if (cause & EMU_RSTCAUSE_EM4)
  {
    printf("-- RSTCAUSE = EM4 wakeup \n");
    BURAM->RET[0].REG += 1; // increment EM4 wakeup counter
  }
  // Print # of EM4 wakeups
  printf("-- Number of EM4 wakeups = %ld \n", BURAM->RET[0].REG);
  //printf("-- BURTC ISR will toggle LED every ~3 seconds \n");
}

void resetBurtc_and_enterEM4(void)
{
  //BURTC_CounterReset(); // reset BURTC counter to wait full ~5 sec before EM4 wakeup
  //printf("-- BURTC counter reset \n");

  // Enter EM4
  printf("Entering EM4 and wake on BURTC compare in ~5 seconds \n\n");
  //RETARGET_SerialFlush(); // delay for printf to finish
  EMU_EnterEM4();
}

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{
  EMU_UnlatchPinRetention();

  initBURTC();

/*
  // Enable the below lines only if EM4 is running on empty project. Otherwise
  // EM4 initialization done during platform init via sl_device_init_emu().
  EMU_EM4Init_TypeDef em4Init = EMU_EM4INIT_DEFAULT;
  EMU_EM4Init(&em4Init);
*/

  // Check RESETCAUSE, update and print EM4 wakeup count
  checkResetCause();

  printf("\r\nIn EM0 \r\n");
  initCMUClocks();
  presenceDetectorPowerON();

  reInitialise_I2C();
  setup_I2C_Read_Write();
  verify_sths34pf80_ID();
  configure_sths34pf80();

}

static void presence_measurement_val_to_buf(int16_t value,
                                            uint8_t *buffer)
{
  int16_t originalValue = value;

  buffer[0] = (uint8_t)((value >> 8) & 0xFF); // Store the upper 8 bits
  buffer[1] = (uint8_t)(value & 0xFF);        // Store the lower 8 bits

  // Reconstructing the value from the buffer
  int16_t reconstructedValue = (int16_t)((buffer[0] << 8) | buffer[1]);

  // Verify if the original and reconstructed values are the same
  if (originalValue != reconstructedValue) {
      //app_log_warning("Error in conversion. Original: %d, Reconstructed: %d\n", originalValue, reconstructedValue);
  }
}

void adv_presence_data(void)
{
  int16_t t_presence_raw = 0;
  uint8_t presence_value[2];
  sl_status_t sc;

  get_presence(1, &t_presence_raw);
  presenceDetectorPowerOFF();
  disable_I2C();

  if(t_presence_raw != 0)
  {
    printf("T_presence_raw: 0x%x (hex) <-> %d (dec)\r\n", t_presence_raw, t_presence_raw);
    presence_measurement_val_to_buf(t_presence_raw, presence_value);

    bcn_beacon_adv_data.uuid[0] = presence_value[0];
    bcn_beacon_adv_data.uuid[1] = presence_value[1];

    /*
     * printf("UINT16_TO_BYTE1: 0x%x, UINT16_TO_BYTE0:  0x%x  ", UINT16_TO_BYTE1(t_presence_raw),
           UINT16_TO_BYTE0(t_presence_raw));

    bcn_beacon_adv_data.uuid[0] = UINT16_TO_BYTE1(t_presence_raw);
    bcn_beacon_adv_data.uuid[1] = UINT16_TO_BYTE0(t_presence_raw);

    */

    //Set Advertising data
    sc = sl_bt_legacy_advertiser_set_data(advertising_set_handle,
                                         sl_bt_advertiser_advertising_data_packet,
                                         sizeof(bcn_beacon_adv_data),
                                         (const uint8_t *)&bcn_beacon_adv_data);
    app_assert_status(sc);

    // Start advertising and enable connections.
    sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                      sl_bt_advertiser_non_connectable);
    app_assert_status(sc);
  }
  enter_EM4 = true;
  sl_bt_system_set_lazy_soft_timer(16384,
                                   0,
                                   advertising_set_handle,
                                   1);
}


/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{

}
/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  int16_t ret_power_min, ret_power_max;
  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Set 0 dBm maximum Transmit Power.
      sc = sl_bt_system_set_tx_power(SL_BT_CONFIG_MIN_TX_POWER, 0,
                                     &ret_power_min, &ret_power_max);
      app_assert_status(sc);
      (void)ret_power_min;
      (void)ret_power_max;
      // Initialize iBeacon ADV data.
      bcn_setup_adv_beaconing();

      //1 Millisecond = 32.768, 500millsecond = 32.768*500 = 16384
      sl_bt_system_set_lazy_soft_timer(16384,
                                       0,
                                       advertising_set_handle,
                                       1);
      adv_presence = true;

      break;

    case sl_bt_evt_system_soft_timer_id:
      printf("sl_bt_evt_system_soft_timer_id is expired \r\n");

      if(enter_EM4)
      {
        /*sc = sl_bt_advertiser_stop(advertising_set_handle);
        app_assert_status(sc); */
        printf("lazy_2_Timer (enter_EM4) -> portC: %d, SDA: %d, SCL: %d\r\n",
                   GPIO_PinOutGet(gpioPortC, 3),
                   GPIO_PinOutGet(gpioPortD, 2),
                   GPIO_PinOutGet(gpioPortD, 3));
        resetBurtc_and_enterEM4();
        enter_EM4 = false;
      }
      if(adv_presence)
      {
        printf("lazy_1_Timer (adv_presence) -> portC: %d, SDA: %d, SCL: %d\r\n",
                   GPIO_PinOutGet(gpioPortC, 3),
                   GPIO_PinOutGet(gpioPortD, 2),
                   GPIO_PinOutGet(gpioPortD, 3));
        adv_presence_data();
        adv_presence = false;
      }
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

static void bcn_setup_adv_beaconing(void)
{
  sl_status_t sc;
  int16_t support_min;
  int16_t support_max;
  int16_t set_min;
  int16_t set_max;
  int16_t rf_path_gain;
  int16_t calculated_power;

  // Create an advertising set.
  sc = sl_bt_advertiser_create_set(&advertising_set_handle);
  app_assert_status(sc);

  sc = sl_bt_system_get_tx_power_setting(&support_min, &support_max, &set_min, &set_max, &rf_path_gain);
  app_assert_status(sc);

  calculated_power = (set_max > 0) ? (set_max + 5) / 10 : (set_max - 5) / 10;
  calculated_power = calculated_power - SIGNAL_LOSS_AT_1_M_IN_DBM;

  if (calculated_power > INT8_MAX) {
    bcn_beacon_adv_data.tx_power = INT8_MAX;
  } else if (calculated_power < INT8_MIN) {
    bcn_beacon_adv_data.tx_power = INT8_MIN;
  } else {
    bcn_beacon_adv_data.tx_power = calculated_power;
  }

  // Set custom advertising data.
  sc = sl_bt_legacy_advertiser_set_data(advertising_set_handle,
                                        0,
                                        sizeof(bcn_beacon_adv_data),
                                        (uint8_t *)(&bcn_beacon_adv_data));
  app_assert_status(sc);

  // Set advertising parameters. 100ms advertisement interval.
  sc = sl_bt_advertiser_set_timing(
    advertising_set_handle,
    160,     // min. adv. interval (milliseconds * 1.6)
    160,     // max. adv. interval (milliseconds * 1.6)
    100,       // adv. duration
    5);      // max. num. adv. events
  app_assert_status(sc);

  // Start advertising in user mode and disable connections.
  // sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                    // sl_bt_legacy_advertiser_non_connectable);
  app_assert_status(sc);
}
