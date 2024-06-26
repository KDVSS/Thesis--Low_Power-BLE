#include <stdio.h>
#include <configure_presence.h>

#include "sl_bluetooth.h"
#include "sl_udelay.h"

#include "app_assert.h"
#include "app.h"
#include "app_timer.h"

#include "em_emu.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_rmu.h"
#include "em_burtc.h"

#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_iadc.h"
#include "em_prs.h"


// Use specified PRS channel
#define IADC_PRS_CH              0

// Set CLK_ADC to 10MHz (this corresponds to a sample rate of 77K with OSR = 32)
// CLK_SRC_ADC; largest division is by 4
#define CLK_SRC_ADC_FREQ        20000000

// CLK_ADC; IADC_SCHEDx PRESCALE has 10 valid bits
#define CLK_ADC_FREQ            10000000

/*
 * Specify the IADC input using the IADC_PosInput_t typedef.  This
 * must be paired with a corresponding macro definition that allocates
 * the corresponding ABUS to the IADC.  These are...
 *
 * GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AEVEN0_ADC0
 * GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AODD0_ADC0
 * GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BEVEN0_ADC0
 * GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BODD0_ADC0
 * GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDEVEN0_ADC0
 * GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDODD0_ADC0
 *
 * ...for port A, port B, and port C/D pins, even and odd, respectively.
 */
#define IADC_INPUT_0_PORT_PIN     iadcPosInputPortBPin0;

#define IADC_INPUT_0_BUS          BBUSALLOC
#define IADC_INPUT_0_BUSALLOC     GPIO_BBUSALLOC_BEVEN0_ADC0

// GPIO output toggle to notify IADC conversion complete
#define GPIO_OUTPUT_0_PORT        gpioPortA
#define GPIO_OUTPUT_0_PIN         4

/*******************************************************************************
***************************   GLOBAL VARIABLES   *******************************
 ******************************************************************************/

/*
 * This example enters EM2 in the main while() loop; Setting this #define to 1
 * enables debug connectivity in EM2, which increases current consumption by
 * about 0.5uA
 */
#define EM2DEBUG                  1

/*******************************************************************************
 ***************************   GLOBAL VARIABLES   ******************************
 ******************************************************************************/

// Stores latest ADC sample and converts to volts
static volatile IADC_Result_t sample;
static volatile double singleResult;

void my_IADC_enable(IADC_TypeDef *iadc);
void my_IADC_disable(IADC_TypeDef *iadc);
void my_IADC_ReadChannel(void);



// Number of 1 KHz ULFRCO clocks between BURTC interrupts
#define BURTC_IRQ_PERIOD  5000
// Macros.
#define UINT16_TO_BYTES(n)            ((uint8_t) (n)), ((uint8_t)((n) >> 8))
#define UINT16_TO_BYTE0(n)            ((uint8_t) (n))
#define UINT16_TO_BYTE1(n)            ((uint8_t) ((n) >> 8))

#define SIGNAL_LOSS_AT_1_M_IN_DBM     41 // The beacon's measured RSSI at 1 m

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

static uint8_t advertising_set_handle_1 = 0xff;

static uint8_t int_part = 0;
static uint8_t decimal_part = 0;

volatile bool enter_EM3 = false;
volatile bool adv_presence = false;
static uint8_t em2_counter = 0;
static bool is_capacitor_valtage_enough = false;



//static app_timer_t app_periodic_timer;

//static void app_periodic_timer_cb(app_timer_t *timer, void *data);

#define SL_I2C_RECOVER_NUM_CLOCKS         10

// I2C pins (SCL = PD2; SDA = PD3)
#define I2C_SCL_PORT            gpioPortD
#define I2C_SCL_PIN             2
#define I2C_SDA_PORT            gpioPortD
#define I2C_SDA_PIN             3

static bool em2_is_enabled = true;
static bool em_mode_2 = false;

void adv_presence_data(void);

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

/*******************************************************************************
 * Clear flag and allow power manager to go lower then EM2
 ******************************************************************************/
static void clear_em2_mode(void);

/*******************************************************************************
 * Set flag and power manager to EM2 maximum sleep mode
 ******************************************************************************/
static void set_em2_mode(void);


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
 * @brief  GPIO Initializer
 *****************************************************************************/
void configureLED(void)
{
  // Configure GPIO as output, will indicate when conversions are being performed
  GPIO_PinModeSet(GPIO_OUTPUT_0_PORT, GPIO_OUTPUT_0_PIN, gpioModePushPull, 0);
}

void my_IADC_enable(IADC_TypeDef *iadc)
{
  iadc->EN_SET = IADC_EN_EN;
}

void my_IADC_disable(IADC_TypeDef *iadc)
{
#if defined(IADC_STATUS_SYNCBUSY)
  while ((iadc->STATUS & IADC_STATUS_SYNCBUSY) != 0U) {
    // Wait for synchronization to finish before disable
  }
#endif
  iadc->EN_CLR = IADC_EN_EN;
#if defined(_IADC_EN_DISABLING_MASK)
  while (IADC0->EN & _IADC_EN_DISABLING_MASK) {
  }
#endif
}

void my_IADC_ReadChannel(void)
{
  // Initialize the IADC
  my_IADC_enable(IADC0);

  // Start single
  IADC_command(IADC0, iadcCmdStartSingle);
}

/**************************************************************************//**
 * @brief  IADC Initializer
 *****************************************************************************/
void initIADC(void)
{
  // Declare init structs
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
  IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

  // Enable IADC clock
  CMU_ClockEnable(cmuClock_IADC0, true);

  // Reset IADC to reset configuration in case it has been modified
  IADC_reset(IADC0);

  // Configure IADC clock source for use while in EM2
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);

  // Modify init structs and initialize
  init.warmup = iadcWarmupKeepWarm;

  // Set the HFSCLK prescale value here
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

  // Configuration 0 is used by both scan and single conversions by default
  // Use internal bandgap (supply voltage in mV) as reference
  initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2;
  initAllConfigs.configs[0].vRef = 1210;
  initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain0P5x;

  // Divides CLK_SRC_ADC to set the CLK_ADC frequency for desired sample rate
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                    CLK_ADC_FREQ,
                                                                    0,
                                                                    iadcCfgModeNormal,
                                                                    init.srcClkPrescale);

  // Single initialization
  initSingle.dataValidLevel = iadcFifoCfgDvl1;

  // Set conversions to run continuously
  initSingle.triggerAction = iadcTriggerActionOnce;

  // Configure Input sources for single ended conversion
  initSingleInput.posInput = IADC_INPUT_0_PORT_PIN;
  initSingleInput.negInput = iadcNegInputGnd;

  // Initialize IADC
  // Note oversampling and digital averaging will affect the offset correction
  // This is taken care of in the IADC_init() function in the emlib
  IADC_init(IADC0, &init, &initAllConfigs);

  // Initialize SingleInput
  IADC_initSingle(IADC0, &initSingle, &initSingleInput);

  // Allocate the analog bus for ADC0 inputs
  GPIO->IADC_INPUT_0_BUS |= IADC_INPUT_0_BUSALLOC;

  // Enable interrupts on data valid level
  IADC_enableInt(IADC0, IADC_IEN_SINGLEDONE);

  // Enable ADC interrupts
  NVIC_ClearPendingIRQ(IADC_IRQn);
  NVIC_EnableIRQ(IADC_IRQn);
}

/**************************************************************************//**
 * @brief  IADC interrupt handler
 *****************************************************************************/
void IADC_IRQHandler(void)
{
  // Read most recent single conversion result
  sample = IADC_readSingleResult(IADC0);

  // Calculate input voltage:
  // For single-ended the result range is 0 to +Vref, i.e.,
  // for Vref = VBGR = 1.21V, and with analog gain = 0.5,
  // 12 bits represents 2.42V full scale IADC range.
  singleResult = sample.data * 2.42 / 0xFFF;

  singleResult = singleResult * 2 ; // To reflect the actual supercap voltage

  IADC_clearInt(IADC0, IADC_IF_SINGLEDONE);

  float float_value = singleResult;

  // Extract the integer part
  int_part = (uint8_t)float_value;

  // Extract the first digit of the fractional part
  decimal_part = (uint8_t)((float_value - int_part) * 10);

  // Print the integer values
  printf("%d.%dV\r\n", int_part, decimal_part);

  printf("***singleResult[%.2lfV], sample.data[%ld]***\r\n", singleResult, sample.data);

  if((singleResult >= 2.5) && (singleResult < 4.2)) // Todo: change to real supercap thresholds
    {
      is_capacitor_valtage_enough = true;
      printf("is_capacitor_valtage_enough value: %d \r\n", is_capacitor_valtage_enough);
    }
  else
    {
      // Disable the IADC
      //IADC_reset(IADC0);
      my_IADC_disable(IADC0);
      em_mode_2 = false;
      clear_em2_mode();
      printf("Voltage NOT within the limit, Enter_EM3 -> portC: %d, SDA: %d, SCL: %d\r\n",
                 GPIO_PinOutGet(gpioPortC, 3),
                 GPIO_PinOutGet(gpioPortD, 2),
                 GPIO_PinOutGet(gpioPortD, 3));
    }
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
 * @brief  BURTC Handler
 *****************************************************************************/
void BURTC_IRQHandler(void)
{
  printf("BURTC_IRQHandler() \r\n");

  //sl_udelay_wait(100000);
  //initGPIO();

  BURTC_IntClear(BURTC_IF_COMP); // compare match
  BURTC_CounterReset(); // reset BURTC counter to wait full ~5 sec before EM3 wakeup
  printf("-- BURTC counter reset \r\n");

  // Enable below two lines if app timer is not used.
  set_em2_mode();
  em_mode_2 = true;
}

/**************************************************************************//**
 * Timer callback
 * Called periodically to time periodic temperature measurements and indications.
 *****************************************************************************/
/*static void app_periodic_timer_cb(app_timer_t *timer, void *data)
{
  (void)data;
  (void)timer;
  printf("\r\n\r\nAPP_PERIODIC_TIMER_CB is called \r\n");
  set_em2_mode();
  em_mode_2 = true;

  printf("App timer exit \r\n");
}
*/
/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{
  //sl_status_t sc;

  initCMUClocks();
  printf("AppInt -> portC: %d, SDA: %d, SCL: %d\r\n",
             GPIO_PinOutGet(gpioPortC, 3),
             GPIO_PinOutGet(gpioPortD, 2),
             GPIO_PinOutGet(gpioPortD, 3));

  presenceDetectorPowerON();
  reInitialise_I2C();
  setup_I2C_Read_Write();
  verify_sths34pf80_ID();

  initBURTC();

  printf("AppInt -> portC: %d, SDA: %d, SCL: %d\r\n",
             GPIO_PinOutGet(gpioPortC, 3),
             GPIO_PinOutGet(gpioPortD, 2),
             GPIO_PinOutGet(gpioPortD, 3));
/*
  // Start timer used for periodic indications.
  sc = app_timer_start(&app_periodic_timer,
                       BURTC_IRQ_PERIOD,
                       app_periodic_timer_cb,
                       NULL,
                       true);
  app_assert_status(sc);
  // Send first indication.
  app_periodic_timer_cb(&app_periodic_timer, NULL);
*/
  // Initialize GPIO
  //configureLED();

  // Initialize the IADC
  initIADC();

#ifdef EM2DEBUG
#if (EM2DEBUG == 1)
  // Use for other purpose if needed
#endif
#endif

}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  if(em_mode_2)
  {
    if(em2_counter == 1)
    {
      my_IADC_ReadChannel();
    }
    em2_counter++;

    if(is_capacitor_valtage_enough == true)
    {
      // Disable the IADC
      //IADC_reset(IADC0);
      my_IADC_disable(IADC0);
      is_capacitor_valtage_enough = false;

      em_mode_2 = false;
      em2_counter = 0;

      if(!GPIO_PinOutGet(gpioPortC, 3)){
          presenceDetectorPowerON();
      }
      reInitialise_I2C();
      verify_sths34pf80_ID();
      configure_sths34pf80();

      //32768
      sl_bt_system_set_lazy_soft_timer(16384,
                                       0,
                                       advertising_set_handle_1,
                                       1);
      adv_presence = true;
      printf("App_Periodic, Adv_presence : %d -> portC: %d, SDA: %d, SCL: %d\r\n", adv_presence,
                 GPIO_PinOutGet(gpioPortC, 3),
                 GPIO_PinOutGet(gpioPortD, 2),
                 GPIO_PinOutGet(gpioPortD, 3));
    }
  }
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  printf("sl_bt_on_event \r\n");

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:

      bcn_setup_adv_beaconing();
      break;

    case sl_bt_evt_system_soft_timer_id:
      printf("sl_bt_evt_system_soft_timer_id is expired \r\n");

      if(adv_presence)
      {
        printf("lazy_1_Timer (adv_presence) -> portC: %d, SDA: %d, SCL: %d\r\n",
                   GPIO_PinOutGet(gpioPortC, 3),
                   GPIO_PinOutGet(gpioPortD, 2),
                   GPIO_PinOutGet(gpioPortD, 3));
        adv_presence_data();
        adv_presence = false;
      }
      if(enter_EM3)
      {
        // The below line is not needed when advertising event is not continuously advertising
        /* sl_status_t sc;
        sc = sl_bt_advertiser_stop(advertising_set_handle);
        app_assert_status(sc);*/

        //BURTC_CounterReset(); // reset BURTC counter to wait full ~5 sec before EM3 wakeup
        clear_em2_mode();
        enter_EM3 = false;
        printf("lazy_2_Timer (enter_EM3) -> portC: %d, SDA: %d, SCL: %d\r\n",
                   GPIO_PinOutGet(gpioPortC, 3),
                   GPIO_PinOutGet(gpioPortD, 2),
                   GPIO_PinOutGet(gpioPortD, 3));
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

void presence_measurement_val_to_buf(int16_t value,
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
  //GPIO_PinModeSet(gpioPortB, 0, gpioModePushPull, 0);


  if(t_presence_raw != 0)
  {
    printf("T_presence_raw: 0x%x (hex) <-> %d (dec)\r\n", t_presence_raw, t_presence_raw);
    presence_measurement_val_to_buf(t_presence_raw, presence_value);

    bcn_beacon_adv_data.uuid[4] = presence_value[0];
    bcn_beacon_adv_data.uuid[5] = presence_value[1];
    bcn_beacon_adv_data.uuid[6] = int_part;
    bcn_beacon_adv_data.uuid[7] = decimal_part;

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
  enter_EM3 = true;
  sl_bt_system_set_lazy_soft_timer(16384,
                                   0,
                                   advertising_set_handle,
                                   1);
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

/*******************************************************************************
 * Set flag and power manager to EM2 maximum sleep mode
 ******************************************************************************/
static void set_em2_mode(void)
{
  if (!em2_is_enabled) {
    em2_is_enabled = true;
    sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM2);
    printf("SL_POWER_MANAGER_EM2 requirement added \r\n");
  }
}

/*******************************************************************************
 * Clear flag and allow power manager to go lower then EM2
 ******************************************************************************/
static void clear_em2_mode(void)
{
  if (em2_is_enabled) {
    em2_is_enabled = false;
    sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM2);
    printf("SL_POWER_MANAGER_EM2 requirement removed \r\n");
  }
}
