#include <stdio.h>
#include <configure_presence.h>

#include "sl_bluetooth.h"
#include "app_assert.h"
#include "app.h"
#include "app_timer.h"
#include "sl_udelay.h"

#include "em_emu.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_rmu.h"
#include "em_burtc.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_iadc.h"

// Set CLK_ADC to 10MHz (this corresponds to a sample rate of 77K with OSR = 32)
// CLK_SRC_ADC; largest division is by 4
#define CLK_SRC_ADC_FREQ        20000000

// CLK_ADC; IADC_SCHEDx PRESCALE has 10 valid bits
#define CLK_ADC_FREQ            10000000

// Macros.
#define UINT16_TO_BYTES(n)            ((uint8_t) (n)), ((uint8_t)((n) >> 8))
#define UINT16_TO_BYTE0(n)            ((uint8_t) (n))
#define UINT16_TO_BYTE1(n)            ((uint8_t) ((n) >> 8))

#define SIGNAL_LOSS_AT_1_M_IN_DBM     41 // The beacon's measured RSSI at 1 m

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

// I2C pins (SCL = PD2; SDA = PD3)
#define I2C_SCL_PORT              gpioPortD
#define I2C_SCL_PIN               2
#define I2C_SDA_PORT              gpioPortD
#define I2C_SDA_PIN               3
#define PRESENCE_DETECTOR_PORT    gpioPortC
#define PRESENCE_DETECTOR_PIN     3
#define PN_MOSFET_PORT            gpioPortB
#define PN_MOSFET_PIN             4

/*
 * This example enters EM2 in the main while() loop; Setting this #define to 1
 * enables debug connectivity in EM2, which increases current consumption by
 * about 0.5uA
 */
#define EM2DEBUG                  1

/*******************************************************************************
 ***************************   GLOBAL VARIABLES   ******************************
 ******************************************************************************/
// The voltage divider ratio, which is the ratio of the resistors in the voltage divider circuit
// This ratio determines how the input voltage is divided before being measured by the ADC
const double voltageDividerRatio = 1.31; // Set to 1.0 if no external voltage divider is used
// A constant for the reference voltage in volts
const double referenceVoltageV = 3.42;
// A constant for the reference voltage in milli volts
const double referenceVoltageMV = 3420;
// A constant for the analog gain correction factor
const double analogGainCorrectionFactor = 2.0;
// A calibration factor to adjust the calculated voltage based on observed discrepancy
const double calibrationFactor = 0.970;

// Number of 1 KHz ULFRCO clocks between BURTC interrupts
static uint32_t burtc_irq_period = 5000;

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;
static uint8_t int_part = 0;
static uint8_t decimal_part = 0;

static bool enter_EM4 = false;
static bool adv_presence = false;
static bool trigger_IADC_Conversions = false;
static bool is_capacitor_voltage_enough = false;
static bool low_voltage = false;
static bool em2_is_enabled = true;
static bool em_mode_2 = false;

void my_IADC_enable(IADC_TypeDef *iadc);
void my_IADC_disable(IADC_TypeDef *iadc);
void my_IADC_ReadChannel(void);
void adv_presence_data(void);
void resetBurtc_and_enterEM4(bool reset_burtc_counter);

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
    { 0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48, 0xD2, \
      0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0 },

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
  //printf("initCMUClocks() \r\n");
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
  //printf("presenceDetectorPowerON()\r\n");
  GPIO_PinModeSet(PRESENCE_DETECTOR_PORT, PRESENCE_DETECTOR_PIN, gpioModePushPull, 1);
}

/**************************************************************************//**
 * @brief  Turn OFF Presence Detector
 *****************************************************************************/
void presenceDetectorPowerOFF(void)
{
  //printf("presenceDetectorPowerOFF()\r\n");
  GPIO_PinModeSet(PRESENCE_DETECTOR_PORT, PRESENCE_DETECTOR_PIN, gpioModePushPull, 0);
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
  initAllConfigs.configs[0].reference = iadcCfgReferenceVddx;
  initAllConfigs.configs[0].vRef = referenceVoltageMV; // Reference voltage in mV
  initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain0P5x; // Analog gain of 0.5x

  // Divides CLK_SRC_ADC to set the CLK_ADC frequency for desired sample rate
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                    CLK_ADC_FREQ,
                                                                    0,
                                                                    iadcCfgModeNormal,
                                                                    init.srcClkPrescale);

  // Single initialization
  initSingle.dataValidLevel = iadcFifoCfgDvl1;

  // Set conversions to run only once
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
 * @brief  Function to change burtc compare value
 *****************************************************************************/
void change_burtc_compare_value(uint32_t burtc_counter)
{
  //printf("change_burtc_compare_value()\r\n");

  BURTC_IntDisable(BURTC_IEN_COMP); // Disable the compare interrupt
  BURTC_IntClear(BURTC_IF_COMP);    // clear any pending interrupt flags

  BURTC_CompareSet(0, burtc_counter);
  BURTC_IntEnable(BURTC_IEN_COMP);  // Re-enable the compare interrupt
}

/**************************************************************************//**
 * @brief  Function to check super capacitor voltage and take action
 *****************************************************************************/
void handle_super_capacitor_voltage(double singleResult)
{
  //printf("handle_super_capacitor_voltage()\r\n");

  // Thresholds for the super capacitor voltage
  const float Vmin = 3.000000;
  const float Vmax = 4.450000;

  // Check if the voltage is below the minimum threshold
  if (singleResult <= Vmin) {
      low_voltage = true;
      BURAM->RET[1].REG = low_voltage;

      // Disable the IADC and clear EM2 mode
      my_IADC_disable(IADC0);
      em_mode_2 = false;
      clear_em2_mode();

      printf("Voltage is less than %.2fV, Enter_EM4 -> "
             "PC3_Presence: %d, SDA: %d, SCL: %d, PB4_MOSFET: %d\r\n",
             Vmin,
             GPIO_PinOutGet(PRESENCE_DETECTOR_PORT, PRESENCE_DETECTOR_PIN),
             GPIO_PinOutGet(I2C_SCL_PORT, I2C_SCL_PIN),
             GPIO_PinOutGet(I2C_SDA_PORT, I2C_SDA_PIN),
             GPIO_PinOutGet(PN_MOSFET_PORT, PN_MOSFET_PIN));

      burtc_irq_period = 60000;
      change_burtc_compare_value(burtc_irq_period);
      resetBurtc_and_enterEM4(true);
      return;
  }

  low_voltage = BURAM->RET[1].REG;
  // Check if the voltage was previously low and now is within the valid range
  if (low_voltage && singleResult >= Vmax) {
      printf("Super Capacitor voltage is now enough.\r\n");
      low_voltage = false;
      BURAM->RET[1].REG = low_voltage;
      burtc_irq_period = 5000;
      change_burtc_compare_value(burtc_irq_period);
      is_capacitor_voltage_enough = true;
      return;
  } else if (low_voltage) {
      // If still in low voltage state, keep disabling IADC and clearing EM2 mode
      my_IADC_disable(IADC0);
      em_mode_2 = false;
      clear_em2_mode();

      printf("Super Capacitor is not fully charged(%0.2fV), Enter_EM4 -> "
             "PC3_Presence: %d, SDA: %d, SCL: %d, PB4_MOSFET: %d\r\n",
             Vmax,
             GPIO_PinOutGet(PRESENCE_DETECTOR_PORT, PRESENCE_DETECTOR_PIN),
             GPIO_PinOutGet(I2C_SCL_PORT, I2C_SCL_PIN),
             GPIO_PinOutGet(I2C_SDA_PORT, I2C_SDA_PIN),
             GPIO_PinOutGet(PN_MOSFET_PORT, PN_MOSFET_PIN));

      resetBurtc_and_enterEM4(true);
      return;
  }
  // Normal scenario when super capacitor has enough voltage.
  is_capacitor_voltage_enough = true;
}

/**************************************************************************//**
 * @brief  IADC interrupt handler
 *****************************************************************************/
void IADC_IRQHandler(void)
{
  // Stores latest ADC sample and converts to volts
  IADC_Result_t sample;
  sample.data = 0;

  // Read most recent single conversion result
  sample = IADC_readSingleResult(IADC0);

  /* Calculate input voltage:
     For single-ended the result range is 0 to +Vref, i.e.,
     for Vref = VBGR = 1.21V, and with analog gain = 0.5,
     12 bits represents 2.43V full scale IADC range.
  */

  // Calculate the intermediate voltage based on the ADC sample data
  double intermediateVoltage = sample.data * referenceVoltageV / 0xFFF;

  // Apply the analog gain correction factor and calibration factor
  double measuredVoltage = (intermediateVoltage * analogGainCorrectionFactor) * calibrationFactor;
  //printf("1.Measured Voltage = %.3lf\r\n", voltageDivider);

  // To reflect the actual supercap voltage, multiple with voltageDividerRatio
  double superCapVoltage = measuredVoltage * voltageDividerRatio;
  //printf("2.SuperCap_Voltage = %.3lf\r\n", superCapVoltage);

  IADC_clearInt(IADC0, IADC_IF_SINGLEDONE);

  float float_value = superCapVoltage;

  // Extract the integer part
  int_part = (uint8_t)float_value;

  // Extract the first digit of the fractional part
  decimal_part = (uint8_t)((float_value - int_part) * 100);

  printf("Sample.data[%ld], Voltage_Divider = %.3lfV, SuperCap_Voltage = %.2lfV,"
         " BLE_adv_volt = %d.%dV\r\n",
         sample.data, measuredVoltage, superCapVoltage, int_part, decimal_part);

  handle_super_capacitor_voltage(superCapVoltage);
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
  if(BURAM->RET[1].REG == 1)
    {
      burtc_irq_period = 60000;
    }
  else if(BURAM->RET[1].REG == 0)
    {
      burtc_irq_period = 5000;
    }
  BURTC_CompareSet(0, burtc_irq_period);

  BURTC_IntEnable(BURTC_IEN_COMP);    // Enable the compare interrupt
  NVIC_EnableIRQ(BURTC_IRQn);
  BURTC_Enable(true);
}

/**************************************************************************//**
 * @brief  BURTC Handler
 *****************************************************************************/
void BURTC_IRQHandler(void)
{
  //printf("\r\nBURTC_IRQHandler()\n");
  BURTC_IntClear(BURTC_IF_COMP); // clear any pending interrupt flags
  BURTC_CounterReset(); // reset BURTC counter to wait burtc_irq_period sec before EM4 wakeup
  //printf("-- BURTC counter reset \r\n");
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
    printf("-- RSTCAUSE = PIN \r\n");
    BURAM->RET[0].REG = 0; // reset EM4 wakeup counter
    BURAM->RET[1].REG = 0; // reset low voltage flag
  }
  else if (cause & EMU_RSTCAUSE_EM4)
  {
    //printf("-- RSTCAUSE = EM4 wakeup \r\n");
    BURAM->RET[0].REG += 1; // increment EM4 wakeup counter
  }
  // Print # of EM4 wakeups and low voltage flag status
  printf ("Number of EM4 wakeups = %ld, Low Voltage flag = %ld\r\n",
          BURAM->RET[0].REG, BURAM->RET[1].REG);
}

void resetBurtc_and_enterEM4(bool reset_burtc_counter)
{
  if(reset_burtc_counter)
   {
      BURTC_CounterReset();
      //printf("Reset BURTC counter to wait full %ld sec before EM4 wakeup \n", burtc_irq_period/1000);
   }
  // Enter EM4
  printf("Entering EM4 and wake on BURTC compare in %ld seconds \r\n\r\n", burtc_irq_period/1000);
  //RETARGET_SerialFlush(); // delay for printf to finish
  EMU_EnterEM4();
}

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{
  //printf("app init()\r\n");
  EMU_UnlatchPinRetention();

  set_em2_mode();
  em_mode_2 = true;
  trigger_IADC_Conversions = true;

/*
  // Enable the below lines only if EM4 is running on empty project. Otherwise
  // EM4 initialization done during platform init via sl_device_init_emu().
  EMU_EM4Init_TypeDef em4Init = EMU_EM4INIT_DEFAULT;
  EMU_EM4Init(&em4Init);
*/

  //printf("\r\nIn EM0\r\n");
  initCMUClocks();
  initBURTC();
  // Initialize the IADC
  initIADC();

  // Check RESETCAUSE, update and print EM4 wakeup count
  checkResetCause();

  GPIO_PinModeSet(PN_MOSFET_PORT, PN_MOSFET_PIN, gpioModePushPull, 0);
  GPIO_PinOutSet(PN_MOSFET_PORT, PN_MOSFET_PIN);
  //GPIO_PinOutClear(PN_MOSFET_PORT, PN_MOSFET_PIN);
  printf("AppInt -> PC3_Presence: %d, SDA: %d, SCL: %d, PB4_MOSFET: %d\r\n",
             GPIO_PinOutGet(PRESENCE_DETECTOR_PORT, PRESENCE_DETECTOR_PIN),
             GPIO_PinOutGet(I2C_SCL_PORT, I2C_SCL_PIN),
             GPIO_PinOutGet(I2C_SDA_PORT, I2C_SDA_PIN),
             GPIO_PinOutGet(PN_MOSFET_PORT, PN_MOSFET_PIN));

}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  if(em_mode_2)
    {
      if(trigger_IADC_Conversions)
      {
        my_IADC_ReadChannel();
        trigger_IADC_Conversions = false;
      }
      if(is_capacitor_voltage_enough == true)
      {
        // Disable the IADC
        //IADC_reset(IADC0);
        my_IADC_disable(IADC0);
        is_capacitor_voltage_enough = false;
        em_mode_2 = false;

        if(!GPIO_PinOutGet(PRESENCE_DETECTOR_PORT, PRESENCE_DETECTOR_PIN)){
            presenceDetectorPowerON();
        }
        reInitialise_I2C();
        setup_I2C_Read_Write();
        verify_sths34pf80_ID();
        configure_sths34pf80();

        //1 Millisecond = 32.768, 500millsecond = 32.768*500 = 16384
        sl_bt_system_set_lazy_soft_timer(16384,
                                         0,
                                         advertising_set_handle,
                                         1);
        adv_presence = true;
        printf("App_Periodic, Adv_presence : %d -> "
               "PC3_Presence: %d, SDA: %d, SCL: %d, PB4_MOSFET: %d\r\n",
               adv_presence,
               GPIO_PinOutGet(PRESENCE_DETECTOR_PORT, PRESENCE_DETECTOR_PIN),
               GPIO_PinOutGet(I2C_SCL_PORT, I2C_SCL_PIN),
               GPIO_PinOutGet(I2C_SDA_PORT, I2C_SDA_PIN),
               GPIO_PinOutGet(PN_MOSFET_PORT, PN_MOSFET_PIN));
      }
    }
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

    bcn_beacon_adv_data.min_num[0] = presence_value[0];
    bcn_beacon_adv_data.min_num[1] = presence_value[1];
    bcn_beacon_adv_data.maj_num[0] = int_part;
    bcn_beacon_adv_data.maj_num[1] = decimal_part;

    /*
     * printf("UINT16_TO_BYTE1: 0x%x, UINT16_TO_BYTE0:  0x%x  ", UINT16_TO_BYTE1(t_presence_raw),
           UINT16_TO_BYTE0(t_presence_raw));

    bcn_beacon_adv_data.min_num[0] = UINT16_TO_BYTE1(t_presence_raw);
    bcn_beacon_adv_data.min_num[1] = UINT16_TO_BYTE0(t_presence_raw);

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
      break;

    case sl_bt_evt_system_soft_timer_id:
      //printf("sl_bt_evt_system_soft_timer_id is expired \r\n");

      if(adv_presence)
      {
        printf("lazy_1_Timer (adv_presence) -> portC: %d, SDA: %d, SCL: %d\r\n",
                   GPIO_PinOutGet(gpioPortC, 3),
                   GPIO_PinOutGet(gpioPortD, 2),
                   GPIO_PinOutGet(gpioPortD, 3));
        adv_presence_data();
        adv_presence = false;
      }
      else if(enter_EM4)
      {
        /*sc = sl_bt_advertiser_stop(advertising_set_handle);
        app_assert_status(sc); */
        printf("lazy_2_Timer (enter_EM4) -> portC: %d, SDA: %d, SCL: %d\r\n",
                   GPIO_PinOutGet(gpioPortC, 3),
                   GPIO_PinOutGet(gpioPortD, 2),
                   GPIO_PinOutGet(gpioPortD, 3));
        clear_em2_mode();
        resetBurtc_and_enterEM4(false);
        enter_EM4 = false;
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

  bd_addr address;
  uint8_t address_type;

  // Retrieve the BT Address.
  sc = sl_bt_system_get_identity_address(&address, &address_type);
  app_assert_status(sc);

  //app_log("BT Address: ");
  for (int i=0; i<5; i++)
    {
      printf("%02X:", address.addr[5-i]);
    }
   printf("%02X (%s)\r\n", address.addr[0], address_type == 0 ? "Public device address": "Static random address");


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
    //printf("SL_POWER_MANAGER_EM2 requirement added \r\n");
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
    //printf("SL_POWER_MANAGER_EM2 requirement removed \r\n");
  }
}