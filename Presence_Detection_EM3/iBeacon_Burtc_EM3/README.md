Project – Adaptive Indoor Light Energy Harvesting BLE node.

**Source code changes:**
1) This project uses the BURTC (Backup Real Time Counter) to wake the device from EM3 mode.
2) BLE ibeacon packets are used to adverstise presence detection samples, voltage of supercap.
3) STHS34PF80 -> Presence detection sensor is configured for (gpioPortC, 3).
4) iADC peripheral is configured for single input conversion to read voltage values from the supercap which is connected to (gpioPortB, 0).

================================================================================

**Project configuration:**
To test this application, you can either create a project based on an example project or start with a "iBeacon - SoC Empty" project based on your hardware. Currently, I am using EFR32BG22 board for this project.

For EM3, download the below components which can be found in "software components" tab under filename.slcp 
1) "Timer" for application
2) "Microsecond Delay" for Utilities
3) "USART" for IO Stream for prints in the serial terminal. (If you analyse the power consumption in power analyser you must unistall the USART component from the simplicity studio.)

================================================================================

**Troubleshooting info:**
To get printf in the serial terminal, download putty and configure below detials:

. Serial Line     : COM3
. Speed           : 115200
. Connection type : Serial

================================================================================

**Additional info:**
#define BURTC_IRQ_PERIOD  5000 /* timer in milliseconds */
static void bcn_setup_adv_beaconing(void)  /* In this function "advertisement interval" can be change based on requirement. */

================================================================================