Setup:

To test this application, you can either create a project based on an example project or start with a "iBeacon - SoC Empty" project based on your hardware. Currently, I am using EFR32BG22 board for this project.

For EM3, download the below components which can be found in "software components" tab under filename.slcp 

1) "Timer" for application
2) "Microsecond Delay" for Utilities
3) "USART" for IO Stream for prints in the serial terminal. (If you analyse the power consumption in power analyser you must unistall the USART component from the simplicity studio.)
4) In this code iADC single input conversion is used voltage reading from the PV cell. 

To get printf in the serial terminal, download putty and configure below detials:

. Serial Line     : COM3
. Speed           : 115200
. Connection type : Serial

Code changes:

#define BURTC_IRQ_PERIOD  5000 /* timer in milliseconds */

static void bcn_setup_adv_beaconing(void)  /* In this function "advertisement interval" can be change based on requirement. */