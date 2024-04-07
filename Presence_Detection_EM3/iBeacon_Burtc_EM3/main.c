/***************************************************************************//**
 * @file
 * @brief main() function.
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
#include "sl_component_catalog.h"
#include "sl_system_init.h"
#include "app.h"
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
#include "sl_power_manager.h"
#endif // SL_CATALOG_POWER_MANAGER_PRESENT
#if defined(SL_CATALOG_KERNEL_PRESENT)
#include "sl_system_kernel.h"
#else // SL_CATALOG_KERNEL_PRESENT
#include "sl_system_process_action.h"
#endif // SL_CATALOG_KERNEL_PRESENT

#include "stdio.h"
#include "em_gpio.h"

/*
#define SLEEP_EM_EVENT_MASK_1      ( SL_POWER_MANAGER_EVENT_TRANSITION_ENTERING_EM2  \
                                  | SL_POWER_MANAGER_EVENT_TRANSITION_LEAVING_EM2  \
                                  | SL_POWER_MANAGER_EVENT_TRANSITION_ENTERING_EM3 \
                                  | SL_POWER_MANAGER_EVENT_TRANSITION_LEAVING_EM3  \
                                  | SL_POWER_MANAGER_EVENT_TRANSITION_LEAVING_EM0)

static const char *Emode[] = { "SL_POWER_MANAGER_EM0", "SL_POWER_MANAGER_EM1",
                               "SL_POWER_MANAGER_EM2", "SL_POWER_MANAGER_EM3"
};

static void events_handler_1(sl_power_manager_em_t from,
                           sl_power_manager_em_t to);
static sl_power_manager_em_transition_event_info_t events_info_1 =
{
  .event_mask = SLEEP_EM_EVENT_MASK_1,
  .on_event = events_handler_1,
};
static sl_power_manager_em_transition_event_handle_t events_handle_1;
*/

int main(void)
{
  // Initialize Silicon Labs device, system, service(s) and protocol stack(s).
  // Note that if the kernel is present, processing task(s) will be created by
  // this call.
  sl_system_init();

  //sl_power_manager_subscribe_em_transition_event(&events_handle_1, &events_info_1);


  // Initialize the application. For example, create periodic timer(s) or
  // task(s) if the kernel is present.
  app_init();

#if defined(SL_CATALOG_KERNEL_PRESENT)
  // Start the kernel. Task(s) created in app_init() will start running.
  sl_system_kernel_start();
#else // SL_CATALOG_KERNEL_PRESENT
  while (1) {
    // Do not remove this call: Silicon Labs components process action routine
    // must be called from the super loop.
    sl_system_process_action();

    // Application process.
    app_process_action();

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
    // Let the CPU go to sleep if the system allows it.
    printf("sl_power_manager_sleep is Enter \n");
    sl_power_manager_sleep();
    printf("sl_power_manager_sleep is exit \n");
#endif
  }
#endif // SL_CATALOG_KERNEL_PRESENT
}

/*
static void events_handler_1(sl_power_manager_em_t from,
                           sl_power_manager_em_t to)
{
  printf("Event Handler: %s-%s \r\n", Emode[from], Emode[to]);

  //(void) from;
  uint32_t out;
  if ((from == SL_POWER_MANAGER_EM3)
      && (to == SL_POWER_MANAGER_EM2)){

  // Wake the USART Tx pin back up
      out = GPIO_PinOutGet(gpioPortC, 3);
      GPIO_PinModeSet(gpioPortC, 3, gpioModePushPull, out);
      printf("Event Handler: Pin value after wake: %ld \r\n", out);


  } else if ((to == SL_POWER_MANAGER_EM3)
         && (from == SL_POWER_MANAGER_EM2)){

    // Sleep the USART Tx pin on series 2 devices to save energy
      out = GPIO_PinOutGet(gpioPortC, 3);
      printf("Event Handler: Pin value after sleep: %ld \r\n", out);
      GPIO_PinModeSet(gpioPortC, 3, gpioModeDisabled, 0);
      out = GPIO_PinOutGet(gpioPortC, 3);
      printf("Event Handler: Pin value(Disabled) after sleep: %ld \r\n", out);
  }
}*/
