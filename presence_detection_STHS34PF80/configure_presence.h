#ifndef CONFIGURE_PRESENCE_H_
#define CONFIGURE_PRESENCE_H_

#include <stdio.h>
#include "sl_status.h"

sl_status_t reInitialise_I2C(void);

void get_presence(int number_of_samples, int16_t *t_presence_raw);
void configure_sths34pf80(void);
void reConfigure_sths34pf80(void);
void verify_sths34pf80_ID(void);
void setup_I2C_Read_Write(void);

void disable_I2C(void);

#endif /* CONFIGURE_PRESENCE_H_ */