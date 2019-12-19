/*
 * app_sht3x.h
 *
 *  Created on: Nov 28, 2019
 *      Author: root
 */

#ifndef APP_SHT3X_H_
#define APP_SHT3X_H_

#include <jendefs.h>
#include <stdlib.h>
#include "AppHardwareApi.h"
#include "dbg.h"

int sht3x_transfer_wait();
uint8_t sht3x_read_value(uint16_t* value);
int sht3x_get_measurements(uint16_t* temp, uint16_t* hum);

#endif /* APP_SHT3X_H_ */
