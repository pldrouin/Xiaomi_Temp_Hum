/*
 * app_sht3x.h
 *
 *  Created on: Nov 28, 2019
 *      Author: Pierre-Luc Drouin
 */

#ifndef APP_SHT3X_H_
#define APP_SHT3X_H_

#define DIO_SHT_VDD (13)
#define DIO_SHT_ALERT (12)

#define SHT3X_STATUS_ALERT (65535)

#include <jendefs.h>
#include <stdlib.h>
#include "pdum_gen.h"
#include "zcl_options.h"
#include "zcl_common.h"
#include "app_zcl_task.h"
#include "AppHardwareApi.h"
#include "dbg.h"

extern bool_t sht3x_alert;

void sht3x_initialise();
int sht3x_transfer_wait(bool_t stop_on_nack);
bool_t sht3x_read_value(uint16_t* value, bool_t last);
int sht3x_send_command(uint8_t msb, uint8_t lsb, bool_t stopafterlsb);
int sht3x_write_alert_limit(uint8_t lsb, uint16_t hum, uint16_t temp);
inline static int sht3x_i2c_clear_alerts(){return sht3x_send_command(0x30,0x41,FALSE);}
int sht3x_read_single_value(uint8_t msb, uint8_t lsb, uint16_t* value);
int sht3x_read_two_values(uint8_t msb, uint8_t lsb, uint16_t* value0, uint16_t* value1);
inline static int sht3x_read_status(uint16_t* value){return sht3x_read_single_value(0xF3, 0x2D, value);}
inline static int sht3x_read_alert_high_set(uint16_t* value){return sht3x_read_single_value(0xE1, 0x1F, value);}
inline static int sht3x_read_alert_high_clear(uint16_t* value){return sht3x_read_single_value(0xE1, 0x14, value);}
inline static int sht3x_read_alert_low_set(uint16_t* value){return sht3x_read_single_value(0xE1, 0x02, value);}
inline static int sht3x_read_alert_low_clear(uint16_t* value){return sht3x_read_single_value(0xE1, 0x09, value);}
int sht3x_get_measurements(uint16_t* temp, uint16_t* hum);
bool_t sht3x_i2c_configure();
bool_t sht3x_i2c_disable();
inline static void sht3x_i2c_configure_nocheck(){vAHI_SiMasterConfigure(TRUE,FALSE,63); sht3x_i2c_clear_alerts(); while(sht3x_send_command(0x20,0x32,FALSE)){}}
inline static void sht3x_i2c_disable_nocheck(){vAHI_SiMasterDisable();}

inline static void crc8_init(uint8_t* crc){*crc = 0xFF;}
void crc8_update(uint8_t* crc, const uint8_t byte);

#endif /* APP_SHT3X_H_ */
