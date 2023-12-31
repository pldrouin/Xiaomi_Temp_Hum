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
#include "AppHardwareApi.h"
#include "dbg.h"

extern uint16_t sht3x_lasthumtemp;

void sht3x_initialise();
int sht3x_transfer_wait(bool_t stop_on_nack);
bool_t sht3x_read_value(uint16_t* value, bool_t last);
int sht3x_send_command(uint8_t msb, uint8_t lsb, bool_t stopafterlsb);
inline static uint16_t sht3x_humtemp_reduced_value(uint16_t hum, uint16_t temp){return (hum&65024)|(temp>>7);}
int sht3x_write_alert_limit(uint8_t lsb, uint16_t humtemp);
inline static int sht3x_i2c_clear_alerts(){return sht3x_send_command(0x30,0x41,FALSE);}
int sht3x_read_single_value(uint8_t msb, uint8_t lsb, uint16_t* value);
int sht3x_read_two_values(uint8_t msb, uint8_t lsb, uint16_t* value0, uint16_t* value1);
inline static int sht3x_read_status(uint16_t* value){return sht3x_read_single_value(0xF3, 0x2D, value);}
inline static int sht3x_read_alert_high_set(uint16_t* value){return sht3x_read_single_value(0xE1, 0x1F, value);}
inline static int sht3x_read_alert_high_clear(uint16_t* value){return sht3x_read_single_value(0xE1, 0x14, value);}
inline static int sht3x_read_alert_low_set(uint16_t* value){return sht3x_read_single_value(0xE1, 0x02, value);}
inline static int sht3x_read_alert_low_clear(uint16_t* value){return sht3x_read_single_value(0xE1, 0x09, value);}
int sht3x_get_measurements(uint16_t* temp, uint16_t* hum);
bool_t sht3x_i2c_initialise();
bool_t sht3x_i2c_enable();
bool_t sht3x_i2c_disable();
bool_t sht3x_i2c_turnoff();

inline static void crc8_init(uint8_t* crc){*crc = 0xFF;}
void crc8_update(uint8_t* crc, const uint8_t byte);

#endif /* APP_SHT3X_H_ */
