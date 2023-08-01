Firmware code that implements a custom firmware for a Xiaomi temperature and humidity ZigBee sensor (WSDCGQ01LM). The firmware is configured to wake up the device and transmit data when the temperature or the humidity changes above a configured threshold. A timer also wakes the device every hour to transmit the battery voltage in addition to the temperature and humidity data.

If the device loses connectivity, it will attempt to reconnect using a timer that gets incrementally longer.

A short button press transmits the device's data and quickly pulses the LED.

Pressing the button between 5 and 10 seconds factory resets the device and tries to repair it to a coordinator. Successful reconnection leads to the LED blinking three times.

Pressing the button for over 10 seconds puts the device into deep sleep mode after the LED lights up for 1 second.

Tested with JN-SW-4170 NXP SDK. The firmware makes de device to act as a Zigbee3 base device.

