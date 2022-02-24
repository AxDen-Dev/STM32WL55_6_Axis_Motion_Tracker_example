
# STM32WL55 6-Axis LoRa Motion Tracker

<img src="./asset/axden_stm32wl55_lora_motion_tracker.jpeg">
<br>

The STM32WL55 6-Axis LoRa Motion Tracker collects key information required for asset tracking such as acceleration, angular velocity, temperature, and GPS location, and charges the battery using solar power.
<br>
<br>
This is an example that provides quick testing of various service scenarios that require long-distance communication of 1Km or more.
<br>

-----------------------

Device can be purchased from the Naver Smart Store.
<br>

[Purchase Link : naver smart store](https://smartstore.naver.com/axden)
<br>
<br>

You can purchase it by contacting sales@axden.io

-----------------------

### Key feature and functions

MCU | Description
:-------------------------:|:-------------------------:
STM32WL55 | LoRa SoC

Sensors | Description
:-------------------------:|:-------------------------:
L76 | GPS sensor
MAX2659 | GPS LNA
LSM6DSl | 6 Axis Accelerometer, Gyroscope
Si7051 | Temperature sensor
Si7201 | Hall Sensor
SPV1050 | Solar battery charger (Max charge current 80mA)
Solar | On board
Battery | 3.7V Lithium Battery


It is a motion tracker capable of LORA communication.
<br>

LoRa wireless communication is performed using STM32WL55 SoC
<br>

Sub-G wireless communication is performed using CC1310 SoC
<br>

It can be turned on and off using Si7201 hall sensor and magnet.
<br>

Use LSM6DSL to collect acceleration and gyro sensor values.
<br>

The temperature value is collected using the Si7051 sensor.
<br>

It operates for 5 years using a battery.
<br>

There is a connector that can connect UART X1 and I2C X1, so it can be expanded.
<br>

-----------------------

### Note

This program is not suitable for mass production and commercialization as an example program.
<br>

B2B customers should contact development@axden.io.
<br>

For B2B customers, we develop firmware optimized for customers' purposes, such as low power, stabilization, and communication with gateways, for free.
<br>

<table>
  <tr align="center">
    <td>Top</td>
    <td>Bottom</td>
  </tr>
  <tr align="center">
    <td><img src="./asset/axden_stm32wl55_lora_top.jpeg"></td>
    <td><img src="./asset/axden_stm32wl55_lora_bottom.jpeg"></td>
  </tr>
</table>
<br>

Works with STM32CubeIDE Version: 1.8.0, freeRTOS CMSIS_V2, Single-core
<br>

STM32WL55 module only support RFO_H
<br>

RF Switch IC MPN : BGS12SN6E6327XTSA1
<br>


-----------------------

### STM32WL55 Radio setup

<img src="./asset/Radio_setup_path.png">
<br>

SubGHZ-Phy / App / subghz_phy_app.c

```
#define REGION_AS923

#if defined( REGION_AS923 )

#define RF_FREQUENCY                                923000000 // Hz

#elif defined( REGION_AU915 )

#define RF_FREQUENCY                                915000000 // Hz

#elif defined( REGION_CN470 )

#define RF_FREQUENCY                                470000000 // Hz

#elif defined( REGION_CN779 )

#define RF_FREQUENCY                                779000000 // Hz

#elif defined( REGION_EU433 )

#define RF_FREQUENCY                                433000000 // Hz

#elif defined( REGION_EU868 )

#define RF_FREQUENCY                                868000000 // Hz

#elif defined( REGION_KR920 )

#define RF_FREQUENCY                                920000000 // Hz

#elif defined( REGION_IN865 )

#define RF_FREQUENCY                                865000000 // Hz

#elif defined( REGION_US915 )

#define RF_FREQUENCY                                915000000 // Hz

#elif defined( REGION_RU864 )

#define RF_FREQUENCY                                864000000 // Hz

#else
#error "Please define a frequency band in the compiler options."
#endif

#define TX_OUTPUT_POWER 14

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
//  1: 250 kHz,
//  2: 500 kHz,
//  3: Reserved]
#define LORA_SPREADING_FACTOR                       10         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
//  2: 4/6,
//  3: 4/7,
//  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define TX_BUFFER_SIZE 128
#define RX_BUFFER_SIZE 128

```
<br>

<img src="./asset/Radio_setup.png">
<br>

-------------------------

### STM32WL55 Pin map

<img src="./asset/motion_tracker_stm32wl55_pinmap.png">
