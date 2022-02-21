
# STM32WL55 6-Axis LoRa Motion Tracker

<img src="./asset/axden_stm32wl55_lora_motion_tracker.jpeg">
<br>

STM32WL55 – LORA SoC
<br>

LSM6DSL -Accelerometer, gyroscope sensor
<br>

Si7051 -Temperature sensor
<br>

Si7201 – Hall Sensor
<br>

868 ~ 928Mhz 의 주파수 대역을 사용합니다.
<br>

LORA 통신이 가능한 Motion tracker 입니다.
<br>

STM32WL55 SoC 를 이용하여 LORA 통신이 가능합니다.
<br>

SI7201 Hall 센서와 자석을 이용하여 On Off 할 수 있습니다.
<br>

LSM6DSL 을 이용하여 가속도, 자이로 센서 값을 수집합니다.
<br>

Si7051 센서를 이용하여 온도 값을 수집합니다.
<br>

배터리를 이용하여 5년동안 사용 가능합니다.
<br>

UART X 1, I2C X 1 를 연결할 수 있는 Connector 가 존재하여 확장이 가능합니다.
<br>

네이버 스마트 스토어에서 구매 가능합니다.
<br>

[네이버 스마트 스토어](https://smartstore.naver.com/axden)
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

-----------------------

### Note
<br>

Works with STM32CubeIDE Version: 1.8.0, freeRTOS CMSIS_V2, Single-core
<br>

STM32WL55 module only support RFO_H
<br>

RF Switch IC MPN : BGS12SN6E6327XTSA1
<br>

-------------------------

### STM32WL55 Pin map

<img src="./asset/motion_tracker_stm32wl55_pinmap.png">