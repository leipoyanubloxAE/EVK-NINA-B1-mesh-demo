#ifndef PCA10040_H
#define PCA10040_H
#ifdef __cplusplus
extern "C" {
#endif
#include "nrf_gpio.h"
// In this case PIN 16 is used as button SW1, if the green led
// should be used it is possible to defined that one instead.
#define LEDS_NUMBER 3
#define LED_1 8  // Red
#define LED_2 18  // Blue
#define LED_3 16  // Green
#define LEDS_ACTIVE_STATE 0
#define LEDS_LIST { LED_1, LED_2, LED_3 }
#define BSP_LED_0 LED_1
#define BSP_LED_1 LED_2
#define LEDS_INV_MASK LEDS_MASK
#define BUTTONS_NUMBER 2
#define BUTTON_1 16  // SW1
#define BUTTON_2 30  // SW2
#define BUTTON_PULL NRF_GPIO_PIN_PULLUP
#define BUTTONS_ACTIVE_STATE 0
#define BUTTONS_LIST { BUTTON_1, BUTTON_2 }
#define BSP_BUTTON_0 BUTTON_1
#define BSP_BUTTON_1 BUTTON_2
#define RX_PIN_NUMBER 5
#define TX_PIN_NUMBER 6
#define CTS_PIN_NUMBER 7
#define RTS_PIN_NUMBER 31
#define HWFC true
#define SPIS_MISO_PIN 12 // SPI MISO signal.
#define SPIS_CSN_PIN 11 // SPI CSN signal.
#define SPIS_MOSI_PIN 13 // SPI MOSI signal.
#define SPIS_SCK_PIN 14 // SPI SCK signal.
#define SPIM0_SCK_PIN 14 // SPI clock GPIO pin number.
#define SPIM0_MOSI_PIN 13 // SPI Master Out Slave In GPIO pin number.
#define SPIM0_MISO_PIN 12 // SPI Master In Slave Out GPIO pin number.
#define SPIM0_SS_PIN 11 // SPI Slave Select GPIO pin number.
// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC {.source = NRF_CLOCK_LF_SRC_XTAL, \
.rc_ctiv = 0, \
.rc_temp_ctiv = 0, \
.xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}
#ifdef __cplusplus
}
#endif

#endif // PCA10040_H
