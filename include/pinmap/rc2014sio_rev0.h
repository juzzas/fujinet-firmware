/* FujiNet Hardware Pin Mapping */
#ifdef PINMAP_RC2014SIO_REV0

/* SD Card - fnFsSD.cpp -- NOT CONNECTED */
#define PIN_CARD_DETECT 12 // fnSystem.h
#define PIN_CARD_DETECT_FIX 15 // fnSystem.h
#define PIN_SD_HOST_CS GPIO_NUM_5 //fnFsSD.cpp
#define PIN_SD_HOST_MISO GPIO_NUM_19
#define PIN_SD_HOST_MOSI GPIO_NUM_23
#define PIN_SD_HOST_SCK GPIO_NUM_18

/* UART - fnuart.cpp */
#define PIN_UART0_RX 3  // USB Serial
#define PIN_UART0_TX 1  // USB Serial
#define PIN_UART1_RX 13  // RS232
#define PIN_UART1_TX 21 // RS232
#define PIN_UART2_RX 33 // not connected
#define PIN_UART2_TX 21 // not connected

/* Buttons - keys.cpp */
#define PIN_BUTTON_A 0  // Button 0 on DEVKITC-VE
#define PIN_BUTTON_B -1 // No Button B
#define PIN_BUTTON_C 39 // Safe reset

/* LEDs - leds.cpp */
#define PIN_LED_WIFI    14
#define PIN_LED_BUS     12
#define PIN_LED_BT      -1 // No BT LED

/* Audio Output - samlib.h */
#define PIN_DAC1 25 // not connected

/* RS232 Pins */
#define PIN_RS232_RTS       33 // (IN) Request to Send
#define PIN_RS232_CTS       26 // (OUT) Clear to Send

#endif /* PINMAP_RC2014SIO_REV0 */
