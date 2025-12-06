/*
PCB SPECIFIC CONFIGURATIONS
*/

#define Theseus 0
#define Lumpy 1
#define BOAT_MODE Theseus

// Important constants
#define LED_PIN 25
#define TIMEOUT_MS 1000
#define AGENT_ATTEMPTS 120

// Pin declarations
// UART
#define TX_PIN 0
#define RX_PIN 1

// Debug
#define SWCLK_PIN 2
#define SWDIO_PIN 3
#define SWDTX_PIN 4
#define SWDRX_PIN 5

// I2C
#define SDA_PIN 20
#define SCL_PIN 21

// SPI
#define SCLK_PIN 10
#define MISO_PIN 12
#define MOSI_PIN 11

// RC Channels
#define RC_CHANNEL1_PIN 28
#define RC_CHANNEL2_PIN 27
#define RC_CHANNEL3_PIN 26

// Chip Selections
#define RUDDER_MOTOR_CS_PIN 13
#define WINCH_MOTOR_CS_PIN 6
#define RUDDER_ENCODER_CS_PIN 7
#define WINCH_ENCODER_CS_PIN 9

// Miscellaneous
#define RUDDER_MOTOR_SLEEP_PIN 17
#define WINCH_MOTOR_SLEEP_PIN 14
#define RELAY_PIN 22 // NOTE: UNUSED BECAUSE OUR CURRENT BJT DRIVER DOESN'T WORK

// Relay
#define CONTACTOR_DRIVER_SEL0_PIN 19
#define CONTACTOR_DRIVER_PWM_PIN 18
#define CONTACTOR_DRIVER_IN_A_PIN 8
#define CONTACTOR_DRIVER_IN_B_PIN 16

// Port declarations
#define I2C_PORT i2c0
#define SPI_PORT spi1