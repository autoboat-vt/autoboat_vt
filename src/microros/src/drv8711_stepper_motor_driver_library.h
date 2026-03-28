#ifndef DRV8711_STEPPER_MOTOR_DRIVER_LIBRARY_H
#define DRV8771_STEPPER_MOTOR_DRIVER_LIBRARY_H

#include "pico/stdlib.h"
#include "hardware/spi.h"


#define DRV8711_ENABLE_BIT 0
#define DRV8711_DIRECTION_BIT 1
#define DRV8711_STEP_BIT 2

typedef enum {
    CTRL_REG_ADDRESS   = 0x00,
    TORQUE_REG_ADDRESS = 0x01,
    OFF_REG_ADDRESS    = 0x02,
    BLANK_REG_ADDRESS  = 0x03,
    DECAY_REG_ADDRESS  = 0x04,
    STALL_REG_ADDRESS  = 0x05,
    DRIVE_REG_ADDRESS  = 0x06,
    STATUS_REG_ADDRESS = 0x07,
} DRV8711_registerAddress;

typedef enum {
    MicroStep256 = 256,
    MicroStep128 = 128,
    MicroStep64  =  64,
    MicroStep32  =  32,
    MicroStep16  =  16,
    MicroStep8   =   8,
    MicroStep4   =   4,
    MicroStep2   =   2,
    MicroStep1   =   1,
} DRV8711_stepMode;

typedef enum {
    Slow                = 0b000,
    SlowIncMixedDec     = 0b001,
    Fast                = 0b010,
    Mixed               = 0b011,
    SlowIncAutoMixedDec = 0b100,
    AutoMixed           = 0b101,
} DRV8711_decayMode;

typedef struct {
    spi_inst_t *spi_port;
    uint8_t cs_pin;
    uint8_t slp_pin;
    uint16_t ctrl_reg;
    uint16_t torque_reg;
    uint16_t off_reg;
    uint16_t blank_reg;
    uint16_t decay_reg;
    uint16_t stall_reg;
    uint16_t drive_reg;
    uint16_t status_reg;
} drv8711;


// PRIVATE METHODS
static inline void select_chip(drv8711 *driver) {
    // I sentence the TI engineer who wrote the datasheet
    // To two semesters of Coopr for being unclear about this
    asm volatile("nop \n nop \n nop");
    gpio_put(SPI_MUX_S0, driver->cs_pin & 0x01);  // Active High
    gpio_put(SPI_MUX_S1, (driver->cs_pin >> 1) & 0x01);  // Active High
    gpio_put(SPI_MUX_S2, (driver->cs_pin >> 2) & 0x01);  // Active High
    asm volatile("nop \n nop \n nop");
}
static inline void deselect_chip(drv8711 *driver) {
    asm volatile("nop \n nop \n nop");
    //Corresponds to multiplexer pin 7 -- Unused
    gpio_put(SPI_MUX_S0,1);  // Active High
    gpio_put(SPI_MUX_S1, 1);  // Active High
    gpio_put(SPI_MUX_S2, 1);  // Active High
    asm volatile("nop \n nop \n nop");
}
static inline void set_cs_pin(drv8711 *driver, uint8_t pin) {
    driver->cs_pin = pin;
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}

static inline void set_slp_pin(drv8711 *driver, uint8_t pin) {
    driver->slp_pin = pin;
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 1);
}

// PUBLIC METHODS
static inline uint16_t drv8711_readReg(drv8711 *driver, DRV8711_registerAddress address) {
    // Bit  0    - Read
    // Bits 1:3  - Register address
    // Bits 4:15 - Irrelevant
    uint16_t send = (0x8 | (address & 0b111)) << 12;
    uint8_t tx[2] = {(send & 0x00FF), (send & 0xFF00) >> 8};
    uint8_t rx[2] = {};
    
    select_chip(driver);
    spi_write_read_blocking(driver->spi_port, tx, rx, 2);
    deselect_chip(driver);
    uint16_t rx0 = rx[0];
    uint16_t rx1 = rx[1];
    uint16_t whatIsRead = ((rx0 & 0x0F) >> 8) | rx1;

    return whatIsRead;
}

static inline void drv8711_writeReg(drv8711 *driver, DRV8711_registerAddress address, uint16_t value) {
    // Bit  0    - Write
    // Bits 1:3  - Register address
    // Bits 4:15 - Data to write
    uint16_t send = ((address & 0b111) << 12) | (value & 0xFFF);
    uint8_t tx[2] = {send >> 8, send & 0x00FF};
    select_chip(driver);
    spi_write_blocking(driver->spi_port, tx, 2);
    deselect_chip(driver);
}

static inline void drv8711_applySettings(drv8711 *driver) {
    drv8711_writeReg(driver, TORQUE_REG_ADDRESS, driver->torque_reg);
    drv8711_writeReg(driver, OFF_REG_ADDRESS, driver->off_reg);
    drv8711_writeReg(driver, BLANK_REG_ADDRESS, driver->blank_reg);
    drv8711_writeReg(driver, DECAY_REG_ADDRESS, driver->decay_reg);
    drv8711_writeReg(driver, DRIVE_REG_ADDRESS, driver->drive_reg);
    drv8711_writeReg(driver, STALL_REG_ADDRESS, driver->stall_reg);
    // Apply CTRL last because it contains the enable bit
    drv8711_writeReg(driver, CTRL_REG_ADDRESS, driver->ctrl_reg);
}

static inline void drv8711_resetSettings(drv8711 *driver) {
    driver->ctrl_reg   = 0xF11; //C10
    driver->torque_reg = 0x1DF; //1FF
    driver->off_reg    = 0x030;
    driver->blank_reg  = 0x080;
    driver->decay_reg  = 0x510;
    driver->stall_reg  = 0x040;
    driver->drive_reg  = 0xA59;
    drv8711_applySettings(driver);
}

static inline bool drv8711_verifySettings(drv8711 *driver) {
    return drv8711_readReg(driver, CTRL_REG_ADDRESS)   == driver->ctrl_reg   &&
           drv8711_readReg(driver, TORQUE_REG_ADDRESS) ==(driver->torque_reg & ~(1 << 10)) &&
           drv8711_readReg(driver, OFF_REG_ADDRESS)    == driver->off_reg    &&
           drv8711_readReg(driver, BLANK_REG_ADDRESS)  == driver->blank_reg  &&
           drv8711_readReg(driver, DECAY_REG_ADDRESS)  == driver->decay_reg  &&
           drv8711_readReg(driver, STALL_REG_ADDRESS)  == driver->stall_reg  &&
           drv8711_readReg(driver, DRIVE_REG_ADDRESS)  == driver->drive_reg;
}

static inline void drv8711_enableDriver(drv8711 *driver) {
    driver->ctrl_reg |= (1 << DRV8711_ENABLE_BIT);
    drv8711_writeReg(driver, CTRL_REG_ADDRESS, driver->ctrl_reg);
}

static inline void drv8711_disableDriver(drv8711 *driver) {
    driver->ctrl_reg &= ~(1 << DRV8711_ENABLE_BIT);
    drv8711_writeReg(driver, CTRL_REG_ADDRESS, driver->ctrl_reg);
}

static inline void drv8711_setAwake(drv8711 *driver) {
    gpio_put(driver->slp_pin, 1);
}

static inline void drv8711_setAsleep(drv8711 *driver) {
    gpio_put(driver->slp_pin, 0);
}

static inline void drv8711_setDirection(drv8711 *driver, bool direction) {
    // Direction is set as either alighning with DIR pin or aligning with its inverse
    // Clockwise is 1, counter-clockwise is 0 for DIR tied to ground
    if (direction) driver->ctrl_reg |= (1 << DRV8711_DIRECTION_BIT);
    else driver->ctrl_reg &= ~(1 << DRV8711_DIRECTION_BIT);
    drv8711_writeReg(driver, CTRL_REG_ADDRESS, driver->ctrl_reg);
}

static inline void drv8711_setCurrent(drv8711 *driver, uint16_t current) {
    // Drawing more than 8 amps is a bad idea
    if (current > 8000) { 
        current = 8000; 
    }
    // From the DRV8711 datasheet, section 7.3.4, equation 2:
    //
    //   Ifs = (2.75 V * TORQUE) / (256 * ISGAIN * Risense)
    //
    // Rearranged:
    //
    //   TORQUE = (256 * ISGAIN * Risense * Ifs) / 2.75 V
    //
    // The 36v4 has an Risense of 30 milliohms, and "current" is in milliamps,
    // so:
    //
    //   TORQUE = (256 * ISGAIN * (30/1000) ohms * (current/1000) A) / 2.75 V
    //          = (7680 * ISGAIN * current) / 2750000
    //
    // We want to pick the highest gain (5, 10, 20, or 40) that will not
    // overflow TORQUE (8 bits, 0xFF max), so we start with a gain of 40 and
    // calculate the TORQUE value needed.
    uint8_t isgainBits = 0b11;
    uint16_t torqueBits = ((uint32_t)768 * current) / 6875;
    // 0b11 is an amplifier gain of 40
    // 0b10 is an amplifier gain of 20
    // 0b01 is an amplifier gain of 10
    // 0b00 is an amplifier gain of 05
    while (torqueBits > 0xFF) {
        isgainBits--;
        torqueBits >>= 1;
    }
    driver->ctrl_reg = (driver->ctrl_reg & 0xCFF) | (isgainBits << 8);
    drv8711_writeReg(driver, CTRL_REG_ADDRESS, driver->ctrl_reg);
    driver->torque_reg = (driver->torque_reg & 0xF00) | torqueBits;
    drv8711_writeReg(driver, TORQUE_REG_ADDRESS, driver->torque_reg);
}

static inline void drv8711_step(drv8711 *driver) {
    // Not writing into drv8711 struct because step is immediately cleared internally
    drv8711_writeReg(driver, CTRL_REG_ADDRESS, driver->ctrl_reg | (1 << DRV8711_STEP_BIT));
}

// void drv8711_setStepMode(drv8711 *driver, DRV8711_stepMode mode) {
//     driver->ctrl_reg = (driver->ctrl_reg & ~0b1110) | ((32 - __builtin_clz(mode) - 1) << 1);
//     drv8711_writeReg(driver, CTRL_REG_ADDRESS, driver->ctrl_reg);
// }

static inline void drv8711_setStepMode(drv8711 *driver, DRV8711_stepMode mode)
  {
    // Pick 1/4 micro-step by default.
    uint8_t sm = 0b0010;
    switch (mode)
    {
    case MicroStep1:   sm = 0b0000; break;
    case MicroStep2:   sm = 0b0001; break;
    case MicroStep4:   sm = 0b0010; break;
    case MicroStep8:   sm = 0b0011; break;
    case MicroStep16:  sm = 0b0100; break;
    case MicroStep32:  sm = 0b0101; break;
    case MicroStep64:  sm = 0b0110; break;
    case MicroStep128: sm = 0b0111; break;
    case MicroStep256: sm = 0b1000; break;
    }
    driver->ctrl_reg = (driver->ctrl_reg & 0b111110000111) | (sm << 3);
    drv8711_writeReg(driver, CTRL_REG_ADDRESS, driver->ctrl_reg);
}

static inline void drv8711_setDecayMode(drv8711 *driver, DRV8711_decayMode mode) {
    driver->decay_reg = (driver->decay_reg & 0b00011111111) | (((uint8_t)mode & 0b111) << 8);
    drv8711_writeReg(driver, DECAY_REG_ADDRESS, driver->decay_reg);
}

static inline uint8_t drv8711_readStatus(drv8711 *driver) {
    return drv8711_readReg(driver, STATUS_REG_ADDRESS);
}

static inline void drv8711_clearStatus(drv8711 *driver) {
    drv8711_writeReg(driver, STATUS_REG_ADDRESS, 0x00);
}

static inline uint8_t drv8711_readFaults(drv8711 *driver) {
    return drv8711_readReg(driver, STATUS_REG_ADDRESS);
}

static inline void drv8711_clearFaults(drv8711 *driver) {
    drv8711_writeReg(driver, STATUS_REG_ADDRESS, driver->status_reg & 0xF00);
}

//INCORRECT
static inline bool drv8711_getDirection(drv8711 *driver) {
    return (driver->ctrl_reg & (1 << DRV8711_DIRECTION_BIT)) != 0;
}




static inline void drv8711_init(
    drv8711 *driver, 
    spi_inst_t *spi_port, 
    uint8_t cs_pin,
    uint8_t slp_pin, 
    DRV8711_decayMode decay_mode,
    DRV8711_stepMode step_mode,
    uint16_t max_winch_current
) {


driver->spi_port = spi_port;
// set_cs_pin(driver, cs_pin);     // Pulled low by default
set_slp_pin(driver, slp_pin);   // Pulled low by default
sleep_ms(1000);

drv8711_clearStatus(driver);
drv8711_setDecayMode(driver, decay_mode);
drv8711_setCurrent(driver, max_winch_current);
drv8711_setStepMode(driver, step_mode);
drv8711_enableDriver(driver);
drv8711_setAwake(driver);

sleep_ms(50);
// drv8711_resetSettings(driver); // Disables DRV8711 by default
}

#endif

