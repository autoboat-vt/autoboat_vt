#ifndef DRV8711_STEPPER_MOTOR_DRIVER_LIBRARY_H
#define DRV8711_STEPPER_MOTOR_DRIVER_LIBRARY_H

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "spi_device.hpp"

#define DRV8711_ENABLE_BIT 0
#define DRV8711_DIRECTION_BIT 1
#define DRV8711_STEP_BIT 2

// activate high

typedef enum
{
    CTRL_REG_ADDRESS = 0x00,
    TORQUE_REG_ADDRESS = 0x01,
    OFF_REG_ADDRESS = 0x02,
    BLANK_REG_ADDRESS = 0x03,
    DECAY_REG_ADDRESS = 0x04,
    STALL_REG_ADDRESS = 0x05,
    DRIVE_REG_ADDRESS = 0x06,
    STATUS_REG_ADDRESS = 0x07,
} DRV8711_registerAddress;

typedef enum
{
    MicroStep256 = 256,
    MicroStep128 = 128,
    MicroStep64 = 64,
    MicroStep32 = 32,
    MicroStep16 = 16,
    MicroStep8 = 8,
    MicroStep4 = 4,
    MicroStep2 = 2,
    MicroStep1 = 1,
} DRV8711_stepMode;

typedef enum
{
    Slow = 0b000,
    SlowIncMixedDec = 0b001,
    Fast = 0b010,
    Mixed = 0b011,
    SlowIncAutoMixedDec = 0b100,
    AutoMixed = 0b101,
} DRV8711_decayMode;

class drv8711 : public spi_device
{

public:
    drv8711(
        spi_inst_t *spi_port,
        uint8_t cs_pin,
        uint8_t slp_pin,
        DRV8711_decayMode decay_mode,
        DRV8711_stepMode step_mode,
        uint16_t max_winch_current);

    // TODO: call chip select and deselect methods

    uint16_t drv8711_readReg(DRV8711_registerAddress address);

    void drv8711_writeReg(DRV8711_registerAddress address, uint16_t value);

    void drv8711_applySettings();

    void drv8711_resetSettings();

    bool drv8711_verifySettings();

    void drv8711_enableDriver();

    void drv8711_disableDriver();

    void drv8711_setAwake();

    void drv8711_setAsleep();

    void drv8711_setDirection(bool direction);

    void drv8711_setCurrent(uint16_t current);

    void drv8711_step();

    // void drv8711_setStepMode(drv8711 *driver, DRV8711_stepMode mode) {
    //     driver->ctrl_reg = (driver->ctrl_reg & ~0b1110) | ((32 - __builtin_clz(mode) - 1) << 1);
    //     drv8711_writeReg(driver, CTRL_REG_ADDRESS, driver->ctrl_reg);
    // }

    void drv8711_setStepMode(DRV8711_stepMode mode);

    void drv8711_setDecayMode(DRV8711_decayMode mode);

    uint8_t drv8711_readStatus();

    void drv8711_clearStatus();

    uint8_t drv8711_readFaults();

    void drv8711_clearFaults();

    // INCORRECT
    bool drv8711_getDirection();

private:
    uint8_t slp_pin;
    uint16_t ctrl_reg;
    uint16_t torque_reg;
    uint16_t off_reg;
    uint16_t blank_reg;
    uint16_t decay_reg;
    uint16_t stall_reg;
    uint16_t drive_reg;
    uint16_t status_reg;
};

#endif
