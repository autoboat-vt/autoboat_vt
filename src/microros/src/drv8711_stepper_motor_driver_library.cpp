#include "drv8711_stepper_motor_driver_library.h"

        //activate high

        drv8711::drv8711(
            spi_inst_t *spi_port,
            uint8_t cs_pin,
            uint8_t slp_pin,
            DRV8711_decayMode decay_mode,
            DRV8711_stepMode step_mode,
            uint16_t max_winch_current
        ) : spi_device(spi_port, cs_pin), slp_pin(slp_pin)  {
           
            gpio_put(slp_pin, 1);
            sleep_ms(1000);
           
            drv8711_clearStatus();
            drv8711_setDecayMode(decay_mode);
            drv8711_setCurrent(max_winch_current);
            drv8711_setStepMode(step_mode);
            drv8711_enableDriver();
            drv8711_setAwake();

            sleep_ms(50);
            // drv8711_resetSettings(driver); // Disables DRV8711 by default
        }


    // TODO: call chip select and deselect methods
   


        uint16_t drv8711::drv8711_readReg(DRV8711_registerAddress address) {
            // Bit  0    - Read
            // Bits 1:3  - Register address
            // Bits 4:15 - Irrelevant
            uint16_t send = (0x8 | (address & 0b111)) << 12;
            uint8_t tx[2] = {(send & 0x00FF), (send & 0xFF00) >> 8};
            uint8_t rx[2] = {};
            
            

            this->cs_high();
            spi_write_read_blocking(spi_port, tx, rx, 2);
            this->cs_low();
            uint16_t rx0 = rx[0];
            uint16_t rx1 = rx[1];
            uint16_t whatIsRead = ((rx0 & 0x0F) >> 8) | rx1;


            return whatIsRead;
        }


        void drv8711::drv8711_writeReg(DRV8711_registerAddress address, uint16_t value) {
            // Bit  0    - Write
            // Bits 1:3  - Register address
            // Bits 4:15 - Data to write
            uint16_t send = ((address & 0b111) << 12) | (value & 0xFFF);
            uint8_t tx[2] = {send >> 8, send & 0x00FF};
            this->cs_high();
            spi_write_blocking(spi_port, tx, 2);
            this->cs_low();
        }


        void drv8711::drv8711_applySettings() {
            drv8711_writeReg(TORQUE_REG_ADDRESS, torque_reg);
            drv8711_writeReg(OFF_REG_ADDRESS, off_reg);
            drv8711_writeReg(BLANK_REG_ADDRESS, blank_reg);
            drv8711_writeReg(DECAY_REG_ADDRESS, decay_reg);
            drv8711_writeReg(DRIVE_REG_ADDRESS, drive_reg);
            drv8711_writeReg(STALL_REG_ADDRESS, stall_reg);
            // Apply CTRL last because it contains the enable bit
            drv8711_writeReg(CTRL_REG_ADDRESS, ctrl_reg);
        }


        void drv8711::drv8711_resetSettings() {
            ctrl_reg   = 0xF11; //C10
            torque_reg = 0x1DF; //1FF
            off_reg    = 0x030;
            blank_reg  = 0x080;
            decay_reg  = 0x510;
            stall_reg  = 0x040;
            drive_reg  = 0xA59;
            drv8711_applySettings();
        }


        bool drv8711::drv8711_verifySettings() {
            return drv8711_readReg(CTRL_REG_ADDRESS) == ctrl_reg   &&
                drv8711_readReg(TORQUE_REG_ADDRESS) ==(torque_reg & ~(1 << 10)) &&
                drv8711_readReg(OFF_REG_ADDRESS)    == off_reg    &&
                drv8711_readReg(BLANK_REG_ADDRESS)  == blank_reg  &&
                drv8711_readReg(DECAY_REG_ADDRESS)  == decay_reg  &&
                drv8711_readReg(STALL_REG_ADDRESS)  == stall_reg  &&
                drv8711_readReg(DRIVE_REG_ADDRESS)  == drive_reg;
        }


        void drv8711::drv8711_enableDriver() {
            ctrl_reg |= (1 << DRV8711_ENABLE_BIT);
            drv8711_writeReg(CTRL_REG_ADDRESS, ctrl_reg);
        }


        void drv8711::drv8711_disableDriver() {
            ctrl_reg &= ~(1 << DRV8711_ENABLE_BIT);
            drv8711_writeReg(CTRL_REG_ADDRESS, ctrl_reg);
        }


        void drv8711::drv8711_setAwake() {
            gpio_put(slp_pin, 1);
        }


        void drv8711::drv8711_setAsleep() {
            gpio_put(slp_pin, 0);
        }


        void drv8711::drv8711_setDirection(bool direction) {
            // Direction is set as either alighning with DIR pin or aligning with its inverse
            // Clockwise is 1, counter-clockwise is 0 for DIR tied to ground
            if (direction) ctrl_reg |= (1 << DRV8711_DIRECTION_BIT);
            else ctrl_reg &= ~(1 << DRV8711_DIRECTION_BIT);
            drv8711_writeReg( CTRL_REG_ADDRESS, ctrl_reg);
        }


        void drv8711::drv8711_setCurrent(uint16_t current) {
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
            ctrl_reg = (ctrl_reg & 0xCFF) | (isgainBits << 8);
            drv8711_writeReg(CTRL_REG_ADDRESS, ctrl_reg);
            torque_reg = (torque_reg & 0xF00) | torqueBits;
            drv8711_writeReg(TORQUE_REG_ADDRESS, torque_reg);
        }  


        void drv8711::drv8711_step() {
            // Not writing into drv8711 struct because step is immediately cleared internally
            drv8711_writeReg(CTRL_REG_ADDRESS, ctrl_reg | (1 << DRV8711_STEP_BIT));
        }


        // void drv8711_setStepMode(drv8711 *driver, DRV8711_stepMode mode) {
        //     driver->ctrl_reg = (driver->ctrl_reg & ~0b1110) | ((32 - __builtin_clz(mode) - 1) << 1);
        //     drv8711_writeReg(driver, CTRL_REG_ADDRESS, driver->ctrl_reg);
        // }


        void drv8711::drv8711_setStepMode(DRV8711_stepMode mode) {
            // Pick 1/4 micro-step by default.
            uint8_t sm = 0b0010;
            switch (mode) {
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
            ctrl_reg = (ctrl_reg & 0b111110000111) | (sm << 3);
            drv8711_writeReg(CTRL_REG_ADDRESS, ctrl_reg);
        }


        void drv8711::drv8711_setDecayMode(DRV8711_decayMode mode) {
            decay_reg = (decay_reg & 0b00011111111) | (((uint8_t)mode & 0b111) << 8);
            drv8711_writeReg(DECAY_REG_ADDRESS, decay_reg);
        }


        uint8_t drv8711::drv8711_readStatus() {
            return drv8711_readReg(STATUS_REG_ADDRESS);
        }


        void drv8711::drv8711_clearStatus() {
            drv8711_writeReg(STATUS_REG_ADDRESS, 0x00);
        }


        uint8_t drv8711::drv8711_readFaults() {
            return drv8711_readReg(STATUS_REG_ADDRESS);
        }


        void drv8711::drv8711_clearFaults() {
            drv8711_writeReg(STATUS_REG_ADDRESS, status_reg & 0xF00);
        }


        //INCORRECT
        bool drv8711::drv8711_getDirection() {
            return (ctrl_reg & (1 << DRV8711_DIRECTION_BIT)) != 0;
        }
