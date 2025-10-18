#include "spi_device.hpp"

        spi_device::spi_device(spi_inst_t* spi_port, uint csPin) : csPin(csPin), spi_port(spi_port) {

            gpio_init(csPin);
            gpio_set_dir(csPin, GPIO_OUT);
            // TODO: replace with decoder
            gpio_put(csPin, 0); // idle (inactive) for DRV8711 (active high)
        }

        // Transfer a single byte
        uint8_t spi_device::transfer(uint8_t data) {
            uint8_t rx;
            cs_select();
            spi_write_read_blocking(spi_port, &data, &rx, 1);
            cs_deselect();
            return rx;
        }

        // Transfer multiple bytes
        void spi_device::transfer(const uint8_t* tx, uint8_t* rx, size_t len) {
            cs_select();
            spi_write_read_blocking(spi_port, tx, rx, len);
            cs_deselect();
        }

        void spi_device::cs_select() {
            asm volatile("nop \n nop \n nop");
            asm volatile("nop \n nop \n nop");
            gpio_put(csPin, 0);  // Active low
            asm volatile("nop \n nop \n nop");
            asm volatile("nop \n nop \n nop");
        }


        void spi_device::cs_deselect() {
            asm volatile("nop \n nop \n nop");
            asm volatile("nop \n nop \n nop");
            gpio_put(csPin, 1);  //Activate high
            asm volatile("nop \n nop \n nop");
            asm volatile("nop \n nop \n nop");
        }


