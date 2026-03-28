#include "spi_device.hpp"

        spi_device::spi_device(spi_inst_t* spi_port, uint csPin) : csPin(csPin), spi_port(spi_port) {

            // gpio_init(csPin);
            // gpio_set_dir(csPin, GPIO_OUT);
            // TODO: replace with decoder
            //gpio_put(csPin, 0); // idle (inactive) for DRV8711 (active high)
        }

        // Transfer a single byte
        uint8_t spi_device::transfer(uint8_t data) {
            uint8_t rx;
            cs_low();
            spi_write_read_blocking(spi_port, &data, &rx, 1);
            cs_high();
            return rx;
        }

        // Transfer multiple bytes
        void spi_device::transfer(const uint8_t* tx, uint8_t* rx, size_t len) {
            cs_low();
            spi_write_read_blocking(spi_port, tx, rx, len);
            cs_high();
        }

        void spi_device::cs_low() {
            asm volatile("nop \n nop \n nop");
            asm volatile("nop \n nop \n nop");
            gpio_put(SPI_MUX_S0, (csPin & 0x01)); 
            gpio_put(SPI_MUX_S1, ((csPin >> 1) & 0x01));  
            gpio_put(SPI_MUX_S2, ((csPin >> 2) & 0x01)); 
    

            asm volatile("nop \n nop \n nop");
            asm volatile("nop \n nop \n nop");
        }

        void spi_device::cs_high() {
            asm volatile("nop \n nop \n nop");
            asm volatile("nop \n nop \n nop");
            //This corresponds to "7" on the multiplexer, an unsused pin.
            gpio_put(SPI_MUX_S0, 1);  // Active Low
            gpio_put(SPI_MUX_S1, 1);  // Active Low
            gpio_put(SPI_MUX_S2, 1);  // Active Low
            asm volatile("nop \n nop \n nop");
            asm volatile("nop \n nop \n nop");
        }

