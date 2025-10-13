#include "spi_device.hpp"


class spi_device {


    public:
        spi_device(uint csPin) {
            
            this->csPin = csPin; 

            gpio_init(csPin);
            gpio_set_dir(csPin, GPIO_OUT);
            // TODO: replace with decoder
            gpio_put(csPin, 0); // idle (inactive) for DRV8711 (active high)
        }


        uint8_t transfer(uint8_t data) : csPin(csPin) {
            cs_select();
            uint8_t rx = bus.transfer(data);
            cs_deselect();
            return rx;
        }


        void transfer(const uint8_t* tx, uint8_t* rx, size_t len) {
            cs_select();
            bus.transfer(tx, rx, len);
            cs_deselect();
        }


    private:
       
       
        SPIBus bus;
        uint csPin;




        static inline void cs_select() {
            asm volatile("nop \n nop \n nop");
            asm volatile("nop \n nop \n nop");
            gpio_put(PICO_SPI_CSN_PIN, 0);  // Active low
            asm volatile("nop \n nop \n nop");
            asm volatile("nop \n nop \n nop");
        }


        static inline void cs_deselect() {
            asm volatile("nop \n nop \n nop");
            asm volatile("nop \n nop \n nop");
            gpio_put(PICO_SPI_CSN_PIN, 1);  //Activate high
            asm volatile("nop \n nop \n nop");
            asm volatile("nop \n nop \n nop");
        }


};
