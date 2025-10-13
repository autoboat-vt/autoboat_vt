#ifndef SPI_DEVICE_H
#define SPI_DEVICE_H


#include <stdint.h>
#include <stddef.h>
#include "pico/stdlib.h"




class spi_device {


    public:
        spi_device(uint csPin) {}


        uint8_t transfer(uint8_t data);


        void transfer(const uint8_t* tx, uint8_t* rx, size_t len);


    private:
        SPIBus bus;
        uint csPin;




        static inline void cs_select();


        static inline void cs_deselect();


};


#endif

