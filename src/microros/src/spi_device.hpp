#ifndef SPI_DEVICE_H
#define SPI_DEVICE_H


#include <stdint.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"




class spi_device {


    public:
        spi_device(spi_inst_t* spi_port, uint csPin) {}


        // Transfer a single byte
        uint8_t transfer(uint8_t data);

        // Transfer multiple bytes
        void transfer(const uint8_t* tx, uint8_t* rx, size_t len);


    private:
        spi_inst_t *spi_port;
        uint csPin;




        static inline void cs_select();


        static inline void cs_deselect();


};


#endif

