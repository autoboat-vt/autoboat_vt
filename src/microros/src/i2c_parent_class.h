#ifndef I2C_DEVICE_H  
#define I2C_DEVICE_H  

#include "hardware/i2c.h"
#include <cmath>  

//Note to self
//#define name_thing second_thing will replace any mention of name_thing with second_thing durring compiling. Basically, variables
//Pointers are like having 100 pizzas, and saying peopel can get them in your car, instead of carrying it all the time
//*points to the adress, &gets the adress
// string* pName = &myName
//dont use when the variable is allready a adress

class I2CDevice{
public:



    //Variables
    uint8_t i2cAddress; //the street where the data lives, reg is their house
    i2c_inst_t* i2cPort; //[Note], struct changed to pointer for struct to help efficency

    I2CDevice(i2c_inst_t* port, uint8_t address);

    int writeByte(uint8_t reg, uint8_t value);

    int readBytes(uint8_t reg, uint8_t *buffer, uint8_t length);

    ~I2CDevice();




};//end I2CDevice

#endif