#include "i2c_parent_class.h" 

//Note to self
//#define name_thing second_thing will replace any mention of name_thing with second_thing durring compiling. Basically, variables
//Pointers are like having 100 pizzas, and saying peopel can get them in your car, instead of carrying it all the time
//*points to the adress, &gets the adress
// string* pName = &myName
//dont use when the variable is allready a adress



    //Variables
   
    //constructor
    I2CDevice::I2CDevice(i2c_inst_t* port, uint8_t address) : i2cAddress(address), i2cPort(port) { //maybe check pointers?

    }
    // sends a byte of data to a address, used for configuration
    int I2CDevice::writeByte(uint8_t reg, uint8_t value){
        uint8_t data[] = {reg, value};
        int result = i2c_write_blocking(i2cPort,i2cAddress,data,2,false);
        return result;
    }

    //i2c_write_blocking(&PicoPin,destination,&dataArray,many bytes?,repeat?) //returns result
    
    int I2CDevice::readBytes(uint8_t reg, uint8_t *buffer, uint8_t length){
        i2c_write_blocking(i2cPort, i2cAddress, &reg, 1, true); //Hey I want to start reading from sensor [reg], leave communications open
        int result = i2c_read_blocking(i2cPort, i2cAddress, buffer, length, false); //Take (#length) bytes and but them at the adress of buffer. Close communications
        return result;
    }
    
    //read and write byte and the base things need for everything, but each individual device will have their own 
    //methods using read and write byte in order to gather data (getGYRO())

    
    //deconstructor
    I2CDevice::~I2CDevice(){
        //Cleanup Code, only primitives and no pointers[?], nothing needed
    }

