#include "GVM_Subclass.h" 



GVM::GVM(i2c_inst_t* port, uint8_t address) : I2CDevice(port,address) {

}

//first battery is cell 0 <-----
int16_t GVM::readVoltage(int cell) {
    
    uint8_t buffer[2];
    if (readBytes(registers[cell-1], buffer, 2) != 2) {
        return -1;
    }
    int16_t result = ((buffer[0] << 8) | buffer[1]);
    return result;

}

 GVM::~GVM(){

 }


