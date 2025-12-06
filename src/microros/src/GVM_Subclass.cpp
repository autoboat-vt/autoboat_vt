#include "GVM_Subclass.h" 

#define INPUT_1_Register 0x0071
#define INPUT_2_Register 0x0072
#define INPUT_3_Register 0x0073
#define INPUT_4_Register 0x0074

GVM::GVM(i2c_inst_t* port, uint8_t address) : I2CDevice(port,address) {
    registers={INPUT_1_Register,INPUT_2_Register,INPUT_3_Register,INPUT_4_Register};
}

//first battery is cell 0 <-----
int32_t GVM::readVoltage(int cell) {
    //1st registers cells 1-4, then 2nd register 5-8, and so on
    int reg = cell/4;
    int start = (cell%4)*8;
    uint8_t buffer[32];
    if (readBytes(registers[reg], buffer, 32) != 32) {
        return -1;
    }
    int32_t result = ((buffer[start] << 24) | (buffer[start+1] << 16) | (buffer[start+2] << 8) | buffer[start+3]);
    return result;

}

 GVM::~GVM(){

 }


