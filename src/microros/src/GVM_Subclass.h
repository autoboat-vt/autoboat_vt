#ifndef GVM_I2CSENSOR_H  
#define GVM_I2CSENSOR_H 

#include "hardware/i2c.h"
#include <cmath>
#include "i2c_parent_class.h"

#define CELL_1_REGISTER 0x01
#define CELL_2_REGISTER 0x16
#define CELL_3_REGISTER 0x18
#define CELL_4_REGISTER 0x1A
#define CELL_5_REGISTER 0x1C
#define CELL_6_REGISTER 0x1E
#define CELL_7_REGISTER 0x20
#define CELL_8_REGISTER 0x22
#define CELL_9_REGISTER 0x24
#define CELL_10_REGISTER 0x26
#define CELL_11_REGISTER 0x28
#define CELL_12_REGISTER 0x2A
#define CELL_13_REGISTER 0x2C
#define CELL_14_REGISTER 0x2E
#define CELL_15_REGISTER 0x30
#define CELL_16_REGISTER 0x32
 

class GVM : public I2CDevice { 
    public:
        uint8_t registers[16]={CELL_1_REGISTER,CELL_2_REGISTER,CELL_3_REGISTER,CELL_4_REGISTER,CELL_5_REGISTER,CELL_6_REGISTER
        ,CELL_7_REGISTER,CELL_8_REGISTER,CELL_9_REGISTER,CELL_10_REGISTER,CELL_11_REGISTER,CELL_12_REGISTER,CELL_13_REGISTER
        ,CELL_14_REGISTER,CELL_15_REGISTER,CELL_16_REGISTER};



    GVM(i2c_inst_t* port, uint8_t address);

    int16_t readVoltage(int cell);    

    ~GVM();




};//end GVM

#endif