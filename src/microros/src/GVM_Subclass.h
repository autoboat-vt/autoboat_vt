#ifndef GVM_I2CSENSOR_H  
#define GVM_I2CSENSOR_H 

#include "hardware/i2c.h"
#include <cmath>
#include "i2c_parent_class.h"

#define INPUT_1_Register 0x0071
#define INPUT_2_Register 0x0072
#define INPUT_3_Register 0x0073
#define INPUT_4_Register 0x0074
 

class GVM : public I2CDevice { 
    public:
        uint16_t registers[4]={INPUT_1_Register,INPUT_2_Register,INPUT_3_Register,INPUT_4_Register};



    GVM(i2c_inst_t* port, uint8_t address);

    int32_t readVoltage(int cell);    

    ~GVM();




};//end GVM

#endif