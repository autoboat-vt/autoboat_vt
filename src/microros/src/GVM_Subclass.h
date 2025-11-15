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
        int registers[4];



    DATATYPE GVM(i2c_inst_t* port, uint8_t address);

    readVoltage(int cell);    

    ~GVM();




};//end GVM

#endif