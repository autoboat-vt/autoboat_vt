#ifndef GVM_I2CSENSOR_H  
#define GVM_I2CSENSOR_H 

#include "hardware/i2c.h"
#include <cmath>
#include "i2c_parent_class.h"

 

class GVMSensor : public I2CDevice { 
    public:
        int registers[4];



    GVM(i2c_inst_t* port, uint8_t address);

    int32_t readVoltage(int cell);    

    ~GVM();




};//end GVM

#endif