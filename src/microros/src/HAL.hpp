#ifndef HAL_H
#define HAL_H

//TODO: include all driver libraries here

namespace HAL {
    void init_stepper();
    void init_contactor();
    void init_hydraulics();
    void init_magnetometer();
    void init_servo();
    void init_pump();
    void init_propeller();
}

#endif