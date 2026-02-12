#include "boat.hpp"

/*
List of all components (PUT THIS IN HAL LATER):
Stepper 
contactor
hydraulic
magnetometer
servo
pump
propeller
*/

Boat::initialize(boat_type bt){
    if(bt == LUMPY){
        return new Lumpy();
    }
    if(bt == THESEUS){
        return new Theseus();
    }
    if(bt == TITAN){
        return new Titan();
    }
    if(bt == DUCKHUNTER){
        return new DuckHunter();
    }
    return nullptr;
}

Lumpy::Lumpy(){
    /*
    List of components
    2x Steppers - rudder + sail
    magnetometer
    */

    //add HAL calls here

    /*
    List of publishers
    2 - stepper
    List of subscribers
    */
    //add pub/sub (directly)
}

Theseus::Theseus(){
    /*
    List of components
    Stepper - rudder
    contactor 
    magnetometer
    propeller (controlled by jetson for the near future)
    pump
    servo - camera
    */
    //add HAL calls here

    /*
    List of publishers
    List of subscribers
    */
    //add pub/sub (directly)
}

DuckHunter::DuckHunter(){
    /*
    List of components
    hydraulic
    pump
    magnetometer
    contactor 
    servo - camera
    */
    //add HAL calls here

    /*
    List of publishers
    List of subscribers
    */
    //add pub/sub (directly)
}

Titan::Titan(){
    /*
    List of components
    2x Steppers - rudder + sail
    magnetometer
    */

    //add HAL calls here

    /*
    List of publishers
    List of subscribers
    */
    //add pub/sub (directly)
}