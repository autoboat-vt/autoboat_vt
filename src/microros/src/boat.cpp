#include "boat.hpp"

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
    //add HAL calls here
}

Theseus::Theseus(){
    //add HAL calls here
}

DuckHunter::DuckHunter(){
    //add HAL calls here
}

Titan::Titan(){
    //add HAL calls here
}