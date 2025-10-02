#ifndef BOAT_H
#define BOAT_H

//Author: Elias Mihkael
//Designed so main does not need to know type of boat 
//Each subclass should handle initialization of hardware peripherals 
//This is done via the HAL class

//add new boat type to enum and declare a class if you want a new baot
enum boat_type {LUMPY, THESEUS, TITAN, DUCKHUNTER};

class Boat{
    public:
        static Boat* initialize(boat_type bt);  
        virtual ~Boat() = default; 
        virtual uint8_t lifecycle() = 0;

}

class Lumpy : public Boat{
    public:
        Lumpy();
        uint8_t lifecycle() override;
    private:
}

class Theseus : public Boat{
    public:
        Theseus();
        uint8_t lifecycle() override;
    private:
}

class Titan : public Boat{
    public:
        Titan();
        uint8_t lifecycle() override;
    private:

}

class DuckHunter : public Boat{
    public:
        DuckHunter();
        uint8_t lifecycle() override;
    private:

}

#endif
