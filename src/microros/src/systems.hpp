#ifndef SYSTEMS_H
#define SYSTEMS_H

#include "boat.hpp"
#include "main_microros_node.h"

class Systems {
    public:
        Systems(boat_type bt);
        void application_loop();

}
#endif