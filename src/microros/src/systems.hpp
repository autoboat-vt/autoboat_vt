#ifndef SYSTEMS_H
#define SYSTEMS_H

#include "boat.hpp"
#include "microros.hpp"
#include "common_libraries.h"
#include "HAL.hpp"



class Systems {
    public:
        Systems(boat_type bt);
        void initalize_cores();
        
        static void application_loop(rcl_timer_t * timer, int64_t last_call_time);
        void initialize_application_loop();
        
        void initialize_microros();
        void check_microros();
        
        void initialize_hal();

        void cleanup();


    private:
        boat_type current_boat;
        
        rclc_executor_t executor;
        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_t support;
        
        rcl_timer_t application_loop_timer; 

        Microros microros_node;

        

};
#endif