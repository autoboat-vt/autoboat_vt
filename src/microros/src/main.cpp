// Include the microros nodes
//cd#include "boat.hpp"

#include "common_libraries.h"
//#include "microros.hpp"
#include "systems.hpp"


// Change when adding new nodes
#define NUMBER_OF_NODES 1




int main()
{
    //instantiate boat system
    // Systems current_boat(THESEUS);


    stdio_init_all();

    while (true) {
        rmw_uros_set_custom_transport(
            true,
            NULL,
            pico_serial_transport_open,
            pico_serial_transport_close,
            pico_serial_transport_write,
            pico_serial_transport_read
        );

        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, 1);

        const int timeout_ms = 1000;
        const uint8_t attempts = 120;
        rmw_uros_ping_agent(timeout_ms, attempts);

        //initialize System
        Systems system = Systems(THESEUS);
        system.initialize_microros();
        system.initialize_hal();
        system.initialize_application_loop();



        while (true) {
            // Ping the agent every few seconds to check connection
            if (rmw_uros_ping_agent(1000, 5) != RMW_RET_OK) {
                break;
            }
            
            system.check_microros();
            
        }
        
        system.cleanup();
    }
}