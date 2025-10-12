// Include the microros nodes
#include "main_microros_node.c"

// Change when adding new nodes
#define NUMBER_OF_NODES 1

// Global microros structs
rclc_executor_t executor_core;
rcl_allocator_t allocator_core;
rclc_support_t support_core;


int main()
{
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

        allocator_core = rcl_get_default_allocator();

        const int timeout_ms = 1000; 
        const uint8_t attempts = 120;
        rmw_uros_ping_agent(timeout_ms, attempts);
        
        rclc_support_init(&support_core, 0, NULL, &allocator_core);
        rclc_executor_init(&executor_core, &support_core.context, 5, &allocator_core);

        application_init(&allocator_core, &support_core, &executor_core);

        while (true) {
            // Ping the agent every few seconds to check connection
            if (rmw_uros_ping_agent(1000, 5) != RMW_RET_OK) {
                break;
            }
        
            rclc_executor_spin_some(&executor_core, RCL_MS_TO_NS(100));
        }
        
        rclc_executor_fini(&executor_core);
        rclc_support_fini(&support_core);
    }
}