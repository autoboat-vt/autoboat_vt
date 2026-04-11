//GPIO0 1000ms
//GPIO1 2500ms
//GPIO2 4500ms

#include <stdio.h>
#include <stdlib.h>

#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "common_libraries.h"
//extern "C" {
#include "FreeRTOS.h"
#include "task.h"
//}
#include "main_microros_node.h"
#include "systems.hpp"


bool sharedReady = false;

void start_rtos_main() {
    stdio_init_all();
    xTaskCreate(microros_task, "microros Task", 2048, NULL, 1, NULL);
    xTaskCreate(node_task, "Node Task", 2048, NULL, 1, NULL);
    vTaskStartScheduler();
}

void microros_task(void *params) {
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

            sharedReady = true;

            while (true) {
                // Ping the agent every few seconds to check connection
                if (rmw_uros_ping_agent(1000, 5) != RMW_RET_OK) {
                    break;
                }
                
                system.check_microros();
                
            }
            
            sharedReady = false;
            system.cleanup();
    }

}

void node_task(void *params) {
    for(;;) {
        while(shared_ready) {
            application_loop();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}


















/*
typedef struct GPIO{
    int pin;
    int on_ms;
    int off_ms;

} GPIO;

void task(void *params);

int main() {

    gpio_init(0);
    gpio_init(1);
    gpio_init(2);

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 1);

    gpio_set_dir(0, GPIO_OUT);
    gpio_set_dir(1, GPIO_OUT);
    gpio_set_dir(2, GPIO_OUT);

    GPIO led0 = {0, 1000, 1000};
    GPIO led1 = {1, 2500, 2500};
    GPIO led2 = {2, 4500, 4500};

    xTaskCreate(task, "gpio0", configMINIMAL_STACK_SIZE, &led0, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(task, "gpio1", configMINIMAL_STACK_SIZE, &led1, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(task, "gpio2", configMINIMAL_STACK_SIZE, &led2, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();
    for(;;);

}


void task(void *params) {

    GPIO *cfg = (GPIO*)params;

    for(;;) {
        gpio_put(cfg->pin, 1);
        vTaskDelay(pdMS_TO_TICKS(cfg->on_ms));
        gpio_put(cfg->pin, 0);
        vTaskDelay(pdMS_TO_TICKS(cfg->off_ms));
    }
}


*/
/*
int main() {

    gpio_set_function(9, GPIO_FUNC_PWM);
    int slice_num = pwm_gpio_to_slice_num(9);
    int channel_num = pwm_gpio_to_channel(9);

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    pwm_set_clkdiv(slice_num, 125.0f);
    pwm_set_wrap(slice_num, 19999);
    pwm_set_enabled(slice_num, true);

    for (;;) {
        gpio_put(25, 1);
        for (int level = 2000; level >= 1000; level -= 10) {
            pwm_set_chan_level(slice_num, channel_num, level);
            sleep_ms(20);
        }       
        
        for (int level = 1000; level <= 2000; level += 10) {
            pwm_set_chan_level(slice_num, channel_num, level);
            sleep_ms(20);
        }
        gpio_put(25, 0);

    }
    
}
*/
