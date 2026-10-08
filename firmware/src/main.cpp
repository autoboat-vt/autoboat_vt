#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "common_libraries.h"

#include "FreeRTOS.h"
#include "task.h"

#include "systems.hpp"

static void microros_task(void *params);
static void node_task(void *params);

// Set by microros_task once the agent is connected and the system is
// initialized; node_task must not touch the HAL before then.
static volatile bool sharedReady = false;

int main() {
    stdio_init_all();
    xTaskCreate(microros_task, "microros Task", 2048, NULL, 1, NULL);
    xTaskCreate(node_task, "Node Task", 2048, NULL, 1, NULL);
    vTaskStartScheduler();
    for (;;) {}
}

static void microros_task(void *params) {
    (void)params;

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

        Systems system = Systems(THESEUS);
        system.initialize_microros();
        system.initialize_hal();

        sharedReady = true;

        while (true) {
            if (rmw_uros_ping_agent(1000, 5) != RMW_RET_OK) {
                break;
            }

            system.check_microros();
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        sharedReady = false;
        system.cleanup();
    }
}

static void node_task(void *params) {
    (void)params;

    for (;;) {
        while (!sharedReady) { vTaskDelay(pdMS_TO_TICKS(10)); }

        Systems::application_loop_step();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
