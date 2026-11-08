#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "common_libraries.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "systems.hpp"

static void microros_connect_task(void *params);
static void microros_pulse_task(void *params);
static void microros_executor_task(void *params);
static void application_task(void *params);

static TaskHandle_t connectTaskHandle = NULL;

// Ping and spin used to run back to back in one loop (microros lines could be written at the same time) so a mutex line is setup
static SemaphoreHandle_t microrosMutex = NULL;

// Set by microros_connect_task once the agent is connected and the system is initialized
static volatile bool sharedReady = false;

// Owned by microros_connect_task; valid only while sharedReady is true.
static Systems *activeSystem = NULL;

int main() {
    stdio_init_all();

    microrosMutex = xSemaphoreCreateMutex();

    xTaskCreate(microros_connect_task, "uROS Connect", 2048, NULL, 1, &connectTaskHandle);
    xTaskCreate(microros_pulse_task, "uROS Pulse", 2048, NULL, 1, NULL);
    xTaskCreate(microros_executor_task, "uROS Executor", 2048, NULL, 1, NULL);
    xTaskCreate(application_task, "Application", 2048, NULL, 1, NULL);

    vTaskStartScheduler();
    for (;;) {}
}

// Waits for the agent, brings the system up, then sleeps until the pulse task reports the agent is gone, tears down, and starts over.
static void microros_connect_task(void *params) {
    (void)params;

    while (true) {
        // connect micro-ROS to the pico serial port
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

        // wait for the agent to respond
        rmw_uros_ping_agent(1000, 120);

        // COME ALIVE THESEUS
        Systems system = Systems(THESEUS);
        system.initialize_microros();
        system.initialize_hal();

        activeSystem = &system;
        sharedReady = true;

        // sleep until microros_pulse_task says the agent is lost
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        xSemaphoreTake(microrosMutex, portMAX_DELAY);
        sharedReady = false;
        activeSystem = NULL;
        system.cleanup();
        xSemaphoreGive(microrosMutex);
    }
}

// Checks the pulse of the agent every 10ms; notifies the connect task when it dies.
static void microros_pulse_task(void *params) {
    (void)params;

    for (;;) {
        if (sharedReady) {
            xSemaphoreTake(microrosMutex, portMAX_DELAY);
            bool alive = !sharedReady || rmw_uros_ping_agent(1000, 5) == RMW_RET_OK;
            xSemaphoreGive(microrosMutex);

            if (!alive) {
                xTaskNotifyGive(connectTaskHandle);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Spins the executor every 10ms.
static void microros_executor_task(void *params) {
    (void)params;

    for (;;) {
        if (sharedReady) {
            xSemaphoreTake(microrosMutex, portMAX_DELAY);
            if (sharedReady) { activeSystem->check_microros(); }
            xSemaphoreGive(microrosMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Application loop, runs alongside micro-ROS as the old node_task did.
static void application_task(void *params) {
    (void)params;

    for (;;) {
        while (!sharedReady) { vTaskDelay(pdMS_TO_TICKS(10)); }

        Systems::application_loop_step();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// TODO: move magnetometer out of application_loop_step into its own task / ISR
