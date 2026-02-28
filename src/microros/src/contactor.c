#include <pico/stdlib.h>
#include <stdio.h>

int main() {

    const uint RLY_PIN = 29; 

    stdio_init_all(); 

    gpio_init(RLY_PIN); 

    gpio_set_dir(RLY_PIN, GPIO_OUT); 

    gpio_put(RLY_PIN, 1); 
    sleep_ms(500); 

    // gpio_put(LED_PIN, 0); 
    // sleep_ms(500); 
}