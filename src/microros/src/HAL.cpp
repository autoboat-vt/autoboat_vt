#include "HAL.hpp"
  std_msgs__msg__Float64 desired_rudder_angle_msg;
  std_msgs__msg__Float64 current_rudder_angle_msg;
  std_msgs__msg__Float64 compass_angle_msg;
  
void HAL::init_spi(){
    spi_init(SPI_PORT, 500 * 1000);
    gpio_set_function(SCLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);

    
    // -----------------------------------------------------
    // INITIALIZE DEFAULT CALLBACK VARIABLE VALUES
    // -----------------------------------------------------
    desired_rudder_angle_msg.data = 0.0;
    current_rudder_angle_msg.data = 0.0;
    compass_angle_msg.data = 0.0;


    gpio_init(14);
    gpio_set_dir(14, GPIO_OUT);

    gpio_init(15);
    gpio_set_dir(15, GPIO_OUT);

    gpio_init(16);
    gpio_set_dir(16, GPIO_OUT);

    // gpio_pull_up(14);
    // gpio_pull_up(15);
    // gpio_pull_up(16);

                gpio_put(14,0);
            gpio_put(15,0);
            gpio_put(16,1);

}