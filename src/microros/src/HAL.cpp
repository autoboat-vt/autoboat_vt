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


    gpio_init(SPI_MUX_S0);
    gpio_set_dir(SPI_MUX_S0, GPIO_OUT);

    gpio_init(SPI_MUX_S1);
    gpio_set_dir(SPI_MUX_S1, GPIO_OUT);

    gpio_init(SPI_MUX_S2);
    gpio_set_dir(SPI_MUX_S2,GPIO_OUT);

    gpio_put(SPI_MUX_S0,0);
    gpio_put(SPI_MUX_S1,0);
    gpio_put(SPI_MUX_S2,0);

}

void HAL::init_i2c(){
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);



}


void HAL::init_rudder_stepper(drv8711* rudderStepperMotorDriver){
      drv8711_init(rudderStepperMotorDriver, SPI_PORT, RUDDER_MOTOR_CS_PIN, RUDDER_MOTOR_SLEEP_PIN, AutoMixed, RUDDER_MICROSTEP, MAX_RUDDER_CURRENT);

}
