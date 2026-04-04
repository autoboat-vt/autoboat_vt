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

    gpio_put(SPI_MUX_S0,1);
    gpio_put(SPI_MUX_S1,1);
    gpio_put(SPI_MUX_S2,1);

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

u_int16_t HAL::debug(drv8711* rudderStepperMotorDriver){
    u_int16_t Control_REG_output = drv8711_readReg(rudderStepperMotorDriver, CTRL_REG_ADDRESS);
    // u_int16_t TORQUE_REG_output = drv8711_readReg(rudderStepperMotorDriver, TORQUE_REG_ADDRESS);
    // u_int16_t OFF_REG_output = drv8711_readReg(rudderStepperMotorDriver, OFF_REG_ADDRESS);
    // u_int16_t BLANK_REG_output = drv8711_readReg(rudderStepperMotorDriver, BLANK_REG_ADDRESS);
    // u_int16_t DECAY_REG_output = drv8711_readReg(rudderStepperMotorDriver, DECAY_REG_ADDRESS);
    // u_int16_t STALL_REG_output = drv8711_readReg(rudderStepperMotorDriver, STALL_REG_ADDRESS);
    // u_int16_t DRIVE_REG_output = drv8711_readReg(rudderStepperMotorDriver, DRIVE_REG_ADDRESS);
    // u_int16_t STATUS_REG_output = drv8711_readReg(rudderStepperMotorDriver, STATUS_REG_ADDRESS);

    return Control_REG_output;
}
