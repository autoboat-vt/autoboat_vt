#include "systems.hpp"

static drv8711 rudderStepperMotorDriver;
static drv8711 winchStepperMotorDriver;

Systems::Systems(boat_type bt)
{
  current_boat = bt;
  initalize_cores();  // assumption is application loop will be called
}

void Systems::initalize_cores()
{
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_executor_init(&executor, &support.context, 5, &allocator);
}

void Systems::initialize_microros()
{
  microros_node.set_cores(&allocator, &support, &executor);
  // change this to rely on boat system
  microros_node.initialize_theseus_peripherals();
  Systems::initialize_debug_readers();
}

void Systems::initialize_debug_readers()
{
  microros_node.initialize_debug();
}

void Systems::check_microros()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

void Systems::initialize_hal()
{
  // write boat specific intialization later
  HAL::init_i2c();
  HAL::init_spi();
  gpio_init(RUDDER_MOTOR_CS_PIN);
  gpio_set_dir(RUDDER_MOTOR_CS_PIN, GPIO_OUT);
  gpio_pull_down(RUDDER_MOTOR_CS_PIN);
  HAL::init_rudder_stepper(&rudderStepperMotorDriver);

  gpio_init(RELAY_PIN);
  gpio_set_dir(RELAY_PIN, GPIO_OUT);
  gpio_pull_up(RELAY_PIN);

}

void Systems::cleanup()
{
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
}

void Systems::initialize_application_loop()
{
  rclc_timer_init_default(&application_loop_timer, &support, RCL_MS_TO_NS(1), application_loop);
  rclc_executor_add_timer(&executor, &application_loop_timer);
}

void Systems::application_loop(rcl_timer_t* timer, int64_t last_call_time)
{
  (void)timer;
  (void)last_call_time;



  // -----------------------------------------------------
  // RUDDER CLOSED LOOP CONTROL
  // -----------------------------------------------------
  float current_rudder_motor_angle = rudderEncoder.get_motor_angle() + RUDDER_ANGLE_OFFSET;
  if (current_rudder_motor_angle >= 180.0f)
    current_rudder_motor_angle -= 360.0f;

  float current_rudder_angle = get_rudder_angle_from_motor_angle(current_rudder_motor_angle);
  float rudder_error = (current_rudder::desired_angle - current_rudder_angle);

  int number_of_steps_rudder = 0;
  bool rudder_step_enabled = false;

  if (fabsf(rudder_error) > ACCEPTABLE_RUDDER_ERROR)
  {
    rudder_step_enabled = true;

    // Set direction
    if (rudder_error > 0) {
      drv8711_setDirection(&rudderStepperMotorDriver, CLOCKWISE);
    }
    else {
      drv8711_setDirection(&rudderStepperMotorDriver, COUNTER_CLOCKWISE);
    }

    number_of_steps_rudder = (int)(abs(rudder_error) * RUDDER_GAIN / MAX_RUDDER_ERROR + RUDDER_GAIN_Q * pow(abs(rudder_error), 2));
    if (number_of_steps_rudder > RUDDER_NUMBER_OF_STEPS_TO_CLIP_AT) {
      number_of_steps_rudder = RUDDER_NUMBER_OF_STEPS_TO_CLIP_AT;
    }
  }

  // -----------------------------------------------------
  // EXECUTE STEPPING (only rudder, no winch)
  // -----------------------------------------------------
  for (int i = 0; i < number_of_steps_rudder; i++)
  {
    gpio_put(LED_PIN, 1);
    if (rudder_step_enabled) {
      drv8711_step(&rudderStepperMotorDriver);
    }

    sleep_us(1000);
  }

  // -----------------------------------------------------
  // PUBLISH FEEDBACK
  // -----------------------------------------------------
  current_rudder::current_rudder_motor_angle_msg.data = current_rudder_motor_angle;
  rcl_publish(&current_rudder::current_rudder_motor_angle_publisher, &current_rudder::current_rudder_motor_angle_msg,
              NULL);

  current_rudder::current_angle_msg.data = current_rudder_angle;
  rcl_publish(&current_rudder::current_rudder_angle_publisher, &current_rudder::current_angle_msg, NULL);

  current_heading::heading_msg.data = fmod((180-(compass.getBearing()/10.0+MAGNETIC_DECLINATION))+360,360);
  rcl_publish(&current_heading::compass_angle_publisher, &current_heading::heading_msg, NULL);
}