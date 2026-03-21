#include "systems.hpp"

Systems::Systems(boat_type bt){
    Boat::Boat* boat = Boat::intialize(bt)

}


void Systems::application_loop() {

    // -----------------------------------------------------
    // RUDDER ENCODER READ
    // -----------------------------------------------------

    float current_rudder_motor_angle = 
        get_motor_angle(&rudderEncoder) + RUDDER_ANGLE_OFFSET;

    // Normalize to [-180, 180)
    if (current_rudder_motor_angle >= 180) {
        current_rudder_motor_angle -= 360;
    }

    float current_rudder_angle = 0;
        // get_rudder_angle_from_motor_angle(current_rudder_motor_angle);

    // -----------------------------------------------------
    // PUBLISH RUDDER DATA
    // -----------------------------------------------------

    current_rudder_motor_angle_msg.data = current_rudder_motor_angle;
    current_rudder_angle_msg.data = current_rudder_angle;

    rcl_publish(&current_rudder_motor_angle_publisher,
                &current_rudder_motor_angle_msg,
                NULL);

    rcl_publish(&current_rudder_angle_publisher,
                &current_rudder_angle_msg,
                NULL);
}