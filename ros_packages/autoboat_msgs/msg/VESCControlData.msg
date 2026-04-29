string control_type_for_vesc
float32 desired_vesc_current
float32 desired_vesc_rpm
float32 desired_vesc_duty_cycle
#only one of the three values (current, rpm, or duty cycle) should be read by the topic based on the control type.