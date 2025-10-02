 
class Application{
    public:
        static void init_microros_node();
        static void zero_rudder();
        static void init_propeller_sub();
        static void init_rudder_pub_sub();
        static void init_current_header_pub();
        static void init_application_loop_timer();
        static void init_SPI_and_I2C_busses();
        static void init_AMT22();
        static void init_drv8711();
        static void init_contactor_driver();
}

#endif