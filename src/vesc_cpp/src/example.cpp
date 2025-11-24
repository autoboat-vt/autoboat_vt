#include <iostream>
#include <unistd.h>
#include <vescuart/VescMotor.hpp>

int main(int argc, char** argv)
{
    vescuart::VescMessage message;
    message.add<int>(2);
//    message.add<float>(42.0);
    message.add<float, uint16_t>(13.37, 1000);
    message.addCRC();

    std::cout << message.checkCRC() << std::endl;
    std::cout << message.get<int>() << std::endl;
//    std::cout << message.get<float>() << std::endl;
    std::cout << message.get<float, uint16_t>(1000) << std::endl;


//    if (argc < 3)
//    {
//        std::cout << "please call this example with at least two parameters:" << std::endl;
//        std::cout << "./vescuart-example <device> <baudrate> [<canId>]" << std::endl;
//        std::cout << std::endl;
//        std::cout << "example:" << std::endl;
//        std::cout << "./vescuart-example /dev/ttyMyDevice 115200 1" << std::endl;
//
//        return 1;
//    }
//
//    // get parameters
//    std::string port = argv[1];
//    int baudrate     = std::stoi(argv[2]);
//    std::cout << "using vesc " << port << " with baudrate " << baudrate << std::endl;
//
//    std::optional<int> canId = std::nullopt;
//    if (argc == 4)
//    {
//        canId = std::stoi(argv[3]);
//        std::cout << "forwarding messages to can id " << *canId << std::endl;
//    }
//    std::cout << std::endl;
//
//    // vesc configuration
//    vescuart::Vesc::Ptr vesc = std::make_shared<vescuart::Vesc>(port, baudrate);
//    vescuart::VescMotor motor(vesc, canId);
//
//    // apply speeds
//    for (int i = 0; i < 5000; i += 20)
//    {
//        if (motor.setRpm(i))
//        {
//            std::cout << "setting rpm to " << i << " ...       " << '\r' << std::flush;
//        }
//        else
//        {
//            std::cout << "ERROR setting rpm to " << i << " !       " << '\r' << std::flush;
//        }
//        usleep(50000);
//    }
//
//    for (int i = 5000; i >= 0; i -= 20)
//    {
//        if (motor.setRpm(i))
//        {
//            std::cout << "setting rpm to " << i << " ...       " << '\r' << std::flush;
//        }
//        else
//        {
//            std::cout << "ERROR setting rpm to " << i << " !       " << '\r' << std::flush;
//        }
//        usleep(50000);
//    }

    return 0;
}
