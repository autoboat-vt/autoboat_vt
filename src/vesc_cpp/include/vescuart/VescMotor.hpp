#ifndef VESCMOTOR_HPP
#define VESCMOTOR_HPP

#include "vescuart/Vesc.hpp"

namespace vescuart
{

class VescMotor
{
  public:
    /**
     * @param port serial port for uart communication
     * @param baudrate baudrate of uart communication
     */
    VescMotor(std::string port, int baudrate);

    /**
     * @param vesc vesc to be used for serial communication
     * @param canId can id of vesc which the motor is connected to
     */
    VescMotor(Vesc::Ptr vesc, std::optional<int> canId = std::nullopt);

    /**
     * @brief sets the rpm of the motor
     *
     * @param rpm rpm to be applied
     * @return true if message has been sent
     */
    bool setRpm(int rpm);

    // -100000 to 100000
    bool setDutyCycle(int dutyCycle);

    // milliamps
    bool setCurrent(int current);

    // milliamps whem braking
    bool setBrakeCurrent(int current);

    bool setPosition(float angle);

    bool requestValues();

  private:
    std::shared_ptr<Vesc> m_vesc;
    std::optional<int> m_canId;
};

} // namespace vescuart

#endif
