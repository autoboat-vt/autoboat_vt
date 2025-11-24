#include "vescuart/VescMotor.hpp"

#include <cstdint>

namespace vescuart
{

VescMotor::VescMotor(std::string port, int baudrate)
    : m_vesc(std::make_shared<Vesc>(port, baudrate)), m_canId(std::nullopt)
{
}

VescMotor::VescMotor(Vesc::Ptr vesc, std::optional<int> canId) : m_vesc(vesc), m_canId(canId) {}

bool VescMotor::setRpm(int rpm)
{
    VescMessage message(VescMessageType::setRpm, m_canId);
    message.add<int32_t>(static_cast<int32_t>(rpm));

    return m_vesc->sendPacket(message);
}

bool VescMotor::setDutyCycle(int dutyCycle)
{
    VescMessage message(VescMessageType::setDuty, m_canId);
    message.add<int32_t>(static_cast<int32_t>(dutyCycle));

    return m_vesc->sendPacket(message);
}

bool VescMotor::setCurrent(int current)
{
    VescMessage message(VescMessageType::setCurrent, m_canId);
    message.add<int32_t>(static_cast<int32_t>(current));

    return m_vesc->sendPacket(message);
}

bool VescMotor::setBrakeCurrent(int current)
{
    VescMessage message(VescMessageType::setCurrentBrake, m_canId);
    message.add<int32_t>(static_cast<int32_t>(current));

    return m_vesc->sendPacket(message);
}

bool VescMotor::setPosition(float angle)
{
    VescMessage message(VescMessageType::setPos, m_canId);
    message.add<float, int32_t>(angle, 1000000);

    return m_vesc->sendPacket(message);
}

bool VescMotor::requestValues() {
    VescMessage message(VescMessageType::getValues, m_canId);

    return m_vesc->sendPacket(message);
}
} // namespace vescuart
