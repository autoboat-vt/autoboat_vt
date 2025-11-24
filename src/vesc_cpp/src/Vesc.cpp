#include "vescuart/Vesc.hpp"

#include <boost/crc.hpp>
#include <iostream>

namespace vescuart
{

Vesc::Vesc(std::string port, int baudrate)
    : m_port(port), m_baudrate(baudrate), m_io(), m_serial(m_io)
{
    // try opening serial port
    connect();

    listenUart();

    boost::thread t(boost::bind(&boost::asio::io_service::run, &m_io));
    m_ioThread.swap(t);
}

Vesc::~Vesc()
{
    m_io.stop();
}

bool Vesc::isConnected() const
{
    return m_serial.is_open();
}

bool Vesc::connect()
{
    if (m_serial.is_open())
    {
        return true;
    }

    try
    {
        boost::system::error_code ec;
        m_serial.open(m_port, ec);
        m_serial.set_option(boost::asio::serial_port_base::baud_rate(m_baudrate));

        return !ec;
    }
    catch (const boost::system::system_error& exception)
    {
    }

    return false;
}

bool Vesc::sendPacket(const VescMessage& message)
{
    // check if serial port is open
    if (!m_serial.is_open())
    {
        if (!connect())
        {
            return false;
        }
    }

    // finish buuilding message
    VescMessage finalMessage = message.finalize();

    // send message
    try
    {
        boost::asio::write(m_serial, boost::asio::buffer(finalMessage.data(), finalMessage.size()));
    }
    catch (const boost::system::system_error& exception)
    {
        m_serial.close();
        return false;
    }

    return true;
}

void Vesc::uartCallback(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if (!error && bytes_transferred > 0)
    {
        std::cout << bytes_transferred << std::endl;
    }
    listenUart();
}

void Vesc::listenUart()
{
    m_serial.async_read_some(boost::asio::buffer(m_receiveBuffer, 16),
                             boost::bind(&Vesc::uartCallback,
                                         this,
                                         boost::asio::placeholders::error,
                                         boost::asio::placeholders::bytes_transferred));
}

} // namespace vescuart
