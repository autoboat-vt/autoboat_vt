#ifndef VESCUART_HPP
#define VESCUART_HPP

#include "vescuart/VescMessage.hpp"

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <string>

namespace vescuart
{

class Vesc
{
  public:
    using Ptr = std::shared_ptr<Vesc>;

    /**
     * @param port serial port for communication
     * @param baudrate baud rate of uart communication
     */
    Vesc(std::string port, int baudrate);

    ~Vesc();

    /**
     * @brief checks if the serial port is connected
     *
     * @return true if connection is open, else false
     */
    bool isConnected() const;

    /**
     * @brief tries ot open the serial port
     *
     * @return true if the connnection is open, else false
     */
    bool connect();

    /**
     * @brief sends a message to the configured device
     *
     * @param message message to send
     *
     * @return true if successful, false if connection is not open
     */
    bool sendPacket(const VescMessage& message);

  private:
    std::string m_port;
    int m_baudrate;
    boost::asio::io_service m_io;
    boost::asio::serial_port m_serial;
    boost::thread m_ioThread;

    uint8_t m_receiveBuffer[16];

    /**
     * @brief gets called on incoming UART message
     *
     * @param error error of transmission
     * @param bytes_transferred amount of bytes transferred
     */
    void uartCallback(const boost::system::error_code& error, std::size_t bytes_transferred);

    /**
     * @brief starts listening to UART messages
     */
    void listenUart();
};

} // namespace vescuart

#endif
