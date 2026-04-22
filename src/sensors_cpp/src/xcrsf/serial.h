/*
 * MIT License
 *
 * Copyright (c) 2025 Vinzenz Weist
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once
#include <string>
#include <asm/termbits.h>

namespace crossfire {
    class UARTSerial {
        /**
         * @brief The file descriptor.
         */
        int uart_fd_ = -1;

        /**
         * @brief The uart path.
         */
        std::string uart_path_{};

        /**
         * @brief The baud rate.
         */
        speed_t baud_rate_ = 0;

        /**
         * @brief Check if port is active.
         *
         * @return True if active otherwise False.
         */
        [[nodiscard]] bool port_active() const;

        /**
         * @brief Reconfigure the port's baud rate.
         *
         * @param baud_rate The baud rate.
         * @return The status code.
         */
        [[nodiscard]] int reconfigure_port(speed_t baud_rate) const;

    public:
        /**
         * @brief Create instance of UARTSerial.
         *
         * @param uart_path The path from the serial connection.
         * @param baud_rate The baud rate.
         */
        UARTSerial(std::string uart_path, speed_t baud_rate);

        /**
         * @brief Destroy instance of UARTSerial.
         */
        ~UARTSerial();

        /**
         * @brief Open an uart (serial) connection.
         *
         * @return The file descriptor.
         */
        int open_port();

        /**
         * @brief Close the current uart (serial) connection.
         *
         * @return The status code.
         */
        [[nodiscard]] int close_port() const;
    };
} // namespace crossfire
