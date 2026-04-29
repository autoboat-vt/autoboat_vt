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
#include <array>
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>
#include <functional>

#include "xcrsf/serial.h"
#include "xcrsf/definitions.h"

namespace crossfire {
    class Handler {
        /**
         * @brief The file descriptor.
         */
        int uart_fd_ = -1;

        /**
         * @brief Indicator of pairing status.
         */
        std::atomic<bool> is_paired_{};

        /**
         * @brief Instance of UARTSerial.
         */
        UARTSerial uart_serial_;

        /**
         * @brief Timestamp for timeout.
         */
        std::chrono::system_clock::time_point timeout_{};

        /**
         * @brief The parser thread.
         */
        std::thread thread_parser_;

        /**
         * @brief The crsf protocol data callback.
         */
        std::function<void(std::vector<uint8_t>&)> state_callback_;

        /**
         * @brief Parse CRSF Protocol message.
         */
        void receive_crsf();

    public:
        /**
         * @brief Create instance of Handler.
         *
         * @param uart_path The path from the serial connection.
         * @param baud_rate The baud rate, default is 420_000 for ExpressLRS Receiver.
         */
        explicit Handler(const std::string& uart_path, speed_t baud_rate = 420000);

        /**
         * @brief Destroy instance of Handler.
         */
        ~Handler();

        /**
         * @brief Open serial connection.
         *
         * @return True on success otherwise False.
         */
        [[nodiscard]] bool open_port();

        /**
        * @brief Close serial connection.
        *
        * @return True on success otherwise False.
        */
        bool close_port();

        /**
         * @brief The pairing status.
         *
         * @return True if paired otherwise False.
         */
        [[nodiscard]] bool is_paired() const;

        /**
         * @brief Set the callback to get crsf protocol data.
         *
         * @param completion The completion block.
         */
        void set_callback(std::function<void(std::vector<uint8_t>&)> completion);

        /**
         * @brief Send CRSF Protocol message.
         *
         * @param packet The packet type.
         * @param payload The payload data.
         * @return True on success otherwise False.
         */
        [[nodiscard]] bool send_crsf(uint8_t packet, const std::vector<uint8_t>& payload) const;
    };
} // namespace crossfire