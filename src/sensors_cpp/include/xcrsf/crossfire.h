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
#include <thread>
#include <mutex>

#include "xcrsf/handler.h"

namespace crossfire {
    class XCrossfire {
        /**
         * @brief The file descriptor.
         */
        Handler handler_;

        /**
         * @brief Mutex lock for channel data.
         */
        std::mutex crsf_lock_;

        /**
         * @brief The channel state.
         */
        std::array<uint16_t, 16> channel_state_{};

        /**
         * @brief The link state.
         */
        CRSFLink link_state_{};

        /**
         * @brief Update the channel and link information.
         *
         * @param crsf_cata The payload data.
         */
        void parse_message(const uint8_t* crsf_cata);

    public:
        /**
         * @brief Create instance of XCrossfire.
         *
         * @param uart_path The path from the serial connection.
         * @param baud_rate The baud rate, default is 420_000 for ExpressLRS Receiver.
         */
        explicit XCrossfire(const std::string& uart_path, speed_t baud_rate = 420000);

        /**
         * @brief Destroy instance of XCrossfire.
         */
        ~XCrossfire();

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
         * @brief Status if receiver is paired.
         *
         * @return True if paired otherwise False.
         */
        [[nodiscard]] bool is_paired() const;

        /**
         * @brief Get the link state information.
         *
         * @return The link statistic's.
         */
        CRSFLink get_link_state();

        /**
         * @brief Get the channel state information.
         *
         * @return The channel's.
         */
        std::array<uint16_t, 16> get_channel_state();

        /**
         * @brief Send vario telemetry data back to the transmitter.
         *
         * @param vertical_speed The speed in m/s.
         * @return True on success otherwise False.
         */
        [[nodiscard]] bool set_telemetry_vario(double vertical_speed) const;

        /**
         * @brief Send battery telemetry data back to the transmitter.
         *
         * @param voltage The Volts from the power source.
         * @param current The Amps from the power source.
         * @param capacity The Milliamps from the power source.
         * @param percent The Percent from the power source.
         * @return True on success otherwise False.
         */
        [[nodiscard]] bool set_telemetry_battery(double voltage, double current, uint32_t capacity, uint8_t percent) const;

        /**
         * @brief Send attitude telemetry data back to the transmitter.
         *
         * @param pitch in radiant's.
         * @param roll in radiant's.
         * @param yaw in radiant's.
         * @return True on success otherwise False.
         */
        [[nodiscard]] bool set_telemetry_attitude(double pitch, double roll, double yaw) const;

        /**
         * @brief Send gps telemetry data back to the transmitter.
         *
         * @param latitude The latitude.
         * @param longitude The longitude.
         * @param groundspeed The ground speed in km/h.
         * @param heading The heading in degree's.
         * @param altitude The altitude in meter.
         * @param satellites The satellite count.
         * @return True on success otherwise False.
         */
        [[nodiscard]] bool set_telemetry_gps(double latitude, double longitude, double groundspeed, double heading, double altitude, uint8_t satellites) const;
    };
} // namespace crossfire