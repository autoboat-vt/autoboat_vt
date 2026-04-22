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

#include <cstring>
#include <utility>

#include "xcrsf/crc.h"
#include "xcrsf/handler.h"
#include "xcrsf/utils.h"

namespace crossfire {
    static constexpr auto STD_MEMORY_ORDER = std::memory_order::relaxed;
    static constexpr auto STD_READ_INTERRUPT = std::chrono::microseconds(100);
    static constexpr auto STD_TIMEOUT = std::chrono::milliseconds(250);

    Handler::Handler(const std::string &uart_path, const speed_t baud_rate): uart_serial_(uart_path, baud_rate) { }

    Handler::~Handler() {
        this->close_port();
    }

    bool Handler::open_port() {
        this->uart_fd_ = this->uart_serial_.open_port();
        if (this->uart_fd_ == -1) { return false; }

        this->is_paired_.store(true, STD_MEMORY_ORDER);
        if (this->thread_parser_.joinable()) { this->thread_parser_.join(); }
        this->thread_parser_ = std::thread{&Handler::receive_crsf, this};
        return true;
    }

    bool Handler::close_port() {
        this->is_paired_.store(false, STD_MEMORY_ORDER);
        if (this->thread_parser_.joinable()) { this->thread_parser_.join(); }
        return this->uart_serial_.close_port() == 0;
    }

    bool Handler::is_paired() const {
        return this->is_paired_.load(STD_MEMORY_ORDER);
    }

    void Handler::set_callback(std::function<void(std::vector<uint8_t>&)> completion) {
        this->state_callback_ = std::move(completion);
    }

    bool Handler::send_crsf(const uint8_t packet, const std::vector<uint8_t>& payload) const {
        if (!this->is_paired_.load(STD_MEMORY_ORDER) || payload.size() > CRSF_MAX_FRAME_SIZE - 4) { return false; }
        const auto length = payload.size(); uint8_t buffer[CRSF_MAX_FRAME_SIZE];
        buffer[0] = CRSF_SYNC_BYTE; buffer[1] = length + 2; buffer[2] = packet;

        std::memcpy(&buffer[3], payload.data(), length); buffer[length + 3] = CRCValidator::get_crc8(&buffer[2], length + 1);
        return write(this->uart_fd_, buffer, length + 4);
    }

    void Handler::receive_crsf() {
        std::vector<uint8_t> buffer(0, 0x0); uint8_t byte = 0x0; this->timeout_ = std::chrono::high_resolution_clock::now();
        while (this->is_paired_.load(STD_MEMORY_ORDER)) {
            const auto is_read = read(this->uart_fd_, &byte, 1);
            if (std::chrono::high_resolution_clock::now() > this->timeout_ + STD_TIMEOUT) { this->is_paired_.store(false, STD_MEMORY_ORDER); }
            if (!is_read) { std::this_thread::sleep_for(STD_READ_INTERRUPT); continue; }
            if (byte != CRSF_SYNC_BYTE && buffer.empty()) { continue; }

            if (buffer.size() < 3 || buffer.size() < buffer[1] + 2) {
                buffer.emplace_back(byte); const auto is_valid = CRCValidator::get_crc8(&buffer[2], buffer[1] - 1) == buffer[buffer[1] + 1];
                if (buffer.size() == buffer[1] + 2 && is_valid) { if (this->state_callback_) { this->state_callback_(buffer); } }
                if (buffer[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) { this->timeout_ = std::chrono::high_resolution_clock::now(); }
                if (buffer.size() >= buffer[1] + 2) { buffer.assign(0, 0x0); }
            }
        }
    }
} // namespace crossfire