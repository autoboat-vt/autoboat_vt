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

#include <xcrsf/utils.h>

namespace crossfire {
    int16_t get_big_endian_int16(const int16_t value) {
        const auto result = value >> 8 | value << 8;
        return static_cast<int16_t>(result);
    }

    uint16_t get_big_endian_uint16(const uint16_t value) {
        return value >> 8 | value << 8;
    }

    int32_t get_big_endian_int32(const int32_t value) {
        const auto result = (value & 0xFF000000) >> 24 | (value & 0x00FF0000) >> 8 | (value & 0x0000FF00) << 8 | (value & 0x000000FF) << 24;
        return static_cast<int32_t>(result);
    }

    uint32_t get_big_endian_uint32(const uint32_t value) {
        return (value & 0xFF000000) >> 24 | (value & 0x00FF0000) >> 8 | (value & 0x0000FF00) << 8 | (value & 0x000000FF) << 24;
    }

    uint16_t get_channel_value(const uint8_t* data, const int index) {
        const int bit_position = index * 11, byte_index = 3 + bit_position / 8, bit_offset = bit_position % 8;
        uint16_t value = data[byte_index] >> bit_offset | data[byte_index + 1] << (8 - bit_offset);
        if (bit_offset > 5) { value |= data[byte_index + 2] << (16 - bit_offset); } return value & 0x07FF;
    }

    double get_attitude(const double value) {
        return value >= 0 ? value / 3.27 * 32767 : (value + 3.27) / 3.27 * 32767 + 32768;
    }
} // namespace crossfire