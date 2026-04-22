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
#include <cstdint>

namespace crossfire {
    /**
     * @brief Swap the byte order an int16_t value.
     *
     * @param value The value to swap.
     * @return The swapped value.
     */
    int16_t get_big_endian_int16(int16_t value);

    /**
     * @brief Swap the byte order of an uint16_t value.
     *
     * @param value The value to swap.
     * @return The swapped value.
     */
    uint16_t get_big_endian_uint16(uint16_t value);

    /**
     * @brief Swap the byte order an int32_t value.
     *
     * @param value The value to swap.
     * @return The swapped value.
     */
    int32_t get_big_endian_int32(int32_t value);

    /**
     * @brief Swap the byte order an uint32_t value.
     *
     * @param value The value to swap.
     * @return The swapped value.
     */
    uint32_t get_big_endian_uint32(uint32_t value);

    /**
     * @brief Parse CRSF Channel information.
     *
     * @param data The payload data.
     * @param index The current index.
     * @return The parsed channel information.
     */
    uint16_t get_channel_value(const uint8_t* data, int index);

    /**
     * @brief Convert attitude value.
     *
     * @param value The current value.
     * @return The converted value.
     */
    double get_attitude(double value);
} // namespace crossfire