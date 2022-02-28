/*
 * MIT License
 *
 * Copyright (c) 2018 Aleksander Alekseev
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
 *
 * Original driver: https://github.com/afiskon/stm32-ssd1306
 *
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#ifndef TURTLEBOT4_BASE__SSD1306_FONTS_HPP_
#define TURTLEBOT4_BASE__SSD1306_FONTS_HPP_

#include <stdint.h>

namespace turtlebot4_base
{

#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_7x10

typedef struct
{
  const uint8_t FontWidth;          /*!< Font width in pixels */
  uint8_t FontHeight;         /*!< Font height in pixels */
  const uint16_t * data;      /*!< Pointer to data font data array */
} FontDef;

#ifdef SSD1306_INCLUDE_FONT_6x8
extern FontDef Font_6x8;
#endif

#ifdef SSD1306_INCLUDE_FONT_7x10
extern FontDef Font_7x10;
#endif

#ifdef SSD1306_INCLUDE_FONT_11x18
extern FontDef Font_11x18;
#endif

#ifdef SSD1306_INCLUDE_FONT_16x26
extern FontDef Font_16x26;
#endif

}  // namespace turtlebot4_base

#endif  // TURTLEBOT4_BASE__SSD1306_FONTS_HPP_
