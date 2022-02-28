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


#include "turtlebot4_base/ssd1306.hpp"

#include <math.h>

#include <cstring>
#include <string>
#include <memory>
#include <iostream>

using turtlebot4_base::Ssd1306;
using turtlebot4_base::SSD1306_t;
using turtlebot4_base::SSD1306_Error_t;

Ssd1306::Ssd1306(std::shared_ptr<I2cInterface> i2c_interface, uint8_t device_id)
: i2c_interface_(i2c_interface),
  device_id_(device_id)
{
}

void Ssd1306::Reset(void)
{
  i2c_interface_->close_bus();
  Init();
}

// Send a byte to the command register
void Ssd1306::WriteCommand(uint8_t byte)
{
  SSD1306_Command_Buffer buf;
  buf.command.control = 0x00;
  buf.command.byte = byte;
  auto ret = i2c_interface_->write_to_bus(buf.raw, sizeof(buf));
  HandleRet(ret);
}

// Send data
void Ssd1306::WriteData(uint8_t * buffer, size_t buff_size)
{
  SSD1306_Page_Buffer buf;
  buf.page.control = 0x40;
  memcpy(buf.page.data, buffer, buff_size);
  auto ret = i2c_interface_->write_to_bus(buf.raw, sizeof(buf));
  HandleRet(ret);
}

void Ssd1306::WritePage(uint8_t page)
{
  WriteCommand(SSD1306_SETPAGE + page);
  WriteCommand(SSD1306_SETLOWCOLUMN);
  WriteCommand(SSD1306_SETHIGHCOLUMN);
  WriteData(&buffer_[SSD1306_WIDTH * page], SSD1306_WIDTH);
}

void Ssd1306::HandleRet(int8_t ret)
{
  if (ret == 0) {
    error_count_ = 0;
  } else {
    error_count_++;
    if (error_count_ >= 3) {
      std::cerr << "I2C Bus Error. Resetting." << std::endl;
      Reset();
    }
  }
}

/* Fills the Screenbuffer with values from a given buffer of a fixed length */
SSD1306_Error_t Ssd1306::FillBuffer(uint8_t * buf, uint32_t len)
{
  SSD1306_Error_t ret = SSD1306_ERR;
  if (len <= SSD1306_BUFFER_SIZE) {
    memcpy(buffer_, buf, len);
    ret = SSD1306_OK;
  }
  return ret;
}

// Initialize the oled screen

void Ssd1306::Init(void)
{
  i2c_interface_->open_bus();
  i2c_interface_->set_device_id(device_id_);

  if (SSD1306_HEIGHT == 32) {
    for (size_t i = 0; i < sizeof(ssd1306_128x32_initData); i++) {
      WriteCommand(ssd1306_128x32_initData[i]);
    }
  } else if (SSD1306_HEIGHT == 64) {
    for (size_t i = 0; i < sizeof(ssd1306_128x64_initData); i++) {
      WriteCommand(ssd1306_128x64_initData[i]);
    }
  }
}

// Fill the whole screen with the given color
void Ssd1306::Fill(SSD1306_COLOR color)
{
  /* Set memory */
  memset(buffer_, (color == Black) ? 0x00 : 0xFF, sizeof(buffer_));
}

// Write the screenbuffer with changed to the screen
void Ssd1306::UpdateScreen(void)
{
  // Write data to each page of RAM. Number of pages
  // depends on the screen height:
  //
  //  * 32px   ==  4 pages
  //  * 64px   ==  8 pages
  //  * 128px  ==  16 pages
  for (uint8_t i = 0; i < SSD1306_HEIGHT / 8; i++) {
    WritePage(i);
  }
}

//    Draw one pixel in the screenbuffer
//    X => X Coordinate
//    Y => Y Coordinate
//    color => Pixel color
void Ssd1306::DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
  if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
    // Don't write outside the buffer
    return;
  }

  // Check if pixel should be inverted
  if (SSD1306.Inverted) {
    color = (SSD1306_COLOR) !color;
  }

  // Draw in the right color
  if (color == White) {
    buffer_[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
  } else {
    buffer_[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
  }
}

// Draw 1 char to the screen buffer
// ch       => char om weg te schrijven
// Font     => Font waarmee we gaan schrijven
// color    => Black or White
char Ssd1306::WriteChar(char ch, FontDef Font, SSD1306_COLOR color)
{
  uint32_t i, b, j;

  // Check if character is valid
  if (ch < 32 || ch > 126) {
    return 0;
  }

  // Check remaining space on current line
  if (SSD1306_WIDTH < (SSD1306.CurrentX + Font.FontWidth) ||
    SSD1306_HEIGHT < (SSD1306.CurrentY + Font.FontHeight))
  {
    // Not enough space on current line
    return 0;
  }

  // Use the font to write
  for (i = 0; i < Font.FontHeight; i++) {
    b = Font.data[(ch - 32) * Font.FontHeight + i];
    for (j = 0; j < Font.FontWidth; j++) {
      if ((b << j) & 0x8000) {
        DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)color);
      } else {
        DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR) !color);
      }
    }
  }

  // The current space is now taken
  SSD1306.CurrentX += Font.FontWidth;

  // Return written char for validation
  return ch;
}

// Write full string to screenbuffer
bool Ssd1306::WriteString(std::string str, FontDef Font, SSD1306_COLOR color)
{
  const char * c_str = str.c_str();
  // Write until null-byte
  while (*c_str) {
    if (WriteChar(*c_str, Font, color) != *c_str) {
      // Char could not be written
      return false;
    }

    // Next char
    c_str++;
  }

  // Everything ok
  return true;
}

// Position the cursor
void Ssd1306::SetCursor(uint8_t x, uint8_t y)
{
  SSD1306.CurrentX = x;
  SSD1306.CurrentY = y;
}

SSD1306_t Ssd1306::GetCursor()
{
  return SSD1306;
}

// Draw line by Bresenhem's algorithm
void Ssd1306::Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
  int32_t deltaX = abs(x2 - x1);
  int32_t deltaY = abs(y2 - y1);
  int32_t signX = ((x1 < x2) ? 1 : -1);
  int32_t signY = ((y1 < y2) ? 1 : -1);
  int32_t error = deltaX - deltaY;
  int32_t error2;

  DrawPixel(x2, y2, color);
  while ((x1 != x2) || (y1 != y2)) {
    DrawPixel(x1, y1, color);
    error2 = error * 2;
    if (error2 > -deltaY) {
      error -= deltaY;
      x1 += signX;
    } else {
      /*nothing to do*/
    }

    if (error2 < deltaX) {
      error += deltaX;
      y1 += signY;
    } else {
      /*nothing to do*/
    }
  }
}
// Draw polyline
void Ssd1306::Polyline(const SSD1306_VERTEX * par_vertex, uint16_t par_size, SSD1306_COLOR color)
{
  uint16_t i;
  if (par_vertex != 0) {
    for (i = 1; i < par_size; i++) {
      Line(par_vertex[i - 1].x, par_vertex[i - 1].y, par_vertex[i].x, par_vertex[i].y, color);
    }
  } else {
    /*nothing to do*/
  }
}
/*Convert Degrees to Radians*/
float Ssd1306::DegToRad(float par_deg)
{
  return par_deg * 3.14 / 180.0;
}
/*Normalize degree to [0;360]*/
uint16_t Ssd1306::NormalizeTo0_360(uint16_t par_deg)
{
  uint16_t loc_angle;
  if (par_deg <= 360) {
    loc_angle = par_deg;
  } else {
    loc_angle = par_deg % 360;
    loc_angle = ((par_deg != 0) ? par_deg : 360);
  }
  return loc_angle;
}
/*DrawArc. Draw angle is beginning from 4 quart of trigonometric circle (3pi/2)
 * start_angle in degree
 * sweep in degree
 */
void Ssd1306::DrawArc(
  uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep,
  SSD1306_COLOR color)
{
#define CIRCLE_APPROXIMATION_SEGMENTS 36
  float approx_degree;
  uint32_t approx_segments;
  uint8_t xp1, xp2;
  uint8_t yp1, yp2;
  uint32_t count = 0;
  uint32_t loc_sweep = 0;
  float rad;

  loc_sweep = NormalizeTo0_360(sweep);

  count = (NormalizeTo0_360(start_angle) * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
  approx_segments = (loc_sweep * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
  approx_degree = loc_sweep / static_cast<float>(approx_segments);
  while (count < approx_segments) {
    rad = DegToRad(count * approx_degree);
    xp1 = x + (int8_t)(sin(rad) * radius);
    yp1 = y + (int8_t)(cos(rad) * radius);
    count++;
    if (count != approx_segments) {
      rad = DegToRad(count * approx_degree);
    } else {
      rad = DegToRad(loc_sweep);
    }
    xp2 = x + (int8_t)(sin(rad) * radius);
    yp2 = y + (int8_t)(cos(rad) * radius);
    Line(xp1, yp1, xp2, yp2, color);
  }
}
// Draw circle by Bresenhem's algorithm
void Ssd1306::DrawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR par_color)
{
  int32_t x = -par_r;
  int32_t y = 0;
  int32_t err = 2 - 2 * par_r;
  int32_t e2;

  if (par_x >= SSD1306_WIDTH || par_y >= SSD1306_HEIGHT) {
    return;
  }

  do {
    DrawPixel(par_x - x, par_y + y, par_color);
    DrawPixel(par_x + x, par_y + y, par_color);
    DrawPixel(par_x + x, par_y - y, par_color);
    DrawPixel(par_x - x, par_y - y, par_color);
    e2 = err;
    if (e2 <= y) {
      y++;
      err = err + (y * 2 + 1);
      if (-x == y && e2 <= x) {
        e2 = 0;
      } else {
        /*nothing to do*/
      }
    } else {
      /*nothing to do*/
    }
    if (e2 > x) {
      x++;
      err = err + (x * 2 + 1);
    } else {
      /*nothing to do*/
    }
  } while (x <= 0);
}

// Draw rectangle
void Ssd1306::DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
  Line(x1, y1, x2, y1, color);
  Line(x2, y1, x2, y2, color);
  Line(x2, y2, x1, y2, color);
  Line(x1, y2, x1, y1, color);
}

// Draw battery
void Ssd1306::DrawBattery(uint8_t x1, uint8_t y1, SSD1306_COLOR color)
{
  Line(x1, y1 + 1, x1, y1 + 6, color);
  Line(x1 + 1, y1, x1 + 1, y1 + 6, color);
  Line(x1 + 2, y1, x1 + 2, y1 + 6, color);
  Line(x1 + 3, y1 + 1, x1 + 3, y1 + 6, color);
  SetCursor(x1 + 6, y1);
}

void Ssd1306::SetContrast(const uint8_t value)
{
  WriteCommand(SSD1306_SETCONTRAST);
  WriteCommand(value);
}

void Ssd1306::SetDisplayOn(const uint8_t on)
{
  uint8_t value;
  if (on) {
    value = 0xAF;  // Display on
    SSD1306.DisplayOn = 1;
  } else {
    value = 0xAE;  // Display off
    SSD1306.DisplayOn = 0;
  }
  WriteCommand(value);
}

uint8_t Ssd1306::GetDisplayOn()
{
  return SSD1306.DisplayOn;
}
