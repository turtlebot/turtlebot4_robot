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

#ifndef TURTLEBOT4_BASE__SSD1306_HPP_
#define TURTLEBOT4_BASE__SSD1306_HPP_

#include <unistd.h>
#include <memory>
#include <string>

#include "turtlebot4_base/i2c_interface.hpp"
#include "turtlebot4_base/ssd1306_fonts.hpp"

namespace turtlebot4_base
{

// SSD1306 OLED height in pixels
#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT          64
#endif

// SSD1306 width in pixels
#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH           128
#endif

#ifndef SSD1306_BUFFER_SIZE
#define SSD1306_BUFFER_SIZE   SSD1306_WIDTH * SSD1306_HEIGHT / 8
#endif

#ifdef SSD1306_INCLUDE_FONT_7x10
#define SSD1306_FONT Font_7x10
#define SSD1306_FONT_HEIGHT Font_7x10.FontHeight
#define SSD1306_FONT_WIDTH Font_7x10.FontWidth
#endif

#ifdef SSD1306_INCLUDE_FONT_6x8
#define SSD1306_HEADER_FONT Font_6x8
#define SSD1306_HEADER_FONT_HEIGHT Font_6x8.FontHeight
#define SSD1306_HEADER_FONT_WIDTH Font_6x8.FontWidth
#endif

#define SSD1306_NUM_LINES ((SSD1306_HEIGHT - SSD1306_HEADER_FONT_HEIGHT - 2) / SSD1306_FONT_HEIGHT)
#define SSD1306_CHAR_PER_LINE (SSD1306_WIDTH / SSD1306_FONT_WIDTH)
#define SSD1306_CHAR_PER_LINE_HEADER (SSD1306_WIDTH / SSD1306_HEADER_FONT_WIDTH)

/** SSD1306 LCD driver commands */
enum ESsd1306Commands
{
  SSD1306_SETLOWCOLUMN     = 0x00,
  SSD1306_SETHIGHCOLUMN    = 0x10,
  SSD1306_MEMORYMODE       = 0x20,
  SSD1306_COLUMNADDR       = 0x21,
  SSD1306_PAGEADDR         = 0x22,
  SSD1306_SETSTARTLINE     = 0x40,
  SSD1306_DEFAULT_ADDRESS  = 0x78,
  SSD1306_SETCONTRAST      = 0x81,
  SSD1306_CHARGEPUMP       = 0x8D,
  SSD1306_SEGREMAP         = 0xA0,
  SSD1306_DISPLAYALLON_RESUME = 0xA4,
  SSD1306_DISPLAYALLON     = 0xA5,
  SSD1306_NORMALDISPLAY    = 0xA6,
  SSD1306_INVERTDISPLAY    = 0xA7,
  SSD1306_SETMULTIPLEX     = 0xA8,
  SSD1306_DISPLAYOFF       = 0xAE,
  SSD1306_DISPLAYON        = 0xAF,
  SSD1306_SETPAGE          = 0xB0,
  SSD1306_COMSCANINC       = 0xC0,
  SSD1306_COMSCANDEC       = 0xC8,
  SSD1306_SETDISPLAYOFFSET = 0xD3,
  SSD1306_SETDISPLAYCLOCKDIV = 0xD5,
  SSD1306_SETPRECHARGE     = 0xD9,
  SSD1306_SETCOMPINS       = 0xDA,
  SSD1306_SETVCOMDETECT    = 0xDB,
  SSD1306_SWITCHCAPVCC     = 0x02,
  SSD1306_NOP              = 0xE3,
};

/** SSD1306 supported memory modes. */
enum ESsd1306MemoryMode
{
  HORIZONTAL_ADDRESSING_MODE  = 0x00,
  VERTICAL_ADDRESSING_MODE    = 0x01,
  PAGE_ADDRESSING_MODE        = 0x02,
};

static const uint8_t ssd1306_128x32_initData[] =
{
  SSD1306_DISPLAYOFF,   // display off
  SSD1306_SETDISPLAYCLOCKDIV, 0x80,
  SSD1306_SETMULTIPLEX, 0x1F,
  SSD1306_SETDISPLAYOFFSET, 0x00,   // --no offset
  SSD1306_SETSTARTLINE | 0x00,
  SSD1306_CHARGEPUMP, 0x14,   // 0x10
  SSD1306_SEGREMAP | 0x01,    // Reverse mapping
  SSD1306_COMSCANDEC,
  SSD1306_SETCOMPINS, 0x02,
  SSD1306_SETCONTRAST, 0x7F,   // contrast value
  SSD1306_SETPRECHARGE, 0x22,   // 0x1F
  SSD1306_SETVCOMDETECT, 0x40,
  SSD1306_MEMORYMODE, HORIZONTAL_ADDRESSING_MODE,
  SSD1306_DISPLAYALLON_RESUME,
  SSD1306_NORMALDISPLAY,
  SSD1306_DISPLAYON,
};

static const uint8_t ssd1306_128x64_initData[] =
{
  SSD1306_DISPLAYOFF,   // display off
  SSD1306_SETDISPLAYCLOCKDIV, 0x80,
  SSD1306_SETMULTIPLEX, 0x3F,
  SSD1306_SETDISPLAYOFFSET, 0x00,   // --no offset
  SSD1306_SETSTARTLINE | 0x00,
  SSD1306_CHARGEPUMP, 0x10,   // 0x10
  SSD1306_SEGREMAP | 0x01,    // Reverse mapping
  SSD1306_COMSCANDEC,
  SSD1306_SETCOMPINS, 0x12,
  SSD1306_SETCONTRAST, 0x01,   // contrast value
  SSD1306_SETPRECHARGE, 0x22,   // 0x1F
  SSD1306_SETVCOMDETECT, 0x40,
  SSD1306_MEMORYMODE, HORIZONTAL_ADDRESSING_MODE,
  SSD1306_DISPLAYALLON_RESUME,
  SSD1306_NORMALDISPLAY,
  SSD1306_DISPLAYON,
};


// Enumeration for screen colors
typedef enum
{
  Black = 0x00,   // Black color, no pixel
  White = 0x01    // Pixel is set. Color depends on OLED
} SSD1306_COLOR;

typedef enum
{
  SSD1306_OK = 0x00,
  SSD1306_ERR = 0x01    // Generic error.
} SSD1306_Error_t;

// Struct to store transformations
typedef struct
{
  uint16_t CurrentX;
  uint16_t CurrentY;
  uint8_t Inverted;
  uint8_t Initialized;
  uint8_t DisplayOn;
} SSD1306_t;
typedef struct
{
  uint8_t x;
  uint8_t y;
} SSD1306_VERTEX;

union SSD1306_Page_Buffer {
  struct
  {
    uint8_t control;
    uint8_t data[SSD1306_WIDTH];
  } page;
  uint8_t raw[SSD1306_WIDTH + 1];
};

union SSD1306_Command_Buffer {
  struct
  {
    uint8_t control;
    uint8_t byte;
  } command;
  uint8_t raw[2];
};

class Ssd1306
{
public:
  Ssd1306(std::shared_ptr<I2cInterface> i2c_interface, uint8_t device_id);

  void Init(void);
  void Fill(SSD1306_COLOR color);
  void UpdateScreen(void);
  void DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
  char WriteChar(char ch, FontDef Font, SSD1306_COLOR color);
  bool WriteString(std::string str, FontDef Font, SSD1306_COLOR color);
  void SetCursor(uint8_t x, uint8_t y);
  SSD1306_t GetCursor();
  void DrawBattery(uint8_t x1, uint8_t y1, SSD1306_COLOR color);
  void Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
  void DrawArc(
    uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep,
    SSD1306_COLOR color);
  void DrawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR color);
  void Polyline(const SSD1306_VERTEX * par_vertex, uint16_t par_size, SSD1306_COLOR color);
  void DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
  /**
   * @brief Sets the contrast of the display.
   * @param[in] value contrast to set.
   * @note Contrast increases as the value increases.
   * @note RESET = 7Fh.
   */
  void SetContrast(const uint8_t value);
  /**
   * @brief Set Display ON/OFF.
   * @param[in] on 0 for OFF, any for ON.
   */
  void SetDisplayOn(const uint8_t on);
  /**
   * @brief Reads DisplayOn state.
   * @return  0: OFF.
   *          1: ON.
   */
  uint8_t GetDisplayOn();

  // Low-level procedures
  void Reset(void);
  void WriteCommand(uint8_t byte);
  void WriteData(uint8_t * buffer, size_t buff_size);
  void WritePage(uint8_t page);
  SSD1306_Error_t FillBuffer(uint8_t * buf, uint32_t len);

  float DegToRad(float par_deg);
  uint16_t NormalizeTo0_360(uint16_t par_deg);

private:
  void HandleRet(int8_t ret);

  std::shared_ptr<I2cInterface> i2c_interface_;
  uint8_t device_id_;
  uint8_t error_count_;

  SSD1306_t SSD1306;
  uint8_t buffer_[SSD1306_BUFFER_SIZE];
};

}  // namespace turtlebot4_base

#endif  // TURTLEBOT4_BASE__SSD1306_HPP_
