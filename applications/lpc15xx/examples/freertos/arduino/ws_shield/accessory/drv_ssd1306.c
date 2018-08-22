/**
 * \file
 *
 * \brief SSD1306 display controller driver.
 *
 * Copyright (c) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <stdarg.h>
#include <FreeRTOS.h>
#include <bsp.h>
#include <board.h>
#include <drv_ssd1306.h>
#include <ssd1306_font.h>
#include <app.h>
#if (APP_CFG_GUI_EN == DEF_ENABLED)
#include <ugui.h>
#endif

/**
 * \internal
 * \brief Initialize the hardware interface
 *
 * Depending on what interface used for interfacing the OLED controller this
 * function will initialize the necessary hardware.
 */
static void ssd1306_interface_init(void)
{
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, SSD1306_RES_PIN_CH, SSD1306_RES_PIN_POS);
    Chip_GPIO_SetPinState(LPC_GPIO, SSD1306_RES_PIN_CH, SSD1306_RES_PIN_POS, true);

    Chip_GPIO_SetPinDIROutput(LPC_GPIO, SSD1306_DC_PIN_CH, SSD1306_DC_PIN_POS);
    Chip_GPIO_SetPinState(LPC_GPIO, SSD1306_DC_PIN_CH, SSD1306_DC_PIN_POS, false);
}

/**
 * \brief Initialize the OLED controller
 *
 * Call this function to initialize the hardware interface and the OLED
 * controller. When initialization is done the display is turned on and ready
 * to receive data.
 */
void ssd1306_init(void)
{
	// Initialize the interface
	ssd1306_interface_init();

	// Do a hard reset of the OLED display controller
	ssd1306_hard_reset();

    #if 0
	// 1/32 Duty (0x0F~0x3F)
	ssd1306_write_command(SSD1306_CMD_SET_MULTIPLEX_RATIO);
	ssd1306_write_command(0x3F);

	// Shift Mapping RAM Counter (0x00~0x3F)
	ssd1306_write_command(SSD1306_CMD_SET_DISPLAY_OFFSET);
	ssd1306_write_command(0x00);

	// Set Mapping RAM Display Start Line (0x00~0x3F)
	ssd1306_write_command(SSD1306_CMD_SET_START_LINE(0x00));

	// Set Column Address 0 Mapped to SEG0
	ssd1306_write_command(SSD1306_CMD_SET_SEGMENT_RE_MAP_COL127_SEG0);

	// Set COM/Row Scan Scan from COM63 to 0
	ssd1306_write_command(SSD1306_CMD_SET_COM_OUTPUT_SCAN_DOWN);

	// Set COM Pins hardware configuration
	ssd1306_write_command(SSD1306_CMD_SET_COM_PINS);
	ssd1306_write_command(0x12);

	ssd1306_set_contrast(0x8F);

	// Disable Entire display On
	ssd1306_write_command(SSD1306_CMD_ENTIRE_DISPLAY_AND_GDDRAM_ON);

	ssd1306_display_invert_disable();

	// Set Display Clock Divide Ratio / Oscillator Frequency (Default => 0x80)
	ssd1306_write_command(SSD1306_CMD_SET_DISPLAY_CLOCK_DIVIDE_RATIO);
	ssd1306_write_command(0x80);

	// Enable charge pump regulator
	ssd1306_write_command(SSD1306_CMD_SET_CHARGE_PUMP_SETTING);
	ssd1306_write_command(0x14);

	// Set VCOMH Deselect Level
	ssd1306_write_command(SSD1306_CMD_SET_VCOMH_DESELECT_LEVEL);
	ssd1306_write_command(0x40); // Default => 0x20 (0.77*VCC)

	// Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	ssd1306_write_command(SSD1306_CMD_SET_PRE_CHARGE_PERIOD);
	ssd1306_write_command(0xF1);

	ssd1306_display_on();
    ssd1306_clear();
    #else
#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF

#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA

#define SSD1306_SETVCOMDETECT 0xDB

#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9

#define SSD1306_SETMULTIPLEX 0xA8

#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10

#define SSD1306_SETSTARTLINE 0x40

#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22

#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8

#define SSD1306_SEGREMAP 0xA0

#define SSD1306_CHARGEPUMP 0x8D

#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2
#define SSD1306_LCDWIDTH                  128

    // Init sequence for 128x64 OLED module
    ssd1306_write_command(SSD1306_DISPLAYOFF);                    // 0xAE
    ssd1306_write_command(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    ssd1306_write_command(0x80);                                  // the suggested ratio 0x80
    ssd1306_write_command(SSD1306_SETMULTIPLEX);                  // 0xA8
    ssd1306_write_command(0x3F);
    ssd1306_write_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
    ssd1306_write_command(0x0);                                   // no offset
    ssd1306_write_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
    ssd1306_write_command(SSD1306_CHARGEPUMP);                    // 0x8D
    //if (vccstate == SSD1306_EXTERNALVCC) 
      //{ ssd1306_write_command(0x10); }
    //else 
      { ssd1306_write_command(0x14); }
    ssd1306_write_command(SSD1306_MEMORYMODE);                    // 0x20
    ssd1306_write_command(0x00);                                  // 0x0 act like ks0108
    ssd1306_write_command(SSD1306_SEGREMAP | 0x1);
    ssd1306_write_command(SSD1306_COMSCANDEC);
    ssd1306_write_command(SSD1306_SETCOMPINS);                    // 0xDA
    ssd1306_write_command(0x12);
    ssd1306_write_command(SSD1306_SETCONTRAST);                   // 0x81
    //if (vccstate == SSD1306_EXTERNALVCC) 
      //{ ssd1306_write_command(0x9F); }
    //else 
      { ssd1306_write_command(0xCF); }
    ssd1306_write_command(SSD1306_SETPRECHARGE);                  // 0xd9
    //if (vccstate == SSD1306_EXTERNALVCC) 
      //{ ssd1306_write_command(0x22); }
    //else 
      { ssd1306_write_command(0xF1); }
    ssd1306_write_command(SSD1306_SETVCOMDETECT);                 // 0xDB
    ssd1306_write_command(0x40);
    ssd1306_write_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    ssd1306_write_command(SSD1306_NORMALDISPLAY);                 // 0xA6
    ssd1306_write_command(SSD1306_DISPLAYON);//--turn on oled panel
    ssd1306_clear();

    ssd1306_write_command(SSD1306_COLUMNADDR);
    ssd1306_write_command(0);   // Column start address (0 = reset)
    ssd1306_write_command(SSD1306_LCDWIDTH-1); // Column end address (127 = reset)
    
    ssd1306_write_command(SSD1306_PAGEADDR);
    ssd1306_write_command(0); // Page start address (0 = reset)
    ssd1306_write_command(7); // Page end address

    #endif
}

/**
 * \brief Display text on OLED screen.
 * \param string String to display.
 */
void ssd1306_write_text(const char *string)
{
	uint8_t *char_ptr;
	uint8_t i;

	while (*string != 0) {
		if (*string < 0x7F) {
			char_ptr = font_table[*string - 32];
			for (i = 1; i <= char_ptr[0]; i++) {
				ssd1306_write_data(char_ptr[i]);
			}
			ssd1306_write_data(0x00);
		}
			string++;
	}
}

#if (APP_CFG_GUI_EN == DEF_ENABLED)
uint8_t shadow_buff[SHADOW_WIDTH_PIXELS];
uint8_t display_buff[(LCD_HEIGHT_PIXELS / 8)][LCD_WIDTH_PIXELS];

void BSP_DrawBMP_BPP_1( UG_S16 xp, UG_S16 yp, UG_BMP* bmp );

void ssd1306_shadow_pset(UG_S16 x, UG_S16 y, UG_COLOR c)
{
    uint32_t ipos;
    uint8_t  b;

    b = 1 << (y & 0x7);

    if (!c){
        shadow_buff[x] |= b;
    } else {
        shadow_buff[x] &= ~b;
    }
}

void ssd1306_shadow_update(void)
{
    uint32_t cp_clm;
    static uint32_t cp_clm_s = 0;

    for (cp_clm = 0; cp_clm < LCD_WIDTH_PIXELS; cp_clm++) {
        display_buff[3][cp_clm] = shadow_buff[cp_clm_s + cp_clm];
    }
    cp_clm_s++;
    if (cp_clm_s >= LCD_WIDTH_PIXELS) {
        cp_clm_s = 0;
    }
}

void ssd1306_pset(UG_S16 x, UG_S16 y, UG_COLOR c)
{
    uint32_t ipos;
    uint8_t  b;

    ipos = y >> 3;
    b = 1 << (y & 0x7);

    if (!c){
        display_buff[ipos][x] |= b;
    } else {
        display_buff[ipos][x] &= ~b;
    }
}

void ssd1306_update(void)
{
    uint32_t pg;
    uint32_t clm;

    for (pg = 0; pg < (LCD_HEIGHT_PIXELS / 8); pg++) {
        ssd1306_set_page_address(pg);
        ssd1306_set_column_address(0);

        for (clm = 0; clm < LCD_WIDTH_PIXELS; clm++) {
            ssd1306_write_data(display_buff[pg][clm]);
        }
    }
}

UG_GUI ssd1306_ugui_shadow;
UG_GUI ssd1306_ugui;

void  BSP_SSD1306_Printf (int16_t x, int16_t y, char  *format, ...)
{
    char  buf_str[32];
    va_list   v_args;


    va_start(v_args, format);
    (void)vsnprintf((char       *)&buf_str[0],
                    (size_t      )sizeof(buf_str),
                    (char const *)format,
                    v_args);
    va_end(v_args);

    UG_PutString(x, y, buf_str);
}

void BSP_SSD1306_uGUI_Init(void)
{
    UG_Init(&ssd1306_ugui_shadow, ssd1306_shadow_pset, SHADOW_WIDTH_PIXELS, SHADOW_HEIGHT_PIXELS);
    UG_FillScreen(0);
    UG_FontSelect(&FONT_6X8);
    BSP_SSD1306_Printf(0, 256, "http://microchip.eefocus.com");
    UG_Init(&ssd1306_ugui, ssd1306_pset, LCD_WIDTH_PIXELS, LCD_HEIGHT_PIXELS);
}

void BSP_DrawBMP_BPP_1( UG_S16 xp, UG_S16 yp, UG_BMP* bmp )
{
    UG_S16 x, y, xs;
    UG_U8 bits;
    UG_U8 *p;
    UG_U8 tmp;
    UG_COLOR c;

    if (bmp->p == NULL) {
        return;
    }

    /* Only support 1 BPP */
    if (bmp->bpp == BMP_BPP_1) {
        p = (UG_U8*)bmp->p;
    } else {
        return;
    }

    xs = xp;
    for (y = 0; y < bmp->height; y++) {
        xp = xs;
        for (x = 0; x < bmp->width; x++) {
            tmp = *p++;
            for (bits = 0; bits < 8; bits++) {
                c = (UG_COLOR)(tmp & 0x80);
                UG_DrawPixel(xp++, yp, c);
                tmp <<= 1;
            }
        }
        yp++;
    }
}

#else

void  BSP_SSD1306_Printf (char  *format, ...)
{
    char  buf_str[32];
    va_list   v_args;


    va_start(v_args, format);
    (void)vsnprintf((char       *)&buf_str[0],
                    (size_t      )sizeof(buf_str),
                    (char const *)format,
                    v_args);
    va_end(v_args);

    ssd1306_write_text(buf_str);
}

#endif  /* (APP_CFG_GUI_EN == DEF_ENABLED) */

