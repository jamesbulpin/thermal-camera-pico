/*
 * Portions Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 * Portions Copyright (c) 2022 James Bulpin
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#include <stdlib.h>
#include <math.h>

#include "MLX90640_API.h"
#include "pico/stdlib.h"
#include "pico/st7789.h"
#include "ThermalCamera.h"
#include "font.h"

// lcd configuration
const struct st7789_config lcd_config = {
    .spi      = PICO_DEFAULT_SPI_INSTANCE,
    .gpio_din = PICO_DEFAULT_SPI_TX_PIN,
    .gpio_clk = PICO_DEFAULT_SPI_SCK_PIN,
    .gpio_cs  = PICO_DEFAULT_SPI_CSN_PIN,
    .gpio_dc  = 20,
    .gpio_rst = 21,
    .gpio_bl  = 22,
};

const int LCD_WIDTH = 240;
const int LCD_HEIGHT = 240;

const int CAMERA_WIDTH = 32;
const int CAMERA_HEIGHT = 24;

const int PIXEL_RATIO = 7;

const int SCALE_LEFT_OFFSET = 176;
const int SCALE_WIDTH = 10;

const int SIDEBAR_WIDTH = 48;
const int SIDEBAR_HEIGHT = 240;

// https://www.programmingalgorithms.com/algorithm/hsl-to-rgb/c/
float HueToRGB(float v1, float v2, float vH)
{
	if (vH < 0)
		vH += 1;

	if (vH > 1)
		vH -= 1;

	if ((6 * vH) < 1)
		return (v1 + (v2 - v1) * 6 * vH);

	if ((2 * vH) < 1)
		return v2;

	if ((3 * vH) < 2)
		return (v1 + (v2 - v1) * ((2.0f / 3) - vH) * 6);

	return v1;
}

uint16_t rgb(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t ret = 0;
    ret |= ((uint16_t)(r & 0xf8)) << 8;
    ret |= ((uint16_t)(g & 0xfc)) << 3;
    ret |= ((uint16_t)(b & 0xf8)) >> 3;
    return ret;
}

uint16_t HSLToRGB(uint8_t h, float s, float l) {
    float v1, v2;
    float hue = (float)h / 360;

    v2 = (l < 0.5) ? (l * (1 + s)) : ((l + s) - (l * s));
    v1 = 2 * l - v2;

    uint8_t r = (unsigned char)(255 * HueToRGB(v1, v2, hue + (1.0f / 3)));
    uint8_t g = (unsigned char)(255 * HueToRGB(v1, v2, hue));
    uint8_t b = (unsigned char)(255 * HueToRGB(v1, v2, hue - (1.0f / 3)));
    return rgb(r, g, b);
}

void renderTemperature(float t, uint16_t *framebuffer, int fbWidth, int x, int y, int size, uint16_t colour, int noSign) {
    int ti = (int)abs(t);
    int units = ti%10;
    int tens = ti/10;
    int tensChar = (tens == 0)?0x20:((tens > 9)?'x':(0x30 + tens));
    if (!noSign) {
        renderCharacter((t < 0)?0x2d:0x20, framebuffer, fbWidth, x, y, colour, 0, size);
    }
    renderCharacter(tensChar, framebuffer, fbWidth, x + (noSign?0:6) * size, y, colour, 0, size);
    renderCharacter(0x30 + units, framebuffer, fbWidth, x + (noSign?6:12) * size, y, colour, 0, size);
}

int main()
{
    // initialize the lcd
    st7789_init(&lcd_config, LCD_WIDTH, LCD_HEIGHT);

    // make screen black
    st7789_fill(0x0000);

    ThermalCamera camera;
    float frame[768];

    // Sidebar display; 
    uint16_t sidebar[SIDEBAR_WIDTH*SIDEBAR_HEIGHT];

    int leftOffset = LCD_WIDTH - CAMERA_HEIGHT * PIXEL_RATIO;
    int crossHairsY = CAMERA_WIDTH * PIXEL_RATIO / 2 - 1;
    int crossHairsX = CAMERA_HEIGHT * PIXEL_RATIO / 2 - 1;

    for (int y = 0; y < LCD_HEIGHT; y++) {
        st7789_set_cursor(LCD_WIDTH - SCALE_LEFT_OFFSET - SCALE_WIDTH, LCD_HEIGHT - y);
        uint16_t pixel = HSLToRGB(y, 1.0, 0.5);
        for (int dx = 0; dx < SCALE_WIDTH; dx++) {
            st7789_put(pixel);
        }
    }

    for (int y = 0; y < SIDEBAR_HEIGHT; y++) {
        for (int x = 0; x < SIDEBAR_WIDTH; x++) {
            sidebar[y*SIDEBAR_WIDTH + x] = 0;
        }
    }

    uint16_t sidebarColour = rgb(255, 255, 255);
    uint16_t sidebarColourHot = rgb(255, 0, 0);
    uint16_t sidebarColourCold = rgb(0, 0, 255);
    uint16_t sidebarColourLabel = rgb(128, 128, 128);

    renderCharacter('C', sidebar, SIDEBAR_WIDTH, 6, SIDEBAR_HEIGHT/2 - 32, sidebarColourLabel, 0, 2);
    renderCharacter('T', sidebar, SIDEBAR_WIDTH, 18, SIDEBAR_HEIGHT/2 - 32, sidebarColourLabel, 0, 2);
    renderCharacter('R', sidebar, SIDEBAR_WIDTH, 30, SIDEBAR_HEIGHT/2 - 32, sidebarColourLabel, 0, 2);

    renderCharacter('M', sidebar, SIDEBAR_WIDTH, 0, SIDEBAR_HEIGHT/2 + 32, sidebarColourLabel, 0, 2);
    renderCharacter('E', sidebar, SIDEBAR_WIDTH, 12, SIDEBAR_HEIGHT/2 + 32, sidebarColourLabel, 0, 2);
    renderCharacter('A', sidebar, SIDEBAR_WIDTH, 24, SIDEBAR_HEIGHT/2 + 32, sidebarColourLabel, 0, 2);
    renderCharacter('N', sidebar, SIDEBAR_WIDTH, 36, SIDEBAR_HEIGHT/2 + 32, sidebarColourLabel, 0, 2);

    while (1) {
        camera.capture(frame);

        float minT = frame[0];
        float maxT = frame[0];
        float mean = 0;
        for (int y = 0; y < CAMERA_HEIGHT; y++) {
            for (int x = 0; x < CAMERA_WIDTH; x++) {
                float val = frame[32 * (23 - y) + x];
                if (val < minT) {
                    minT = val;
                }
                if (val > maxT) {
                    maxT = val;
                }
                mean += val;
            }
        }
        mean = mean / (CAMERA_HEIGHT * CAMERA_WIDTH);
        float centre = frame[32 * (23 - 11) + 15];
        centre += frame[32 * (23 - 11) + 16];
        centre += frame[32 * (23 - 12) + 15];
        centre += frame[32 * (23 - 12) + 16];
        centre = centre / 4.0;
        float actualMinT = minT;
        float actualMaxT = maxT;
        if ((maxT - minT) < 5) {
            float margin = (5.0 - (maxT - minT))/2.0;
            minT -= margin;
            maxT += margin;
        }
        minT = std::floor(minT);
        maxT = std::ceil(maxT);
        float range = maxT - minT;

        renderTemperature(maxT, sidebar, SIDEBAR_WIDTH, 0, 2, 2, sidebarColourHot, 0);
        renderTemperature(minT, sidebar, SIDEBAR_WIDTH, 0, SIDEBAR_HEIGHT - 16, 2, sidebarColourCold, 0);
        renderTemperature(abs(centre), sidebar, SIDEBAR_WIDTH, 0, SIDEBAR_HEIGHT/2 - 14, 4, (centre<0)?sidebarColourCold:sidebarColour, 1);
        renderTemperature(mean, sidebar, SIDEBAR_WIDTH, 0, SIDEBAR_HEIGHT/2 + 50, 2, sidebarColour, 0);
        renderTemperature(actualMaxT, sidebar, SIDEBAR_WIDTH, 0, 20, 2, sidebarColour, 0);
        renderTemperature(actualMinT, sidebar, SIDEBAR_WIDTH, 0, SIDEBAR_HEIGHT - 34, 2, sidebarColour, 0);

        for (int x = 0; x < CAMERA_WIDTH; x++) {
            for (int px = 0; px < PIXEL_RATIO; px++) {
                st7789_set_cursor(leftOffset, LCD_HEIGHT - (x * PIXEL_RATIO + px));
                int lcdX = LCD_WIDTH - leftOffset;
                int lcdY = x * PIXEL_RATIO + px;
                for (int y = 0; y < CAMERA_HEIGHT; y++) {
                    float val = frame[32 * (23 - y) + x];
                    uint8_t hue = 240 - (uint8_t)(240.0*(val - minT)/range);
                    uint16_t pixel = HSLToRGB(hue, 1.0, 0.5);
                    for (int py = 0; py < PIXEL_RATIO; py++) {
                        uint16_t p = pixel;

                        // Crosshairs
                        if ((lcdY == crossHairsY) || (lcdY == (crossHairsY+1))) {
                            if ((lcdX > (crossHairsX - 20)) && (lcdX <= (crossHairsX + 20))) {
                                p = 0; // Black
                            }
                        }
                        if ((lcdX == crossHairsX) || (lcdX == (crossHairsX+1))) {
                            if ((lcdY > (crossHairsY - 20)) && (lcdY <= (crossHairsY + 20))) {
                                p = 0; // Black
                            }
                        }

                        st7789_put(p);
                        lcdX--;
                    }
                }
            }
        }

        for (int y = 0; y < LCD_HEIGHT; y++) {
            st7789_set_cursor(0, LCD_HEIGHT - y);
            for (int dx = 0; dx < SIDEBAR_WIDTH; dx++) {
                st7789_put(sidebar[y*SIDEBAR_WIDTH + SIDEBAR_WIDTH - 1 - dx]);
            }
        }
    }
}
