/*
 * Portions Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 * Portions Copyright (c) 2022 James Bulpin
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#include <stdlib.h>

#include "MLX90640_API.h"
#include "pico/stdlib.h"
#include "pico/st7789.h"
#include "ThermalCamera.h"

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

int main()
{
    // initialize the lcd
    st7789_init(&lcd_config, LCD_WIDTH, LCD_HEIGHT);

    // make screen black
    st7789_fill(0x0000);

    ThermalCamera camera;
    float frame[768];

    while (1) {
        camera.capture(frame);

        float minT = frame[0];
        float maxT = frame[0];
        for (int y = 0; y < CAMERA_HEIGHT; y++) {
            for (int x = 0; x < CAMERA_WIDTH; x++) {
                float val = frame[32 * (23 - y) + x];
                if (val < minT) {
                    minT = val;
                }
                if (val > maxT) {
                    maxT = val;
                }
            }
        }
        float range = maxT - minT;
        if (range < 5) {
            range = 5;
        }

        int leftOffset = LCD_WIDTH - CAMERA_HEIGHT * PIXEL_RATIO;
        for (int x = 0; x < CAMERA_WIDTH; x++) {
            for (int px = 0; px < PIXEL_RATIO; px++) {
                st7789_set_cursor(leftOffset, LCD_HEIGHT - (x * PIXEL_RATIO + px));
                for (int y = 0; y < CAMERA_HEIGHT; y++) {
                    float val = frame[32 * (23 - y) + x];
                    uint8_t hue = 240 - (uint8_t)(240.0*(val - minT)/range);
                    uint16_t pixel = HSLToRGB(hue, 1.0, 0.5);
                    for (int py = 0; py < PIXEL_RATIO; py++) {
                        st7789_put(pixel);
                    }
                }
            }
        }
    }
}
