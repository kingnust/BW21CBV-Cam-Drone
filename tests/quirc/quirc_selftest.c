#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "quirc.h"

enum { MODULE_COUNT = 25 };

static const char expected_payload[] = "BW21-QR-EVERY-FRAME-2026";
static const char modules[] =
    "0000000000000000000000000"
    "0000000000000000000000000"
    "0011111110111110111111100"
    "0010000010011000100000100"
    "0010111010011000101110100"
    "0010111010001110101110100"
    "0010111010010010101110100"
    "0010000010001110100000100"
    "0011111110101010111111100"
    "0000000000100000000000000"
    "0011011010011100100000100"
    "0010011000010100111011000"
    "0000111111110110000111000"
    "0001110101100010011100000"
    "0010111011101010001001100"
    "0000000000110011010111000"
    "0011111110000001010101000"
    "0010000010000110000100100"
    "0010111010100000110011000"
    "0010111010100011101110000"
    "0010111010010110101001100"
    "0010000010111100111101000"
    "0011111110101100001001000"
    "0000000000000000000000000"
    "0000000000000000000000000";

static void render(uint8_t *image, int width, int height, int module_size)
{
    const int rendered_size = MODULE_COUNT * module_size;
    const int left = (width - rendered_size) / 2;
    const int top = (height - rendered_size) / 2;
    memset(image, 255, width * height);
    for (int module_y = 0; module_y < MODULE_COUNT; module_y++) {
        for (int module_x = 0; module_x < MODULE_COUNT; module_x++) {
            if (modules[module_y * MODULE_COUNT + module_x] != '1') continue;
            for (int y = 0; y < module_size; y++) {
                memset(image + (top + module_y * module_size + y) * width +
                           left + module_x * module_size,
                       0, module_size);
            }
        }
    }
}

static int run_case(int width, int height, int module_size)
{
    struct quirc *decoder = quirc_new();
    if (!decoder || quirc_resize(decoder, width, height) < 0) {
        fprintf(stderr, "%dx%d decoder allocation failed\n", width, height);
        quirc_destroy(decoder);
        return 1;
    }
    render(quirc_begin(decoder, NULL, NULL), width, height, module_size);
    quirc_end(decoder);

    const int candidates = quirc_count(decoder);
    printf("%dx%d candidates=%d\n", width, height, candidates);
    for (int index = 0; index < candidates; index++) {
        struct quirc_code code;
        struct quirc_data data;
        quirc_extract(decoder, index, &code);
        const quirc_decode_error_t error = quirc_decode(&code, &data);
        printf("%dx%d candidate=%d size=%d result=%s payload=%.*s\n",
               width, height, index, code.size, quirc_strerror(error),
               error == QUIRC_SUCCESS ? data.payload_len : 0,
               error == QUIRC_SUCCESS ? (const char *)data.payload : "");
        if (error == QUIRC_SUCCESS &&
            data.payload_len == (int)strlen(expected_payload) &&
            memcmp(data.payload, expected_payload, data.payload_len) == 0) {
            quirc_destroy(decoder);
            return 0;
        }
    }
    quirc_destroy(decoder);
    return 1;
}

int main(void)
{
    const int full_result = run_case(640, 480, 10);
    const int fast_result = run_case(320, 240, 5);
    return full_result || fast_result;
}
