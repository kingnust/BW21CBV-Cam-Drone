#include "OnDeviceVision.h"

#include "BuildConfig.h"

#if BW21CAM_ENABLE_ONDEVICE_VISION

#include <SPI.h>
#include <JPEGDEC_Libraries/JPEGDEC.h>
#include <NNObjectDetection.h>
#include <QRCodeScanner.h>
#include <StreamIO.h>
#include <VideoStream.h>
#include <QRCodeScanner_Libraries/zbar/zbar.h>
#include <semphr.h>

#include "quirc.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace OnDeviceVision {
namespace {

using namespace zbar;

constexpr uint8_t NN_CHANNEL = 3;
constexpr uint16_t NN_WIDTH = 576;
constexpr uint16_t NN_HEIGHT = 320;
constexpr uint8_t NN_FPS = 10;
constexpr uint8_t QR_CHANNEL = 1;
constexpr uint16_t QR_WIDTH = 640;
constexpr uint16_t QR_HEIGHT = 480;
constexpr uint16_t QR_FAST_WIDTH = QR_WIDTH / 2;
constexpr uint16_t QR_FAST_HEIGHT = QR_HEIGHT / 2;
constexpr uint8_t QR_FPS = 10;
constexpr uint8_t QR_JPEG_QUALITY = 9;
constexpr size_t QR_PIXELS = static_cast<size_t>(QR_WIDTH) * QR_HEIGHT;
constexpr size_t QR_FAST_PIXELS = static_cast<size_t>(QR_FAST_WIDTH) * QR_FAST_HEIGHT;
constexpr uint32_t QR_FULL_RETRY_MS = 250;
constexpr uint32_t ZBAR_FALLBACK_MS = 1500;
constexpr uint32_t COLOR_SAMPLE_MS = 250;
constexpr uint32_t QR_REAPPEAR_MS = 1800;
constexpr uint32_t QR_OBSERVATION_INTERVAL_MS = 250;
constexpr uint32_t OBJECT_HOLD_MS = 700;
constexpr float OBJECT_BOX_NEW_WEIGHT = 0.45f;
constexpr uint8_t COLOR_COUNT = 12;

enum ColorId : uint8_t {
    COLOR_BLACK,
    COLOR_WHITE,
    COLOR_GREY,
    COLOR_RED,
    COLOR_ORANGE,
    COLOR_YELLOW,
    COLOR_GREEN,
    COLOR_CYAN,
    COLOR_BLUE,
    COLOR_PURPLE,
    COLOR_PINK,
    COLOR_BROWN
};

const char* const COLOR_NAMES[COLOR_COUNT] = {
    "black", "white", "grey", "red", "orange", "yellow",
    "green", "cyan", "blue", "purple", "pink", "brown"
};

struct ColorAccumulator {
    uint32_t count[COLOR_COUNT];
    uint32_t total;
};

struct DecodeContext {
    uint8_t* luma;
    uint16_t width;
    uint16_t height;
    ObjectResult objects[BW21CAM_VISION_MAX_OBJECTS];
    ColorAccumulator objectColors[BW21CAM_VISION_MAX_OBJECTS];
    uint8_t objectCount;
    uint8_t darkest;
    uint8_t brightest;
    uint32_t histogram[256];
    uint32_t decodedPixels;
    uint64_t luminanceSum;
};

VideoSetting qrConfig(QR_WIDTH, QR_HEIGHT, QR_FPS, VIDEO_JPEG, 1);
VideoSetting nnConfig(NN_WIDTH, NN_HEIGHT, NN_FPS, VIDEO_RGB, 0);
NNObjectDetection objectDetector;
StreamIO nnStreamer(1, 1);
JPEGDEC jpegDecoder;
SemaphoreHandle_t stateMutex = nullptr;
SemaphoreHandle_t analysisMutex = nullptr;
volatile bool visionActive = false;
Status state = {};
DecodeContext decodeContext = {};
uint8_t qrLuma[QR_PIXELS] = {};
uint8_t qrFastLuma[QR_FAST_PIXELS] = {};
zbar_image_scanner_t* qrScanner = nullptr;
zbar_image_t* qrImage = nullptr;
struct quirc* quircFullDecoder = nullptr;
struct quirc* quircFastDecoder = nullptr;
struct quirc_code quircCode = {};
struct quirc_data quircData = {};
uint32_t lastFullScanAtMs = 0;
uint32_t lastZbarScanAtMs = 0;
uint32_t lastColorSampleAtMs = 0;

constexpr char QR_SELF_TEST_PAYLOAD[] = "BW21-QR-EVERY-FRAME-2026";
constexpr char QR_SELF_TEST_MODULES[] =
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
static_assert(sizeof(QR_SELF_TEST_MODULES) - 1 == 25U * 25U,
              "QR self-test pattern must be 25 by 25 modules");

float clampUnit(float value)
{
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

void copyText(char* destination, size_t capacity, const char* source)
{
    if (!capacity) return;
    if (!source) source = "";
    strncpy(destination, source, capacity - 1);
    destination[capacity - 1] = 0;
}

bool isValidUtf8Text(const char* text, size_t length)
{
    size_t i = 0;
    while (i < length) {
        const uint8_t first = static_cast<uint8_t>(text[i]);
        if (first == 0 || first < 32 || first == 127) return false;
        if (first < 128) {
            i++;
            continue;
        }
        uint8_t continuationCount = 0;
        uint32_t codePoint = 0;
        if ((first & 0xe0) == 0xc0) {
            continuationCount = 1;
            codePoint = first & 0x1f;
        } else if ((first & 0xf0) == 0xe0) {
            continuationCount = 2;
            codePoint = first & 0x0f;
        } else if ((first & 0xf8) == 0xf0) {
            continuationCount = 3;
            codePoint = first & 0x07;
        } else {
            return false;
        }
        if (i + continuationCount >= length) return false;
        for (uint8_t offset = 1; offset <= continuationCount; offset++) {
            const uint8_t next = static_cast<uint8_t>(text[i + offset]);
            if ((next & 0xc0) != 0x80) return false;
            codePoint = (codePoint << 6) | (next & 0x3f);
        }
        const uint32_t minimum = continuationCount == 1 ? 0x80U
                                 : continuationCount == 2 ? 0x800U
                                                          : 0x10000U;
        if (codePoint < minimum || codePoint > 0x10ffffU ||
            (codePoint >= 0xd800U && codePoint <= 0xdfffU)) {
            return false;
        }
        i += continuationCount + 1;
    }
    return length > 0;
}

ColorId classifyColor(uint8_t red, uint8_t green, uint8_t blue)
{
    const uint8_t maximum = std::max(red, std::max(green, blue));
    const uint8_t minimum = std::min(red, std::min(green, blue));
    const uint8_t delta = maximum - minimum;
    if (maximum < 45) return COLOR_BLACK;

    const uint16_t saturation = maximum ? (static_cast<uint16_t>(delta) * 255U) / maximum : 0;
    if (saturation < 36) {
        if (maximum > 210) return COLOR_WHITE;
        if (maximum < 82) return COLOR_BLACK;
        return COLOR_GREY;
    }

    int16_t hue;
    if (maximum == red) {
        hue = static_cast<int16_t>(60 * (static_cast<int16_t>(green) - blue) / delta);
    } else if (maximum == green) {
        hue = static_cast<int16_t>(120 + 60 * (static_cast<int16_t>(blue) - red) / delta);
    } else {
        hue = static_cast<int16_t>(240 + 60 * (static_cast<int16_t>(red) - green) / delta);
    }
    if (hue < 0) hue += 360;

    if (hue < 15 || hue >= 345) return COLOR_RED;
    if (hue < 42) return maximum < 155 ? COLOR_BROWN : COLOR_ORANGE;
    if (hue < 70) return COLOR_YELLOW;
    if (hue < 165) return COLOR_GREEN;
    if (hue < 195) return COLOR_CYAN;
    if (hue < 255) return COLOR_BLUE;
    if (hue < 292) return COLOR_PURPLE;
    return COLOR_PINK;
}

void addColor(ColorAccumulator& accumulator, ColorId color)
{
    accumulator.count[color]++;
    accumulator.total++;
}

void finishColor(const ColorAccumulator& accumulator, char* name,
                 size_t capacity, uint8_t& confidence)
{
    uint8_t winner = COLOR_GREY;
    uint32_t winnerCount = 0;
    for (uint8_t i = 0; i < COLOR_COUNT; i++) {
        if (accumulator.count[i] > winnerCount) {
            winner = i;
            winnerCount = accumulator.count[i];
        }
    }
    copyText(name, capacity, accumulator.total ? COLOR_NAMES[winner] : "unknown");
    confidence = accumulator.total
                     ? static_cast<uint8_t>((winnerCount * 100U) / accumulator.total)
                     : 0;
}

int drawJpegBlock(JPEGDRAW* draw)
{
    if (!decodeContext.luma || !decodeContext.width || !decodeContext.height) return 0;
    const int width = draw->iWidthUsed > 0 ? draw->iWidthUsed : draw->iWidth;
    if (draw->iBpp == 8) {
        const uint8_t* pixels = reinterpret_cast<const uint8_t*>(draw->pPixels);
        for (int row = 0; row < draw->iHeight; row++) {
            const int y = draw->y + row;
            if (y < 0 || y >= decodeContext.height) continue;
            for (int column = 0; column < width; column++) {
                const int x = draw->x + column;
                if (x < 0 || x >= decodeContext.width) continue;
                const uint8_t luminance = pixels[row * draw->iWidth + column];
                decodeContext.luma[static_cast<size_t>(y) * decodeContext.width + x] =
                    luminance;
                decodeContext.histogram[luminance]++;
                decodeContext.decodedPixels++;
                decodeContext.luminanceSum += luminance;
                decodeContext.darkest = std::min(decodeContext.darkest, luminance);
                decodeContext.brightest = std::max(decodeContext.brightest, luminance);
            }
        }
        return 1;
    }

    for (int row = 0; row < draw->iHeight; row++) {
        const int y = draw->y + row;
        if (y < 0 || y >= decodeContext.height) continue;
        for (int column = 0; column < width; column++) {
            const int x = draw->x + column;
            if (x < 0 || x >= decodeContext.width) continue;

            const uint16_t pixel = draw->pPixels[row * draw->iWidth + column];
            const uint8_t red = static_cast<uint8_t>(((pixel >> 11) & 0x1f) * 255U / 31U);
            const uint8_t green = static_cast<uint8_t>(((pixel >> 5) & 0x3f) * 255U / 63U);
            const uint8_t blue = static_cast<uint8_t>((pixel & 0x1f) * 255U / 31U);
            const uint8_t luminance = static_cast<uint8_t>(
                (77U * red + 150U * green + 29U * blue) >> 8);
            decodeContext.luma[static_cast<size_t>(y) * decodeContext.width + x] =
                luminance;
            decodeContext.histogram[luminance]++;
            decodeContext.decodedPixels++;
            decodeContext.luminanceSum += luminance;
            decodeContext.darkest = std::min(decodeContext.darkest, luminance);
            decodeContext.brightest = std::max(decodeContext.brightest, luminance);

            if ((x & 3) || (y & 3)) continue;
            const ColorId color = classifyColor(red, green, blue);
            for (uint8_t i = 0; i < decodeContext.objectCount; i++) {
                const ObjectResult& object = decodeContext.objects[i];
                const float marginX = (object.xMax - object.xMin) * 0.12f;
                const float marginY = (object.yMax - object.yMin) * 0.12f;
                const float normalizedX = static_cast<float>(x) / decodeContext.width;
                const float normalizedY = static_cast<float>(y) / decodeContext.height;
                if (normalizedX >= object.xMin + marginX && normalizedX <= object.xMax - marginX &&
                    normalizedY >= object.yMin + marginY && normalizedY <= object.yMax - marginY) {
                    addColor(decodeContext.objectColors[i], color);
                }
            }
        }
    }
    return 1;
}

bool stretchContrast(uint8_t* luma, size_t pixels)
{
    if (!luma || !pixels) return false;
    const uint32_t tail = std::max<uint32_t>(1, pixels / 100U);
    uint32_t cumulative = 0;
    uint8_t darkest = decodeContext.darkest;
    uint8_t brightest = decodeContext.brightest;
    for (uint16_t value = 0; value < 256; value++) {
        cumulative += decodeContext.histogram[value];
        if (cumulative >= tail) {
            darkest = static_cast<uint8_t>(value);
            break;
        }
    }
    cumulative = 0;
    for (int value = 255; value >= 0; value--) {
        cumulative += decodeContext.histogram[value];
        if (cumulative >= tail) {
            brightest = static_cast<uint8_t>(value);
            break;
        }
    }
    const uint16_t span = brightest - darkest;
    if (span < 32 || span >= 220) return false;
    for (size_t i = 0; i < pixels; i++) {
        const uint8_t value = luma[i];
        if (value <= darkest) luma[i] = 0;
        else if (value >= brightest) luma[i] = 255;
        else luma[i] = static_cast<uint8_t>(
            (static_cast<uint16_t>(value - darkest) * 255U) / span);
    }
    return true;
}

uint16_t toPermille(float value)
{
    return static_cast<uint16_t>(std::lround(clampUnit(value) * 1000.0f));
}

QrGeometry geometryFromQuirc(const struct quirc_code& code, uint16_t width,
                             uint16_t height, bool fullResolution, bool mirrored)
{
    QrGeometry geometry = {};
    if (width < 2 || height < 2) return geometry;

    float centerX = 0.0f;
    float centerY = 0.0f;
    float side = 0.0f;
    float twiceArea = 0.0f;
    for (uint8_t i = 0; i < 4; i++) {
        const quirc_point& current = code.corners[i];
        const quirc_point& next = code.corners[(i + 1) & 3];
        centerX += current.x;
        centerY += current.y;
        side += std::hypot(static_cast<float>(next.x - current.x),
                           static_cast<float>(next.y - current.y));
        twiceArea += static_cast<float>(current.x * next.y - next.x * current.y);
    }
    side *= 0.25f;
    const float area = std::fabs(twiceArea) * 0.5f;
    if (side < 4.0f || area < 16.0f) return geometry;

    const float rotation = std::atan2(
        static_cast<float>(code.corners[1].y - code.corners[0].y),
        static_cast<float>(code.corners[1].x - code.corners[0].x));
    geometry.valid = true;
    geometry.fullResolution = fullResolution;
    geometry.mirrored = mirrored;
    geometry.centerXPermille = toPermille((centerX * 0.25f) / (width - 1));
    geometry.centerYPermille = toPermille((centerY * 0.25f) / (height - 1));
    geometry.sidePermille = toPermille(side / width);
    geometry.areaPermille = toPermille(area / (static_cast<float>(width) * height));
    geometry.rotationCdeg = static_cast<int16_t>(std::lround(
        rotation * (18000.0f / 3.14159265358979323846f)));
    return geometry;
}

QrGeometry geometryFromZbar(const zbar_symbol_t* symbol)
{
    QrGeometry geometry = {};
    const unsigned int count = zbar_symbol_get_loc_size(symbol);
    if (count < 4) return geometry;

    int minimumX = QR_WIDTH;
    int minimumY = QR_HEIGHT;
    int maximumX = -1;
    int maximumY = -1;
    for (unsigned int i = 0; i < count; i++) {
        const int x = zbar_symbol_get_loc_x(symbol, i);
        const int y = zbar_symbol_get_loc_y(symbol, i);
        minimumX = std::min(minimumX, x);
        minimumY = std::min(minimumY, y);
        maximumX = std::max(maximumX, x);
        maximumY = std::max(maximumY, y);
    }
    const float width = maximumX - minimumX;
    const float height = maximumY - minimumY;
    const float side = (width + height) * 0.5f;
    if (width < 4.0f || height < 4.0f) return geometry;

    geometry.valid = true;
    geometry.fullResolution = true;
    geometry.zbarFallback = true;
    geometry.centerXPermille = toPermille(
        ((minimumX + maximumX) * 0.5f) / (QR_WIDTH - 1));
    geometry.centerYPermille = toPermille(
        ((minimumY + maximumY) * 0.5f) / (QR_HEIGHT - 1));
    geometry.sidePermille = toPermille(side / QR_WIDTH);
    geometry.areaPermille = toPermille(
        (width * height) / (static_cast<float>(QR_WIDTH) * QR_HEIGHT));
    return geometry;
}

bool acceptQrPayloadLocked(const uint8_t* payload, size_t length, uint32_t now,
                           const QrGeometry& geometry)
{
    if (!payload || length == 0 || length >= QR_PAYLOAD_CAPACITY ||
        !isValidUtf8Text(reinterpret_cast<const char*>(payload), length)) {
        state.qrDecodeErrors++;
        return false;
    }

    const char* text = reinterpret_cast<const char*>(payload);
    const bool changed = strncmp(state.qrPayload, text, length) != 0 ||
                         state.qrPayload[length] != 0;
    const bool reappeared = state.qrSeenAtMs && now - state.qrSeenAtMs > QR_REAPPEAR_MS;
    const bool observationDue = !state.qrPublishedAtMs ||
                                now - state.qrPublishedAtMs >= QR_OBSERVATION_INTERVAL_MS;
    state.qrSeenAtMs = now;
    state.qrDecodes++;
    state.qrGeometry = geometry;
    if (changed || reappeared || observationDue || state.qrSequence == 0) {
        memcpy(state.qrPayload, payload, length);
        state.qrPayload[length] = 0;
        state.qrPublishedAtMs = now;
        state.qrSequence++;
    } else {
        state.qrDuplicates++;
    }
    return true;
}

void transposeQrCode(struct quirc_code& code)
{
    for (int y = 0; y < code.size; y++) {
        for (int x = 0; x < y; x++) {
            const int first = y * code.size + x;
            const int second = x * code.size + y;
            const bool firstSet = code.cell_bitmap[first >> 3] & (1U << (first & 7));
            const bool secondSet = code.cell_bitmap[second >> 3] & (1U << (second & 7));
            if (firstSet != secondSet) {
                code.cell_bitmap[first >> 3] ^= 1U << (first & 7);
                code.cell_bitmap[second >> 3] ^= 1U << (second & 7);
            }
        }
    }
}

bool scanWithQuirc(struct quirc* decoder, const uint8_t* luma, size_t pixels,
                   bool fullResolution, uint32_t now, int* candidates = nullptr)
{
    uint8_t* image = quirc_begin(decoder, nullptr, nullptr);
    if (!image || !luma) return false;
    memcpy(image, luma, pixels);
    quirc_end(decoder);

    const int count = quirc_count(decoder);
    if (candidates) *candidates = count;
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    state.qrScanPasses++;
    state.quircScanPasses++;
    if (fullResolution) state.quircFullScans++;
    else state.quircFastScans++;
    state.quircCandidates += count;
    state.qrCandidates += count;
    state.qrLastScanDetail = static_cast<int8_t>(std::min(count, 127));
    if (count == 0) state.qrNoFinderCenters++;
    xSemaphoreGive(stateMutex);

    for (int index = 0; index < count; index++) {
        quirc_extract(decoder, index, &quircCode);
        quirc_decode_error_t error = quirc_decode(&quircCode, &quircData);
        bool mirrored = false;
        if (error != QUIRC_SUCCESS) {
            transposeQrCode(quircCode);
            error = quirc_decode(&quircCode, &quircData);
            mirrored = error == QUIRC_SUCCESS;
        }

        xSemaphoreTake(stateMutex, portMAX_DELAY);
        if (error == QUIRC_SUCCESS) {
            if (mirrored) state.quircMirroredDecodes++;
            const QrGeometry geometry = geometryFromQuirc(
                quircCode, fullResolution ? QR_WIDTH : QR_FAST_WIDTH,
                fullResolution ? QR_HEIGHT : QR_FAST_HEIGHT,
                fullResolution, mirrored);
            const bool accepted = acceptQrPayloadLocked(
                quircData.payload, quircData.payload_len, now, geometry);
            if (!accepted) state.quircDecodeErrors++;
            xSemaphoreGive(stateMutex);
            if (accepted) return true;
        } else {
            state.quircDecodeErrors++;
            state.qrDecodeErrors++;
            xSemaphoreGive(stateMutex);
        }
    }
    return false;
}

bool scanWithZbar(uint32_t now, int& scanDetail)
{
    scanDetail = 0;
    zbar_scan_image(qrScanner, qrImage, &scanDetail);
    const zbar_symbol_t* symbol = zbar_image_first_symbol(qrImage);
    bool accepted = false;

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    state.zbarFallbackScans++;
    for (; symbol; symbol = zbar_symbol_next(symbol)) {
        if (zbar_symbol_get_type(symbol) != ZBAR_QRCODE) continue;
        const uint8_t* payload = reinterpret_cast<const uint8_t*>(
            zbar_symbol_get_data(symbol));
        const size_t sourceLength = zbar_symbol_get_data_length(symbol);
        accepted = acceptQrPayloadLocked(
            payload, sourceLength, now, geometryFromZbar(symbol));
        if (accepted) break;
    }
    xSemaphoreGive(stateMutex);
    return accepted;
}

bool setQrDensity(int density)
{
    return zbar_image_scanner_set_config(
               qrScanner, ZBAR_NONE, ZBAR_CFG_X_DENSITY, density) == 0 &&
           zbar_image_scanner_set_config(
               qrScanner, ZBAR_NONE, ZBAR_CFG_Y_DENSITY, density) == 0;
}

void renderQrSelfTest(uint8_t* luma, uint16_t width, uint16_t height,
                      uint8_t moduleSize)
{
    memset(luma, 255, static_cast<size_t>(width) * height);
    constexpr int moduleCount = 25;
    const int renderedSize = moduleCount * moduleSize;
    const int left = (width - renderedSize) / 2;
    const int top = (height - renderedSize) / 2;
    for (int moduleY = 0; moduleY < moduleCount; moduleY++) {
        for (int moduleX = 0; moduleX < moduleCount; moduleX++) {
            if (QR_SELF_TEST_MODULES[moduleY * moduleCount + moduleX] != '1') continue;
            for (int y = 0; y < moduleSize; y++) {
                memset(luma + static_cast<size_t>(top + moduleY * moduleSize + y) *
                                  width + left + moduleX * moduleSize,
                       0, moduleSize);
            }
        }
    }
}

bool runQuircSelfTest(struct quirc* decoder, uint8_t* luma, size_t pixels)
{
    uint8_t* image = quirc_begin(decoder, nullptr, nullptr);
    if (!image) return false;
    memcpy(image, luma, pixels);
    quirc_end(decoder);
    const int count = quirc_count(decoder);
    for (int index = 0; index < count; index++) {
        quirc_extract(decoder, index, &quircCode);
        quirc_decode_error_t error = quirc_decode(&quircCode, &quircData);
        if (error != QUIRC_SUCCESS) {
            transposeQrCode(quircCode);
            error = quirc_decode(&quircCode, &quircData);
        }
        if (error == QUIRC_SUCCESS &&
            quircData.payload_len == strlen(QR_SELF_TEST_PAYLOAD) &&
            memcmp(quircData.payload, QR_SELF_TEST_PAYLOAD,
                   quircData.payload_len) == 0) {
            return true;
        }
    }
    return false;
}

bool runZbarSelfTest()
{
    for (int density : {2, 1}) {
        if (!setQrDensity(density)) return false;
        int scanDetail = 0;
        const int decoded = zbar_scan_image(qrScanner, qrImage, &scanDetail);
        for (const zbar_symbol_t* symbol = zbar_image_first_symbol(qrImage); symbol;
             symbol = zbar_symbol_next(symbol)) {
            if (decoded <= 0 || zbar_symbol_get_type(symbol) != ZBAR_QRCODE) continue;
            const size_t length = zbar_symbol_get_data_length(symbol);
            if (length == strlen(QR_SELF_TEST_PAYLOAD) &&
                memcmp(zbar_symbol_get_data(symbol), QR_SELF_TEST_PAYLOAD, length) == 0) {
                return true;
            }
        }
    }
    return false;
}

float intersectionOverUnion(const ObjectResult& first, const ObjectResult& second)
{
    const float left = std::max(first.xMin, second.xMin);
    const float top = std::max(first.yMin, second.yMin);
    const float right = std::min(first.xMax, second.xMax);
    const float bottom = std::min(first.yMax, second.yMax);
    const float intersection = std::max(0.0f, right - left) *
                               std::max(0.0f, bottom - top);
    const float firstArea = std::max(0.0f, first.xMax - first.xMin) *
                            std::max(0.0f, first.yMax - first.yMin);
    const float secondArea = std::max(0.0f, second.xMax - second.xMin) *
                             std::max(0.0f, second.yMax - second.yMin);
    const float combined = firstArea + secondArea - intersection;
    return combined > 0.0f ? intersection / combined : 0.0f;
}

void retainObjectHistory(ObjectResult* objects, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        float bestOverlap = 0.3f;
        int best = -1;
        for (uint8_t j = 0; j < state.objectCount; j++) {
            if (strcmp(objects[i].name, state.objects[j].name) != 0) continue;
            const float overlap = intersectionOverUnion(objects[i], state.objects[j]);
            if (overlap > bestOverlap) {
                bestOverlap = overlap;
                best = j;
            }
        }
        if (best >= 0) {
            const ObjectResult& previous = state.objects[best];
            copyText(objects[i].color, sizeof(objects[i].color), previous.color);
            objects[i].colorConfidence = previous.colorConfidence;
            const float previousWeight = 1.0f - OBJECT_BOX_NEW_WEIGHT;
            objects[i].xMin = previous.xMin * previousWeight +
                              objects[i].xMin * OBJECT_BOX_NEW_WEIGHT;
            objects[i].yMin = previous.yMin * previousWeight +
                              objects[i].yMin * OBJECT_BOX_NEW_WEIGHT;
            objects[i].xMax = previous.xMax * previousWeight +
                              objects[i].xMax * OBJECT_BOX_NEW_WEIGHT;
            objects[i].yMax = previous.yMax * previousWeight +
                              objects[i].yMax * OBJECT_BOX_NEW_WEIGHT;
        }
    }
}

void applyObjectColors()
{
    for (uint8_t current = 0; current < state.objectCount; current++) {
        float bestOverlap = 0.3f;
        int best = -1;
        for (uint8_t analyzed = 0; analyzed < decodeContext.objectCount; analyzed++) {
            if (strcmp(state.objects[current].name,
                       decodeContext.objects[analyzed].name) != 0) {
                continue;
            }
            const float overlap = intersectionOverUnion(
                state.objects[current], decodeContext.objects[analyzed]);
            if (overlap > bestOverlap) {
                bestOverlap = overlap;
                best = analyzed;
            }
        }
        if (best >= 0) {
            finishColor(decodeContext.objectColors[best], state.objects[current].color,
                        sizeof(state.objects[current].color),
                        state.objects[current].colorConfidence);
        }
    }
}

void objectResultCallback(std::vector<ObjectDetectionResult> results)
{
    if (!visionActive) return;

    ObjectResult objects[BW21CAM_VISION_MAX_OBJECTS] = {};
    uint8_t count = 0;
    for (size_t i = 0; i < results.size() && count < BW21CAM_VISION_MAX_OBJECTS; i++) {
        if (results[i].score() < 35) continue;
        ObjectResult& object = objects[count++];
        copyText(object.name, sizeof(object.name), results[i].name());
        copyText(object.color, sizeof(object.color), "unknown");
        object.score = static_cast<uint8_t>(std::min(results[i].score(), 100));
        object.xMin = clampUnit(results[i].xMin());
        object.yMin = clampUnit(results[i].yMin());
        object.xMax = clampUnit(results[i].xMax());
        object.yMax = clampUnit(results[i].yMax());
    }

    const uint32_t now = millis();
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    if (!visionActive) {
        xSemaphoreGive(stateMutex);
        return;
    }
    state.yoloFrames++;
    if (count) {
        retainObjectHistory(objects, count);
        memcpy(state.objects, objects, sizeof(objects));
        state.objectCount = count;
        state.objectSequence++;
        state.objectSeenAtMs = now;
    } else if (!state.objectCount || now - state.objectSeenAtMs > OBJECT_HOLD_MS) {
        if (state.objectCount) state.objectSequence++;
        memset(state.objects, 0, sizeof(state.objects));
        state.objectCount = 0;
        state.objectSeenAtMs = 0;
    }
    xSemaphoreGive(stateMutex);
}

bool decodeAnalysisJpeg(const uint8_t* jpeg, size_t length, bool fullResolution,
                        bool sampleObjectColors)
{
    uint8_t* luma = fullResolution ? qrLuma : qrFastLuma;
    const uint16_t width = fullResolution ? QR_WIDTH : QR_FAST_WIDTH;
    const uint16_t height = fullResolution ? QR_HEIGHT : QR_FAST_HEIGHT;
    const size_t pixels = static_cast<size_t>(width) * height;

    memset(luma, 255, pixels);
    memset(&decodeContext, 0, sizeof(decodeContext));
    decodeContext.luma = luma;
    decodeContext.width = width;
    decodeContext.height = height;
    decodeContext.darkest = 255;

    if (sampleObjectColors) {
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        decodeContext.objectCount = state.objectCount;
        memcpy(decodeContext.objects, state.objects, sizeof(state.objects));
        xSemaphoreGive(stateMutex);
    }

    bool decodeOk = jpegDecoder.openFLASH(const_cast<uint8_t*>(jpeg), length,
                                          drawJpegBlock) != 0;
    if (decodeOk) {
        jpegDecoder.setPixelType(sampleObjectColors ? RGB565_LITTLE_ENDIAN
                                                    : EIGHT_BIT_GRAYSCALE);
        decodeOk = jpegDecoder.getWidth() == QR_WIDTH &&
                   jpegDecoder.getHeight() == QR_HEIGHT &&
                   jpegDecoder.decode(0, 0,
                                      fullResolution ? 0 : JPEG_SCALE_HALF) != 0;
        jpegDecoder.close();
    }
    return decodeOk && decodeContext.decodedPixels == pixels;
}

void processAnalysisJpeg(const uint8_t* jpeg, size_t length, uint32_t frameSequence)
{
    if (!visionActive || !state.ready || !jpeg || !length) return;
    const uint32_t started = millis();
    uint32_t jpegDecodeMs = 0;
    uint32_t qrScanMs = 0;
    bool accepted = false;
    bool decodedAny = false;
    uint8_t darkest = 255;
    uint8_t brightest = 0;
    uint8_t mean = 0;

    uint32_t stageStarted = millis();
    const bool fastDecodeOk = decodeAnalysisJpeg(jpeg, length, false, false);
    jpegDecodeMs += millis() - stageStarted;
    decodedAny = fastDecodeOk;
    if (fastDecodeOk) {
        darkest = decodeContext.darkest;
        brightest = decodeContext.brightest;
        mean = static_cast<uint8_t>(decodeContext.luminanceSum /
                                    decodeContext.decodedPixels);
        int candidates = 0;
        stageStarted = millis();
        accepted = scanWithQuirc(quircFastDecoder, qrFastLuma, QR_FAST_PIXELS,
                                 false, millis(), &candidates);
        if (!accepted && stretchContrast(qrFastLuma, QR_FAST_PIXELS)) {
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            state.qrEnhancedScans++;
            xSemaphoreGive(stateMutex);
            accepted = scanWithQuirc(quircFastDecoder, qrFastLuma, QR_FAST_PIXELS,
                                     false, millis());
        }
        qrScanMs += millis() - stageStarted;
    }

    const uint32_t now = millis();
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    const bool haveObjects = state.objectCount > 0;
    xSemaphoreGive(stateMutex);
    const bool fullScanDue = !accepted && now - lastFullScanAtMs >= QR_FULL_RETRY_MS;
    const bool colorSampleDue = haveObjects && now - lastColorSampleAtMs >= COLOR_SAMPLE_MS;
    if (fullScanDue || colorSampleDue) {
        stageStarted = millis();
        const bool fullDecodeOk = decodeAnalysisJpeg(jpeg, length, true, haveObjects);
        jpegDecodeMs += millis() - stageStarted;
        decodedAny = decodedAny || fullDecodeOk;
        if (fullDecodeOk) {
            darkest = decodeContext.darkest;
            brightest = decodeContext.brightest;
            mean = static_cast<uint8_t>(decodeContext.luminanceSum /
                                        decodeContext.decodedPixels);
            if (haveObjects) {
                xSemaphoreTake(stateMutex, portMAX_DELAY);
                applyObjectColors();
                xSemaphoreGive(stateMutex);
                lastColorSampleAtMs = now;
            }
            if (fullScanDue) {
                lastFullScanAtMs = now;
                int fullCandidates = 0;
                stageStarted = millis();
                accepted = scanWithQuirc(quircFullDecoder, qrLuma, QR_PIXELS,
                                         true, millis(), &fullCandidates);
                const bool zbarDue = now - lastZbarScanAtMs >= ZBAR_FALLBACK_MS;
                if (!accepted && (fullCandidates > 0 || zbarDue) &&
                    stretchContrast(qrLuma, QR_PIXELS)) {
                    xSemaphoreTake(stateMutex, portMAX_DELAY);
                    state.qrEnhancedScans++;
                    xSemaphoreGive(stateMutex);
                    accepted = scanWithQuirc(quircFullDecoder, qrLuma, QR_PIXELS,
                                             true, millis());
                }
                if (!accepted && zbarDue) {
                    lastZbarScanAtMs = now;
                    int scanDetail = 0;
                    setQrDensity(1);
                    accepted = scanWithZbar(millis(), scanDetail);
                }
                qrScanMs += millis() - stageStarted;
            }
        }
    }

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    state.frameSequence = frameSequence;
    state.analyzedFrames++;
    if (!fastDecodeOk) state.jpegDecodeErrors++;
    if (fastDecodeOk) state.qrCheckedFrames++;
    if (decodedAny) {
        state.qrDarkest = darkest;
        state.qrBrightest = brightest;
        state.qrMean = mean;
    }
    state.lastJpegDecodeMs = jpegDecodeMs;
    state.lastQrScanMs = qrScanMs;
    state.lastProcessMs = millis() - started;
    state.maxProcessMs = std::max(state.maxProcessMs, state.lastProcessMs);
    xSemaphoreGive(stateMutex);
}

void analysisTask(void*)
{
    uint32_t sequence = 0;
    for (;;) {
        if (!visionActive) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        xSemaphoreTake(analysisMutex, portMAX_DELAY);
        if (!visionActive) {
            xSemaphoreGive(analysisMutex);
            continue;
        }
        uint32_t imageAddress = 0;
        uint32_t imageLength = 0;
        Camera.getImage(QR_CHANNEL, &imageAddress, &imageLength);
        processAnalysisJpeg(reinterpret_cast<const uint8_t*>(imageAddress),
                            imageLength, ++sequence);
        xSemaphoreGive(analysisMutex);
        vTaskDelay(1);
    }
}

}  // namespace

void configureCamera()
{
    qrConfig.setJpegQuality(QR_JPEG_QUALITY);
    Camera.configVideoChannel(QR_CHANNEL, qrConfig);
    Camera.configVideoChannel(NN_CHANNEL, nnConfig);
}

bool begin()
{
    stateMutex = xSemaphoreCreateMutex();
    analysisMutex = xSemaphoreCreateMutex();
    if (!stateMutex || !analysisMutex) return false;
    state.enabled = false;

    qrScanner = zbar_image_scanner_create(2, 2);
    qrImage = zbar_image_create();
    if (!qrScanner || !qrImage) return false;
    zbar_image_scanner_set_config(qrScanner, ZBAR_NONE, ZBAR_CFG_ENABLE, 0);
    zbar_image_scanner_set_config(qrScanner, ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
    const uint32_t format = static_cast<uint32_t>('Y') |
                            (static_cast<uint32_t>('8') << 8) |
                            (static_cast<uint32_t>('0') << 16) |
                            (static_cast<uint32_t>('0') << 24);
    zbar_image_set_format(qrImage, format);
    zbar_image_set_size(qrImage, QR_WIDTH, QR_HEIGHT);
    zbar_image_set_data(qrImage, qrLuma, QR_PIXELS, nullptr);
    quircFullDecoder = quirc_new();
    quircFastDecoder = quirc_new();
    if (!quircFullDecoder || !quircFastDecoder ||
        quirc_resize(quircFullDecoder, QR_WIDTH, QR_HEIGHT) < 0 ||
        quirc_resize(quircFastDecoder, QR_FAST_WIDTH, QR_FAST_HEIGHT) < 0) {
        Serial.println("quirc allocation failed");
        return false;
    }
    state.quircReady = true;
    renderQrSelfTest(qrLuma, QR_WIDTH, QR_HEIGHT, 10);
    state.quircFullSelfTestPassed = runQuircSelfTest(
        quircFullDecoder, qrLuma, QR_PIXELS);
    renderQrSelfTest(qrFastLuma, QR_FAST_WIDTH, QR_FAST_HEIGHT, 5);
    state.quircFastSelfTestPassed = runQuircSelfTest(
        quircFastDecoder, qrFastLuma, QR_FAST_PIXELS);
    state.quircSelfTestPassed = state.quircFastSelfTestPassed &&
                                state.quircFullSelfTestPassed;
    renderQrSelfTest(qrLuma, QR_WIDTH, QR_HEIGHT, 10);
    state.zbarSelfTestPassed = runZbarSelfTest();
    state.qrSelfTestPassed = state.quircSelfTestPassed && state.zbarSelfTestPassed;
    memset(qrLuma, 255, sizeof(qrLuma));
    Serial.print("QR decoder self-test: quirc=");
    Serial.print(state.quircSelfTestPassed ? "PASS" : "FAIL");
    Serial.print(" fast=");
    Serial.print(state.quircFastSelfTestPassed ? "PASS" : "FAIL");
    Serial.print(" full=");
    Serial.print(state.quircFullSelfTestPassed ? "PASS" : "FAIL");
    Serial.print(" zbar=");
    Serial.print(state.zbarSelfTestPassed ? "PASS" : "FAIL");
    Serial.print(" overall=");
    Serial.println(state.qrSelfTestPassed ? "PASS" : "FAIL");

    objectDetector.configVideo(nnConfig);
    objectDetector.configThreshold(0.35f, 0.45f);
    objectDetector.setResultCallback(objectResultCallback);
    objectDetector.modelSelect(OBJECT_DETECTION, DEFAULT_YOLOV4TINY, NA_MODEL, NA_MODEL);
    objectDetector.begin();

    nnStreamer.registerInput(Camera.getStream(NN_CHANNEL));
    nnStreamer.setStackSize();
    nnStreamer.setTaskPriority();
    nnStreamer.registerOutput(objectDetector);
    if (nnStreamer.begin() != 0) return false;
    nnStreamer.pause();

    state.ready = true;
    if (xTaskCreate(analysisTask, "CameraAnalysis", 10 * 1024, nullptr, 1,
                    nullptr) != pdPASS) {
        state.ready = false;
        return false;
    }
    if (BW21CAM_VISION_DEFAULT_ENABLED && !setEnabled(true)) {
        state.ready = false;
        return false;
    }
    Serial.print("On-device vision ready: fast/full quirc + timed ZBar fallback + "
                 "YOLOv4-tiny; mode=");
    Serial.println(state.enabled ? "vision" : "camera-only");
    return true;
}

bool setEnabled(bool enabled)
{
    if (!stateMutex || !analysisMutex) return false;

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    const bool ready = state.ready;
    const bool current = state.enabled;
    xSemaphoreGive(stateMutex);
    if (!ready) return false;
    if (enabled == current) return true;

    if (enabled) {
        xSemaphoreTake(analysisMutex, portMAX_DELAY);
        Camera.channelBegin(QR_CHANNEL);
        Camera.channelBegin(NN_CHANNEL);
        visionActive = true;
        lastFullScanAtMs = 0;
        lastZbarScanAtMs = 0;
        lastColorSampleAtMs = 0;
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        state.enabled = true;
        xSemaphoreGive(stateMutex);
        nnStreamer.resume();
        xSemaphoreGive(analysisMutex);
    } else {
        visionActive = false;
        nnStreamer.pause();
        xSemaphoreTake(analysisMutex, portMAX_DELAY);
        Camera.channelEnd(NN_CHANNEL);
        Camera.channelEnd(QR_CHANNEL);
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        state.enabled = false;
        state.qrSeenAtMs = 0;
        state.qrPublishedAtMs = 0;
        state.qrGeometry = {};
        state.qrPayload[0] = 0;
        state.objectCount = 0;
        state.objectSeenAtMs = 0;
        memset(state.objects, 0, sizeof(state.objects));
        xSemaphoreGive(stateMutex);
        xSemaphoreGive(analysisMutex);
    }

    Serial.print("Vision mode: ");
    Serial.println(enabled ? "ON" : "OFF (camera-only)");
    return true;
}

void getStatus(Status& status)
{
    if (!stateMutex) {
        memset(&status, 0, sizeof(status));
        return;
    }
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    status = state;
    xSemaphoreGive(stateMutex);
}

bool lockQrJpeg(const uint8_t*& jpeg, size_t& length)
{
    jpeg = nullptr;
    length = 0;
    if (!visionActive || !state.ready || !analysisMutex ||
        xSemaphoreTake(analysisMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
        return false;
    }
    if (!visionActive) {
        xSemaphoreGive(analysisMutex);
        return false;
    }

    uint32_t imageAddress = 0;
    uint32_t imageLength = 0;
    Camera.getImage(QR_CHANNEL, &imageAddress, &imageLength);
    if (!imageAddress || !imageLength) {
        xSemaphoreGive(analysisMutex);
        return false;
    }
    jpeg = reinterpret_cast<const uint8_t*>(imageAddress);
    length = imageLength;
    return true;
}

void unlockQrJpeg()
{
    if (analysisMutex) xSemaphoreGive(analysisMutex);
}

}  // namespace OnDeviceVision

#else

namespace OnDeviceVision {

void configureCamera() {}
bool begin() { return true; }
bool setEnabled(bool) { return false; }
void getStatus(Status& status)
{
    memset(&status, 0, sizeof(status));
}
bool lockQrJpeg(const uint8_t*&, size_t&) { return false; }
void unlockQrJpeg() {}

}  // namespace OnDeviceVision

#endif
