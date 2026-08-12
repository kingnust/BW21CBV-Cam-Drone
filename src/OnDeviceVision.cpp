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

#include <algorithm>
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
constexpr uint16_t QR_HEIGHT = 360;
constexpr uint8_t QR_FPS = 10;
constexpr uint8_t QR_JPEG_QUALITY = 5;
constexpr size_t QR_PIXELS = static_cast<size_t>(QR_WIDTH) * QR_HEIGHT;
constexpr uint32_t QR_REAPPEAR_MS = 1800;
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
Status state = {};
DecodeContext decodeContext = {};
uint8_t qrLuma[QR_PIXELS] = {};
zbar_image_scanner_t* qrScanner = nullptr;
zbar_image_t* qrImage = nullptr;

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
    const int width = draw->iWidthUsed > 0 ? draw->iWidthUsed : draw->iWidth;
    if (draw->iBpp == 8) {
        const uint8_t* pixels = reinterpret_cast<const uint8_t*>(draw->pPixels);
        for (int row = 0; row < draw->iHeight; row++) {
            const int y = draw->y + row;
            if (y < 0 || y >= QR_HEIGHT) continue;
            for (int column = 0; column < width; column++) {
                const int x = draw->x + column;
                if (x < 0 || x >= QR_WIDTH) continue;
                const uint8_t luminance = pixels[row * draw->iWidth + column];
                qrLuma[static_cast<size_t>(y) * QR_WIDTH + x] = luminance;
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
        if (y < 0 || y >= QR_HEIGHT) continue;
        for (int column = 0; column < width; column++) {
            const int x = draw->x + column;
            if (x < 0 || x >= QR_WIDTH) continue;

            const uint16_t pixel = draw->pPixels[row * draw->iWidth + column];
            const uint8_t red = static_cast<uint8_t>(((pixel >> 11) & 0x1f) * 255U / 31U);
            const uint8_t green = static_cast<uint8_t>(((pixel >> 5) & 0x3f) * 255U / 63U);
            const uint8_t blue = static_cast<uint8_t>((pixel & 0x1f) * 255U / 31U);
            const uint8_t luminance = static_cast<uint8_t>(
                (77U * red + 150U * green + 29U * blue) >> 8);
            qrLuma[static_cast<size_t>(y) * QR_WIDTH + x] = luminance;
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
                const float normalizedX = static_cast<float>(x) / QR_WIDTH;
                const float normalizedY = static_cast<float>(y) / QR_HEIGHT;
                if (normalizedX >= object.xMin + marginX && normalizedX <= object.xMax - marginX &&
                    normalizedY >= object.yMin + marginY && normalizedY <= object.yMax - marginY) {
                    addColor(decodeContext.objectColors[i], color);
                }
            }
        }
    }
    return 1;
}

void stretchContrast()
{
    const uint32_t lowerTarget = QR_PIXELS / 50;
    const uint32_t upperTarget = QR_PIXELS - lowerTarget;
    uint32_t cumulative = 0;
    uint8_t lower = decodeContext.darkest;
    uint8_t upper = decodeContext.brightest;
    for (uint16_t value = 0; value < 256; value++) {
        cumulative += decodeContext.histogram[value];
        if (cumulative >= lowerTarget) {
            lower = static_cast<uint8_t>(value);
            break;
        }
    }
    cumulative = 0;
    for (uint16_t value = 0; value < 256; value++) {
        cumulative += decodeContext.histogram[value];
        if (cumulative >= upperTarget) {
            upper = static_cast<uint8_t>(value);
            break;
        }
    }
    const uint16_t span = upper - lower;
    if (span < 40 || span > 220) return;
    for (size_t i = 0; i < QR_PIXELS; i++) {
        const uint8_t value = qrLuma[i];
        if (value <= lower) qrLuma[i] = 0;
        else if (value >= upper) qrLuma[i] = 255;
        else qrLuma[i] = static_cast<uint8_t>(
            (static_cast<uint16_t>(value - lower) * 255U) / span);
    }
}

bool recordQrResult(uint32_t now, int& scanDetail)
{
    scanDetail = 0;
    const int decoded = zbar_scan_image(qrScanner, qrImage, &scanDetail);
    const zbar_symbol_t* symbol = zbar_image_first_symbol(qrImage);
    bool accepted = false;

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    state.qrScanPasses++;
    state.qrLastScanDetail = static_cast<int8_t>(scanDetail);
    if (scanDetail == -1) state.qrNoFinderCenters++;
    if (scanDetail == -2) {
        state.qrCandidates++;
        state.qrDecodeErrors++;
    }
    if (decoded > 0) state.qrCandidates += decoded;
    for (; symbol; symbol = zbar_symbol_next(symbol)) {
        if (zbar_symbol_get_type(symbol) != ZBAR_QRCODE) continue;
        const char* payload = zbar_symbol_get_data(symbol);
        const size_t sourceLength = zbar_symbol_get_data_length(symbol);
        const size_t length = std::min(sourceLength, QR_PAYLOAD_CAPACITY - 1);
        if (!isValidUtf8Text(payload, length)) {
            state.qrDecodeErrors++;
            continue;
        }

        const bool changed = strncmp(state.qrPayload, payload, length) != 0 ||
                             state.qrPayload[length] != 0;
        const bool reappeared = state.qrSeenAtMs && now - state.qrSeenAtMs > QR_REAPPEAR_MS;
        state.qrSeenAtMs = now;
        state.qrDecodes++;
        if (changed || reappeared || state.qrSequence == 0) {
            memcpy(state.qrPayload, payload, length);
            state.qrPayload[length] = 0;
            state.qrSequence++;
        } else {
            state.qrDuplicates++;
        }
        accepted = true;
        break;
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

bool runQrSelfTest()
{
    memset(qrLuma, 255, sizeof(qrLuma));
    constexpr int moduleCount = 25;
    constexpr int moduleSize = 10;
    constexpr int renderedSize = moduleCount * moduleSize;
    constexpr int left = (QR_WIDTH - renderedSize) / 2;
    constexpr int top = (QR_HEIGHT - renderedSize) / 2;
    for (int moduleY = 0; moduleY < moduleCount; moduleY++) {
        for (int moduleX = 0; moduleX < moduleCount; moduleX++) {
            if (QR_SELF_TEST_MODULES[moduleY * moduleCount + moduleX] != '1') continue;
            for (int y = 0; y < moduleSize; y++) {
                memset(qrLuma + static_cast<size_t>(top + moduleY * moduleSize + y) *
                                    QR_WIDTH + left + moduleX * moduleSize,
                       0, moduleSize);
            }
        }
    }

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
                memset(qrLuma, 255, sizeof(qrLuma));
                return true;
            }
        }
    }
    memset(qrLuma, 255, sizeof(qrLuma));
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

void processAnalysisJpeg(const uint8_t* jpeg, size_t length, uint32_t frameSequence)
{
    if (!state.ready || !jpeg || !length) return;
    const uint32_t started = millis();
    memset(qrLuma, 255, sizeof(qrLuma));
    memset(&decodeContext, 0, sizeof(decodeContext));
    decodeContext.darkest = 255;

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    decodeContext.objectCount = state.objectCount;
    memcpy(decodeContext.objects, state.objects, sizeof(state.objects));
    xSemaphoreGive(stateMutex);

    const bool sampleObjectColors = decodeContext.objectCount > 0;
    bool decodeOk = jpegDecoder.openFLASH(const_cast<uint8_t*>(jpeg), length,
                                          drawJpegBlock) != 0;
    if (decodeOk) {
        jpegDecoder.setPixelType(sampleObjectColors ? RGB565_LITTLE_ENDIAN
                                                    : EIGHT_BIT_GRAYSCALE);
        decodeOk = jpegDecoder.getWidth() == QR_WIDTH &&
                   jpegDecoder.getHeight() == QR_HEIGHT &&
                   jpegDecoder.decode(0, 0, 0) != 0;
        jpegDecoder.close();
    }

    if (decodeOk) {
        int scanDetail = 0;
        setQrDensity((frameSequence & 1U) ? 1 : 2);
        const bool decoded = recordQrResult(millis(), scanDetail);
        if (!decoded && scanDetail == -2) {
            stretchContrast();
            setQrDensity(1);
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            state.qrEnhancedScans++;
            xSemaphoreGive(stateMutex);
            recordQrResult(millis(), scanDetail);
        }
    }

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    state.frameSequence = frameSequence;
    state.analyzedFrames++;
    if (!decodeOk) state.jpegDecodeErrors++;
    if (decodeOk) state.qrCheckedFrames++;
    if (decodeOk && sampleObjectColors) applyObjectColors();
    if (decodeOk && decodeContext.decodedPixels) {
        state.qrDarkest = decodeContext.darkest;
        state.qrBrightest = decodeContext.brightest;
        state.qrMean = static_cast<uint8_t>(
            decodeContext.luminanceSum / decodeContext.decodedPixels);
    }
    state.lastProcessMs = millis() - started;
    state.maxProcessMs = std::max(state.maxProcessMs, state.lastProcessMs);
    xSemaphoreGive(stateMutex);
}

void analysisTask(void*)
{
    uint32_t sequence = 0;
    for (;;) {
        uint32_t imageAddress = 0;
        uint32_t imageLength = 0;
        Camera.getImage(QR_CHANNEL, &imageAddress, &imageLength);
        processAnalysisJpeg(reinterpret_cast<const uint8_t*>(imageAddress),
                            imageLength, ++sequence);
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
    if (!stateMutex) return false;
    state.enabled = true;

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
    state.qrSelfTestPassed = runQrSelfTest();
    Serial.print("QR decoder self-test: ");
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
    Camera.channelBegin(QR_CHANNEL);
    Camera.channelBegin(NN_CHANNEL);

    state.ready = true;
    if (xTaskCreate(analysisTask, "CameraAnalysis", 10 * 1024, nullptr, 1,
                    nullptr) != pdPASS) {
        state.ready = false;
        return false;
    }
    Serial.println("On-device vision ready: dedicated every-frame QR/color + YOLOv4-tiny");
    return true;
}

void getStatus(Status& status)
{
    if (!stateMutex) {
        memset(&status, 0, sizeof(status));
        status.enabled = true;
        return;
    }
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    status = state;
    xSemaphoreGive(stateMutex);
}

}  // namespace OnDeviceVision

#else

namespace OnDeviceVision {

void configureCamera() {}
bool begin() { return true; }
void getStatus(Status& status)
{
    memset(&status, 0, sizeof(status));
}

}  // namespace OnDeviceVision

#endif
