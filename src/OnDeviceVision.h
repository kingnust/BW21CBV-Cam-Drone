#pragma once

#include <Arduino.h>

#include "AppConfig.h"

namespace OnDeviceVision {

constexpr size_t QR_PAYLOAD_CAPACITY = 193;
constexpr size_t OBJECT_NAME_CAPACITY = 24;
constexpr size_t COLOR_NAME_CAPACITY = 12;

struct ObjectResult {
    char name[OBJECT_NAME_CAPACITY];
    char color[COLOR_NAME_CAPACITY];
    uint8_t score;
    uint8_t colorConfidence;
    float xMin;
    float yMin;
    float xMax;
    float yMax;
};

struct QrGeometry {
    bool valid;
    bool fullResolution;
    bool mirrored;
    bool zbarFallback;
    uint16_t centerXPermille;
    uint16_t centerYPermille;
    uint16_t sidePermille;
    uint16_t areaPermille;
    int16_t rotationCdeg;
};

struct Status {
    bool enabled;
    bool ready;
    bool qrSelfTestPassed;
    bool quircReady;
    bool quircSelfTestPassed;
    bool quircFastSelfTestPassed;
    bool quircFullSelfTestPassed;
    bool zbarSelfTestPassed;
    uint32_t frameSequence;
    uint32_t analyzedFrames;
    uint32_t qrCheckedFrames;
    uint32_t qrScanPasses;
    uint32_t qrEnhancedScans;
    uint32_t jpegDecodeErrors;
    uint32_t qrCandidates;
    uint32_t qrNoFinderCenters;
    uint32_t qrDecodes;
    uint32_t qrDecodeErrors;
    uint32_t qrDuplicates;
    uint32_t quircScanPasses;
    uint32_t quircFastScans;
    uint32_t quircFullScans;
    uint32_t quircCandidates;
    uint32_t quircDecodeErrors;
    uint32_t quircMirroredDecodes;
    uint32_t zbarFallbackScans;
    uint32_t qrSequence;
    uint32_t qrSeenAtMs;
    uint32_t qrPublishedAtMs;
    QrGeometry qrGeometry;
    uint32_t lastProcessMs;
    uint32_t maxProcessMs;
    uint32_t lastJpegDecodeMs;
    uint32_t lastQrScanMs;
    int8_t qrLastScanDetail;
    uint8_t qrDarkest;
    uint8_t qrMean;
    uint8_t qrBrightest;
    char qrPayload[QR_PAYLOAD_CAPACITY];
    uint32_t objectSequence;
    uint32_t objectSeenAtMs;
    uint32_t yoloFrames;
    uint8_t objectCount;
    ObjectResult objects[BW21CAM_VISION_MAX_OBJECTS];
};

void configureCamera();
bool begin();
bool setEnabled(bool enabled);
void getStatus(Status& status);
bool lockQrJpeg(const uint8_t*& jpeg, size_t& length);
void unlockQrJpeg();

}  // namespace OnDeviceVision
