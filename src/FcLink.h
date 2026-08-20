#pragma once

#include <Arduino.h>

namespace FcLink {

struct QrObservation {
    const char* payload;
    bool geometryValid;
    bool fullResolution;
    bool mirrored;
    bool zbarFallback;
    uint16_t centerXPermille;
    uint16_t centerYPermille;
    uint16_t sidePermille;
    uint16_t areaPermille;
    int16_t rotationCdeg;
};

void begin();
void update();
bool publishQr(const QrObservation& observation);

bool connected();
uint32_t requestCount();
uint32_t responseCount();
uint32_t checksumErrorCount();
uint32_t qrSendCount();
uint32_t qrAckCount();
uint32_t qrRejectCount();
uint8_t apiMajor();
uint8_t apiMinor();

}  // namespace FcLink
