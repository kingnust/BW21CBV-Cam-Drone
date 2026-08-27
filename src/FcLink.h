#pragma once

#include <Arduino.h>
#include "DroneProtoCameraProtocol.h"

namespace FcLink {

using QrObservation = DroneProtoCameraProtocol::QrObservation;

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
