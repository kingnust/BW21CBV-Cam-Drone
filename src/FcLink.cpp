#include "FcLink.h"

#include "AppConfig.h"
#include "DroneProtoCameraProtocol.h"

namespace FcLink {

#if BW21CAM_ENABLE_FC_LINK

namespace {

constexpr uint8_t MSP_API_VERSION = 1;
constexpr uint32_t API_QUERY_INTERVAL_MS = 1000;
constexpr uint32_t LINK_TIMEOUT_MS = 2500;
constexpr uint32_t QR_RETRY_INTERVAL_MS = 500;
namespace Protocol = DroneProtoCameraProtocol;

enum ParseState : uint8_t {
    SYNC_DOLLAR,
    SYNC_PROTOCOL,
    V1_DIRECTION,
    V1_SIZE,
    V1_COMMAND,
    V1_PAYLOAD,
    V1_CHECKSUM,
    V2_DIRECTION,
    V2_HEADER,
    V2_PAYLOAD,
    V2_CHECKSUM
};

ParseState parseState = SYNC_DOLLAR;
uint16_t parseSize = 0;
uint16_t parseCommand = 0;
uint16_t parseIndex = 0;
uint8_t parseChecksum = 0;
uint8_t parseHeader[5] = {};
uint8_t parsePayload[128] = {};

uint32_t lastQueryMs = 0;
uint32_t lastResponseMs = 0;
uint32_t requests = 0;
uint32_t responses = 0;
uint32_t checksumErrors = 0;
uint8_t protocolVersion = 0;
uint8_t majorVersion = 0;
uint8_t minorVersion = 0;

uint16_t qrSequence = 0;
uint8_t qrPayload[Protocol::QrMaxPayloadSize] = {};
uint8_t qrPayloadLength = 0;
uint32_t lastQrSendMs = 0;
uint32_t qrSends = 0;
uint32_t qrAcks = 0;
uint32_t qrRejects = 0;
bool qrPending = false;

void sendApiRequest()
{
    static const uint8_t request[] = {'$', 'M', '<', 0, MSP_API_VERSION, MSP_API_VERSION};
    Serial1.write(request, sizeof(request));
    Serial1.flush();
    requests++;
}

void sendMspV2(uint16_t command, const uint8_t* payload, uint16_t length)
{
    uint8_t frame[Protocol::QrMaxFrameSize] = {};
    size_t frameLength = 0;
    if (!Protocol::buildMspV2Frame(command, payload, length,
                                   frame, sizeof(frame), frameLength)) {
        return;
    }
    Serial1.write(frame, frameLength);
    Serial1.flush();
}

void acceptV1Frame(uint32_t now)
{
    if (parseCommand == MSP_API_VERSION && parseSize >= 3) {
        protocolVersion = parsePayload[0];
        majorVersion = parsePayload[1];
        minorVersion = parsePayload[2];
        responses++;
        lastResponseMs = now;
    }
}

void acceptV2Frame(uint32_t now)
{
    if (parseCommand != Protocol::Msp2CameraQr || !qrPending) {
        return;
    }

    uint8_t status = 0xff;
    if (!Protocol::parseQrAck(parsePayload, parseSize, qrPayload[0], qrSequence, status)) {
        return;
    }

    if (status == 0 || status == 1) {
        qrAcks++;
    } else {
        qrRejects++;
    }
    qrPending = false;
    lastResponseMs = now;
}

void parseByte(uint8_t value, uint32_t now)
{
    switch (parseState) {
        case SYNC_DOLLAR:
            parseState = value == '$' ? SYNC_PROTOCOL : SYNC_DOLLAR;
            break;
        case SYNC_PROTOCOL:
            parseState = value == 'M' ? V1_DIRECTION
                                      : (value == 'X' ? V2_DIRECTION : SYNC_DOLLAR);
            break;
        case V1_DIRECTION:
            parseState = (value == '>' || value == '!') ? V1_SIZE : SYNC_DOLLAR;
            break;
        case V1_SIZE:
            parseSize = value;
            parseIndex = 0;
            parseChecksum = value;
            parseState = parseSize <= sizeof(parsePayload) ? V1_COMMAND : SYNC_DOLLAR;
            break;
        case V1_COMMAND:
            parseCommand = value;
            parseChecksum ^= value;
            parseState = parseSize ? V1_PAYLOAD : V1_CHECKSUM;
            break;
        case V1_PAYLOAD:
            parsePayload[parseIndex++] = value;
            parseChecksum ^= value;
            if (parseIndex >= parseSize) {
                parseState = V1_CHECKSUM;
            }
            break;
        case V1_CHECKSUM:
            if (value == parseChecksum) {
                acceptV1Frame(now);
            } else {
                checksumErrors++;
            }
            parseState = SYNC_DOLLAR;
            break;
        case V2_DIRECTION:
            parseIndex = 0;
            parseChecksum = 0;
            parseState = (value == '>' || value == '!') ? V2_HEADER : SYNC_DOLLAR;
            break;
        case V2_HEADER:
            parseHeader[parseIndex++] = value;
            parseChecksum = Protocol::crc8DvbS2(parseChecksum, value);
            if (parseIndex == sizeof(parseHeader)) {
                parseCommand = parseHeader[1] |
                               (static_cast<uint16_t>(parseHeader[2]) << 8);
                parseSize = parseHeader[3] |
                            (static_cast<uint16_t>(parseHeader[4]) << 8);
                parseIndex = 0;
                parseState = parseSize <= sizeof(parsePayload)
                                 ? (parseSize ? V2_PAYLOAD : V2_CHECKSUM)
                                 : SYNC_DOLLAR;
            }
            break;
        case V2_PAYLOAD:
            parsePayload[parseIndex++] = value;
            parseChecksum = Protocol::crc8DvbS2(parseChecksum, value);
            if (parseIndex >= parseSize) {
                parseState = V2_CHECKSUM;
            }
            break;
        case V2_CHECKSUM:
            if (value == parseChecksum) {
                acceptV2Frame(now);
            } else {
                checksumErrors++;
            }
            parseState = SYNC_DOLLAR;
            break;
    }
}

}  // namespace

void begin()
{
    Serial1.begin(BW21CAM_FC_BAUD);
    Serial.println("FC link enabled on Serial1 D21(TX)/D22(RX)");
}

void update()
{
    const uint32_t now = millis();
    if (now - lastQueryMs >= API_QUERY_INTERVAL_MS) {
        lastQueryMs = now;
        sendApiRequest();
    }

    if (qrPending && now - lastQrSendMs >= QR_RETRY_INTERVAL_MS) {
        lastQrSendMs = now;
        sendMspV2(Protocol::Msp2CameraQr, qrPayload, qrPayloadLength);
        qrSends++;
    }

    while (Serial1.available() > 0) {
        parseByte(static_cast<uint8_t>(Serial1.read()), now);
    }
}

bool publishQr(const QrObservation& observation)
{
    if (!observation.payload || !observation.payload[0] || qrPending) {
        return false;
    }

    qrSequence++;
    size_t payloadLength = 0;
    if (!Protocol::buildQrPayload(observation, qrSequence, qrPayload,
                                  sizeof(qrPayload), payloadLength)) {
        return false;
    }
    qrPayloadLength = static_cast<uint8_t>(payloadLength);
    lastQrSendMs = 0;
    qrPending = true;
    return true;
}

bool connected()
{
    return lastResponseMs && millis() - lastResponseMs <= LINK_TIMEOUT_MS;
}

uint32_t requestCount() { return requests; }
uint32_t responseCount() { return responses; }
uint32_t checksumErrorCount() { return checksumErrors; }
uint32_t qrSendCount() { return qrSends; }
uint32_t qrAckCount() { return qrAcks; }
uint32_t qrRejectCount() { return qrRejects; }
uint8_t apiMajor() { return majorVersion; }
uint8_t apiMinor() { return minorVersion; }

#else

void begin() {}
void update() {}
bool publishQr(const QrObservation&) { return false; }
bool connected() { return false; }
uint32_t requestCount() { return 0; }
uint32_t responseCount() { return 0; }
uint32_t checksumErrorCount() { return 0; }
uint32_t qrSendCount() { return 0; }
uint32_t qrAckCount() { return 0; }
uint32_t qrRejectCount() { return 0; }
uint8_t apiMajor() { return 0; }
uint8_t apiMinor() { return 0; }

#endif

}  // namespace FcLink
