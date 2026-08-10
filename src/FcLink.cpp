#include "FcLink.h"

#include "AppConfig.h"

namespace FcLink {

#if BW21CAM_ENABLE_FC_LINK

namespace {

constexpr uint8_t MSP_API_VERSION = 1;
constexpr uint16_t MSP2_CAMERA_QR = 0x3001;
constexpr uint32_t API_QUERY_INTERVAL_MS = 1000;
constexpr uint32_t LINK_TIMEOUT_MS = 2500;
constexpr uint32_t QR_RETRY_INTERVAL_MS = 500;
constexpr size_t QR_MAX_PAYLOAD = 96;

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
uint8_t qrPayload[5 + QR_MAX_PAYLOAD] = {};
uint8_t qrPayloadLength = 0;
uint32_t lastQrSendMs = 0;
uint32_t qrSends = 0;
uint32_t qrAcks = 0;
uint32_t qrRejects = 0;
bool qrPending = false;

uint8_t crc8DvbS2(uint8_t crc, uint8_t value)
{
    crc ^= value;
    for (uint8_t bit = 0; bit < 8; bit++) {
        crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                           : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

void sendApiRequest()
{
    static const uint8_t request[] = {'$', 'M', '<', 0, MSP_API_VERSION, MSP_API_VERSION};
    Serial1.write(request, sizeof(request));
    Serial1.flush();
    requests++;
}

void sendMspV2(uint16_t command, const uint8_t* payload, uint16_t length)
{
    const uint8_t header[] = {
        '$', 'X', '<', 0,
        static_cast<uint8_t>(command),
        static_cast<uint8_t>(command >> 8),
        static_cast<uint8_t>(length),
        static_cast<uint8_t>(length >> 8)
    };

    uint8_t crc = 0;
    for (size_t i = 3; i < sizeof(header); i++) {
        crc = crc8DvbS2(crc, header[i]);
    }
    Serial1.write(header, sizeof(header));
    for (uint16_t i = 0; i < length; i++) {
        crc = crc8DvbS2(crc, payload[i]);
    }
    Serial1.write(payload, length);
    Serial1.write(crc);
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
    if (parseCommand != MSP2_CAMERA_QR || parseSize < 4 || !qrPending) {
        return;
    }

    const uint16_t sequence = parsePayload[2] |
                              (static_cast<uint16_t>(parsePayload[3]) << 8);
    if (parsePayload[0] != 1 || sequence != qrSequence) {
        return;
    }

    if (parsePayload[1] == 0 || parsePayload[1] == 1) {
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
            parseChecksum = crc8DvbS2(parseChecksum, value);
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
            parseChecksum = crc8DvbS2(parseChecksum, value);
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
        sendMspV2(MSP2_CAMERA_QR, qrPayload, qrPayloadLength);
        qrSends++;
    }

    while (Serial1.available() > 0) {
        parseByte(static_cast<uint8_t>(Serial1.read()), now);
    }
}

bool publishQr(const char* payload)
{
    if (!payload || !payload[0] || qrPending) {
        return false;
    }

    size_t length = strlen(payload);
    if (length > QR_MAX_PAYLOAD) {
        length = QR_MAX_PAYLOAD;
    }

    qrSequence++;
    qrPayload[0] = 1;
    qrPayload[1] = 1;
    qrPayload[2] = static_cast<uint8_t>(qrSequence);
    qrPayload[3] = static_cast<uint8_t>(qrSequence >> 8);
    qrPayload[4] = static_cast<uint8_t>(length);
    memcpy(&qrPayload[5], payload, length);
    qrPayloadLength = static_cast<uint8_t>(5 + length);
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
bool publishQr(const char*) { return false; }
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

