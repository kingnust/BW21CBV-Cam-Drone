#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace DroneProtoCameraProtocol {

constexpr uint16_t Msp2CameraQr = 0x3001;
constexpr size_t QrMaxTextLength = 96;
constexpr uint8_t ProtocolVersion = 2;
constexpr uint8_t MessageQr = 1;
constexpr size_t QrHeaderSize = 16;
constexpr size_t QrMaxPayloadSize = QrHeaderSize + QrMaxTextLength;
constexpr size_t MspV2FrameOverhead = 9;
constexpr size_t QrMaxFrameSize = QrMaxPayloadSize + MspV2FrameOverhead;

constexpr uint8_t QrFlagGeometryValid = 1 << 0;
constexpr uint8_t QrFlagFullResolution = 1 << 1;
constexpr uint8_t QrFlagMirrored = 1 << 2;
constexpr uint8_t QrFlagZbarFallback = 1 << 3;

struct QrObservation {
  const char *payload;
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

inline void writeU16(uint8_t *destination, uint16_t value)
{
  destination[0] = static_cast<uint8_t>(value);
  destination[1] = static_cast<uint8_t>(value >> 8);
}

inline uint8_t crc8DvbS2(uint8_t crc, uint8_t value)
{
  crc ^= value;
  for(uint8_t bit = 0; bit < 8; ++bit)
  {
    crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                       : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}

inline bool buildQrPayload(const QrObservation& observation, uint16_t sequence,
                           uint8_t *output, size_t capacity, size_t& outputLength)
{
  outputLength = 0;
  if(output == nullptr || capacity < QrHeaderSize || observation.payload == nullptr ||
     observation.payload[0] == '\0')
  {
    return false;
  }

  size_t length = std::strlen(observation.payload);
  if(length > QrMaxTextLength) length = QrMaxTextLength;
  if(capacity < QrHeaderSize + length) return false;

  output[0] = ProtocolVersion;
  output[1] = MessageQr;
  writeU16(&output[2], sequence);
  output[4] = static_cast<uint8_t>(length);
  output[5] = (observation.geometryValid ? QrFlagGeometryValid : 0) |
              (observation.fullResolution ? QrFlagFullResolution : 0) |
              (observation.mirrored ? QrFlagMirrored : 0) |
              (observation.zbarFallback ? QrFlagZbarFallback : 0);
  writeU16(&output[6], observation.centerXPermille);
  writeU16(&output[8], observation.centerYPermille);
  writeU16(&output[10], observation.sidePermille);
  writeU16(&output[12], observation.areaPermille);
  writeU16(&output[14], static_cast<uint16_t>(observation.rotationCdeg));
  std::memcpy(&output[QrHeaderSize], observation.payload, length);
  outputLength = QrHeaderSize + length;
  return true;
}

inline bool buildMspV2Frame(uint16_t command, const uint8_t *payload, size_t length,
                            uint8_t *output, size_t capacity, size_t& outputLength,
                            char direction = '<')
{
  outputLength = 0;
  if(output == nullptr || (payload == nullptr && length != 0) || length > 0xffff ||
     capacity < length + MspV2FrameOverhead ||
     (direction != '<' && direction != '>' && direction != '!'))
  {
    return false;
  }

  output[0] = '$';
  output[1] = 'X';
  output[2] = static_cast<uint8_t>(direction);
  output[3] = 0;
  output[4] = static_cast<uint8_t>(command);
  output[5] = static_cast<uint8_t>(command >> 8);
  output[6] = static_cast<uint8_t>(length);
  output[7] = static_cast<uint8_t>(length >> 8);

  uint8_t crc = 0;
  for(size_t i = 3; i < 8; ++i) crc = crc8DvbS2(crc, output[i]);
  if(length != 0) std::memcpy(&output[8], payload, length);
  for(size_t i = 0; i < length; ++i) crc = crc8DvbS2(crc, payload[i]);
  output[8 + length] = crc;
  outputLength = length + MspV2FrameOverhead;
  return true;
}

inline bool parseQrAck(const uint8_t *payload, size_t length,
                       uint8_t expectedVersion, uint16_t expectedSequence,
                       uint8_t& status)
{
  if(payload == nullptr || length < 4) return false;
  const uint16_t sequence = static_cast<uint16_t>(payload[2]) |
                            (static_cast<uint16_t>(payload[3]) << 8);
  if(payload[0] != expectedVersion || sequence != expectedSequence) return false;
  status = payload[1];
  return true;
}

} // namespace DroneProtoCameraProtocol
