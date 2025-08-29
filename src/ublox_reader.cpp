#include "ublox_reader.h"
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

// -------------------- UBX helpers --------------------

static void ubxChecksum(const uint8_t* data, size_t len, uint8_t& ckA, uint8_t& ckB) {
  // checksum over Class, ID, Length(2), Payload
  ckA = 0; ckB = 0;
  for (size_t i = 0; i < len; ++i) {
    ckA = ckA + data[i];
    ckB = ckB + ckA;
  }
}

static bool looksLikeFullUBX(const uint8_t* buf) {
  return buf[0] == 0xB5 && buf[1] == 0x62;
}

// Extract pointers and lengths from a buffer that is either:
//  - full UBX packet starting with 0xB5 0x62, or
//  - "short" form starting at Class byte (no sync chars).
// Returns true on success and fills out references.
static bool getUbxSlices(const uint8_t* buf, size_t size,
                         const uint8_t*& clsIdLenPayload, size_t& clsIdLenPayloadLen,
                         const uint8_t*& payload, uint16_t& payloadLen,
                         uint8_t& msgClass, uint8_t& msgId,
                         const uint8_t*& ck) {
  if (size < 6) return false;

  if (looksLikeFullUBX(buf)) {
    // [0]=B5 [1]=62 [2]=class [3]=id [4..5]=len [6..] payload [..] ckA ckB
    msgClass = buf[2];
    msgId    = buf[3];
    payloadLen = static_cast<uint16_t>(buf[4] | (buf[5] << 8));

    const size_t packetLen = 6 + payloadLen + 2; // header + payload + checksum
    if (size < packetLen) return false;

    payload = buf + 6;
    ck = buf + 6 + payloadLen;
    clsIdLenPayload      = buf + 2;                 // start at class
    clsIdLenPayloadLen   = 4 + payloadLen;          // class,id,len(2),payload
    return true;
  } else {
    // Assume buffer starts at Class byte: [0]=class [1]=id [2..3]=len [4..] payload [..] ckA ckB (optional)
    msgClass = buf[0];
    msgId    = buf[1];
    payloadLen = static_cast<uint16_t>(buf[2] | (buf[3] << 8));

    if (size < 4 + payloadLen) return false;

    payload = buf + 4;
    clsIdLenPayload      = buf;                     // already at class
    clsIdLenPayloadLen   = 4 + payloadLen;          // class,id,len(2),payload
    // checksum may or may not be present in this short representation
    ck = (size >= 4 + payloadLen + 2) ? (buf + 4 + payloadLen) : nullptr;
    return true;
  }
}

// -------------------- Message parsers --------------------

static int NAV_POSLLH(const uint8_t* p, classId* gps) {
  // Offsets per datasheet: iTOW[0..3] lon[4..7] lat[8..11] height[12..15] hMSL[16..19] hAcc[20..23] vAcc[24..27]
  memcpy(&gps->iTOW,   p + 0,  4);
  memcpy(&gps->lon,    p + 4,  4);
  memcpy(&gps->lat,    p + 8,  4);
  memcpy(&gps->height, p + 12, 4);
  memcpy(&gps->hMSL,   p + 16, 4);
  memcpy(&gps->hAcc,   p + 20, 4);
  memcpy(&gps->vAcc,   p + 24, 4);
  return 0;
}

static int NAV_PVT(const uint8_t* p, uint16_t len, classId* gps) {
  // We only need lon/lat/height/hMSL; check length is enough
  // NAV-PVT payload offsets (u-blox): lon @24 (I4), lat @28 (I4), height @32 (I4), hMSL @36 (I4), hAcc @40 (U4), vAcc @44 (U4)
  if (len < 48) return 1;
  memcpy(&gps->lon,    p + 24, 4);
  memcpy(&gps->lat,    p + 28, 4);
  memcpy(&gps->height, p + 32, 4);
  memcpy(&gps->hMSL,   p + 36, 4);
  memcpy(&gps->hAcc,   p + 40, 4);
  memcpy(&gps->vAcc,   p + 44, 4);
  // iTOW at offset 0
  memcpy(&gps->iTOW,   p + 0,  4);
  return 0;
}

// -------------------- Hex reader --------------------

static vector<uint8_t> hexToBytes(const string& rawHex) {
  vector<uint8_t> bytes;
  stringstream ss(rawHex);
  string token;
  while (ss >> token) {
    bytes.push_back(static_cast<uint8_t>(stoul(token, nullptr, 16)));
  }
  return bytes;
}

// -------------------- Public functions (headers) --------------------

// Backwards-compatible: tries to decode assuming the pointer points either to
// the full UBX packet (with 0xB5 0x62) or directly at Class byte.
// Because we don’t have a length here, checksum is only validated if present.
int decodeUBX(uint8_t* buffer, classId* gps) {
  // Attempt to read a reasonable amount by trusting the length field inside.
  // We'll assume there are at least 6+payload+2 bytes available in memory from 'buffer'.
  const uint8_t* clsIdLenPayload = nullptr;
  size_t clsIdLenPayloadLen = 0;
  const uint8_t* payload = nullptr;
  uint16_t payloadLen = 0;
  uint8_t msgClass = 0, msgId = 0;
  const uint8_t* ck = nullptr;

  // We don't know 'size'. Use a large sentinel for safe arithmetic; slices will check internally using the length field.
  // This relies on caller supplying a whole packet in memory (which is the case in readUbloxFile below).
  size_t pretendSize = 4096;

  if (!getUbxSlices(buffer, pretendSize, clsIdLenPayload, clsIdLenPayloadLen,
                    payload, payloadLen, msgClass, msgId, ck)) {
    return 1;
  }

  // If checksum bytes are present, validate them.
  if (ck) {
    uint8_t ckA = 0, ckB = 0;
    ubxChecksum(clsIdLenPayload, clsIdLenPayloadLen, ckA, ckB);
    if (ck[0] != ckA || ck[1] != ckB) {
      cerr << "UBX checksum failed\n";
      return 1;
    }
  }

  if (msgClass == 0x01 && msgId == 0x02) {           // NAV-POSLLH
    return NAV_POSLLH(payload, gps);
  } else if (msgClass == 0x01 && msgId == 0x07) {    // NAV-PVT
    return NAV_PVT(payload, payloadLen, gps);
  }

  return 1; // unsupported message
}

GPS gpsFromData(const classId& gps) {
  GPS out;
  out.lat    = gps.lat * 1e-7;     // degrees
  out.lon    = gps.lon * 1e-7;     // degrees
  out.height = gps.height / 1000.0; // meters (mm -> m)
  return out;
}

pair<GPS, GPS> readUbloxFile(const string& filename) {
  ifstream file(filename);
  if (!file.is_open()) {
    cerr << "Error: cannot open file " << filename << endl;
    return {{0.0, 0.0}, {0.0, 0.0}};
  }

  string rawStart, rawGoal;
  getline(file, rawStart);
  getline(file, rawGoal);

  cout << "Raw UBX Start: " << rawStart << endl;
  cout << "Raw UBX Goal : " << rawGoal << endl;

  vector<uint8_t> startBytes = hexToBytes(rawStart);
  vector<uint8_t> goalBytes  = hexToBytes(rawGoal);

  classId gpsStartData{}, gpsGoalData{};

  // Prefer decoding with full size (so checksum can be validated if present)
  auto decodeWithSize = [](const vector<uint8_t>& bytes, classId* out) -> bool {
    if (bytes.size() < 4) return false;

    const uint8_t* clsIdLenPayload = nullptr;
    size_t clsIdLenPayloadLen = 0;
    const uint8_t* payload = nullptr;
    uint16_t payloadLen = 0;
    uint8_t msgClass = 0, msgId = 0;
    const uint8_t* ck = nullptr;

    if (!getUbxSlices(bytes.data(), bytes.size(), clsIdLenPayload, clsIdLenPayloadLen,
                      payload, payloadLen, msgClass, msgId, ck)) {
      return false;
    }

    // Validate checksum if present (present for full packets and for short form when caller included it)
    if (ck) {
      uint8_t ckA = 0, ckB = 0;
      ubxChecksum(clsIdLenPayload, clsIdLenPayloadLen, ckA, ckB);
      if (ck[0] != ckA || ck[1] != ckB) {
        cerr << "UBX checksum failed (line)\n";
        return false;
      }
    }

    if (msgClass == 0x01 && msgId == 0x02) {
      return NAV_POSLLH(payload, out) == 0;
    } else if (msgClass == 0x01 && msgId == 0x07) {
      return NAV_PVT(payload, payloadLen, out) == 0;
    }
    cerr << "Unsupported UBX message: class 0x" << hex << int(msgClass)
         << " id 0x" << int(msgId) << dec << "\n";
    return false;
  };

  bool okStart = decodeWithSize(startBytes, &gpsStartData);
  bool okGoal  = decodeWithSize(goalBytes,  &gpsGoalData);

  // Fallback to legacy decode if needed (when lines omit checksum but we still want to try)
  if (!okStart && !startBytes.empty()) {
    okStart = (decodeUBX(startBytes.data(), &gpsStartData) == 0);
  }
  if (!okGoal && !goalBytes.empty()) {
    okGoal = (decodeUBX(goalBytes.data(), &gpsGoalData) == 0);
  }

  if (!okStart) cerr << "Failed to decode START UBX line\n";
  if (!okGoal)  cerr << "Failed to decode GOAL UBX line\n";

  GPS startGPS = gpsFromData(gpsStartData);
  GPS goalGPS  = gpsFromData(gpsGoalData);

  file.close();
  return {startGPS, goalGPS};
}

//check readme file for the logic and breif explanation of the code
















