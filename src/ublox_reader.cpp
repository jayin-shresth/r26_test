#include "ublox_reader.h"
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;
//workspace 
static int NAV_POSLLH(uint8_t *buffer, classId *gps) {
  memcpy(&gps->iTOW, buffer + 0, 4);
  memcpy(&gps->lon, buffer + 4, 4);
  memcpy(&gps->lat, buffer + 8, 4);
  memcpy(&gps->height, buffer + 12, 4);
  memcpy(&gps->hMSL, buffer + 16, 4);
  memcpy(&gps->hAcc, buffer + 20, 4);
  memcpy(&gps->vAcc, buffer + 24, 4);
  return 0;
}

static vector<uint8_t> hexToBytes(const string &rawHex) {
  vector<uint8_t> bytes;
  stringstream ss(rawHex);
  string token;
  while (ss >> token) {
    bytes.push_back(static_cast<uint8_t>(stoul(token, nullptr, 16)));
  }
  return bytes;
}

int decodeUBX(uint8_t *buffer, classId *gps) {
    // buffer[0] = class, buffer[1] = id, buffer[2-3] = length
    if (buffer[0] == 0x01 && buffer[1] == 0x02) { // NAV-POSLLH
        // Payload starts right after class+id+len = 4 bytes
        return NAV_POSLLH(buffer + 4, gps);
    }
    return 1; // not supported
}

GPS gpsFromData(const classId &gps) {
  GPS out;
  out.lat = gps.lat * 1e-7;
  out.lon = gps.lon * 1e-7;
  out.height = gps.height / 1000.0;
  return out;
}

pair<GPS, GPS> readUbloxFile(const string &filename) {
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
  vector<uint8_t> goalBytes = hexToBytes(rawGoal);

  classId gpsStartData, gpsGoalData;
  decodeUBX(startBytes.data(), &gpsStartData);
  decodeUBX(goalBytes.data(), &gpsGoalData);

  GPS startGPS = gpsFromData(gpsStartData);
  GPS goalGPS = gpsFromData(gpsGoalData);

  file.close();

  return {startGPS, goalGPS};
}