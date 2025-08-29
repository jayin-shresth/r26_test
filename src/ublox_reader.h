#ifndef UBLOX_READER_H
#define UBLOX_READER_H

#include <cstdint>
#include <string>
#include <utility>

// store GPS NAV data
struct GPS {
  double lat;    // degrees
  double lon;    // degrees
  double height; // meters
};

// struct for GPS class IDs
struct classId {
  uint32_t iTOW;
  int32_t lon;    // in 1e-7 degrees
  int32_t lat;    // in 1e-7 degrees
  int32_t height; // mm
  int32_t hMSL;   // mm
  uint32_t hAcc;  // mm
  uint32_t vAcc;  // mm
};

/*Decode UBX data
 *
 * param: uint*_t buffer-> raw gps data
 * 	  classId *gps-> store decoded data
 *
 * ret: 0 if successfull else 1*/
int decodeUBX(uint8_t *buffer, classId *gps);

// Convert classId to GPS struct (degrees, meters)
GPS gpsFromData(const classId &gps);

// Read UBX GPS data from file containing two hex lines (start and goal)
std::pair<GPS, GPS> readUbloxFile(const std::string &filename);

#endif // UBLOX_READER_H
