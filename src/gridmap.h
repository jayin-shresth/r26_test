#ifndef GRIDMAP_H
#define GRIDMAP_H

#include "ublox_reader.h"
#include <cmath>
#include <utility>
#include <vector>

class Gridmapper {
public:
  Gridmapper(GPS origin, double cellsize, int rows, int cols);

  /* transform gps to gridmap
   * 
   * param: const GPS &point-> start and goal GPS coord
   *
   * ret: pair<int, int> transformed grid coords*/
  std::pair<int, int> gpstogrid(const GPS &point) const;

  // generate and print gridmap
  const std::vector<std::vector<bool>> &getGrid() const;

private:
  GPS origin;
  double cellsize;
  int rows, cols;
  std::vector<std::vector<bool>> grid;

  static double deg2rad(double deg);
  bool isvalid(int row, int col) const;
  void makemap();
  void printgrid() const;
};


#endif