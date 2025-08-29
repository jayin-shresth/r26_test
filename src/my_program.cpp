#define _USE_MATH_DEFINES
#include "gridmap.h"
#include "odometry.h"
#include "planning.h"
#include "ublox_reader.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>
#include <sstream>

using namespace std;

// Helper to convert angle to unit direction

pair<double, double> directionFromAngle(double angle_deg) {
  double rad = angle_deg * M_PI / 180.0;
  return {cos(rad), sin(rad)};

}

int main(int argc, char *argv[]) {


  if (argc < 3) {
    cerr << "Usage: " << argv[0] << " <gps_data_file> <odom_commands_output_file>" << endl;
    return 1;
}

  // store file path to GPS data
  string gps_data = argv[1];

  // file path to store result
  string odom_commands = argv[2];

  // decode GPS data from file
  auto result = readUbloxFile(gps_data);
  if(static_cast<int>(result.first.lat)==0 && static_cast<int>(result.first.lon)==0 && static_cast<int>(result.second.lat)==0 && static_cast<int>(result.second.lon)==0)
  {
    cout<<"Error: Invalid GPS Coordinates"<<endl;
    return 1;

  }

  cout << "Start -> Lat: " << result.first.lat << " Lon: " << result.first.lon
       << endl;
  cout << "Goal  -> Lat: " << result.second.lat << " Lon: " << result.second.lon
       << endl;

  // Initialize Gridmapper with start as origin
  GPS origin = {result.first.lat, result.first.lon};
  double cellsize = 1.0; // meters per grid cell    
  int rows = 10, cols = 10;
  Gridmapper grid(origin, cellsize, rows, cols);

  // Convert start and goal GPS to grid coordinates
  pair<int, int> start = grid.gpstogrid(result.first);
  pair<int, int> goal = grid.gpstogrid(result.second);

  cout << "Start (grid) -> (" << start.first << "," << start.second << ")"
       << endl;
  cout << "Goal  (grid) -> (" << goal.first << "," << goal.second << ")"
       << endl;

  // Path planning
  Planner planner(grid.getGrid());
  auto path = planner.pathplanning(start, goal);

  // print planned path
  cout << "Planned Path:" << endl;
  for (auto &p : path) {
    cout << "(" << p.first << "," << p.second << ") ";
  }
  cout << endl;

  // Odometry commands
  cout << "\nOdometry Commands" << endl;
  double wheel_radius = 0.05; // meters
  double rpm = 120;           // wheel speed
  Odometry odo(wheel_radius, rpm);
  auto commands = odo.computeCommands(path);

  // computing total time and sec
    // computing total time and sec
  ofstream result_file(odom_commands);

// check if file is open
if (!result_file.is_open()) {
    cerr << "Error: cannot open file " << odom_commands << endl;
    return 1;
}

// --- writing result to file (trim trailing zeros properly) ---
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4) << commands.time_sec;
    std::string time_str = oss.str();

    // trim trailing zeros
    time_str.erase(time_str.find_last_not_of('0') + 1, std::string::npos);
    if (time_str.back() == '.') {
        time_str.pop_back();
    }

    result_file << time_str << "\n"
                << static_cast<int>(round(commands.angle_deg)) << "\n";
}
}

//check readme file for the logic and breif explanation of the code.
