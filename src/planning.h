#ifndef PLANNING_H
#define PLANNING_H

#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

using namespace std;

class Planner {
public:
  Planner(const vector<vector<bool>> &grid);

  /* A-start Path Planning algorithm
   *
   * to compute path over gridmap
   * path: pair<int,int> start (start pos)
   * 	     pair<int,int> goal (goal pos)
   *
   * return: vector<pair<int,int>>
   * 		(path over gridmap) */
  vector<pair<int, int>> pathplanning(pair<int, int> start,
                                      pair<int, int> goal);

private:
  // store grid map
  vector<vector<bool>> grid;
  int rows, cols; // number of rows and columns

  bool isvalid(int x, int y) const; // to check if position is within gridmap
  double heuristic(int x1, int y1, int x2,
                   int y2) const; // heuristic/cost function for path planner
};

#endif