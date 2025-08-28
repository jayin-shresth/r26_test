#include "planning.h"
#include <cmath>
#include <vector>
#include <queue>
#include <map>
#include <algorithm>

using namespace std;

Planner::Planner(const vector<vector<bool>> &grid) : grid(grid) {
  rows = grid.size();
  cols = grid[0].size();
}

bool Planner::isvalid(int x, int y) const {
  return (x >= 0 && x < rows && y >= 0 && y < cols && !grid[x][y]);
}

double Planner::heuristic(int x1, int y1, int x2, int y2) const {
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

vector<pair<int, int>> Planner::pathplanning(pair<int, int> start,
                                             pair<int, int> goal) {
    vector<pair<int,int>> path;

    // priority queue (f = g + h, node = (x,y))
    using Node = pair<double, pair<int,int>>;
    priority_queue<Node, vector<Node>, greater<Node>> open;

    map<pair<int,int>, double> g;                // cost so far
    map<pair<int,int>, pair<int,int>> parent;    // backtracking

    // 4 directions (up, down, left, right)
    vector<pair<int,int>> dirs = {{1,0},{-1,0},{0,1},{0,-1}};

    g[start] = 0.0;
    parent[start] = {-1,-1};
    open.push({heuristic(start.first, start.second, goal.first, goal.second), start});

    while(!open.empty()) {
        auto current = open.top().second;
        open.pop();

        if(current == goal) break; // goal found ✅

        for(auto d : dirs) {
            int nx = current.first + d.first;
            int ny = current.second + d.second;

            if(!isvalid(nx, ny)) continue;

            double tentative_g = g[current] + 1.0; // each move cost = 1
            pair<int,int> neighbor = {nx, ny};

            if(g.find(neighbor) == g.end() || tentative_g < g[neighbor]) {
                g[neighbor] = tentative_g;
                double f = tentative_g + heuristic(nx, ny, goal.first, goal.second);
                open.push({f, neighbor});
                parent[neighbor] = current;
            }
        }
    }

    // reconstruct path
    if(parent.find(goal) == parent.end()) {
        return path; // empty → no path
    }

    for(pair<int,int> at = goal; at != make_pair(-1,-1); at = parent[at]) {
        path.push_back(at);
    }
    reverse(path.begin(), path.end());

    return path;
}
}
