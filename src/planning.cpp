#define _USE_MATH_DEFINES
#include "planning.h"
#include <cmath>
#include <vector>
#include <queue>
#include <map>
#include <algorithm>
#include <set>

using namespace std;

Planner::Planner(const vector<vector<bool>> &grid) : grid(grid) {
  rows = grid.size();
  cols = grid[0].size();
}

bool Planner::isvalid(int x, int y) const {
  return (x >= 0 && x < rows && y >= 0 && y < cols && !grid[x][y]);
}

// Octile distance heuristic (works for 8 directions)
double Planner::heuristic(int x1, int y1, int x2, int y2) const {
  double dx = abs(x1 - x2);
  double dy = abs(y1 - y2);
  return (dx + dy) + (1.4142 - 2.0) * min(dx, dy);
}

vector<pair<int, int>> Planner::pathplanning(pair<int, int> start,
                                             pair<int, int> goal) {
    vector<pair<int,int>> path;

    using Node = pair<pair<double,double>, pair<int,int>>;
    // priority_queue: smaller f, if equal then prefer larger g (to force zigzag)
    auto cmp = [](const Node &a, const Node &b) {
        if (a.first.first == b.first.first) {
            return a.first.second < b.first.second; // prefer larger g
        }
        return a.first.first > b.first.first;
    };

    priority_queue<Node, vector<Node>, decltype(cmp)> open(cmp);

    map<pair<int,int>, double> g;             // cost so far
    map<pair<int,int>, pair<int,int>> parent; // backtracking
    set<pair<int,int>> closed;                // visited set

    // 8-connected grid (allow diagonals)
    vector<pair<int,int>> dirs = {
        {1,0},{-1,0},{0,1},{0,-1},
        {1,1},{-1,-1},{1,-1},{-1,1}
    };

    g[start] = 0.0;
    parent[start] = {-1,-1};
    open.push({{heuristic(start.first,start.second,goal.first,goal.second),0.0}, start});

    while(!open.empty()) {
        auto current = open.top().second;
        open.pop();

        if(closed.count(current)) continue;
        closed.insert(current);

        if(current == goal) break;

        for(auto d : dirs) {
            int nx = current.first + d.first;
            int ny = current.second + d.second;

            if(!isvalid(nx, ny)) continue;

            double step_cost = (d.first != 0 && d.second != 0) ? 1.4142 : 1.0;
            double tentative_g = g[current] + step_cost;
            pair<int,int> neighbor = {nx, ny};

            if(g.find(neighbor) == g.end() || tentative_g < g[neighbor]) {
                g[neighbor] = tentative_g;
                double f = tentative_g + heuristic(nx, ny, goal.first, goal.second);
                open.push({{f, tentative_g}, neighbor});
                parent[neighbor] = current;
            }
        }
    }

    // reconstruct path
    if(parent.find(goal) == parent.end()) {
        return path; // no path
    }

    for(pair<int,int> at = goal; at != make_pair(-1,-1); at = parent[at]) {
        path.push_back(at);
    }
    reverse(path.begin(), path.end());

    return path;
}
