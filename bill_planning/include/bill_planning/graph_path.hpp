#ifndef BILL_PLANNING_GRAPH_PATH_HPP
#define BILL_PLANNING_GRAPH_PATH_HPP

#include <bits/stdc++.h>
#include "bill_planning/position.hpp"
#include "ros/ros.h"
#include <math.h>
using namespace std;

class GraphPath
{
    public:
        GraphPath();
        void add_edge(int src, int dest);
        void getShortestPath(std::list<TilePosition> &drivePoints, TilePosition start, TilePosition dest, bool scanOnReach);

    private:
        bool BFS(int src, int dest, int v, int pred[], int dist[]);

        vector<int> adj[36];
};
#endif //BILL_PLANNING_GRAPH_PATH_HPP
