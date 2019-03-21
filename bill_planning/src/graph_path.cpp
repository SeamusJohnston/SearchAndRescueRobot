#include "bill_planning/graph_path.hpp"
// NOTE THIS CODE WAS TAKEN FROM
// https://www.geeksforgeeks.org/shortest-path-unweighted-graph/

// FATHER FORGIVE ME FOR I HAVE SINNED IN THE FOLLOWING FUNCTIONS
bool isValidVerticalLink(int i)
{
    return  (i != 2) &&
            (i != (7 - 6)) && (i != 7) &&
            (i != (10 - 6)) && (i != 10) &&
            (i != (17 - 6)) && (i != 17) &&
            (i != (18 - 6)) && (i != 18) &&
            (i != (20 - 6)) && (i != 20) &&
            (i != (25 - 6)) && (i != 25) &&
            (i != (28 - 6)) && (i != 28) &&
            (i != 27);
}

bool isValidHorizontalLink(int i)
{
    return  (i != (2 - 1)) && (i != 2) &&
            (i != (7 - 1)) && (i != 7) &&
            (i != (10 - 1)) && (i != 10) &&
            (i != (17 - 1)) &&
            (i != 18) &&
            (i != (20 - 1)) && (i != 20) &&
            (i != (25 - 1)) && (i != 25) &&
            (i != (28 - 1)) && (i != 28);
}

GraphPath::GraphPath()
{
    // Populate the edges of our graph
    // THIS FOR LOOP POPULATES ROWS 0 TO 4
    for (int i = 0; i < 30; ++i)
    {
        // Link the current tile and the tile above it
        if (isValidVerticalLink(i))
        {
            add_edge(i, i + 6);
        }

        // For tiles not on the far right edge, we should connect them with their right hand neighbors
        if ((i%6 != 5) && isValidHorizontalLink(i))
        {
            add_edge(i, i+1);
        }
    }

    // THIS FOR LOOP POPULATES ROW 5
    for (int i = 30; i < 35; ++i)
    {
        if ((i != 33) && (i != 32))
        {
            add_edge(i, i+1);
        }
    }
}

// utility function to form edge between two vertices source and dest
void GraphPath::add_edge(int src, int dest)
{
    adj[src].push_back(dest);
    adj[dest].push_back(src);
}

// a modified version of BFS that stores predecessor of each vertex in array p and its distance from source in array d
bool GraphPath::BFS(int src, int dest, int v, int pred[], int dist[])
{
    // a queue to maintain queue of vertices whose adjacency list is to be scanned as per normal DFS algorithm
    list<int> queue;

    // boolean array visited[] which stores the information whether ith vertex is reached at least once in the Breadth first search
    bool visited[v];

    // initially all vertices are unvisited so v[i] for all i is false and as no path is yet constructed dist[i] for all i set to infinity
    for (int i = 0; i < v; i++) {
        visited[i] = false;
        dist[i] = INT_MAX;
        pred[i] = -1;
    }

    // now source is first to be visited and distance from source to itself should be 0
    visited[src] = true;
    dist[src] = 0;
    queue.push_back(src);

    // standard BFS algorithm
    while (!queue.empty()) {
        int u = queue.front();
        queue.pop_front();
        for (int i = 0; i < adj[u].size(); i++) {
            if (visited[adj[u][i]] == false) {
                visited[adj[u][i]] = true;
                dist[adj[u][i]] = dist[u] + 1;
                pred[adj[u][i]] = u;
                queue.push_back(adj[u][i]);

                // We stop BFS when we find destination.
                if (adj[u][i] == dest)
                    return true;
            }
        }
    }

    return false;
}

// utility function to print the shortest distance between source vertex and destination vertex
void GraphPath::getShortestPath(std::list<TilePosition> &drivePoints, TilePosition startTile, TilePosition targetTile, bool scanOnReach)
{
    int s = (startTile.y * 6) + startTile.x;
    int dest = (targetTile.y * 6) + targetTile.x;

    int v = 36;
    // predecessor[i] array stores predecessor of i and distance array stores distance of i from s
    int pred[v], dist[v];

    if (BFS(s, dest, v, pred, dist) == false)
    {
        ROS_INFO("Given target and destination are not connected");
        return;
    }

    // vector path stores the shortest path
    vector<int> path;
    int crawl = dest;
    path.push_back(crawl);

    while (pred[crawl] != -1)
    {
        path.push_back(pred[crawl]);
        crawl = pred[crawl];
    }

    int firstX = path[path.size() - 2] % 6;
    int firstY = (int)floor(path[path.size() - 2]/6);

    bool prevXDelta = startTile.x != firstX;
    bool prevYDelta = startTile.y != firstY;

    // printing path from source to destination
    for (int i = path.size() - 1; i >= 0; i--)
    {
        int pointY = (int)floor(path[i]/6);
        int pointX = path[i] % 6;

        if (i != 0)
        {
            int nextY = (int)floor(path[i - 1]/6);
            int nextX = path[i - 1] % 6;

            if ((nextX != pointX) && (prevYDelta))
            {
                prevXDelta = true;
                prevYDelta = false;
                drivePoints.emplace_back(pointX, pointY);
            }

            if ((nextY != pointY) && (prevXDelta))
            {
                prevXDelta = false;
                prevYDelta = true;
                drivePoints.emplace_back(pointX, pointY);
            }
        }
        else
        {
            drivePoints.emplace_back(pointX, pointY, scanOnReach);
        }
    }
}
