// Models.cpp: Core Data Structures
#include <bits/stdc++.h>
using namespace std;

// Constants
const double INF = 1e18;
const double PI = 3.14159265358979323846;

// Enums
enum EdgeType
{
    ROAD,
    METRO,
    BUS_BIKOLPO,
    BUS_UTTARA,
    WALKING
};

// Structs
struct Point
{
    double lat, lon;
};

struct Edge
{
    int to_node_id;
    double weight_distance;
    EdgeType type;
    vector<Point> geometry;
};

struct Node
{
    int id;
    double lat, lon;
    string name;
};

// Global Graph
vector<Node> nodes;
map<int, vector<Edge>> adj;

// Helpers
double toRadians(double degree) { return degree * PI / 180.0; }

double getDistance(double lat1, double lon1, double lat2, double lon2)
{
    double R = 6371;
    double dLat = toRadians(lat2 - lat1);
    double dLon = toRadians(lon2 - lon1);
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(toRadians(lat1)) * cos(toRadians(lat2)) *
                   sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

// Node Lookup
int getNodeID(double lat, double lon)
{
    for (const auto &node : nodes)
    {
        if (abs(node.lat - lat) < 1e-6 && abs(node.lon - lon) < 1e-6)
            return node.id;
    }
    return -1;
}

int getNearestNode(double lat, double lon)
{
    int nearest_id = -1;
    double min_dist = INF;

    for (const auto &node : nodes)
    {
        double d = getDistance(lat, lon, node.lat, node.lon);
        if (d < min_dist)
        {
            min_dist = d;
            nearest_id = node.id;
        }
    }
    return nearest_id;
}

int getOrCreateNodeID(double lat, double lon)
{
    int existing = getNodeID(lat, lon);
    if (existing != -1)
        return existing;

    int id = (int)nodes.size();
    nodes.push_back({id, lat, lon, ""});
    return id;
}

string getEdgeTypeName(EdgeType t)
{
    switch (t)
    {
    case ROAD:
        return "Car";
    case METRO:
        return "Metro";
    case BUS_BIKOLPO:
        return "Bikolpo Bus";
    case BUS_UTTARA:
        return "Uttara Bus";
    case WALKING:
        return "Walk";
    default:
        return "Unknown";
    }
}
