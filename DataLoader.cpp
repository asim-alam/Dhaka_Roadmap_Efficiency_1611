// DataLoader.cpp: CSV Parsing
#include <bits/stdc++.h>
#include "Models.cpp"
using namespace std;

// Add Edge to Adjacency List (Bi-directional)
void addEdge(int u, int v, double dist, EdgeType type, vector<Point> geom)
{
    Edge e1 = {v, dist, type, geom};
    adj[u].push_back(e1);

    reverse(geom.begin(), geom.end());
    Edge e2 = {u, dist, type, geom};
    adj[v].push_back(e2);
}

// Parse Roadmap-Dhaka.csv
void parseRoadMap(const string &filename)
{
    ifstream file(filename);
    string line;
    while (getline(file, line))
    {
        if (line.rfind("DhakaStreet", 0) != 0)
            continue;

        stringstream ss(line);
        string token;
        vector<string> parts;
        while (getline(ss, token, ','))
            parts.push_back(token);

        if (parts.size() < 6)
            continue;

        double dist = stod(parts.back());
        vector<Point> polyline;
        for (size_t i = 1; i < parts.size() - 2; i += 2)
        {
            polyline.push_back({stod(parts[i + 1]), stod(parts[i])}); // Lat, Lon
        }

        if (polyline.size() < 2)
            continue;

        int u = getOrCreateNodeID(polyline[0].lat, polyline[0].lon);
        int v = getOrCreateNodeID(polyline.back().lat, polyline.back().lon);
        addEdge(u, v, dist, ROAD, polyline);
    }
    cout << "Loaded Roadmap. Nodes: " << nodes.size() << ", Edges: " << adj.size() << endl;
}

// Parse Transport CSVs
void parseRouteMap(const string &filename, EdgeType type)
{
    ifstream file(filename);
    string line;
    while (getline(file, line))
    {
        stringstream ss(line);
        string token;
        vector<string> parts;
        while (getline(ss, token, ','))
            parts.push_back(token);

        if (parts.empty())
            continue;

        string tType = parts[0];
        if (type == METRO && tType != "DhakaMetroRail")
            continue;
        if ((type == BUS_BIKOLPO || type == BUS_UTTARA) && tType.find("DhakaBus") == string::npos)
            continue;

        string startName = parts[parts.size() - 2];
        string endName = parts[parts.size() - 1];

        // Clean Names
        endName.erase(remove(endName.begin(), endName.end(), '\r'), endName.end());
        endName.erase(remove(endName.begin(), endName.end(), '\n'), endName.end());

        vector<Point> polyline;
        for (size_t i = 1; i < parts.size() - 2; i += 2)
        {
            polyline.push_back({stod(parts[i + 1]), stod(parts[i])});
        }

        if (polyline.empty())
            continue;

        // Create/Find Nodes for Stations
        int u = -1, v = -1;
        // Simple search for existing named node or create new
        u = getOrCreateNodeID(polyline[0].lat, polyline[0].lon);
        if (nodes[u].name.empty())
            nodes[u].name = startName;

        v = getOrCreateNodeID(polyline.back().lat, polyline.back().lon);
        if (nodes[v].name.empty())
            nodes[v].name = endName;

        // Calc Distance
        double dist = 0;
        for (size_t i = 0; i < polyline.size() - 1; i++)
        {
            dist += getDistance(polyline[i].lat, polyline[i].lon, polyline[i + 1].lat, polyline[i + 1].lon);
        }

        addEdge(u, v, dist, type, polyline);
    }
}

// Link Stations to Nearest Road
void linkStationsToRoads()
{
    int count = 0;
    for (auto &node : nodes)
    {
        if (node.name.empty())
            continue; // Skip if not a station (roughly)

        int nearest = -1;
        double minDist = INF;

        // Find nearest unnamed node (Road intersection)
        for (auto &other : nodes)
        {
            if (node.id == other.id || !other.name.empty())
                continue;

            double d = getDistance(node.lat, node.lon, other.lat, other.lon);
            if (d < minDist)
            {
                minDist = d;
                nearest = other.id;
            }
        }

        if (nearest != -1 && minDist < 0.5)
        { // < 500m
            vector<Point> geom = {{node.lat, node.lon}, {nodes[nearest].lat, nodes[nearest].lon}};
            addEdge(node.id, nearest, minDist, WALKING, geom);
            count++;
        }
    }
    cout << "Linked " << count << " stations to road network." << endl;
}
