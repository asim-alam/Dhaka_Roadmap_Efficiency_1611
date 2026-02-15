// Main.cpp: Unity Build Driver

#include "DataLoader.cpp"
#include "Solver.cpp"

void runP1(int s, int e)
{
    cout << "=== Problem 1: Shortest Distance (Car) ===" << endl;
    map<EdgeType, double> cost;
    cost[ROAD] = 1;
    vector<EdgeType> m = {ROAD};
    SolutionPath p = dijkstra_standard(s, e, false, cost, m);
    printPathDescription(p, -1, {}, {}, 1);
    exportKML(p, "solution_p1.kml", "ff0000ff");
}

void runP2(int s, int e)
{
    cout << "=== Problem 2: Cheapest (Car, Metro) ===" << endl;
    map<EdgeType, double> cost;
    cost[ROAD] = 20;
    cost[METRO] = 5;
    cost[WALKING] = 0;
    vector<EdgeType> m = {ROAD, METRO, WALKING};
    SolutionPath p = dijkstra_standard(s, e, true, cost, m);
    printPathDescription(p, -1, cost, {}, 2);
    exportKML(p, "solution_p2.kml", "ffff0000");
}

void runP3(int s, int e)
{
    cout << "=== Problem 3: Cheapest (Car, Metro, Bus) ===" << endl;
    map<EdgeType, double> cost;
    cost[ROAD] = 20;
    cost[METRO] = 5;
    cost[BUS_BIKOLPO] = 7;
    cost[BUS_UTTARA] = 7;
    cost[WALKING] = 0;
    vector<EdgeType> m = {ROAD, METRO, BUS_BIKOLPO, BUS_UTTARA, WALKING};
    SolutionPath p = dijkstra_standard(s, e, true, cost, m);
    printPathDescription(p, -1, cost, {}, 3);
    exportKML(p, "solution_p3.kml", "ff00ff00");
}

void runP4(int s, int e, double st)
{
    cout << "=== Problem 4: Cheapest with Start Time ===" << endl;
    map<EdgeType, double> cost;
    cost[ROAD] = 20;
    cost[METRO] = 5;
    cost[BUS_BIKOLPO] = 7;
    cost[BUS_UTTARA] = 7;
    cost[WALKING] = 0;
    map<EdgeType, double> speed;
    speed[ROAD] = 30;
    speed[METRO] = 30;
    speed[BUS_BIKOLPO] = 30;
    speed[BUS_UTTARA] = 30;
    speed[WALKING] = 2;
    vector<EdgeType> m = {ROAD, METRO, BUS_BIKOLPO, BUS_UTTARA, WALKING};

    SolutionPath p = dijkstra_time_dependent(s, e, st, 0, cost, speed, m, 4);
    printPathDescription(p, st, cost, speed, 4);
    exportKML(p, "solution_p4.kml", "ff00ffff");
}

void runP5(int s, int e, double st)
{
    cout << "=== Problem 5: Fastest with Start Time ===" << endl;
    map<EdgeType, double> cost;
    cost[ROAD] = 20;
    cost[METRO] = 5;
    cost[BUS_BIKOLPO] = 7;
    cost[BUS_UTTARA] = 7;
    cost[WALKING] = 0;
    map<EdgeType, double> speed;
    speed[ROAD] = 10;
    speed[METRO] = 10;
    speed[BUS_BIKOLPO] = 10;
    speed[BUS_UTTARA] = 10;
    speed[WALKING] = 2;
    vector<EdgeType> m = {ROAD, METRO, BUS_BIKOLPO, BUS_UTTARA, WALKING};

    SolutionPath p = dijkstra_time_dependent(s, e, st, 1, cost, speed, m, 5);
    printPathDescription(p, st, cost, speed, 5);
    exportKML(p, "solution_p5.kml", "ff800080");
}

void runP6(int s, int e, double st, double dl)
{
    cout << "=== Problem 6: Constrained Cheapest (Deadlined) ===" << endl;
    map<EdgeType, double> cost;
    cost[ROAD] = 20;
    cost[METRO] = 5;
    cost[BUS_BIKOLPO] = 7;
    cost[BUS_UTTARA] = 10;
    cost[WALKING] = 0;
    map<EdgeType, double> speed;
    speed[ROAD] = 20;
    speed[METRO] = 15;
    speed[BUS_BIKOLPO] = 10;
    speed[BUS_UTTARA] = 12;
    speed[WALKING] = 2;
    vector<EdgeType> m = {ROAD, METRO, BUS_BIKOLPO, BUS_UTTARA, WALKING};

    SolutionPath p = dijkstra_constrained(s, e, st, dl, cost, speed, m);
    printPathDescription(p, st, cost, speed, 6);
    exportKML(p, "solution_p6.kml", "ff0080ff");
}

int main()
{
    cout << "Loading Data..." << endl;
    parseRoadMap("Dataset/Roadmap-Dhaka.csv");
    parseRouteMap("Dataset/Routemap-DhakaMetroRail.csv", METRO);
    parseRouteMap("Dataset/Routemap-BikolpoBus.csv", BUS_BIKOLPO);
    parseRouteMap("Dataset/Routemap-UttaraBus.csv", BUS_UTTARA);
    linkStationsToRoads();

    double sLat = 23.834145, sLon = 90.363833;
    double eLat = 23.721444, eLon = 90.378868;
    double startTime = 17 + 43.0 / 60.0;
    double deadline = 20 + 40.0 / 60.0;

    cout << "\nChoose Input Mode:\n1. Default\n2. Custom" << endl;
    int choice;
    if (cin >> choice && choice == 2)
    {
        cout << "\n--- Demo: Source(23.834145 90.363833) Dest(23.721444 90.378868) ---" << endl;
        cout << "Enter Source Lat: ";
        cin >> sLat;
        cout << "Enter Source Lon: ";
        cin >> sLon;
        cout << "Enter Dest Lat: ";
        cin >> eLat;
        cout << "Enter Dest Lon: ";
        cin >> eLon;

        double sh, sm, dh, dm;
        cout << "Start Time (HH MM): ";
        cin >> sh >> sm;
        startTime = sh + sm / 60.0;
        cout << "Deadline (HH MM): ";
        cin >> dh >> dm;
        deadline = dh + dm / 60.0;
    }

    int startNode = getNearestNode(sLat, sLon);
    int endNode = getNearestNode(eLat, eLon);

    if (startNode == -1 || endNode == -1)
    {
        cout << "Error: Nodes not found." << endl;
        return 1;
    }

    double walkSrc = getDistance(sLat, sLon, nodes[startNode].lat, nodes[startNode].lon);
    double walkDst = getDistance(nodes[endNode].lat, nodes[endNode].lon, eLat, eLon);

    cout << fixed << setprecision(2);
    cout << "Initial Walk: " << walkSrc << " km (" << (int)(walkSrc / 2.0 * 60) << " min) to Node " << startNode << endl;
    cout << "Final Walk: " << walkDst << " km (" << (int)(walkDst / 2.0 * 60) << " min) from Node " << endNode << endl;

    double gStart = startTime + walkSrc / 2.0;
    double gDead = deadline - walkDst / 2.0;

    runP1(startNode, endNode);
    runP2(startNode, endNode);
    runP3(startNode, endNode);
    runP4(startNode, endNode, gStart);
    runP5(startNode, endNode, gStart);
    runP6(startNode, endNode, gStart, gDead);

    return 0;
}
