// Solver.cpp: Pathfinding Algorithms
#include <bits/stdc++.h>
using namespace std;

// State for Dijkstra
struct State
{
    int u;
    double cost, time, dist;
};

struct ParentInfo
{
    int parent_node;
    Edge edge_taken;
};

// Schedules
struct Schedule
{
    double start, end, interval;
};
Schedule default_sched = {6.0, 23.0, 0.25};
Schedule sched_bikalpa = {7.0, 22.0, 20.0 / 60.0};
Schedule sched_uttara = {6.0, 23.0, 10.0 / 60.0};
Schedule sched_metro_p6 = {1.0, 23.0, 5.0 / 60.0};

double getWaitingTime(double current_time, EdgeType type, int problem_id)
{
    if (type == ROAD || type == WALKING)
        return 0.0;

    Schedule s = default_sched;
    if (problem_id == 6)
    {
        if (type == METRO)
            s = sched_metro_p6;
        else if (type == BUS_BIKOLPO)
            s = sched_bikalpa;
        else if (type == BUS_UTTARA)
            s = sched_uttara;
    }

    if (current_time < s.start)
        return s.start - current_time;
    if (current_time > s.end)
        return INF;

    double elapsed = current_time - s.start;
    double next_dep = s.start + ceil(elapsed / s.interval) * s.interval;

    if (abs(next_dep - current_time) < 1e-9)
        return 0;
    if (next_dep < current_time)
        next_dep += s.interval;

    return max(0.0, next_dep - current_time);
}

struct SolutionPath
{
    vector<int> nodes;
    vector<Edge> edges;
    double total_cost = 0, total_dist = 0, total_time = 0;
};

// Standard Dijkstra (Distance or Cost)
SolutionPath dijkstra_standard(int start, int end, bool opt_cost, map<EdgeType, double> rates, vector<EdgeType> modes)
{
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    map<int, double> dist;
    map<int, ParentInfo> parent;

    for (const auto &n : nodes)
        dist[n.id] = INF;
    dist[start] = 0;
    pq.push({0, start});

    while (!pq.empty())
    {
        double d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (d > dist[u])
            continue;
        if (u == end)
            break;

        for (auto &e : adj[u])
        {
            bool allowed = false;
            for (auto t : modes)
                if (e.type == t)
                    allowed = true;
            if (!allowed)
                continue;

            double weight = opt_cost ? e.weight_distance * rates[e.type] : e.weight_distance;
            if (dist[u] + weight < dist[e.to_node_id])
            {
                dist[e.to_node_id] = dist[u] + weight;
                parent[e.to_node_id] = {u, e};
                pq.push({dist[e.to_node_id], e.to_node_id});
            }
        }
    }

    SolutionPath path;
    if (dist[end] == INF)
        return path;

    for (int v = end; v != start; v = parent[v].parent_node)
    {
        path.nodes.push_back(v);
        path.edges.push_back(parent[v].edge_taken);
        path.total_dist += parent[v].edge_taken.weight_distance;
        path.total_cost += parent[v].edge_taken.weight_distance * rates[parent[v].edge_taken.type];
    }
    path.nodes.push_back(start);
    reverse(path.nodes.begin(), path.nodes.end());
    reverse(path.edges.begin(), path.edges.end());
    return path;
}

// Time-Dependent Dijkstra (Cost or Time)
// Target: 0 = Cost, 1 = Time
SolutionPath dijkstra_time_dependent(int start, int end, double start_time, int target, map<EdgeType, double> rates, map<EdgeType, double> speeds, vector<EdgeType> modes, int pid)
{
    struct State
    {
        double val;
        int u;
        double curr_time, acc_cost;
        bool operator>(const State &o) const { return val > o.val; }
    };

    priority_queue<State, vector<State>, greater<State>> pq;
    map<int, double> best;
    map<int, ParentInfo> parent;
    map<int, double> arrival;

    for (const auto &n : nodes)
        best[n.id] = INF;
    best[start] = 0;
    pq.push({0, start, start_time, 0});
    arrival[start] = start_time;

    while (!pq.empty())
    {
        State top = pq.top();
        pq.pop();

        if (top.val > best[top.u])
            continue;
        if (top.u == end)
            break;

        for (auto &e : adj[top.u])
        {
            bool allowed = false;
            for (auto t : modes)
                if (e.type == t)
                    allowed = true;
            if (!allowed)
                continue;

            double wait = getWaitingTime(top.curr_time, e.type, pid);
            if (wait == INF)
                continue;

            double travel = 0;
            if (e.type == WALKING)
                travel = e.weight_distance / 2.0;
            else if (speeds[e.type] > 0)
                travel = e.weight_distance / speeds[e.type];

            double next_time = top.curr_time + wait + travel;
            double next_cost = top.acc_cost + e.weight_distance * rates[e.type];
            double new_val = (target == 0) ? next_cost : (next_time - start_time);

            if (new_val < best[e.to_node_id])
            {
                best[e.to_node_id] = new_val;
                parent[e.to_node_id] = {top.u, e};
                arrival[e.to_node_id] = next_time;
                pq.push({new_val, e.to_node_id, next_time, next_cost});
            }
        }
    }

    SolutionPath path;
    if (best[end] == INF)
        return path;

    path.total_cost = (target == 0) ? best[end] : 0;
    path.total_time = arrival[end] - start_time;

    for (int v = end; v != start; v = parent[v].parent_node)
    {
        path.nodes.push_back(v);
        path.edges.push_back(parent[v].edge_taken);
    }
    path.nodes.push_back(start);
    reverse(path.nodes.begin(), path.nodes.end());
    reverse(path.edges.begin(), path.edges.end());

    if (target == 1)
    {
        path.total_cost = 0;
        for (auto &e : path.edges)
            path.total_cost += e.weight_distance * rates[e.type];
    }
    for (auto &e : path.edges)
        path.total_dist += e.weight_distance;
    return path;
}

// Re-implementing P6 properly compacted
SolutionPath runP6(int start, int end, double start_time, double deadline, map<EdgeType, double> rates, map<EdgeType, double> speeds, vector<EdgeType> modes)
{

    struct State
    {
        double cost, time;
        int u, p_idx;
        bool operator>(const State &o) const { return cost > o.cost; }
    };
    struct Trace
    {
        int u, p_idx;
        Edge e;
        double time;
    };

    vector<Trace> traces;
    priority_queue<State, vector<State>, greater<State>> pq;
    map<int, vector<pair<double, double>>> pareto;

    traces.push_back({start, -1, {}, start_time});
    pq.push({0, start_time, start, 0});
    pareto[start].push_back({0, start_time});

    int final_idx = -1;
    double min_cost = INF;

    while (!pq.empty())
    {
        State top = pq.top();
        pq.pop();
        if (top.cost >= min_cost)
            continue;
        if (top.time > deadline)
            continue;

        if (top.u == end)
        {
            if (top.cost < min_cost)
            {
                min_cost = top.cost;
                final_idx = top.p_idx;
            }
            continue;
        }

        for (auto &e : adj[top.u])
        {
            bool ok = false;
            for (auto m : modes)
                if (e.type == m)
                    ok = true;
            if (!ok)
                continue;

            double wait = getWaitingTime(top.time, e.type, 6);
            if (wait == INF)
                continue;

            double travel = (e.type == WALKING) ? e.weight_distance / 2.0 : (speeds[e.type] > 0 ? e.weight_distance / speeds[e.type] : 0);
            double n_time = top.time + wait + travel;
            if (n_time > deadline)
                continue;

            double n_cost = top.cost + e.weight_distance * rates[e.type];

            bool dominated = false;
            for (auto &p : pareto[e.to_node_id])
                if (p.first <= n_cost && p.second <= n_time)
                    dominated = true;
            if (dominated)
                continue;

            pareto[e.to_node_id].push_back({n_cost, n_time});
            traces.push_back({e.to_node_id, top.p_idx, e, n_time});
            pq.push({n_cost, n_time, e.to_node_id, (int)traces.size() - 1});
        }
    }

    SolutionPath p;
    if (final_idx == -1)
        return p;
    p.total_cost = min_cost;
    p.total_time = traces[final_idx].time - start_time;

    int curr = final_idx;
    while (curr != -1)
    {
        if (traces[curr].p_idx != -1)
        {
            p.nodes.push_back(traces[curr].u);
            p.edges.push_back(traces[curr].e);
        }
        else
            p.nodes.push_back(traces[curr].u);
        curr = traces[curr].p_idx;
    }
    reverse(p.nodes.begin(), p.nodes.end());
    reverse(p.edges.begin(), p.edges.end());
    for (auto &e : p.edges)
        p.total_dist += e.weight_distance;
    return p;
}

// Wrapper to match signature
SolutionPath dijkstra_constrained(int start, int end, double st, double dl, map<EdgeType, double> r, map<EdgeType, double> s,
                                  vector<EdgeType> m)
{
    return runP6(start, end, st, dl, r, s, m);
}

// Output
string formatTime(double h)
{
    int hr = (int)h;
    int mn = (int)((h - hr) * 60 + 0.5);
    string suff = "AM";
    if (hr >= 12)
    {
        suff = "PM";
        if (hr > 12)
            hr -= 12;
    }
    if (hr == 0)
        hr = 12;
    if (hr > 12)
        hr -= 12;
    stringstream ss;
    ss << hr << ":" << setfill('0') << setw(2) << mn << " " << suff;
    return ss.str();
}

void printPathDescription(SolutionPath path, double start_hour, map<EdgeType, double> rates, map<EdgeType, double> speeds, int pid)
{
    if (path.nodes.empty())
    {
        cout << "No path found." << endl;
        return;
    }

    cout << "Problem No: " << pid << endl;
    cout << "Nodes: " << path.nodes.size() << endl;
    if (start_hour >= 0)
        cout << "Start: " << formatTime(start_hour) << endl;

    double curr = start_hour;
    for (size_t i = 0; i < path.edges.size(); i++)
    {
        Edge e = path.edges[i];
        string name = getEdgeTypeName(e.type);
        string from = nodes[path.nodes[i]].name.empty() ? "RoadIntersection" : nodes[path.nodes[i]].name;
        string to = nodes[path.nodes[i + 1]].name.empty() ? "RoadIntersection" : nodes[path.nodes[i + 1]].name;

        double wait = 0, travel = 0, cost = e.weight_distance * rates[e.type];
        if (start_hour >= 0)
        {
            wait = getWaitingTime(curr, e.type, pid);
            double sp = (e.type == WALKING) ? 2.0 : speeds[e.type];
            travel = (sp > 0) ? e.weight_distance / sp : 0;

            cout << formatTime(curr) << " - " << formatTime(curr + wait + travel)
                 << ", Cost: " << fixed << setprecision(2) << cost << " Tk: "
                 << "Ride " << name << " (" << e.weight_distance << " km) from " << from << " to " << to;
            if (wait > 0)
                cout << " (Wait: " << (int)(wait * 60) << " min)";
            cout << endl;
            curr += wait + travel;
        }
        else
        {
            cout << "Ride " << name << " (" << e.weight_distance << " km) from " << from << " to " << to << " Cost: " << cost << endl;
        }
    }
    cout << "Total Dist: " << path.total_dist << " km\nTotal Cost: " << path.total_cost << " Tk" << endl;
    if (start_hour >= 0)
        cout << "Arrival: " << formatTime(curr) << endl;
    cout << endl;
}

void exportKML(SolutionPath path, string file, string color)
{
    ofstream kml(file);
    kml << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://earth.google.com/kml/2.1\">\n<Document>\n";
    kml << "<Style id=\"p\"><LineStyle><color>" << color << "</color><width>4</width></LineStyle></Style>\n";
    kml << "<Placemark><name>Route</name><styleUrl>#p</styleUrl><LineString><tessellate>1</tessellate><coordinates>\n";
    for (auto &e : path.edges)
        for (auto &p : e.geometry)
            kml << fixed << setprecision(6) << p.lon << "," << p.lat << ",0\n";
    kml << "</coordinates></LineString></Placemark></Document></kml>";
    kml.close();
    cout << "Exported " << file << endl;
}
