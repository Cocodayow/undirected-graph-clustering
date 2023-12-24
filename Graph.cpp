#include "Graph.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <set>
#include <queue>
#include <limits>
#include <algorithm> 
#include <functional>

using namespace std;

unordered_map<string, unordered_map<string, double>> adjacency_list;

Graph::Graph(const char* const & edgelist_csv_fn){
    ifstream file(edgelist_csv_fn);
    if (!file) {
        return;
    }

    string line;
    while (getline(file, line)) {
        istringstream iss(line);
        string u, v;
        double weight;
        getline(iss, u, ',');
        getline(iss, v, ',');
        iss >> weight;

        // Check if the nodes exist in the adjacency list
        if (adjacency_list.find(u) == adjacency_list.end()) {
            adjacency_list[u] = unordered_map<string, double>();
        }
        if (adjacency_list.find(v) == adjacency_list.end()) {
            adjacency_list[v] = unordered_map<string, double>();
        }

        adjacency_list[u][v] = weight;
        adjacency_list[v][u] = weight;
    }

    file.close();
}


unsigned int Graph::num_nodes(){
    return adjacency_list.size();
}

vector<string> Graph::nodes(){
    vector<string> result;
    for (const auto& node : adjacency_list) {
        result.push_back(node.first);
    }
    return result;
}

unsigned int Graph::num_edges(){
    int count = 0;
    for (const auto& node : adjacency_list) {
        count += node.second.size();
    }
    return count / 2;
}

double Graph::edge_weight(const string& u, const string& v) {
    auto iter = adjacency_list.find(u);
    if (iter != adjacency_list.end()) {
        auto innerIter = iter->second.find(v);
        if (innerIter != iter->second.end()) {
            return innerIter->second;
        }
    }
    return -1;
}

unsigned int Graph::num_neighbors(const string& node) {
    auto iter = adjacency_list.find(node);
    if (iter != adjacency_list.end()) {
        return iter->second.size();
    }
    return 0;
}

vector<string> Graph::neighbors(const string& node){
    auto iter = adjacency_list.find(node);
    if (iter != adjacency_list.end()) {
        vector<string> result;
        for (const auto& neighbor : iter->second) {
            result.push_back(neighbor.first);
        }
        return result;
    }
    return {};
}

vector<string> Graph::shortest_path_unweighted(string const & start_label, string const & end_label){
    unordered_map<string, bool> visited;
    unordered_map<string, string> parent;
    queue<string> q;
    q.push(start_label);
    visited[start_label] = true;

    while (!q.empty()) {
        string current = q.front();
        q.pop();

        if (current == end_label) {
            vector<string> path;
            while (current != start_label) {
                path.insert(path.begin(), current);
                current = parent[current];
            }
            path.insert(path.begin(), start_label);
            return path;
        }

        for (const string& neighbor : neighbors(current)) {
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                parent[neighbor] = current;
                q.push(neighbor);
            }
        }
    }

    return {};
}
vector<tuple<string, string, double>> Graph::shortest_path_weighted(const string& start_label, const string& end_label) {
    priority_queue<pair<double, string>, vector<pair<double, string>>, greater<pair<double, string>>> pq;
    unordered_map<string, double> distances;
    unordered_map<string, string> previous;
     if (start_label == end_label) {
        return { make_tuple(start_label, start_label, -1) };
    }


    for (const auto& node : adjacency_list) {
        distances[node.first] = numeric_limits<double>::infinity();
        previous[node.first] = "";
    }

    distances[start_label] = 0.0;
    pq.emplace(0.0, start_label);

    while (!pq.empty()) {
        string current_node = pq.top().second;
        double current_distance = pq.top().first;
        pq.pop();

        if (current_node == end_label)
            break;

        // Check if the current distance is outdated (due to relaxed edges)
        if (current_distance > distances[current_node])
            continue;

        for (const auto& neighbor : adjacency_list[current_node]) {
            string neighbor_node = neighbor.first;
            double edge_weight = neighbor.second;

            double total_distance = distances[current_node] + edge_weight;
            if (total_distance < distances[neighbor_node]) {
                distances[neighbor_node] = total_distance;
                previous[neighbor_node] = current_node;
                pq.emplace(total_distance, neighbor_node);
            }
        }
    }

    vector<tuple<string, string, double>> shortest_path;
    string current_node = end_label;
    while (current_node != "") {
        string previous_node = previous[current_node];
        if (previous_node != "") {
            double edge_weight = adjacency_list[previous_node][current_node];
            shortest_path.emplace_back(previous_node, current_node, edge_weight);
        }
        current_node = previous_node;
    }

    reverse(shortest_path.begin(), shortest_path.end());
    return shortest_path;
}


vector<vector<string>> Graph::connected_components(double const & threshold) {
    unordered_map<string, bool> visited;
    vector<vector<string>> components;

    for (const string& node : nodes()) {
        if (!visited[node]) {
            vector<string> component;
            queue<string> q;
            q.push(node);
            visited[node] = true;

            while (!q.empty()) {
                string current = q.front();
                q.pop();
                component.push_back(current);

                for (const string& neighbor : neighbors(current)) {
                    if (!visited[neighbor] && edge_weight(current, neighbor) <= threshold) {
                        visited[neighbor] = true;
                        q.push(neighbor);
                    }
                }
            }

            components.push_back(component);
        }
    }

    return components;
}


double Graph::smallest_connecting_threshold(const string& u, const string& v) {
    if (u == v) {
        return 0.0;
    }

    // Priority queue to store nodes with their corresponding distances
    priority_queue<pair<double, string>, vector<pair<double, string>>, greater<pair<double, string>>> pq;

    // Map to store the smallest connecting thresholds from u to each node
    unordered_map<string, double> thresholds;

    // Initialize thresholds
    for (const auto& node : adjacency_list) {
        thresholds[node.first] = numeric_limits<double>::infinity();
    }

    thresholds[u] = 0.0;
    pq.emplace(0.0, u);

    while (!pq.empty()) {
        string current_node = pq.top().second;
        pq.pop();

        // If the smallest connecting threshold to v is found, return it
        if (current_node == v) {
            return thresholds[v];
        }

        for (const auto& neighbor : adjacency_list[current_node]) {
            string neighbor_node = neighbor.first;
            double edge_weight = neighbor.second;

            double current_threshold = max(thresholds[current_node], edge_weight);
            if (current_threshold < thresholds[neighbor_node]) {
                thresholds[neighbor_node] = current_threshold;
                pq.emplace(current_threshold, neighbor_node);
            }
        }
    }

    return -1;
}

