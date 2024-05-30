#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <algorithm>
#include <queue>
#include <chrono>
#include "Algorithms.h"
using namespace std;

// Function to generate a manual graph
map<string, vector<pair<string, int>>> generateManualGraph(int nodes, int edges) {
    map<string, vector<pair<string, int>>> graph;

    for (int i = 0; i < edges; ++i) {
        string u, v;
        int w;
        cin >> u >> v >> w;
        graph[u].push_back(make_pair(v, w));
        graph[v].push_back(make_pair(u, w));
    }
    return graph;
}

// Function to generate a random graph
map<string, vector<pair<string, int>>> generateRandomGraph(int nodes, int edges) {
    map<string, vector<pair<string, int>>> graph;
    vector<string> vertices;
    // Generating random vertices
    for (int i = 0; i < nodes; ++i) {
        vertices.push_back("city" + to_string(i));
    }
    // Shuffling vertices to get random edges
    random_shuffle(vertices.begin(), vertices.end());
    srand(time(0));
    // Generating random edges
    for (int i = 0; i < edges; ++i) {
        int u = rand() % nodes;
        int v = rand() % nodes;
        // Checking if the edge already exists
        while (u == v || find_if(graph[vertices[u]].begin(), graph[vertices[u]].end(),
            [&](const pair<string, int>& edge) { return edge.first == vertices[v]; }) != graph[vertices[u]].end()) {
            u = rand() % nodes;
            v = rand() % nodes;
        }
        // Generating random weight
        int weight = rand() % 20 + 1;
        graph[vertices[u]].push_back(make_pair(vertices[v], weight));
        graph[vertices[v]].push_back(make_pair(vertices[u], weight));
    }
    return graph;
}

// Function to print the graph
void printGraph(const map<string, vector<pair<string, int>>>& graph) {
    for (const auto& vertex : graph) {
        cout << vertex.first << " : ";
        for (const auto& neighbour : vertex.second) {
            cout << "(" << neighbour.first << ", " << neighbour.second << ") ";
        }
        cout << endl;
    }
}

// Structure to store the node information
struct Node {
    string city;
    vector<string> path;
    int cost;
    int heuristic;

    bool operator>(const Node& other) const {
        return cost > other.cost;
    }
};

// Function to solve the Travelling Salesman Problem using Depth First Search
void TravellingSalesmanProblemDFS(const map<string, vector<pair<string, int>>>& graph, const string& start, const string& current, int cost, int& min_cost, vector<string>& current_path, set<string>& visited, vector<string>& best_path) {
    // If the cost exceeds the minimum cost, return
    if (cost >= min_cost) return;
    
    // Add the current city to the path
    current_path.push_back(current);
    visited.insert(current);
    // If all cities are visited, check if the path is valid
    if (visited.size() == graph.size()) {
        for (auto& neighbor : graph.at(current)) {
            // If the path is valid, update the minimum cost and best path
            if (neighbor.first == start) {
                int total_cost = cost + neighbor.second;
                if (total_cost < min_cost) {
                    min_cost = total_cost;
                    best_path = current_path;
                    best_path.push_back(start);
                }
                break;
            }
        }
    }
    // Otherwise, visit the unvisited neighbors
    else {
        for (auto& neighbour : graph.at(current)) {
            // If the neighbor is not visited, visit it
            if (visited.find(neighbour.first) == visited.end()) {
                TravellingSalesmanProblemDFS(graph, start, neighbour.first, cost + neighbour.second, min_cost, current_path, visited, best_path);
            }
        }
    }
    current_path.pop_back();
    visited.erase(current);
}

// Function to solve the Travelling Salesman Problem using Uniform Cost Search
int TravellingSalesmanProblemUCS(const map<string, vector<pair<string, int>>>& graph, const string& start, vector<string>& best_path) {
    priority_queue<Node, vector<Node>, greater<Node>> pq;
    pq.push({ start, {start}, 0 });

    int min_cost = INT_MAX;
    // While the priority queue is not empty
    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        // If the path is complete and the path is valid, update the minimum cost and best path
        if (current.path.size() == graph.size() + 1 && current.path.back() == start) {
            if (current.cost < min_cost) {
                min_cost = current.cost;
                best_path = current.path;
            }
            continue;
        }
        // Visit the unvisited neighbors
        for (auto& neighbour : graph.at(current.city)) {
            // If the neighbor is not visited or the path is complete and the neighbor is the start city
            if (find(current.path.begin(), current.path.end(), neighbour.first) == current.path.end() ||
                (current.path.size() == graph.size() && neighbour.first == start)) {
                // Add the neighbor to the path
                vector<string> new_path = current.path;
                new_path.push_back(neighbour.first);
                pq.push({ neighbour.first, new_path, current.cost + neighbour.second });
            }
        }
    }
    return min_cost;
}

// Function to calculate the minimum spanning tree cost
int mstCost(const map<string, vector<pair<string, int>>>& graph, const set<string>& unvisited, const string& start) {
    // If all cities are visited, return 0
    if (unvisited.empty()) return 0;
    
    map<string, bool> inMST;
    map<string, int> key;
    map<string, string> parent;
    // Initialize the key and inMST maps
    for (const string& node : unvisited) {
        key[node] = INT_MAX;
        inMST[node] = false;
    }
    // Set the key of the start city to 0
    key[*unvisited.begin()] = 0;
    // Perform Prim's algorithm
    for (size_t count = 0; count < unvisited.size(); count++) {
        int min = INT_MAX;
        string min_node;
        // Find the minimum key value
        for (const string& node : unvisited) {
            if (!inMST[node] && key[node] < min) {
                min = key[node];
                min_node = node;
            }
        }

        inMST[min_node] = true;
        // Update the key values
        for (const auto& neighbour : graph.at(min_node)) {
            if (unvisited.find(neighbour.first) != unvisited.end() && !inMST[neighbour.first] && neighbour.second < key[neighbour.first]) {
                key[neighbour.first] = neighbour.second;
                parent[neighbour.first] = min_node;
            }
        }
    }
    // Calculate the total cost
    int total_cost = 0;
    for (const string& node : unvisited) {
        total_cost += key[node];
    }
    // Find the minimum edge cost to the start city from the unvisited cities and return the total cost
    int min_edge_back = INT_MAX;
    for (const string& node : unvisited) {
        for (const auto& neighbour : graph.at(node)) {
            if (neighbour.first == start) {
                min_edge_back = min(min_edge_back, neighbour.second);
            }
        }
    }
    return total_cost + min_edge_back;
}

// Function to calculate the heuristic cost
int heuristic(const map<string, vector<pair<string, int>>>& graph, const string& current, const set<string>& unvisited, const string& start) {
    if (unvisited.empty()) return 0;
    // Find the minimum edge cost to the start city from the unvisited cities and return the minimum spanning tree cost
    int min_edge_cost = INT_MAX;
    for (const string& node : unvisited) {
        for (const auto& neighbour : graph.at(node)) {
            if (neighbour.first == start) {
                min_edge_cost = min(min_edge_cost, neighbour.second);
            }
        }
    }
    return mstCost(graph, unvisited, start) + min_edge_cost;
}

// Function to solve the Travelling Salesman Problem using A*
int TravellingSalesmanProblemAStar(const map<string, vector<pair<string, int>>>& graph, const string& start, vector<string>& best_path) {
    priority_queue<Node, vector<Node>, greater<Node>> pq;
    set<string> all_cities;
    // Insert all cities into the set
    for (const auto& pair : graph) {
        all_cities.insert(pair.first);
    }
    // Push the start city into the priority queue
    pq.push({ start, {start}, 0, heuristic(graph, start, all_cities, start) });

    int min_cost = INT_MAX;
    // While the priority queue is not empty
    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();
        // If the path is complete and the path is valid, update the minimum cost and best path
        if (current.path.size() == graph.size() + 1 && current.path.back() == start) {
            if (current.cost < min_cost) {
                min_cost = current.cost;
                best_path = current.path;
            }
            continue;
        }
        // Visit the unvisited neighbors
        for (auto& neighbour : graph.at(current.city)) {
            // If the neighbor is not visited or the path is complete and the neighbor is the start city
            if (find(current.path.begin(), current.path.end(), neighbour.first) == current.path.end() ||
                (current.path.size() == graph.size() && neighbour.first == start)) {
                // Add the neighbor to the path
                vector<string> new_path = current.path;
                new_path.push_back(neighbour.first);
                set<string> unvisited(all_cities);
                // Erase the visited cities from the set
                for (const string& city : new_path) {
                    unvisited.erase(city);
                }
                // Push the neighbor into the priority queue
                pq.push({ neighbour.first, new_path, current.cost + neighbour.second, heuristic(graph, neighbour.first, unvisited, start) });
            }
        }
    }

    return min_cost;
}
