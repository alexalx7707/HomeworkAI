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

int main() {
    // Generate a graph and print it
    cout << "Press 1 for random graph or 2 for custom graph: ";
    int choice;
    cin >> choice;
    map<string, vector<pair<string, int>>> graph;
    if (choice == 2) {
        int n;
        cin >> n;
        graph = generateManualGraph(n, (n * (n - 1)) / 2);
    }
    else {
        int n;
        cin >> n;
        graph = generateRandomGraph(n, (n * (n - 1)) / 2);
    }
    printGraph(graph);
    
    chrono::time_point<chrono::high_resolution_clock> start, end;
    int minCost = INT_MAX;
    vector<string> current_path;
    vector<string> best_path;
    set<string> visited;
    string start_node;
    for (const auto& node : graph) {
		start_node = node.first;
		break;
	}
    // DFS
    start = chrono::high_resolution_clock::now();
    TravellingSalesmanProblemDFS(graph, start_node, start_node, 0, minCost, current_path, visited, best_path);
    end = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed_seconds = end - start;
    cout << "-----------------------------DFS-----------------------------" << endl;
    cout << "Minimum cost: " << minCost << endl;
    cout << "Path: ";
    for (const string& node : best_path) {
        cout << node << " ";
    }
    cout << "\nTime taken by DFS: " << elapsed_seconds.count() << endl;

    // UCS
    minCost = INT_MAX;
    current_path.clear();
    best_path.clear();
    start = chrono::high_resolution_clock::now();
    minCost = TravellingSalesmanProblemUCS(graph, start_node, best_path);
    end = chrono::high_resolution_clock::now();
    elapsed_seconds = end - start;
    cout << "-----------------------------UCS-----------------------------" << endl;
    cout << "Minimum cost: " << minCost << endl;
    cout << "Path: ";
    for (const string& node : best_path) {
        cout << node << " ";
    }
    cout << "\nTime taken by UCS: " << elapsed_seconds.count() << endl;

    // A*
    minCost = INT_MAX;
    current_path.clear();
    best_path.clear();
    start = chrono::high_resolution_clock::now();
    minCost = TravellingSalesmanProblemAStar(graph, start_node, best_path);
    end = chrono::high_resolution_clock::now();
    elapsed_seconds = end - start;
    cout << "-----------------------------A*-----------------------------" << endl;
    cout << "Minimum cost: " << minCost << endl;
    cout << "Path: ";
    for (const string& node : best_path) {
        cout << node << " ";
    }
    cout << "\nTime taken by A*: " << elapsed_seconds.count() << endl;

    return 0;
}
