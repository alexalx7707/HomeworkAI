#pragma once
#include <string>
#include <vector>
#include <map>
#include <set>
#include <utility>

using namespace std;
// Function prototypes
map<string, vector<pair<string, int>>> generateManualGraph(int nodes, int edges);
map<string, vector<pair<string, int>>> generateRandomGraph(int nodes, int edges);
void printGraph(const map<string, vector<pair<string, int>>>& graph);

void TravellingSalesmanProblemDFS(const map<string, vector<pair<string, int>>>& graph, const string& start, const string& current, int cost, int& min_cost, vector<string>& current_path, set<string>& visited, vector<string>& best_path);

int TravellingSalesmanProblemUCS(const map<string, vector<pair<string, int>>>& graph, const string& start, vector<string>& best_path);

int TravellingSalesmanProblemAStar(const map<string, vector<pair<string, int>>>& graph, const string& start, vector<string>& best_path);
int mstCost(const map<string, vector<pair<string, int>>>& graph, const set<string>& unvisited, const string& start);
int heuristic(const map<string, vector<pair<string, int>>>& graph, const string& current, const set<string>& unvisited, const string& start);
