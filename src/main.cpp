#include <iostream>
#include <vector>
#include <tuple>
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <numeric> // Para std::iota

using namespace std;

// Estructura para representar un arco
struct Edge {
    int u, v, weight;
    bool operator<(const Edge& other) const {
        return weight < other.weight;
    }
};

// Estructura para representar un grafo
class Graph {
public:
    int n;
    vector<vector<int>> adjMatrix;

    Graph(int size, vector<vector<int>>& matrix) : n(size), adjMatrix(matrix) {}
};

// Estructura para representar un grafo con capacidades para flujo
class FlowGraph {
public:
    int n;
    vector<vector<int>> capacity;

    FlowGraph(int size, vector<vector<int>>& matrix) : n(size), capacity(matrix) {}
};

// Función para encontrar el representante de un nodo (Union-Find)
int findParent(int u, vector<int>& parent) {
    if (u != parent[u]) {
        parent[u] = findParent(parent[u], parent); // Compresión de camino
    }
    return parent[u];
}

// Función para encontrar el MST usando el algoritmo de Kruskal
vector<Edge> kruskal(Graph& graph) {
    vector<Edge> edges;
    for (int i = 0; i < graph.n; i++) {
        for (int j = i + 1; j < graph.n; j++) {
            if (graph.adjMatrix[i][j] > 0) {
                edges.push_back({i, j, graph.adjMatrix[i][j]});
            }
        }
    }
    sort(edges.begin(), edges.end());

    vector<int> parent(graph.n);
    iota(parent.begin(), parent.end(), 0);

    vector<Edge> mst;
    for (const auto& edge : edges) {
        int uRoot = findParent(edge.u, parent);
        int vRoot = findParent(edge.v, parent);
        if (uRoot != vRoot) {
            mst.push_back(edge);
            parent[uRoot] = vRoot;
        }
    }
    return mst;
}

// Algoritmo TSP (Fuerza bruta)
pair<vector<int>, int> travelingSalesman(vector<vector<int>>& dist) {
    int n = dist.size();
    vector<int> cities(n);
    iota(cities.begin(), cities.end(), 0);

    vector<int> bestRoute;
    int minCost = numeric_limits<int>::max();

    do {
        int currentCost = 0;
        for (int i = 0; i < n - 1; i++) {
            currentCost += dist[cities[i]][cities[i + 1]];
        }
        currentCost += dist[cities[n - 1]][cities[0]];
        if (currentCost < minCost) {
            minCost = currentCost;
            bestRoute = cities;
        }
    } while (next_permutation(cities.begin(), cities.end()));

    return {bestRoute, minCost};
}

// Algoritmo Ford-Fulkerson para flujo máximo
int fordFulkerson(FlowGraph& graph, int source, int sink) {
    vector<vector<int>> residual = graph.capacity;
    vector<int> parent(graph.n);

    auto bfs = [&]() {
        fill(parent.begin(), parent.end(), -1);
        queue<int> q;
        q.push(source);
        parent[source] = source;

        while (!q.empty()) {
            int u = q.front();
            q.pop();

            for (int v = 0; v < graph.n; v++) {
                if (parent[v] == -1 && residual[u][v] > 0) {
                    parent[v] = u;
                    if (v == sink) return true;
                    q.push(v);
                }
            }
        }
        return false;
    };

    int maxFlow = 0;
    while (bfs()) {
        int pathFlow = numeric_limits<int>::max();
        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            pathFlow = min(pathFlow, residual[u][v]);
        }
        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            residual[u][v] -= pathFlow;
            residual[v][u] += pathFlow;
        }
        maxFlow += pathFlow;
    }
    return maxFlow;
}

// Búsqueda lineal para encontrar la central más cercana
vector<pair<int, int>> nearestCentral(vector<pair<int, int>>& houses, vector<pair<int, int>>& centrals) {
    vector<pair<int, int>> closest(houses.size());
    for (size_t i = 0; i < houses.size(); i++) {
        double minDistance = numeric_limits<double>::max();
        pair<int, int> nearest;
        for (const auto& central : centrals) {
            double distance = sqrt(pow(houses[i].first - central.first, 2) + pow(houses[i].second - central.second, 2));
            if (distance < minDistance) {
                minDistance = distance;
                nearest = central;
            }
        }
        closest[i] = nearest;
    }
    return closest;
}

int main() {
    int n;
    cin >> n;

    vector<vector<int>> distanceMatrix(n, vector<int>(n));
    vector<vector<int>> capacityMatrix(n, vector<int>(n));

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            cin >> distanceMatrix[i][j];
        }
    }

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            cin >> capacityMatrix[i][j];
        }
    }

    vector<pair<int, int>> houses(n);
    for (int i = 0; i < n; i++) {
        cin >> houses[i].first >> houses[i].second;
    }

    Graph graph(n, distanceMatrix);
    auto mst = kruskal(graph);
    cout << "1. Forma de cablear las colonias con fibra:" << endl;
    for (const auto& edge : mst) {
        cout << "(" << char('A' + edge.u) << ", " << char('A' + edge.v) << ")" << endl;
    }

    auto tspResult = travelingSalesman(distanceMatrix);
    cout << "2. Ruta a seguir por el personal: ";
    for (int city : tspResult.first) {
        cout << char('A' + city) << " ";
    }
    cout << char('A' + tspResult.first[0]) << endl;

    FlowGraph flowGraph(n, capacityMatrix);
    cout << "3. Flujo máximo de información: " << fordFulkerson(flowGraph, 0, n - 1) << endl;

    cout << "4. Centrales más cercanas:" << endl;
    auto closestCentrals = nearestCentral(houses, houses);
    for (const auto& central : closestCentrals) {
        cout << "(" << central.first << ", " << central.second << ")" << endl;
    }

    return 0;
}

