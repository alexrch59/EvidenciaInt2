#define CATCH_CONFIG_MAIN  // Esto crea el punto de entrada principal para Catch2
#include <catch2/catch.hpp>
#include <cmath>
#include <vector>
#include <tuple>

// Incluye tu código de producción
#include "main.cpp"

// Prueba de ejemplo para el cálculo de la distancia
TEST_CASE("Cálculo de distancia entre dos puntos", "[distance]") {
    std::pair<int, int> house1 = {0, 0};
    std::pair<int, int> house2 = {3, 4};
    double expectedDistance = 5.0;
    
    double computedDistance = sqrt(pow(house1.first - house2.first, 2) + pow(house1.second - house2.second, 2));
    
    REQUIRE(computedDistance == Approx(expectedDistance).epsilon(0.01));
}

// Prueba de ejemplo para el algoritmo de Kruskal
TEST_CASE("Prueba de Kruskal - Grafo de ejemplo", "[kruskal]") {
    std::vector<std::vector<int>> adjMatrix = {
        {0, 10, 0, 0, 0},
        {10, 0, 20, 0, 0},
        {0, 20, 0, 30, 0},
        {0, 0, 30, 0, 40},
        {0, 0, 0, 40, 0}
    };
    
    Graph graph(5, adjMatrix);
    auto mst = kruskal(graph);
    
    // Verifica que los arcos estén en el orden correcto según el peso
    REQUIRE(mst.size() == 4);  // Debe haber 4 arcos en el MST
    REQUIRE(mst[0].weight == 10);
    REQUIRE(mst[1].weight == 20);
    REQUIRE(mst[2].weight == 30);
    REQUIRE(mst[3].weight == 40);
}

// Prueba de ejemplo para el algoritmo de Ford-Fulkerson
TEST_CASE("Ford-Fulkerson - Flujo máximo de un grafo", "[fordFulkerson]") {
    std::vector<std::vector<int>> capacityMatrix = {
        {0, 10, 10, 0, 0},
        {0, 0, 5, 15, 0},
        {0, 0, 0, 10, 10},
        {0, 0, 0, 0, 10},
        {0, 0, 0, 0, 0}
    };
    
    FlowGraph flowGraph(5, capacityMatrix);
    
    // El flujo máximo entre el nodo 0 y el nodo 4 debería ser 20
    int maxFlow = fordFulkerson(flowGraph, 0, 4);
    REQUIRE(maxFlow == 20);
}
