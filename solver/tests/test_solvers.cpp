#include "solvers/astar_solver.h"
#include "solvers/bfs_solver.h"

#include "graph/graph.h"

#include "models/models.h"
#include "models/test/test.h"

#include <gtest/gtest.h>

#include <chrono>
#include <iostream>

using namespace mapf::graph;
using namespace mapf::models;
using namespace mapf::solver;

// Builds a width x height grid graph with bidirectional edges
Graph BuildGridGraph(uint32_t width, uint32_t height) {
    Nodes nodes;
    Edges edges;
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            NodeId id = y * width + x;
            nodes.emplace(id, Node(id));
            if (x + 1 < width) {
                NodeId right = id + 1;
                edges.emplace(Edge(id, right));
                edges.emplace(Edge(right, id));
            }
            if (y + 1 < height) {
                NodeId down = id + width;
                edges.emplace(Edge(id, down));
                edges.emplace(Edge(down, id));
            }
        }
    }
    return Graph(std::move(nodes), std::move(edges));
}

// Runs both solvers on the same problem and collects metrics
struct BenchmarkResult {
    size_t bfs_nodes_expanded;
    size_t astar_nodes_expanded;
    double bfs_time_ms;
    double astar_time_ms;
    size_t bfs_makespan;
    size_t astar_makespan;
};

BenchmarkResult RunBenchmark(const MAPFProblem& problem) {
    BenchmarkResult result{};

    BFSSolver bfs;
    AStarSolver astar;

    auto t0 = std::chrono::high_resolution_clock::now();
    auto bfs_solution = bfs.FindSolution(problem);
    auto t1 = std::chrono::high_resolution_clock::now();
    auto astar_solution = astar.FindSolution(problem);
    auto t2 = std::chrono::high_resolution_clock::now();

    result.bfs_time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.astar_time_ms =
        std::chrono::duration<double, std::milli>(t2 - t1).count();

    result.bfs_nodes_expanded = bfs.GetNodesExpanded();
    result.astar_nodes_expanded = astar.GetNodesExpanded();

    if (!bfs_solution.agent_paths.empty()) {
        result.bfs_makespan = bfs_solution.agent_paths.begin()->second.path.size() - 1;
    }
    if (!astar_solution.agent_paths.empty()) {
        result.astar_makespan = astar_solution.agent_paths.begin()->second.path.size() - 1;
    }

    return result;
}

void PrintBenchmark(const std::string& name, const BenchmarkResult& r) {
    std::cout << "\n--- " << name << " ---" << std::endl;
    std::cout << "  Nodes expanded:  BFS=" << r.bfs_nodes_expanded
              << "  A*=" << r.astar_nodes_expanded
              << "  (A* is " << (r.bfs_nodes_expanded / std::max(r.astar_nodes_expanded, size_t(1)))
              << "x fewer)" << std::endl;
    std::cout << "  Time (ms):       BFS=" << r.bfs_time_ms
              << "  A*=" << r.astar_time_ms << std::endl;
    std::cout << "  Makespan:        BFS=" << r.bfs_makespan
              << "  A*=" << r.astar_makespan << std::endl;
}

// Correctness tests from starter code (4 nodes, 2 agents)

class MAPFProblemTest1 : public testing::Test {
  protected:
    MAPFProblemTest1() {
        auto nodes = Nodes{{0, Node(0)}, {1, Node(1)}, {2, Node(2)}, {3, Node(3)}};
        auto edges = Edges{
            Edge(0, 1),
            Edge(1, 3),
            Edge(3, 2),
            Edge(2, 0),
            Edge(1, 0),
            Edge(3, 1),
            Edge(2, 3),
            Edge(0, 2)};
        auto graph = Graph(std::move(nodes), std::move(edges));
        problem = MAPFProblem({std::move(graph), {{0, {0, {0, 3}}}, {1, {1, {3, 0}}}}});
    }

    MAPFProblem problem;
};

class MAPFProblemTest2 : public testing::Test {
  protected:
    MAPFProblemTest2() {
        auto nodes = Nodes{{0, Node(0)}, {1, Node(1)}, {2, Node(2)}, {3, Node(3)}};
        auto edges = Edges{Edge(0, 1), Edge(1, 0), Edge(1, 2), Edge(2, 1), Edge(1, 3), Edge(3, 1)};
        auto graph = Graph(std::move(nodes), std::move(edges));
        problem = MAPFProblem({std::move(graph), {{0, {0, {0, 2}}}, {1, {1, {2, 0}}}}});
    }

    MAPFProblem problem;
};

TEST_F(MAPFProblemTest1, BFSSolver) {
    auto solver = BFSSolver();
    ASSERT_VALID_SOLUTION(problem, solver.FindSolution(problem));
}

TEST_F(MAPFProblemTest1, AStarSolver) {
    auto solver = AStarSolver();
    ASSERT_VALID_SOLUTION(problem, solver.FindSolution(problem));
}

TEST_F(MAPFProblemTest2, BFSSolver) {
    auto solver = BFSSolver();
    ASSERT_VALID_SOLUTION(problem, solver.FindSolution(problem));
}

TEST_F(MAPFProblemTest2, AStarSolver) {
    auto solver = AStarSolver();
    ASSERT_VALID_SOLUTION(problem, solver.FindSolution(problem));
}

// Benchmark: small grid, minimal difference expected
TEST(Benchmark, Grid3x3_2Agents) {
    auto graph = BuildGridGraph(3, 3);
    AgentTasks tasks{
        {0, {0, {0, 8}}},
        {1, {1, {8, 0}}},
    };
    MAPFProblem problem(std::move(graph), std::move(tasks));

    BFSSolver bfs;
    AStarSolver astar;
    ASSERT_VALID_SOLUTION(problem, bfs.FindSolution(problem));
    ASSERT_VALID_SOLUTION(problem, astar.FindSolution(problem));

    auto r = RunBenchmark(problem);
    PrintBenchmark("3x3 grid, 2 agents", r);

    EXPECT_LE(astar.GetNodesExpanded(), bfs.GetNodesExpanded());
    EXPECT_EQ(r.bfs_makespan, r.astar_makespan);
}

// Benchmark: corridor with a single side pocket (bottleneck)
// 0 — 1 — 2 — 3 — 4 — 5 — 6 — 7
//             |
//             8
// Agents swap ends: 0->7 and 7->0. Only node 8 allows them to pass.
TEST(Benchmark, CorridorBottleneck_2Agents) {
    Nodes nodes;
    Edges edges;
    for (uint32_t i = 0; i < 9; ++i) {
        nodes.emplace(i, Node(i));
    }
    // main corridor
    for (uint32_t i = 0; i < 7; ++i) {
        edges.emplace(Edge(i, i + 1));
        edges.emplace(Edge(i + 1, i));
    }
    // side pocket at node 3
    edges.emplace(Edge(3, 8));
    edges.emplace(Edge(8, 3));
    auto graph = Graph(std::move(nodes), std::move(edges));

    AgentTasks tasks{
        {0, {0, {0, 7}}},
        {1, {1, {7, 0}}},
    };
    MAPFProblem problem(std::move(graph), std::move(tasks));

    BFSSolver bfs;
    AStarSolver astar;
    ASSERT_VALID_SOLUTION(problem, bfs.FindSolution(problem));
    ASSERT_VALID_SOLUTION(problem, astar.FindSolution(problem));

    auto r = RunBenchmark(problem);
    PrintBenchmark("corridor with bottleneck, 2 agents", r);

    EXPECT_LE(astar.GetNodesExpanded(), bfs.GetNodesExpanded());
    EXPECT_EQ(r.bfs_makespan, r.astar_makespan);
}

// Benchmark: 3 agents, larger state space
TEST(Benchmark, Grid4x4_3Agents) {
    auto graph = BuildGridGraph(4, 4);
    AgentTasks tasks{
        {0, {0, {0, 15}}},
        {1, {1, {3, 12}}},
        {2, {2, {12, 3}}},
    };
    MAPFProblem problem(std::move(graph), std::move(tasks));

    BFSSolver bfs;
    AStarSolver astar;
    ASSERT_VALID_SOLUTION(problem, bfs.FindSolution(problem));
    ASSERT_VALID_SOLUTION(problem, astar.FindSolution(problem));

    auto r = RunBenchmark(problem);
    PrintBenchmark("4x4 grid, 3 agents", r);

    EXPECT_LE(astar.GetNodesExpanded(), bfs.GetNodesExpanded());
    EXPECT_EQ(r.bfs_makespan, r.astar_makespan);
}

// Benchmark: 4 agents, all doing diagonal swaps — hardest case
TEST(Benchmark, Grid4x4_4Agents) {
    auto graph = BuildGridGraph(4, 4);
    AgentTasks tasks{
        {0, {0, {0, 15}}},
        {1, {1, {15, 0}}},
        {2, {2, {3, 12}}},
        {3, {3, {12, 3}}},
    };
    MAPFProblem problem(std::move(graph), std::move(tasks));

    BFSSolver bfs;
    AStarSolver astar;
    ASSERT_VALID_SOLUTION(problem, bfs.FindSolution(problem));
    ASSERT_VALID_SOLUTION(problem, astar.FindSolution(problem));

    auto r = RunBenchmark(problem);
    PrintBenchmark("4x4 grid, 4 agents", r);

    EXPECT_LE(astar.GetNodesExpanded(), bfs.GetNodesExpanded());
    EXPECT_EQ(r.bfs_makespan, r.astar_makespan);
}
