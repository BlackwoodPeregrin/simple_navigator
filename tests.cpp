#include <gtest/gtest.h>

#include <iostream>

#include "s21_graph.hpp"
#include "s21_graph_algorithms.hpp"

TEST(graph, constructors) {
  Graph A;
  Graph B(A);
  Graph C(std::move(A));
}

TEST(graph, save_load) {
  Graph A;
  A.LoadGraphFromFile("graph_txt/5.txt");
  A.ExportGraphToDot("graph_dot/5.dot");
  A.LoadGraphFromFile("graph_txt/4.txt");
  A.ExportGraphToDot("graph_dot/4.dot");
}

TEST(graph, sizes) {
  Graph A;
  A.LoadGraphFromFile("graph_txt/5.txt");
  ASSERT_TRUE(A.IsGraphInizialize());
  ASSERT_EQ(A.MinVertex(), 1);
  ASSERT_EQ(A.MaxVertex(), 5);
}

TEST(graph, properties) {
  Graph A;
  A.LoadGraphFromFile("graph_txt/5.txt");
  ASSERT_EQ(A.GraphType(), Graph::ORIENTED);
  ASSERT_EQ(A.GraphWeights(), Graph::WEIGHTED);
  ASSERT_TRUE(A.PositiveWeights());
}

TEST(graph, matrix) {
  Graph A;
  A.LoadGraphFromFile("graph_txt/5.txt");
  ASSERT_DOUBLE_EQ(A.graphMatrix()(0, 0), 0.0);
  ASSERT_DOUBLE_EQ(A.graphMatrix()(1, 2), 5.0);
  ASSERT_DOUBLE_EQ(A.graphMatrix()(2, 4), 10.0);
}

TEST(graph, generation) {
  Graph A;
  A.GenerateOrientedGraph(5);
  ASSERT_EQ(A.MinVertex(), 1);
  ASSERT_EQ(A.MaxVertex(), 5);
  ASSERT_EQ(A.GraphType(), Graph::ORIENTED);
  ASSERT_EQ(A.GraphWeights(), Graph::WEIGHTED);
  ASSERT_TRUE(A.PositiveWeights());

  A.GenerateUnorientedGraph(5, false);
  ASSERT_EQ(A.MinVertex(), 1);
  ASSERT_EQ(A.MaxVertex(), 5);
  ASSERT_EQ(A.GraphType(), Graph::UNORIENTED);
  ASSERT_EQ(A.GraphWeights(), Graph::UNWEIGHTED);
  ASSERT_TRUE(A.PositiveWeights());

  A.GenerateUnorientedGraph(4, true);
  ASSERT_EQ(A.MaxVertex(), 4);
  ASSERT_EQ(A.GraphType(), Graph::UNORIENTED);
  ASSERT_EQ(A.GraphWeights(), Graph::WEIGHTED);
  ASSERT_TRUE(A.PositiveWeights());
}

TEST(algorithm, depth_first) {
  Graph A;
  A.LoadGraphFromFile("graph_txt/4.txt");
  GraphAlgorithms gg;
  std::vector<int> res = gg.DepthFirstSearch(A, 1);
  std::vector<int> exp{1, 3, 4, 5, 2};
  ASSERT_EQ(res, exp);
}

TEST(algorithm, breadth_first) {
  Graph A;
  A.LoadGraphFromFile("graph_txt/4.txt");
  GraphAlgorithms gg;
  std::vector<int> res = gg.BreadthFirstSearch(A, 1);
  std::vector<int> exp{1, 2, 3, 4, 5};
  ASSERT_EQ(res, exp);
}

TEST(algorithm, shortest_path) {
  Graph A;
  A.LoadGraphFromFile("graph_txt/4.txt");
  GraphAlgorithms gg;
  std::vector<int> res_path = gg.GetVectorShortestPathBetweenVertices(A, 1, 5);
  std::vector<int> exp_path{1, 3, 4, 5};
  double res_length = gg.GetShortestPathBetweenVertices(A, 1, 5);
  Matrix<double> res_mx = gg.GetShortestPathsBetweenAllVertices(A);

  ASSERT_EQ(res_path, exp_path);
  ASSERT_DOUBLE_EQ(res_length, 3.0);
  ASSERT_DOUBLE_EQ(res_mx(0, 4), 3.0);
  ASSERT_DOUBLE_EQ(res_mx(1, 2), 2.0);
  ASSERT_DOUBLE_EQ(res_mx(0, 1), 3.0);
}

TEST(algorithm, spanning_tree) {
  Graph A;
  A.LoadGraphFromFile("graph_txt/4.txt");
  GraphAlgorithms gg;
  Matrix<double> res = gg.GetLeastSpanningTree(A);
  ASSERT_DOUBLE_EQ(res(0, 1), 0.0);
  ASSERT_DOUBLE_EQ(res(0, 2), 1.0);
  ASSERT_DOUBLE_EQ(res(2, 0), 1.0);
  ASSERT_DOUBLE_EQ(res(3, 1), 1.0);
  ASSERT_DOUBLE_EQ(res(4, 3), 1.0);
}

TEST(algorithm, tsm_complete) {
  Graph A;
  A.LoadGraphFromFile("graph_txt/5.txt");
  GraphAlgorithms gg;
  TsmResult res_ants = gg.SolveTravelingSalesmanProblem(A);
  TsmResult res_brut = gg.SolveTsmBrute(A);
  TsmResult res_nrst = gg.SolveTsmNearestNeighbour(A);
  ASSERT_LE(res_brut.distance, res_ants.distance);
  ASSERT_LE(res_brut.distance, res_nrst.distance);
  ASSERT_EQ(res_ants.vertices.size(), 6);
  ASSERT_EQ(res_brut.vertices.size(), 6);
  ASSERT_EQ(res_nrst.vertices.size(), 6);
  ASSERT_EQ(res_ants.vertices[0], res_ants.vertices[5]);
  ASSERT_NE(res_ants.vertices[0], res_ants.vertices[2]);
}

TEST(algorithm, tsm_incomplete) {
  Graph A;
  A.LoadGraphFromFile("graph_txt/4.txt");
  GraphAlgorithms gg;
  TsmResult res_ants = gg.SolveTravelingSalesmanProblem(A);
  ASSERT_LE(res_ants.vertices.size(), 9);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
