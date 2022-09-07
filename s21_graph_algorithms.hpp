#ifndef SRC_S21_GRAPH_SLGORITHMS_HPP_
#define SRC_S21_GRAPH_SLGORITHMS_HPP_

#include <ctime>
#include <set>
#include <vector>

#include "queue/s21_queue.hpp"
#include "s21_graph.hpp"
#include "stack/s21_stack.hpp"

struct TsmResult {
  std::vector<int> vertices;
  double distance;
};

class GraphAlgorithms {
  unsigned random_seed_{(unsigned)time(0)};

 public:
  /*---обход графа в глубину---*/
  std::vector<int> DepthFirstSearch(const Graph &graph, const int &startVertex);
  /*---обход графа в ширину---*/
  std::vector<int> BreadthFirstSearch(const Graph &graph,
                                      const int &startVertex);
  /*---возвращает вектор вершин кратчайшего пути---*/
  std::vector<int> GetVectorShortestPathBetweenVertices(const Graph &graph,
                                                        const int &vertex1,
                                                        const int &vertex2);
  /*---возвращает значение кратчайшего пути---*/
  double GetShortestPathBetweenVertices(const Graph &graph, const int &vertex1,
                                        const int &vertex2);
  /*---возвращает матрицу смежности кратчайших путей между всеми вершинами---*/
  Matrix<double> GetShortestPathsBetweenAllVertices(const Graph &graph);
  /*---возвращает матрицу смежностиминимального остновного дерева---*/
  Matrix<double> GetLeastSpanningTree(const Graph &graph);
  /*---алгоритмы на решение задачи коммивояжера---*/
  TsmResult SolveTravelingSalesmanProblem(const Graph &graph);
  TsmResult SolveTsmBrute(const Graph &graph);
  TsmResult SolveTsmNearestNeighbour(const Graph &graph);

 private:
  /*---приватные фунции для обхода графа в глубину---*/
  void RebuildUnVisitedVertexStack_(s21::Stack<int> *unVisitedVertex,
                                    const int &vertex);
  /*---приватные фунции для поиска кратчайшего пути между двум] вершинами---*/
  std::vector<int> InverseVector(const std::vector<int> &current);
  /*---приватные фунции для поиска минимального остновного дерева---*/
  int GetRandomVertex_(std::set<int> *unVisitedVertex);
  bool IsHaveVectorEdge_(const std::vector<std::pair<int, int>> &vecEdge,
                         const std::pair<int, int> &edge);
  std::pair<int, int> FindMinimalEdge_(
      const std::vector<std::pair<int, int>> &edgeNeighbors,
      const Matrix<double> &matrixGraph);
  Matrix<double> GenerateMatrixSpanningTree_(
      const std::vector<std::pair<int, int>> &vectorSpanningTree,
      const Matrix<double> &matrixGraph);
  /*---приватные фунции для задачи коммивояжера---*/
  static constexpr double kEvaporationCoef = 0.8;
  TsmResult GenerateRandomPath(const Matrix<double> &paths, unsigned *seed);
  TsmResult GeneratePopularPath(const Matrix<double> &paths, unsigned *seed,
                                Matrix<double> *phero);
  void EvaporatePheromones(Matrix<double> *mx);
  void LeavingTrails(const TsmResult &path, Matrix<double> *phero);
  TsmResult InsertActualPaths(const TsmResult &data, const Graph &graph);
  int Roll(int base, unsigned *seed);
  TsmResult RecBruteSearch(int cur, std::set<int> future,
                           const Matrix<double> &paths);
};

#endif  // SRC_S21_GRAPH_SLGORITHMS_HPP_
