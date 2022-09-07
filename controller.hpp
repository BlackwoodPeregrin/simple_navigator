#ifndef SRC_CONTROLLER_HPP_
#define SRC_CONTROLLER_HPP_

#include "s21_graph_algorithms.hpp"

class Controller {
 public:
  enum TsmAlg { kAnts, kBrute, kNeighbour };

  Controller();
  Controller(const Controller &other);
  Controller(Controller &&other);
  ~Controller();
  void operator=(const Controller &other);
  void operator=(Controller &&other);

  std::pair<Matrix<double>, std::string> GetAdjacencyMatrix();
  std::pair<bool, std::string> GetTypeGraph();
  std::pair<bool, std::string> GetTypeWeightedGraph();

  std::pair<std::string, bool> GenerateOrientedGraph(const int &size);
  std::pair<std::string, bool> GenerateUnorientedWeightedGraph(const int &size);
  std::pair<std::string, bool> GenerateUnorientedGraph(const int &size);
  std::pair<std::string, bool> LoadGraphFromFile(const std::string &file_name);
  std::pair<std::string, bool> SaveGraphToFileDot(const std::string &file_name);
  std::pair<std::string, bool> SaveGraphToFileTxt(const std::string &file_name);
  std::pair<std::string, bool> BreadthFirstSearch(const int &startVertex);
  std::pair<std::string, bool> DepthFirstSearch(const int &startVertex);
  std::pair<std::string, bool> GetVectorShortestPathBetweenVertices(
      const int &vertex1, const int &vertex2);
  std::pair<Matrix<double>, std::string> GetShortestPathBetweenAllVertices();
  std::pair<Matrix<double>, std::string> GetLeastSpanningTree();

  std::pair<std::string, bool> SolveTsm(unsigned mode = kAnts);
  std::pair<std::string, bool> SolveTsmFast(unsigned mode = kAnts);

 private:
  Graph *graph_;
  GraphAlgorithms graph_alg_;
};

#endif  // SRC_CONTROLLER_HPP_
