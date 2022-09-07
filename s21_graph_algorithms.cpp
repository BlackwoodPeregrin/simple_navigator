#include "s21_graph_algorithms.hpp"

#include <stdlib.h>

#include <array>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <map>

std::vector<int> GraphAlgorithms::DepthFirstSearch(const Graph &graph,
                                                   const int &startVertex) {
  if (startVertex > graph.MaxVertex() || startVertex < graph.MinVertex()) {
    throw std::invalid_argument("Start vertex out of range graph.");
  }

  /*---матрица смежности входящего графа---*/
  Matrix<double> dataGraph(graph.graphMatrix());
  /*---посещенные веришны---*/
  std::set<int> visitedVertex;
  /*---не посещенные веришны---*/
  s21::Stack<int> unVisitedVertex{startVertex};
  /*---резулультат обхода вершин в порядке их обхода---*/
  std::vector<int> resultPath;

  while (!unVisitedVertex.empty()) {
    int curVertex = unVisitedVertex.pop();
    visitedVertex.insert(curVertex);
    resultPath.push_back(curVertex);
    for (int j = 0; j < dataGraph.get_columns(); ++j) {
      if (dataGraph(curVertex - 1, j) != 0) {
        int potencialVertex = j + 1;
        /*---если вершина не посещена, то добавляем ее в стэк---*/
        if (visitedVertex.find(potencialVertex) == visitedVertex.end()) {
          RebuildUnVisitedVertexStack_(&unVisitedVertex, potencialVertex);
        }
      }
    }
  }
  return resultPath;
}

std::vector<int> GraphAlgorithms::BreadthFirstSearch(const Graph &graph,
                                                     const int &startVertex) {
  if (startVertex > graph.MaxVertex() || startVertex < graph.MinVertex()) {
    throw std::invalid_argument("Start vertex out of range graph.");
  }

  /*---матрица смежности входящего графа---*/
  Matrix<double> dataGraph(graph.graphMatrix());
  /*---посещенные веришны---*/
  std::set<int> visitedVertex;
  /*---не посещенные веришны---*/
  s21::Queue<int> unVisitedVertex{startVertex};
  /*---резулультат обхода вершин в порядке их обхода---*/
  std::vector<int> resultPath;

  while (!unVisitedVertex.empty()) {
    int curVertex = unVisitedVertex.pop();
    if (visitedVertex.insert(curVertex).second == true) {
      resultPath.push_back(curVertex);
    }
    for (int j = 0; j < dataGraph.get_columns(); ++j) {
      if (dataGraph(curVertex - 1, j) != 0) {
        int potencialVertex = j + 1;
        /*---если вершина не посещена, то добавляем ее в очередь---*/
        if (visitedVertex.find(potencialVertex) == visitedVertex.end()) {
          unVisitedVertex.push(potencialVertex);
        }
      }
    }
  }
  return resultPath;
}

std::vector<int> GraphAlgorithms::GetVectorShortestPathBetweenVertices(
    const Graph &graph, const int &vertex1, const int &vertex2) {
  if (vertex1 > graph.MaxVertex() || vertex1 < graph.MinVertex() ||
      vertex2 > graph.MaxVertex() || vertex2 < graph.MinVertex()) {
    throw std::invalid_argument("Vertexes out of range graph.");
  }

  if (graph.PositiveWeights() == false) {
    throw std::invalid_argument("Weigts grath is negative");
  }

  /*---текущая вершина---*/
  int curVertex = vertex1;
  /*---кратчайший путь до текущей вершины---*/
  double curLenPath = 0;
  /*---переменная для хранения бесконечности---*/
  double inf = std::numeric_limits<double>::infinity();
  /*---матрица смежности входящего графа---*/
  Matrix<double> dataGraph(graph.graphMatrix());

  /*---key - посещенная вершина, value - кратчайший путь до нее---*/
  std::set<int> visitedVertex{curVertex};
  /*---key - вершина, value - родитель вершины по кратчайшему пути---*/
  std::map<int, int> visitedVertexAndParent;
  for (int i = 0; i < dataGraph.get_columns(); ++i) {
    int vertex = i + 1;
    if (vertex == vertex1) {
      /*---стартовая вершина является роидителем самой себе---*/
      visitedVertexAndParent.insert({vertex, vertex});
    } else {
      /*---родитель пока что не установлен---*/
      visitedVertexAndParent.insert({vertex, inf});
    }
  }

  /*---временные значения расстояний до всех вершин---*/
  std::vector<double> searchPath;

  for (int j = 0; j < dataGraph.get_columns(); ++j) {
    if (j == curVertex - 1) {
      searchPath.push_back(0);
    } else {
      searchPath.push_back(inf);  // пути между вершинами нет
    }
  }

  while (true) {
    for (size_t i = 0; i < searchPath.size(); ++i) {
      int vertex = i + 1;
      if (visitedVertex.find(vertex) == visitedVertex.end()) {
        if (dataGraph(curVertex - 1, vertex - 1) != 0) {
          double possibleLenPath =
              curLenPath + dataGraph(curVertex - 1, vertex - 1);
          if (possibleLenPath < searchPath[i]) {
            searchPath[i] = possibleLenPath;
            /*---обновляем родителя для текущей вершины---*/
            visitedVertexAndParent[vertex] = curVertex;
          }
        }
      }
    }
    //
    std::map<int, double> unique;
    for (size_t i = 0; i < searchPath.size(); ++i) {
      int vertex = i + 1;
      double valuePath = searchPath[i];
      if (valuePath != inf &&
          visitedVertex.find(vertex) == visitedVertex.end()) {
        unique.insert({vertex, valuePath});
      }
    }
    //
    if (unique.size() == 0) {
      break;  // путей больше нет
    }
    //
    auto iter = unique.begin();
    curVertex = (*iter).first;
    curLenPath = (*iter).second;
    for (++iter; iter != unique.end(); ++iter) {
      if (curLenPath > (*iter).second) {
        curVertex = (*iter).first;
        curLenPath = (*iter).second;
      }
    }
    //
    visitedVertex.insert(curVertex);
    /*---если нашли минимальный путь не обходя все вершины то выходим из
     * цикла---*/
    if (visitedVertex.find(vertex2) != visitedVertex.end()) {
      break;
    }
  }

  if (visitedVertex.find(vertex2) == visitedVertex.end()) {
    throw std::invalid_argument(
        "Can't build path beetween vertexes: " + std::to_string(vertex1) +
        " -> " + std::to_string(vertex2));
  }

  std::vector<int> shortestPath;
  auto iter1 = visitedVertexAndParent.find(vertex2);
  while ((*iter1).first != vertex1) {
    shortestPath.push_back((*iter1).first);
    iter1 = visitedVertexAndParent.find((*iter1).second);
  }
  shortestPath.push_back(vertex1);

  return InverseVector(shortestPath);
}

double GraphAlgorithms::GetShortestPathBetweenVertices(const Graph &graph,
                                                       const int &vertex1,
                                                       const int &vertex2) {
  std::vector<int> shortestPath =
      GetVectorShortestPathBetweenVertices(graph, vertex1, vertex2);
  //
  Matrix<double> dataGraph(graph.graphMatrix());
  double valueShortestPath = 0;
  //
  for (size_t i = 0; i < shortestPath.size() - 1; ++i) {
    int curVertex = shortestPath[i];
    int nextVertex = shortestPath[i + 1];
    valueShortestPath += dataGraph(curVertex - 1, nextVertex - 1);
  }

  return valueShortestPath;
}

Matrix<double> GraphAlgorithms::GetShortestPathsBetweenAllVertices(
    const Graph &graph) {
  if (graph.PositiveWeights() == false) {
    throw std::out_of_range("weigts grath is negative");
  }

  /*---матрица смежности входящего графа---*/
  Matrix<double> dataGraph(graph.graphMatrix());

  /*---заменяем значения 0 если пути нет на бесконечность---*/
  for (int i = 0; i < dataGraph.get_rows(); ++i) {
    for (int j = 0; j < dataGraph.get_columns(); ++j) {
      if (dataGraph(i, j) == 0 && (i != j)) {
        dataGraph(i, j) = std::numeric_limits<double>::infinity();
      }
    }
  }

  for (int k = 0; k < dataGraph.get_rows(); ++k) {
    for (int i = 0; i < dataGraph.get_rows(); ++i) {
      for (int j = 0; j < dataGraph.get_columns(); ++j) {
        if ((dataGraph(i, j) > dataGraph(i, k) + dataGraph(k, j))) {
          dataGraph(i, j) = dataGraph(i, k) + dataGraph(k, j);
        }
      }
    }
  }
  return dataGraph;
}

Matrix<double> GraphAlgorithms::GetLeastSpanningTree(const Graph &graph) {
  if (graph.PositiveWeights() == false) {
    throw std::out_of_range("Weigts grath is negative.");
  }
  if (graph.GraphType() == Graph::GraphType::ORIENTED) {
    throw std::out_of_range("Can't use algoritm with oriented graph.");
  }
  /*---матрица смежности входящего графа---*/
  Matrix<double> dataGraph(graph.graphMatrix());
  /*---не посещенные веришны---*/
  std::set<int> unVisitedVertex;
  /*---в начале алгоритма все вершины являются не посещеными---*/
  for (int vertex = 1; vertex <= dataGraph.get_columns(); ++vertex) {
    unVisitedVertex.insert(vertex);
  }
  /*---веришна с которой начинаем поиск остновного дерева---*/
  int startVertex = GetRandomVertex_(&unVisitedVertex);
  /*---посещенные веришны---*/
  std::set<int> visitedVertex{startVertex};
  /*---вектор ребер минимального остновного дерева---*/
  std::vector<std::pair<int, int>> vectorEdgeResult;
  while (!unVisitedVertex.empty()) {
    std::vector<std::pair<int, int>> edgeNeighbors;
    for (auto iter = visitedVertex.begin(); iter != visitedVertex.end();
         ++iter) {
      int curVertex = (*iter) - 1;
      for (int j = 0; j < dataGraph.get_columns(); ++j) {
        /*---добавляем ребро в вектро соседей только в том случае если этого
         * ребра нет еще в остновном дереве и вершина соединяющая это ребро не
         * была посещена---*/
        if (dataGraph(curVertex, j) != 0 &&
            IsHaveVectorEdge_(vectorEdgeResult, {curVertex + 1, j + 1}) ==
                false &&
            visitedVertex.find(j + 1) == visitedVertex.end()) {
          edgeNeighbors.push_back({curVertex + 1, j + 1});
        }
      }
    }
    std::pair<int, int> minEdge = FindMinimalEdge_(edgeNeighbors, dataGraph);
    /*---добавляем ребро в минимальное остновное дерево---*/
    vectorEdgeResult.push_back(minEdge);
    /*---удаляем вершину к которой ведет ребро из непосещенных---*/
    unVisitedVertex.erase(minEdge.second);
    /*---добавляем вершину к которой ведет ребро в посещенные---*/
    visitedVertex.insert(minEdge.second);
  }

  return GenerateMatrixSpanningTree_(vectorEdgeResult, dataGraph);
}

/*======== приватные фунции для поиска кратчайшего пути между двумя вершинами
 * =================*/

void GraphAlgorithms::RebuildUnVisitedVertexStack_(
    s21::Stack<int> *unVisitedVertex, const int &vertex) {
  std::vector<int> temp;
  while (!unVisitedVertex->empty()) {
    int curStep = unVisitedVertex->pop();
    if (curStep != vertex) {
      temp.push_back(curStep);
    }
  }
  for (auto iter = --temp.end(); iter != --temp.begin(); --iter) {
    unVisitedVertex->push(*iter);
  }
  unVisitedVertex->push(vertex);
}

std::vector<int> GraphAlgorithms::InverseVector(
    const std::vector<int> &current) {
  std::vector<int> inverseVector;
  for (int i = current.size() - 1; i >= 0; --i) {
    inverseVector.push_back(current[i]);
  }
  return inverseVector;
}

/*======== приватные фунции для поиска минимального остновного дерева
 * =================*/

int GraphAlgorithms::GetRandomVertex_(std::set<int> *unVisitedVertex) {
  int vertex = rand_r(&random_seed_) % unVisitedVertex->size() + 1;
  unVisitedVertex->erase(vertex);
  return vertex;
}

bool GraphAlgorithms::IsHaveVectorEdge_(
    const std::vector<std::pair<int, int>> &vecEdge,
    const std::pair<int, int> &edge) {
  //
  std::set<int> valueInputEdge{edge.first, edge.second};
  for (size_t i = 0; i < vecEdge.size(); ++i) {
    std::set<int> valueCurEdge{vecEdge[i].first, vecEdge[i].second};
    if (valueCurEdge == valueInputEdge) {
      return true;
    }
  }
  return false;
}

std::pair<int, int> GraphAlgorithms::FindMinimalEdge_(
    const std::vector<std::pair<int, int>> &edgeNeighbors,
    const Matrix<double> &dataGraph) {
  //
  std::pair<int, int> minEdge = edgeNeighbors.front();
  for (int i = 1; i < (int)edgeNeighbors.size(); ++i) {
    int curRow = minEdge.first - 1;
    int curCol = minEdge.second - 1;
    int potencialRow = edgeNeighbors[i].first - 1;
    int potencialCol = edgeNeighbors[i].second - 1;
    if (dataGraph(curRow, curCol) > dataGraph(potencialRow, potencialCol)) {
      minEdge = edgeNeighbors[i];
    }
  }
  return minEdge;
}

Matrix<double> GraphAlgorithms::GenerateMatrixSpanningTree_(
    const std::vector<std::pair<int, int>> &vectorSpanningTree,
    const Matrix<double> &matrixGraph) {
  Matrix<double> resMatrix(matrixGraph.get_rows(), matrixGraph.get_columns());
  for (size_t i = 0; i < vectorSpanningTree.size(); ++i) {
    std::pair<int, int> edge = vectorSpanningTree[i];
    resMatrix(edge.first - 1, edge.second - 1) =
        matrixGraph(edge.first - 1, edge.second - 1);
    resMatrix(edge.second - 1, edge.first - 1) =
        matrixGraph(edge.second - 1, edge.first - 1);
  }
  return resMatrix;
}

// ========== 4: Travelling salesman problem ==========

TsmResult GraphAlgorithms::SolveTravelingSalesmanProblem(const Graph &graph) {
  TsmResult res{};
  int mx_size = graph.MaxVertex();
  Matrix<double> paths(GetShortestPathsBetweenAllVertices(graph));
  unsigned random_seed = time(0);
  Matrix<double> pheromone(mx_size, mx_size);
  for (int i{}; i < mx_size; i++) {
    for (int j{}; j < mx_size; j++) {
      pheromone(i, j) = 0.5 / mx_size;
    }
  }

  // initial walkthrough
  for (int i{}; i < mx_size * 5; i++) {
    TsmResult path = GenerateRandomPath(paths, &random_seed);
    LeavingTrails(path, &pheromone);
    if (res.distance == 0 || path.distance < res.distance) res = path;
  }

  // repeating walkthroughs
  int silence{};
  while (silence < 5) {
    silence++;
    EvaporatePheromones(&pheromone);
    std::vector<TsmResult> path;
    path.resize(mx_size);
    for (int j{}; j < mx_size; j++) {
      path[j] = GeneratePopularPath(paths, &random_seed, &pheromone);
      if (path[j].distance < res.distance) {
        res = path[j];
        silence = 0;
      }
    }
    for (int j{}; j < mx_size; j++) {
      LeavingTrails(path[j], &pheromone);
    }
  }
  res = InsertActualPaths(res, graph);
  return res;
}

TsmResult GraphAlgorithms::GenerateRandomPath(const Matrix<double> &paths,
                                              unsigned *seed) {
  TsmResult res{};
  int mx_size = paths.get_rows();
  for (int i{}; i < mx_size; i++) res.vertices.push_back(i);
  for (int i(1); i < mx_size - 1; i++) {
    std::swap(res.vertices[i], res.vertices[Roll(mx_size - i, seed) + i]);
  }
  res.vertices.push_back(0);

  auto iter = res.vertices.begin();
  while ((iter + 1) != res.vertices.end()) {
    res.distance += paths(*iter, *(iter + 1));
    iter++;
  }
  return res;
}

TsmResult GraphAlgorithms::GeneratePopularPath(const Matrix<double> &paths,
                                               unsigned *seed,
                                               Matrix<double> *phero) {
  TsmResult res{};
  res.vertices.push_back(0);
  int current_pos = 0;
  int mx_size = paths.get_rows();
  std::set<int> not_visited{};
  for (int i(1); i < mx_size; i++) not_visited.insert(i);

  while (not_visited.size()) {
    std::map<int, double> attractiveness{};
    double total{};
    for (auto i : not_visited) {
      double step_attr =
          (*phero)(current_pos, i) * (1.0 / paths(current_pos, i));
      attractiveness[i] = step_attr;
      total += step_attr;
    }
    double rnd = Roll(1000000, seed) / 1000000.0;
    for (auto i : not_visited) {
      rnd -= attractiveness[i];
      if (rnd < 0) {
        res.distance += paths(current_pos, i);
        current_pos = i;
        not_visited.erase(i);
        res.vertices.push_back(i);
        break;
      }
    }
  }
  res.distance += paths(current_pos, 0);
  res.vertices.push_back(0);
  return res;
}

void GraphAlgorithms::EvaporatePheromones(Matrix<double> *mx) {
  for (int i{}; i < mx->get_rows(); i++) {
    for (int j{}; j < mx->get_columns(); j++) {
      (*mx)(i, j) *= kEvaporationCoef;
    }
  }
}

void GraphAlgorithms::LeavingTrails(const TsmResult &path,
                                    Matrix<double> *phero) {
  auto iter = path.vertices.begin();
  while ((iter + 1) != path.vertices.end()) {
    (*phero)(*iter, *(iter + 1)) += 1.0 / path.distance;
    iter++;
  }
}

TsmResult GraphAlgorithms::InsertActualPaths(const TsmResult &data,
                                             const Graph &graph) {
  TsmResult res{};
  res.distance = data.distance;
  for (size_t i{}; i < data.vertices.size(); i++) {
    res.vertices.push_back(data.vertices[i] + 1);
    if (i < data.vertices.size() - 1) {
      if (graph.graphMatrix()(data.vertices[i], data.vertices[i + 1]) == 0) {
        std::vector<int> actual_path = GetVectorShortestPathBetweenVertices(
            graph, data.vertices[i] + 1, data.vertices[i + 1] + 1);
        for (size_t j{1}; j < actual_path.size() - 1; j++)
          res.vertices.push_back(actual_path[j]);
      }
    }
  }
  return res;
}

int GraphAlgorithms::Roll(int base, unsigned *seed) {
  int res{};
  if (base > 0) {
    res = rand_r(seed) % base;
  }
  return res;
}

// ========== 6: Alternative tsm algorithms ===========

TsmResult GraphAlgorithms::SolveTsmBrute(const Graph &graph) {
  TsmResult res{};
  Matrix<double> paths(GetShortestPathsBetweenAllVertices(graph));

  std::set<int> points{};
  for (int i(1); i < graph.MaxVertex(); i++) points.insert(i);
  res.vertices.push_back(0);

  res = RecBruteSearch(0, points, paths);

  res = InsertActualPaths(res, graph);
  return res;
}

TsmResult GraphAlgorithms::SolveTsmNearestNeighbour(const Graph &graph) {
  TsmResult res{};
  Matrix<double> paths(GetShortestPathsBetweenAllVertices(graph));

  std::set<int> points{};
  for (int i(1); i < graph.MaxVertex(); i++) points.insert(i);
  res.vertices.push_back(0);
  int curr_pos{};

  while (points.size() > 0) {
    double shortest{};
    int new_pos{};
    for (int i : points) {
      if (shortest == 0 || paths(curr_pos, i) < shortest) {
        shortest = paths(curr_pos, i);
        new_pos = i;
      }
    }
    res.vertices.push_back(new_pos);
    res.distance += paths(curr_pos, new_pos);
    points.erase(new_pos);
    curr_pos = new_pos;
  }
  res.vertices.push_back(0);
  res.distance += paths(curr_pos, 0);

  res = InsertActualPaths(res, graph);
  return res;
}

TsmResult GraphAlgorithms::RecBruteSearch(int cur, std::set<int> future,
                                          const Matrix<double> &paths) {
  TsmResult res{};
  if (future.size() == 0) {
    res.vertices.push_back(cur);
    res.vertices.push_back(0);
    res.distance = paths(cur, 0);
  } else {
    for (int i : future) {
      std::set<int> far_future(future);
      far_future.erase(i);
      TsmResult candidate = RecBruteSearch(i, far_future, paths);
      candidate.vertices.insert(candidate.vertices.begin(), cur);
      candidate.distance += paths(candidate.vertices[0], candidate.vertices[1]);
      if (res.distance == 0 || candidate.distance < res.distance) {
        res = candidate;
      }
    }
  }
  return res;
}
