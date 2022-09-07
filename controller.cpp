#include "controller.hpp"

Controller::Controller() : graph_(new Graph) {}

Controller::Controller(const Controller &other) : graph_(new Graph) {
  *this = other;
}

Controller::Controller(Controller &&other) : graph_(new Graph) {
  *this = std::move(other);
}

Controller::~Controller() { delete graph_; }

void Controller::operator=(const Controller &other) {
  *graph_ = (*other.graph_);
  graph_alg_ = other.graph_alg_;
}

void Controller::operator=(Controller &&other) {
  *graph_ = std::move(*other.graph_);
  std::swap(graph_alg_, other.graph_alg_);
}

std::pair<Matrix<double>, std::string> Controller::GetAdjacencyMatrix() {
  Matrix<double> result(1, 1);
  std::string error{};
  try {
    result = std::move((*graph_).graphMatrix());
  } catch (const std::exception &e) {
    error = e.what();
  }
  return {result, error};
}

std::pair<bool, std::string> Controller::GetTypeGraph() {
  bool result;
  std::string error{};
  try {
    result = (*graph_).GraphType();
  } catch (const std::exception &e) {
    error = e.what();
  }
  return {result, error};
}

std::pair<bool, std::string> Controller::GetTypeWeightedGraph() {
  bool result;
  std::string error{};
  try {
    result = (*graph_).GraphWeights();
  } catch (const std::exception &e) {
    error = e.what();
  }
  return {result, error};
}

std::pair<std::string, bool> Controller::GenerateOrientedGraph(
    const int &size) {
  std::pair<std::string, bool> result{{}, {}};
  try {
    graph_->GenerateOrientedGraph(size);
    result.first = "Oriented graph generate correct.";
    result.second = true;
  } catch (const std::exception &e) {
    result.first = e.what();
    result.second = false;
  }
  return result;
}

std::pair<std::string, bool> Controller::GenerateUnorientedWeightedGraph(
    const int &size) {
  std::pair<std::string, bool> result{{}, {}};
  try {
    graph_->GenerateUnorientedGraph(size, true);
    result.first = "Unoriented weighted graph generate correct.";
    result.second = true;
  } catch (const std::exception &e) {
    result.first = e.what();
    result.second = false;
  }
  return result;
}

std::pair<std::string, bool> Controller::GenerateUnorientedGraph(
    const int &size) {
  std::pair<std::string, bool> result{{}, {}};
  try {
    graph_->GenerateUnorientedGraph(size, false);
    result.first = "Unoriented graph generate correct.";
    result.second = true;
  } catch (const std::exception &e) {
    result.first = e.what();
    result.second = false;
  }
  return result;
}

std::pair<std::string, bool> Controller::LoadGraphFromFile(
    const std::string &file_name) {
  std::pair<std::string, bool> result{{}, {}};
  try {
    graph_->LoadGraphFromFile(file_name);
    result.first = "Graph load correct.";
    result.second = true;
  } catch (const std::exception &e) {
    result.first = e.what();
    result.second = false;
  }
  return result;
}

std::pair<std::string, bool> Controller::SaveGraphToFileDot(
    const std::string &file_name) {
  std::pair<std::string, bool> result{{}, {}};
  try {
    graph_->ExportGraphToDot(file_name);
    result.first = "Graph export correct.";
    result.second = true;
  } catch (const std::out_of_range &e) {
    result.first = e.what();
    result.second = false;
  }
  return result;
}

std::pair<std::string, bool> Controller::SaveGraphToFileTxt(
    const std::string &file_name) {
  std::pair<std::string, bool> result{{}, {}};
  try {
    graph_->ExportGraphToTxt(file_name);
    result.first = "Graph export correct.";
    result.second = true;
  } catch (const std::out_of_range &e) {
    result.first = e.what();
    result.second = false;
  }
  return result;
}

std::pair<std::string, bool> Controller::BreadthFirstSearch(
    const int &startVertex) {
  std::pair<std::string, bool> result{{}, {}};
  try {
    std::vector<int> data = graph_alg_.BreadthFirstSearch(*graph_, startVertex);
    result.first = "Result: [";
    for (size_t i = 0; i < data.size(); ++i) {
      if (i == data.size() - 1) {
        result.first += std::to_string(data[i]) + "];";
      } else {
        result.first += std::to_string(data[i]) + ", ";
      }
    }
    result.second = true;
  } catch (const std::exception &e) {
    result.first = e.what();
    result.second = false;
  }
  return result;
}

std::pair<std::string, bool> Controller::DepthFirstSearch(
    const int &startVertex) {
  std::pair<std::string, bool> result{{}, {}};
  try {
    std::vector<int> data = graph_alg_.DepthFirstSearch(*graph_, startVertex);
    result.first = "Result: [";
    for (size_t i = 0; i < data.size(); ++i) {
      if (i == data.size() - 1) {
        result.first += std::to_string(data[i]) + "];";
      } else {
        result.first += std::to_string(data[i]) + ", ";
      }
    }
    result.second = true;
  } catch (const std::exception &e) {
    result.first = e.what();
    result.second = false;
  }
  return result;
}

std::pair<std::string, bool> Controller::GetVectorShortestPathBetweenVertices(
    const int &vertex1, const int &vertex2) {
  std::pair<std::string, bool> result{{}, {}};
  try {
    std::vector<int> path = graph_alg_.GetVectorShortestPathBetweenVertices(
        *graph_, vertex1, vertex2);
    result.first = "Result Path:\t";
    for (size_t i = 0; i < path.size(); ++i) {
      if (i == path.size() - 1) {
        result.first += std::to_string(path[i]) + "\n";
      } else {
        result.first += std::to_string(path[i]) + " -> ";
      }
    }
    double lenPath =
        graph_alg_.GetShortestPathBetweenVertices(*graph_, vertex1, vertex2);
    result.first += "Result Len Path:\t" + std::to_string(lenPath);
    result.second = true;
  } catch (const std::exception &e) {
    result.first = e.what();
    result.second = false;
  }
  return result;
}

std::pair<Matrix<double>, std::string>
Controller::GetShortestPathBetweenAllVertices() {
  Matrix<double> result(1, 1);
  std::string error{};
  try {
    result = std::move(graph_alg_.GetShortestPathsBetweenAllVertices(*graph_));
  } catch (const std::exception &e) {
    error = e.what();
  }
  return {result, error};
}

std::pair<Matrix<double>, std::string> Controller::GetLeastSpanningTree() {
  Matrix<double> result(1, 1);
  std::string error{};
  try {
    result = std::move(graph_alg_.GetLeastSpanningTree(*graph_));
  } catch (const std::exception &e) {
    error = e.what();
  }
  return {result, error};
}

std::pair<std::string, bool> Controller::SolveTsm(unsigned mode) {
  std::pair<std::string, bool> res{};
  try {
    TsmResult answer{};
    if (mode == kAnts) {
      answer = graph_alg_.SolveTravelingSalesmanProblem(*graph_);
    } else if (mode == kBrute) {
      answer = graph_alg_.SolveTsmBrute(*graph_);
    } else {
      answer = graph_alg_.SolveTsmNearestNeighbour(*graph_);
    }
    res.first = "Result Path:\t";
    for (size_t i{}; i < answer.vertices.size(); i++) {
      res.first += std::to_string(answer.vertices[i]);
      if (i < answer.vertices.size() - 1) res.first += " -> ";
    }
    res.first += "\nDistance:\t";
    if (answer.distance - (int)answer.distance == 0)
      res.first += std::to_string((int)answer.distance);
    else
      res.first += std::to_string(answer.distance);
    res.second = true;
  } catch (const std::exception &e) {
    res.first = e.what();
    res.second = false;
  }
  return res;
}

std::pair<std::string, bool> Controller::SolveTsmFast(unsigned mode) {
  TsmResult res;
  std::pair<std::string, bool> result_func{};
  try {
    if (mode == kAnts) {
      res = graph_alg_.SolveTravelingSalesmanProblem(*graph_);
    } else if (mode == kBrute) {
      res = graph_alg_.SolveTsmBrute(*graph_);
    } else {
      res = graph_alg_.SolveTsmNearestNeighbour(*graph_);
    }
    result_func.second = true;
  } catch (const std::exception &e) {
    result_func.second = false;
  }
  return result_func;
}
