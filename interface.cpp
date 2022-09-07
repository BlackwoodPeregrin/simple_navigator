#include "interface.hpp"

#include <chrono>
#include <iostream>
#include <sstream>

MainWindow::MainWindow()
    : m_exit_(false), m_draw_(true), controller_(new Controller) {}

MainWindow::MainWindow(const MainWindow &window)
    : m_exit_(false), m_draw_(true), controller_(new Controller) {
  *this = window;
}

MainWindow::MainWindow(MainWindow &&window)
    : m_exit_(false), m_draw_(true), controller_(new Controller) {
  *this = std::move(window);
}

MainWindow::~MainWindow() { delete controller_; }

void MainWindow::operator=(const MainWindow &window) {
  m_exit_ = window.m_exit_;
  m_draw_ = window.m_draw_;
  *controller_ = (*window.controller_);
  file_name_ = window.file_name_;
}

void MainWindow::operator=(MainWindow &&window) {
  std::swap(m_exit_, window.m_exit_);
  std::swap(m_draw_, window.m_draw_);
  *controller_ = std::move((*window.controller_));
  file_name_ = std::move(window.file_name_);
}

void MainWindow::show() {
  while (m_exit_ == false) {
    if (m_draw_ == true) {
      PrintUi();
      m_draw_ = false;
    }
    ScanInput();
  }
}

void MainWindow::ScanInput() {
  std::string choose = ProtectedInput();
  PrintUi();
  if (choose == "1") {
    std::cout << choose << std::endl;
    slotButtonOneClicked();
  } else if (choose == "2") {
    std::cout << choose << std::endl;
    slotButtonTwoClicked();
  } else if (choose == "3") {
    std::cout << choose << std::endl;
    slotButtonThreeClicked();
  } else if (choose == "4") {
    std::cout << choose << std::endl;
    slotButtonFourClicked();
  } else if (choose == "5") {
    std::cout << choose << std::endl;
    slotButtonFiveClicked();
  } else if (choose == "6") {
    std::cout << choose << std::endl;
    slotButtonSixClicked();
  } else if (choose == "7") {
    std::cout << choose << std::endl;
    slotButtonSevenClicked();
  } else if (choose == "8") {
    std::cout << choose << std::endl;
    slotButtonEightClicked();
  } else if (choose == "9") {
    std::cout << choose << std::endl;
    slotButtonNineClicked();
  } else if (choose == "10") {
    std::cout << choose << std::endl;
    slotButtonTenClicked();
  } else if (choose == "11") {
    std::cout << choose << std::endl;
    slotButtonElevenClicked();
  } else if (choose == "12") {
    std::cout << choose << std::endl;
    slotButtonTwelveClicked();
  } else if (choose == "13") {
    std::cout << choose << std::endl;
    slotButtonThirteenClicked();
  } else if (choose == "14") {
    std::cout << choose << std::endl;
    slotButtonFourteenClicked();
  } else if (choose == "15") {
    std::cout << choose << std::endl;
    slotButtonFifteenClicked();
  } else if (choose == "16") {
    std::cout << choose << std::endl;
    slotButtonSixteenClicked();
  } else if (choose == "17") {
    std::cout << "\e[1;40mProgramm Exit.\e[0m" << std::endl;
    m_exit_ = true;
  } else {
    std::cout << "Error, incorrect choose. Try again." << std::endl;
  }
}

void MainWindow::PrintUi() {
  std::cout.flush();
  std::cin.clear();
  system("clear");
  std::cout << ui;
}

bool MainWindow::PrintQuestion() {
  std::cout << "\e[1;93mTry again? (Yes/no): ";
  std::string answer = ProtectedInput();
  std::cout << "\e[0m";
  if (answer == "Yes" || answer == "yes") {
    return true;
  }
  return false;
}

void MainWindow::PauseFunc() {
  std::cout << "\e[1;94mEnter any key to continue ... ";
  std::string key = ProtectedInput();
  std::cout << "\e[0m" << std::endl;
  m_draw_ = true;
}

bool MainWindow::CorrectInput(int *temp, const std::string &message) {
  std::cout << message << std::endl;
  std::string line = ProtectedInput();
  bool correct = false;
  if (InputNumberCorrect(line) == true) {
    try {
      *temp = std::stoi(line);
      correct = true;
    } catch (const std::exception &e) {
      std::cerr << "\e[1;91mIncorrect input. Try again.\e[0m" << '\n';
    }
  } else {
    std::cout << "\e[1;91mIncorrect input. Try again.\e[0m" << std::endl;
  }
  return correct;
}

bool MainWindow::InputNumberCorrect(const std::string &number) {
  size_t i = (number[0] == '-') ? 1 : 0;
  for (; i < number.size(); ++i) {
    if (number[i] < '0' || number[i] > '9') {
      return false;
    }
  }
  return true;
}

std::string MainWindow::ReadNameFile() {
  std::cout << "Input file name:" << std::endl;
  std::string file_name = ProtectedInput();
  return file_name;
}

void MainWindow::PrintMatrixToTerminal(const Matrix<double> &matrix) {
  for (int i = 0; i < matrix.get_rows(); ++i) {
    std::cout << "\e[1;92m[";
    for (int j = 0; j < matrix.get_columns(); ++j) {
      if (j != matrix.get_columns() - 1) {
        std::cout << matrix(i, j) << ", ";
      } else {
        std::cout << matrix(i, j) << "]";
      }
    }
    std::cout << "\e[0m" << std::endl;
  }
}

std::string MainWindow::ProtectedInput() {
  std::string res{};
  std::cin >> res;
  if (std::cin.eof()) exit(0);
  return res;
}

void MainWindow::slotButtonOneClicked() {
  auto result = controller_->GetAdjacencyMatrix();
  if (result.second.empty()) {
    std::cout << "\e[1;93mAdjacency matrix\e[0m" << std::endl;
    PrintMatrixToTerminal(result.first);
  } else {
    std::cout << "\e[1;91m" + result.second + "\e[0m" << std::endl;
  }
  PauseFunc();
}

void MainWindow::slotButtonTwoClicked() {
  while (true) {
    int size{};
    int correct = CorrectInput(&size, "Input graph size:");
    if (correct) {
      auto result = controller_->GenerateOrientedGraph(size);
      if (result.second == false) {
        std::cout << "\e[1;91m" + result.first + "\e[0m" << std::endl;
        if (PrintQuestion() == false) {
          m_draw_ = true;
          break;
        }
      } else {
        std::cout << "\e[1;92m" + result.first + "\e[0m" << std::endl;
        file_name_.clear();
        PauseFunc();
        break;
      }
    }
  }
}

void MainWindow::slotButtonThreeClicked() {
  while (true) {
    int size{};
    int correct = CorrectInput(&size, "Input graph size:");
    if (correct) {
      auto result = controller_->GenerateUnorientedWeightedGraph(size);
      if (result.second == false) {
        std::cout << "\e[1;91m" + result.first + "\e[0m" << std::endl;
        if (PrintQuestion() == false) {
          m_draw_ = true;
          break;
        }
      } else {
        std::cout << "\e[1;92m" + result.first + "\e[0m" << std::endl;
        file_name_.clear();
        PauseFunc();
        break;
      }
    }
  }
}

void MainWindow::slotButtonFourClicked() {
  while (true) {
    int size{};
    int correct = CorrectInput(&size, "Input graph size:");
    if (correct) {
      auto result = controller_->GenerateUnorientedGraph(size);
      if (result.second == false) {
        std::cout << "\e[1;91m" + result.first + "\e[0m" << std::endl;
        if (PrintQuestion() == false) {
          m_draw_ = true;
          break;
        }
      } else {
        std::cout << "\e[1;92m" + result.first + "\e[0m" << std::endl;
        file_name_.clear();
        PauseFunc();
        break;
      }
    }
  }
}

void MainWindow::slotButtonFiveClicked() {
  while (true) {
    std::string file_name = ReadNameFile();
    auto result = controller_->LoadGraphFromFile(file_name);
    if (result.second == false) {
      std::cout << "\e[1;91m" + result.first + "\e[0m" << std::endl;
      if (PrintQuestion() == false) {
        m_draw_ = true;
        break;
      }
    } else {
      std::cout << "\e[1;92m" + result.first + "\e[0m" << std::endl;
      file_name_ = file_name;
      PauseFunc();
      break;
    }
  }
}

void MainWindow::slotButtonSixClicked() {
  std::string file_name = ReadNameFile();
  if (file_name.length() < 4 ||
      file_name.substr(file_name.length() - 4) != ".dot") {
    file_name += ".dot";
  }
  auto result = controller_->SaveGraphToFileDot(file_name);
  if (result.second == false) {
    std::cout << "\e[1;91m" + result.first + "\e[0m" << std::endl;
  } else {
    std::cout << "\e[1;92m" + result.first + "\e[0m" << std::endl;
  }
  PauseFunc();
}

void MainWindow::slotButtonSevenClicked() {
  std::string file_name = ReadNameFile();
  if (file_name.length() < 4 ||
      file_name.substr(file_name.length() - 4) != ".txt") {
    file_name += ".txt";
  }
  auto result = controller_->SaveGraphToFileTxt(file_name);
  if (result.second == false) {
    std::cout << "\e[1;91m" + result.first + "\e[0m" << std::endl;
  } else {
    std::cout << "\e[1;92m" + result.first + "\e[0m" << std::endl;
  }
  PauseFunc();
}

void MainWindow::slotButtonEightClicked() {
  std::string tgt_name = ReadNameFile();
  if (tgt_name.length() < 4 ||
      tgt_name.substr(tgt_name.length() - 4) != ".png") {
    tgt_name += ".png";
  }
  auto result = controller_->SaveGraphToFileDot("tmp.dot");

  if (result.first == "Graph export correct.") {
    std::string command = "dot -Tpng tmp.dot -o";
    command += tgt_name;
    const char *c_com = command.c_str();
    system(c_com);
  } else {
    std::cout << "something went wrong\n";
  }
  PauseFunc();
}

void MainWindow::slotButtonNineClicked() {
  while (true) {
    int startVertex{};
    int correct = CorrectInput(&startVertex, "Input start vertex:");
    if (correct) {
      auto result = controller_->BreadthFirstSearch(startVertex);
      if (result.second == true) {
        std::cout << "\e[1;92m" + result.first + "\e[0m" << std::endl;
        PauseFunc();
        break;
      } else {
        std::cout << "\e[1;91m" + result.first + "\e[0m" << std::endl;
        if (result.first.find("out of range") == std::string::npos) {
          PauseFunc();
          break;
        }
        if (PrintQuestion() == false) {
          m_draw_ = true;
          break;
        }
      }
    }
  }
}

void MainWindow::slotButtonTenClicked() {
  while (true) {
    int startVertex{};
    int correct = CorrectInput(&startVertex, "Input start vertex:");
    if (correct) {
      auto result = controller_->DepthFirstSearch(startVertex);
      if (result.second == true) {
        std::cout << "\e[1;92m" + result.first + "\e[0m" << std::endl;
        PauseFunc();
        break;
      } else {
        std::cout << "\e[1;91m" + result.first + "\e[0m" << std::endl;
        if (result.first.find("out of range") == std::string::npos) {
          PauseFunc();
          break;
        }
        if (PrintQuestion() == false) {
          m_draw_ = true;
          break;
        }
      }
    }
  }
}

void MainWindow::slotButtonElevenClicked() {
  while (true) {
    int startVertex{};
    int finishVertex{};
    bool correct = CorrectInput(&startVertex, "Input start vertex:");
    if (correct) {
      correct = CorrectInput(&finishVertex, "Input finish vertex:");
      if (correct) {
        auto result = controller_->GetVectorShortestPathBetweenVertices(
            startVertex, finishVertex);

        if (result.second == true) {
          std::cout << "\e[1;92m" + result.first + "\e[0m" << std::endl;
          PauseFunc();
          break;
        } else {
          std::cout << "\e[1;91m" + result.first + "\e[0m" << std::endl;
          if (result.first == "Graph not inizialize.") {
            PauseFunc();
            break;
          } else {
            if (PrintQuestion() == false) {
              m_draw_ = true;
              break;
            }
          }
        }
      }
    }
  }
}

void MainWindow::slotButtonTwelveClicked() {
  auto result = controller_->GetShortestPathBetweenAllVertices();
  /*---ошибок не выялено---*/
  if (result.second.empty()) {
    std::cout << "\e[1;93mAdjacency Matrix between all pairs of vertices:\e[0m"
              << std::endl;
    Matrix<double> resMatrix(std::move(result.first));
    PrintMatrixToTerminal(resMatrix);
    /*---ошибоки есть, печатаем их в консоль---*/
  } else {
    std::cout << "\e[1;91m" + result.second + "\e[0m" << std::endl;
  }
  PauseFunc();
}

void MainWindow::slotButtonThirteenClicked() {
  auto result = controller_->GetLeastSpanningTree();
  /*---ошибок не выялено---*/
  if (result.second.empty()) {
    std::cout << "\e[1;93mAdjacency matrix minimum spanning tree\e[0m"
              << std::endl;
    Matrix<double> resMatrix(std::move(result.first));
    PrintMatrixToTerminal(resMatrix);
    /*---ошибоки есть, печатаем их в консоль---*/
  } else {
    std::cout << "\e[1;91m" + result.second + "\e[0m" << std::endl;
  }
  PauseFunc();
}

void MainWindow::slotButtonFourteenClicked() {
  auto result = controller_->SolveTsm();
  if (result.second == true) {
    std::cout << "\e[1;92m" + result.first + "\e[0m" << std::endl;
  } else {
    std::cout << "\e[1;91m" + result.first + "\e[0m" << std::endl;
  }
  PauseFunc();
}

void MainWindow::slotButtonFifteenClicked() {
  int num{};
  while (true) {
    if (CorrectInput(&num, "Input number of tests:") == true) break;
  }
  std::pair<std::string, bool> result_func;
  auto catching_error = controller_->SolveTsm();
  if (!catching_error.second) {
    std::cout << "\e[1;91m" + catching_error.first + "\e[0m" << std::endl;
  } else {
    auto start = std::chrono::high_resolution_clock::now();
    for (int i{}; i < num; i++)
      result_func = controller_->SolveTsmFast(Controller::kAnts);
    auto finish = std::chrono::high_resolution_clock::now();
    double duration =
        (std::chrono::duration_cast<std::chrono::milliseconds>(finish - start))
            .count() /
        1000.0;
    if (result_func.second == true) {
      std::cout << "\e[1;93mAnts Colony:\e[1;92m\t\t" << duration / num
                << "\e[0m"
                << "\e[1;93m s\e[0m" << std::endl;
    } else {
      std::cout << "\e[1;91m" + result_func.first + "\e[0m" << std::endl;
    }

    start = std::chrono::high_resolution_clock::now();
    for (int i{}; i < num; i++)
      result_func = controller_->SolveTsmFast(Controller::kBrute);
    finish = std::chrono::high_resolution_clock::now();
    duration =
        (std::chrono::duration_cast<std::chrono::milliseconds>(finish - start))
            .count() /
        1000.0;
    if (result_func.second == true) {
      std::cout << "\e[1;93mBrute Force:\e[1;92m\t\t" << duration / num
                << "\e[0m"
                << "\e[1;93m s\e[0m" << std::endl;
    } else {
      std::cout << "\e[1;91m" + result_func.first + "\e[0m" << std::endl;
    }

    start = std::chrono::high_resolution_clock::now();
    for (int i{}; i < num; i++)
      result_func = controller_->SolveTsmFast(Controller::kNeighbour);
    finish = std::chrono::high_resolution_clock::now();
    duration =
        (std::chrono::duration_cast<std::chrono::milliseconds>(finish - start))
            .count() /
        1000.0;
    if (result_func.second == true) {
      std::cout << "\e[1;93mNearest Neighbour:\e[1;92m\t" << duration / num
                << "\e[0m"
                << "\e[1;93m s\e[0m" << std::endl;
    } else {
      std::cout << "\e[1;91m" + result_func.first + "\e[0m" << std::endl;
    }
  }
  PauseFunc();
}

void MainWindow::slotButtonSixteenClicked() {
  auto result = controller_->GetAdjacencyMatrix();
  if (result.second.empty()) {
    std::cout << "⋆★⋆ Info About Graph ⋆★⋆" << std::endl;
    if (!file_name_.empty()) {
      std::cout << "\e[1;93mName Loading File:\e[1;92m " << file_name_
                << "\e[0m" << std::endl;
    } else {
      std::cout << "\e[1;93mName Loading File:\e[1;92m no choose\e[0m"
                << std::endl;
    }

    std::cout << "\e[1;93mSize Graph:\e[1;92m " << result.first.get_rows()
              << "\e[0m" << std::endl;
    auto typeGraph = controller_->GetTypeGraph();
    if (typeGraph.second.empty()) {
      if (typeGraph.first == Graph::GraphType::ORIENTED) {
        std::cout << "\e[1;93mType Graph:\e[1;92m Oriented\e[0m" << std::endl;
      } else {
        std::cout << "\e[1;93mType Graph:\e[1;92m Unoriented\e[0m" << std::endl;
      }
      auto typeWeightedGraph = controller_->GetTypeWeightedGraph();
      if (typeWeightedGraph.second.empty()) {
        if (typeWeightedGraph.first == Graph::GraphWeights::WEIGHTED) {
          std::cout << "\e[1;93mHave Weights Graph:\e[1;92m Yes\e[0m"
                    << std::endl;
        } else {
          std::cout << "\e[1;93mHave Weights Graph:\e[1;92m No\e[0m"
                    << std::endl;
        }
      } else {
        std::cout << "\e[1;91m" + result.second + "\e[0m" << std::endl;
      }
    } else {
      std::cout << "\e[1;91m" + result.second + "\e[0m" << std::endl;
    }
  } else {
    std::cout << "\e[1;91m" + result.second + "\e[0m" << std::endl;
  }
  PauseFunc();
}
