#ifndef SRC_INTERFACE_HPP_
#define SRC_INTERFACE_HPP_

#include "controller.hpp"
#include "s21_graph_algorithms.hpp"

class MainWindow {
 public:
  MainWindow();
  MainWindow(const MainWindow &window);
  MainWindow(MainWindow &&window);
  ~MainWindow();

  void operator=(const MainWindow &window);
  void operator=(MainWindow &&window);

  void show();

 private:
  void ScanInput();
  void PrintUi();
  bool PrintQuestion();
  void PauseFunc();

  bool CorrectInput(int *temp, const std::string &message);
  bool InputNumberCorrect(const std::string &number);
  std::string ReadNameFile();
  void PrintMatrixToTerminal(const Matrix<double> &matrix);
  std::string ProtectedInput();

 private:  // slots
  void slotButtonOneClicked();
  void slotButtonTwoClicked();
  void slotButtonThreeClicked();
  void slotButtonFourClicked();
  void slotButtonFiveClicked();
  void slotButtonSixClicked();
  void slotButtonSevenClicked();
  void slotButtonEightClicked();
  void slotButtonNineClicked();
  void slotButtonTenClicked();
  void slotButtonElevenClicked();
  void slotButtonTwelveClicked();
  void slotButtonThirteenClicked();
  void slotButtonFourteenClicked();
  void slotButtonFifteenClicked();
  void slotButtonSixteenClicked();

 private:
  bool m_exit_;
  bool m_draw_;
  Controller *controller_;
  std::string file_name_;
};

static const char ui[] =
    "                           ⋆★⋆ SIMPLE NAVIGATOR ⋆★⋆                       "
    "      \n"
    "╔═════════════════════════════════════════════════════════════════════════"
    "═════╗\n"
    "║  \e[1;97m1. Print adjacency matrix graph.\e[0m                          "
    "     "
    "        "
    "     ║\n"
    "║  \e[1;90m2. Generate oriented graph.\e[0m                               "
    "             "
    "     ║\n"
    "║  \e[1;90m3. Generate unoriented weighted graph.\e[0m                    "
    "             "
    "     ║\n"
    "║  \e[1;90m4. Generate unoriented graph.\e[0m                             "
    "             "
    "     ║\n"
    "║  \e[1;96m5. Load graph from file.\e[0m                                  "
    "             "
    "     ║\n"
    "║  \e[1;96m6. Save graph to file.(.dot)\e[0m                              "
    "             "
    "     ║\n"
    "║  \e[1;96m7. Save graph to file.(.txt)\e[0m                              "
    "             "
    "     ║\n"
    "║  \e[1;35m8. Create a PNG file with an image of the downloaded "
    "graph.\e[0m            "
    "     ║\n"
    "║  \e[1;40m9. Traversal of the graph in width.\e[0m                       "
    "             "
    "     ║\n"
    "║ \e[1;40m10. Traversal of the graph in depth.\e[0m                       "
    "             "
    "     ║\n"
    "║ \e[1;33m11. Find the shortest path between two vertices in graph.\e[0m  "
    "             "
    "     ║\n"
    "║ \e[1;33m12. Search for the shortest paths between all pairs of vertices "
    "in a "
    "graph.\e[0m  ║\n"
    "║ \e[1;33m13. Search for the minimum spanning tree in the graph.\e[0m     "
    "             "
    "     ║\n"
    "║ \e[1;32m14. Solve the traveling salesman problem.\e[0m                  "
    "             "
    "     ║\n"
    "║ \e[1;32m15. Test of three algorithms for solving the traveling salesman "
    "problem.\e[0m     ║\n"
    "║ \e[1;97m16. Print graph characteristics.\e[0m                           "
    "             "
    "     ║\n"
    "║ \e[1;40m17. Exit.\e[0m                                                  "
    "     "
    "        "
    "     ║\n"
    "╚═════════════════════════════════════════════════════════════════════════"
    "═════╝\n";

#endif  // SRC_INTERFACE_HPP_
