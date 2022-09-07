CC=g++ -std=c++17
CFLAGS=-Wall -Wextra -Werror
LFLAGS=-lgtest

OS = $(shell uname -s)
ifeq ($(OS), Linux)
	LFLAGS+=-pthread -lsubunit
	OPEN=-xdg-open
	MEMTEST=valgrind
else
	OPEN=-open
	MEMTEST=leaks
endif

GRAPH=s21_graph
ALGMS=s21_graph_algorithms
NAVIG=simple_navigator

all : rebuild

build : $(GRAPH).a $(ALGMS).a $(NAVIG)

test : tests

tests : tests.o $(GRAPH).a $(ALGMS).a
	$(CC) tests.o $(GRAPH).a $(ALGMS).a -o tests.out $(LFLAGS)
	./tests.out

tests.o : tests.cpp
	$(CC) -c $(CFLAGS) tests.cpp -o tests.o

$(NAVIG) : main.o interface.o controller.o $(GRAPH).a $(ALGMS).a
	$(CC) main.o interface.o controller.o $(GRAPH).a $(ALGMS).a -o $(NAVIG) $(LFLAGS)

main.o : main.cpp
	$(CC) -c $(CFLAGS) -fPIC main.cpp

interface.o : interface.cpp interface.hpp
	$(CC) -c $(CFLAGS) -fPIC interface.cpp

controller.o : controller.cpp controller.hpp
	$(CC) -c $(CFLAGS) -fPIC controller.cpp

$(GRAPH).a : $(GRAPH).o
	ar rcs $(GRAPH).a $(GRAPH).o
	ranlib $(GRAPH).a

$(ALGMS).a : $(ALGMS).o
	ar rcs $(ALGMS).a $(ALGMS).o
	ranlib $(ALGMS).a

$(GRAPH).o : $(GRAPH).cpp $(GRAPH).hpp
	$(CC) -c $(CFLAGS) -fPIC $(GRAPH).cpp

$(ALGMS).o : $(ALGMS).cpp $(ALGMS).hpp
	$(CC) -c $(CFLAGS) -fPIC $(ALGMS).cpp

gcov : gcov_report

gcov_report : clean
	$(MAKE) CFLAGS="$(CFLAGS) --coverage" LFLAGS="$(LFLAGS) --coverage" build
	$(MAKE) LFLAGS="$(LFLAGS) --coverage" test
	lcov -t tests.out -o tests.info -c -d . --no-external
	genhtml -o report tests.info
	$(OPEN) report/index.html

report_clean :
	$(RM) -rf *.gcda *.gcno *.info *.gch report

clean : report_clean
	$(RM) -rf *.o *.a *.out valgrind-out.txt CPPLINT.cfg simple_navigator tmp.dot

rebuild : clean build

checks : linter cppcheck $(MEMTEST)

cpplint : linter

linter :
	cp ../materials/linters/CPPLINT.cfg CPPLINT.cfg
	python3 ../materials/linters/cpplint.py --extensions=cpp *.cpp
	python3 ../materials/linters/cpplint.py --extensions=hpp *.hpp
	$(RM) ./CPPLINT.cfg

cppcheck :
	cppcheck --std=c++17 --enable=all --check-config --suppress=missingIncludeSystem *.cpp *.hpp

leaks : test
	CK_FORK=no leaks --atExit -- ./tests.out

valgrind : test
	valgrind --undef-value-errors=no --log-file=valgrind-out.txt ./tests.out

