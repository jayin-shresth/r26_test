CXX = g++
CXXFLAGS = -std=c++17 -Og -g -Wall

# default target
all: my_program

# build main executable
my_program: src/*.cpp
	$(CXX) $(CXXFLAGS) src/*.cpp -Llib -lm -o my_program.exe

check: my_program
	@echo "--------------------------------------------"
	@echo "Checking..."
	@echo "Test-1: "
	./my_program test/testcase1.txt test/result1.txt
	diff -Z test/result1.txt test/expected1.txt
	@echo "Test-2: "
	./my_program test/testcase2.txt test/result2.txt
	diff -Z test/result2.txt test/expected2.txt
	@echo "Test-3: "
	./my_program test/testcase3.txt test/result3.txt
	diff -Z test/result3.txt test/expected3.txt
	@echo "**** Success: ***"
	@echo "--------------------------------------------"


clean:
	rm -f my_program test/result*

.PHONY: all clean check

