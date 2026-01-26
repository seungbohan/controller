CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Iinclude -Isrc -Isim -Itests

SRC = tests/test_runner.cpp src/controller_core.cpp sim/plant.cpp
OUT = controller_tests

all: $(OUT)

$(OUT): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(OUT) $(SRC)

clean:
	rm -f $(OUT)
