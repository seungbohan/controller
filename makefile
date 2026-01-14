CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra

TARGET := controller

SRC := src/main.cpp sim/plant.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

clean:
	rm -f $(TARGET)
