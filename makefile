CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Iinclude -Isrc -Isim -Itests

# --- test binary ---
TEST_SRC = tests/test_runner.cpp src/controller_core.cpp sim/plant.cpp
TEST_OUT = controller_tests

# --- runtime demo binary (FakeCAN) ---
DEMO_SRC = runtime/main_fakecan_demo.cpp src/controller_core.cpp sim/plant.cpp
DEMO_OUT = fakecan_demo

# --- runtime keyboard demo binary (FakeCAN + Keyboard) ---
KEY_SRC = runtime/main_fakecan_keyboard_demo.cpp src/controller_core.cpp sim/plant.cpp
KEY_OUT = fakecan_key_demo

all: $(TEST_OUT) $(DEMO_OUT) $(KEY_OUT)

$(TEST_OUT): $(TEST_SRC)
	$(CXX) $(CXXFLAGS) -o $(TEST_OUT) $(TEST_SRC)

$(DEMO_OUT): $(DEMO_SRC)
	$(CXX) $(CXXFLAGS) -o $(DEMO_OUT) $(DEMO_SRC)

$(KEY_OUT): $(KEY_SRC)
	$(CXX) $(CXXFLAGS) -o $(KEY_OUT) $(KEY_SRC)

clean:
	rm -f $(TEST_OUT) $(DEMO_OUT) $(KEY_OUT)
