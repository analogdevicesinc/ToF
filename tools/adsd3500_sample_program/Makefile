CXX     = aarch64-linux-gnu-g++
SRC_DIR = src
INCLUDE_DIR = include
CRC_DIR = crc32
DEPTH_COMPUTE_DIR = depthComputeLibrary
DEPTH_COMPUTE_SRC = $(wildcard $(DEPTH_COMPUTE_DIR)/*.cpp)
CXXCFLAGS = -I$(INCLUDE_DIR) -std=c++11  
CRCS = $(CRC_DIR)/*.c

all: clean run_adsd3500

run_adsd3500: $(SRC_DIR)/run_adsd3500.cpp $(SRC_DIR)/adsd3500_util.cpp $(CRCS) $(DEPTH_COMPUTE_SRC)
	$(CXX) $^ -o $@ $(CXXCFLAGS)

clean:
	$(RM) -rf run_adsd3500
