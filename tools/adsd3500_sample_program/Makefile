CXX     = aarch64-linux-gnu-g++
SRC_DIR = src
INCLUDE_DIR = include
CRC_DIR = crc32
CXXCFLAGS = -I$(INCLUDE_DIR) -std=c++11  
CRCS = $(CRC_DIR)/*.c

all: clean run_adsd3500

run_adsd3500: $(SRC_DIR)/run_adsd3500.cpp $(SRC_DIR)/adsd3500_util.cpp $(CRCS)
	$(CXX) $^ -o $@

clean:
	$(RM) -rf run_adsd3500
