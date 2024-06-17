CXX     = aarch64-linux-gnu-g++
SRC_DIR = src
INCLUDE_DIR = include
DEPTH_COMPUTE_DIR = depthComputeLibrary
SO_DIR = $(DEPTH_COMPUTE_DIR)/libs
SO_FILES = $(wildcard $(SO_DIR)/*.so)
CXXCFLAGS = -I$(INCLUDE_DIR) -I$(DEPTH_COMPUTE_DIR) -std=c++11  
LDFLAGS = $(addprefix -L,$(SO_DIR)) -ltofi_config -ltofi_compute

all: clean run_adsd3500

run_adsd3500: $(SRC_DIR)/run_adsd3500.cpp $(SRC_DIR)/adsd3500_util.cpp $(SRC_DIR)/adsd3500_interrupt_notifier.cpp
	$(CXX) $^ -o $@ $(CXXCFLAGS) $(LDFLAGS) 

clean:
	$(RM) -rf run_adsd3500
	$(RM) -rf *.bin *.txt
