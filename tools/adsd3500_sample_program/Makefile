CXX     = aarch64-linux-gnu-g++
SRC_DIR = src
INCLUDE_DIR = include
DEPTH_COMPUTE_DIR = depthComputeLibrary
CLOSED_SOURCE_DIR = $(DEPTH_COMPUTE_DIR)/closedSourceLibs
OPEN_SOURCE_DIR = $(DEPTH_COMPUTE_DIR)/openSourceFiles
DCL_HEADER_DIR = $(DEPTH_COMPUTE_DIR)/include
SRCS = $(SRC_DIR)/run_adsd3500.cpp $(SRC_DIR)/adsd3500_util.cpp $(SRC_DIR)/adsd3500_interrupt_notifier.cpp
CXXFLAGS += -I$(INCLUDE_DIR) -I$(DCL_HEADER_DIR) -std=c++11

ifeq ($(USE_OPEN_SOURCE), 1)
    SRCS += $(wildcard $(OPEN_SOURCE_DIR)/*.cpp)
else
    LDFLAGS = -L$(CLOSED_SOURCE_DIR) -ltofi_config -ltofi_compute
endif

all: clean prebuild run_adsd3500

prebuild:
ifeq ($(USE_OPEN_SOURCE), 1)
	@echo "*** Building with Open-source Depth Compute files. ***"
else
	@echo "*** Building with Closed-source Depth Compute libraries. ***"
endif

run_adsd3500: $(SRCS)
	@echo "*** Building the target. ***"
	$(CXX) $^ -o $@ $(CXXFLAGS) $(LDFLAGS)

clean:
	@echo "*** Cleaning the Repository. ***"
	$(RM) -rf run_adsd3500
	$(RM) -rf *.bin *.txt
