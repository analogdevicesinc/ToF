CXX     = aarch64-linux-gnu-g++
SRC_DIR = src
INCLUDE_DIR = include
DEPTH_COMPUTE_DIR = depthComputeLibrary
CLOSED_SOURCE_DIR = $(DEPTH_COMPUTE_DIR)/closedSourceLibs
OPEN_SOURCE_DIR = $(DEPTH_COMPUTE_DIR)/openSourceFiles
DCL_HEADER_DIR = $(DEPTH_COMPUTE_DIR)/include
SRCS = $(SRC_DIR)/run_adsd3500.cpp $(SRC_DIR)/adsd3500_util.cpp $(SRC_DIR)/adsd3500_interrupt_notifier.cpp
CXXFLAGS += -I$(INCLUDE_DIR) -I$(DCL_HEADER_DIR) -std=c++11

ifeq ($(OPEN_SOURCE_MODE), 1)
	CXXFLAGS += -DOPEN_SOURCE_MODE
    SRCS += $(wildcard $(OPEN_SOURCE_DIR)/*.cpp)
else
	LDFLAGS = -L$(CLOSED_SOURCE_DIR) -ltofi_config -ltofi_compute
	ifeq ($(FLOAT_LIBS), 1)
		CXXFLAGS += -DFLOAT_LIBS
	endif
endif

all: clean prebuild run_adsd3500

prebuild:
ifeq ($(OPEN_SOURCE_MODE), 1)
	@echo "*** Using Open-source Depth Compute files. ***"
else
	@echo "*** Using Closed-source Depth Compute libraries. ***"
ifeq ($(FLOAT_LIBS), 1)
	@echo "*** Floating Point Depth Compute Libraries will be used to run the target. ***"
	@echo "*** Pleas copy the .so files from 'depthComputeLibrary/closedSourceLibs/floatingPoint/' to '/usr/lib/' location.***\n" 
else 
	@echo "*** Fixed Point Depth Compute Libraries will be used to run the target. ***\n"
	@echo "*** Pleas copy the .so files from 'depthComputeLibrary/closedSourceLibs/fixedPoint/' to '/usr/lib/' location. ***\n" 
endif
endif

run_adsd3500: $(SRCS)
	@echo "*** Building the target. ***"
	$(CXX) $^ -o $@ $(CXXFLAGS) $(LDFLAGS)

clean:
	@echo "*** Cleaning the Repository. ***"
	$(RM) -rf run_adsd3500
	$(RM) -rf *.bin *.txt
