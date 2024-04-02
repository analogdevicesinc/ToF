# the compiler: g++ for C++ program
#CXX = g++
CXX	= aarch64-linux-gnu-g++
SRC_DIR = src
INCLUDE_DIR = include
BIN_DIR = bin
CRC_DIR = crc32
CXXCFLAGS = -I$(INCLUDE_DIR) -std=c++11	
CRCS = $(CRC_DIR)/*.c

#all: clean reset_adsd3500 set_image_mode get_image_mode start_stream stop_stream receive_frames set_fps get_fps read_ccb

all: clean run_adsd3500 start_stream set_image_mode get_image_mode stop_stream

first_frame: $(SRC_DIR)/firstFrame.cpp $(SRC_DIR)/adsd3500_util.cpp $(CRCS)
	$(CXX) $^ -o $@
	mv $@ $(BIN_DIR)

reset_adsd3500: $(SRC_DIR)/ResetAdsd3500.cpp $(SRC_DIR)/adsd3500_util.cpp $(CRCS)
	$(CXX) $^ -o $@
	mv $@ $(BIN_DIR)

set_fps: $(SRC_DIR)/SetFps.cpp $(SRC_DIR)/adsd3500_util.cpp $(CRCS)
	$(CXX) $^ -o $@
	mv $@ $(BIN_DIR)

get_fps: $(SRC_DIR)/GetFps.cpp $(SRC_DIR)/adsd3500_util.cpp $(CRCS)
	$(CXX) $^ -o $@
	mv $@ $(BIN_DIR)

set_image_mode: $(SRC_DIR)/SetImageMode.cpp $(SRC_DIR)/adsd3500_util.cpp $(CRCS)
	$(CXX) $^ -o $@
	mv $@ $(BIN_DIR)

get_image_mode: $(SRC_DIR)/GetImageMode.cpp $(SRC_DIR)/adsd3500_util.cpp $(CRCS)
	$(CXX) $^ -o $@
	mv $@ $(BIN_DIR)

start_stream: $(SRC_DIR)/StartStream.cpp $(SRC_DIR)/adsd3500_util.cpp $(CRCS)
	$(CXX) $^ -o $@
	mv $@ $(BIN_DIR)

stop_stream: $(SRC_DIR)/StopStream.cpp $(SRC_DIR)/adsd3500_util.cpp $(CRCS)
	$(CXX) $^ -o $@
	mv $@ $(BIN_DIR)

receive_frames: $(SRC_DIR)/ReceiveFrames.cpp $(SRC_DIR)/adsd3500_util.cpp $(CRCS)
	$(CXX) $^ -o $@
	mv $@ $(BIN_DIR)

run_adsd3500: $(SRC_DIR)/run_adsd3500.cpp $(SRC_DIR)/adsd3500_util.cpp $(CRCS)
	$(CXX) $^ -o $@
	mv $@ $(BIN_DIR)

read_ccb: $(SRC_DIR)/ReadCCB.cpp $(SRC_DIR)/adsd3500_util.cpp $(CRCS)
	$(CXX) $^ -o $@
	mv $@ $(BIN_DIR)

clean:
	mkdir -p $(BIN_DIR)
	$(RM) -rf $(BIN_DIR)/*
