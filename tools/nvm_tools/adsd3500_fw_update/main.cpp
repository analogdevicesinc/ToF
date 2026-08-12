/****************************************************************************
# Copyright (c) 2024 - Analog Devices Inc. All Rights Reserved.
# This software is proprietary & confidential to Analog Devices, Inc.
# and its licensors.
# *****************************************************************************
# *****************************************************************************/

#include <iostream>
#include <string>
#include <sys/stat.h>
#include "Adsd3500.h"

int main(int argc, char *argv[]) {

#ifdef NXP
	int user;
	user = getuid();
	if (user != 0)
	{
		std::cout << "Please run the application with sudo" << std::endl;
		return 1;
	}
#endif

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			std::cout << "Usage: " << argv[0] << " <ADCAM_Fw_Update_X.Y.Z.bin> [--force]" << std::endl;
			std::cout << std::endl;
			std::cout << "Firmware update utility for ADSD3500 devices." << std::endl;
			std::cout << "Supports single-device (master only) and dual-device (master + slave)" << std::endl;
			std::cout << "configurations. Device presence is detected automatically at runtime." << std::endl;
			std::cout << std::endl;
			std::cout << "Arguments:" << std::endl;
			std::cout << "  <ADCAM_Fw_Update_X.Y.Z.bin>" << std::endl;
			std::cout << "      Dual-slot .bin file (256 KB minimum, 2 x 128 KB slots)." << std::endl;
			std::cout << "      Slot 0 (offset 0x00000): master firmware (chunkType=0x54)" << std::endl;
			std::cout << "      Slot 1 (offset 0x20000): slave  firmware (chunkType=0x60)" << std::endl;
			std::cout << std::endl;
			std::cout << "  --force  (optional, only required for downgrade)" << std::endl;
			std::cout << "      Allow flashing a firmware version older than the installed one." << std::endl;
			std::cout << "      Not needed for normal upgrades." << std::endl;
			std::cout << std::endl;
			std::cout << "  -h, --help" << std::endl;
			std::cout << "      Print this help message and exit." << std::endl;
			std::cout << std::endl;
			std::cout << "Examples:" << std::endl;
			std::cout << "  " << argv[0] << " ADCAM_Fw_Update_8.1.0.bin" << std::endl;
			std::cout << "  " << argv[0] << " ADCAM_Fw_Update_8.1.0.bin --force  # downgrade only" << std::endl;
			return 0;
		}
	}

	if (argc < 2 || argc > 3) {
		std::cerr << "Usage: " << argv[0] << " <ADCAM_Fw_Update_X.Y.Z.bin> [--force]" << std::endl;
		std::cerr << "       .bin file must be at least 262144 bytes (2 x 128 KB slots)" << std::endl;
		std::cerr << "       --force   Required only for firmware downgrade" << std::endl;
		std::cerr << "       -h, --help  Show full help" << std::endl;
		return 1;
	}

	bool force = false;
	const char *filename = nullptr;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--force") == 0) {
			force = true;
		} else {
			filename = argv[i];
		}
	}

	if (!filename) {
		std::cerr << "Error: no firmware file specified." << std::endl;
		return 1;
	}

	if (!validate_ext(filename)) {
		std::cerr << "Error: file must have a '.bin' extension." << std::endl;
		return 1;
	}

	struct stat st;
	if (stat(filename, &st) != 0) {
		std::cerr << "Error: cannot access file '" << filename << "'" << std::endl;
		return 1;
	}
	if (st.st_size < (2 * ADI_DUAL_FW_SLOT_SIZE)) {
		std::cerr << "Error: file '" << filename << "' is " << st.st_size
		          << " bytes — must be at least 262144 bytes (2 x 128 KB slots)." << std::endl;
		return 1;
	}

	auto adsd = Adsd3500(filename, force);
}
