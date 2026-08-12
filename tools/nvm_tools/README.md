# nvm_tools

This project contains utilities for NVM write operations and firmware update support for the ADSD3500 device.

## Prerequisites

- CMake version 3.0 or later
- GNU Make
- A supported Linux environment
- Required development libraries and headers for your target platform

## Directory Structure

- `NVM_WRITE/` – NVM write utility
- `adsd3500_fw_update/` – ADSD3500 firmware update tool

## Build Instructions (NXP Platform)

Follow the steps below to build the project for the NXP platform.

```bash
mkdir build
cd build
cmake -DNXP=ON ..
make
```

## Clean Build (Recommended)

Before switching platforms or rebuilding from scratch, perform a clean build:

```bash
rm -rf build
```
