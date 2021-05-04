// Copyright (c) Microsoft Corporation. All rights reserved.
// Modified by Analog Devices, Inc.
// Licensed under the MIT License.

/**
 * Private header file for the command and streaming interface
 */

 //************************ Includes *****************************
#include <inttypes.h>
#include <stdbool.h>
#include <usbcommand.h>

// Exteranl dependencis
#include <libusb.h>
// Ensure we have LIBUSB_API_VERSION defined if not defined by libusb.h
#ifndef LIBUSB_API_VERSION
#define LIBUSB_API_VERSION 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

//**************Symbolic Constant Macros (defines)  *************
#define USB_CMD_MAX_WAIT_TIME 2000
#define USB_CMD_MAX_XFR_COUNT 8 // Upper limit to the number of outstanding transfer
#ifdef _WIN32
#define USB_CMD_MAX_XFR_POOL 80000000 // Memory pool size for outstanding transfers (based on empirical testing)
#else
#define USB_CMD_MAX_XFR_POOL 10000000 // Memory pool size for outstanding transfers (based on empirical testing)
#endif
#define USB_CMD_PORT_DEPTH 8

#define USB_CMD_EVENT_WAIT_TIME 1
#define USB_MAX_TX_DATA 128
#define USB_CMD_PACKET_TYPE 0x06022009
#define USB_CMD_PACKET_TYPE_RESPONSE 0x0A6FE000

/****************/
//Device Specific
#define ADI_CCD_VID  0x1D6B//Verified
#define ADI_CCD_PID  0x0102//Verified
#define FAKE_VID     0x0C45
#define FAKE_PID     0x671D
#define ADI_FPGA_VID 0x04b3
#define ADI_FPGA_PID 0x0006
/****************/
#define USB_CMD_DEFAULT_CONFIG 1

#define USB_CMD_DEPTH_INTERFACE 0
#define USB_CMD_DEPTH_IN_ENDPOINT 0x02
#define USB_CMD_DEPTH_OUT_ENDPOINT 0x81
#define USB_CMD_DEPTH_STREAM_ENDPOINT 0x83

#define USB_CMD_IMU_INTERFACE 2
#define USB_CMD_IMU_IN_ENDPOINT 0x04
#define USB_CMD_IMU_OUT_ENDPOINT 0x83
#define USB_CMD_IMU_STREAM_ENDPOINT 0x82

#define MAX_SERIAL_NUMBER_LENGTH                                                                                       \
    (13 * 2) // Current schema is for 12 digits plus NULL, the extra size is in case that grows in the future.

//************************ Typedefs *****************************
typedef struct _usb_async_transfer_data_t
{
	struct _usbcmd_context_t* usbcmd;
	struct libusb_transfer* bulk_transfer;
	//k4a_image_t image;
	uint32_t list_index;
} usb_async_transfer_data_t;

typedef struct _usbcmd_context_t
{
	//allocation_source_t source;

	// LIBUSB properties
	libusb_device_handle* libusb;
	libusb_context* libusb_context;

	uint8_t index;
	uint16_t pid;
	uint8_t interface;
	uint8_t cmd_tx_endpoint;
	uint8_t cmd_rx_endpoint;
	uint8_t stream_endpoint;
	uint32_t transaction_id;

	unsigned char serial_number[MAX_SERIAL_NUMBER_LENGTH];
	//guid_t container_id;

	//usb_cmd_stream_cb_t* callback;
	void* stream_context;
	bool stream_going;
	usb_async_transfer_data_t* transfer_list[USB_CMD_MAX_XFR_COUNT];
	size_t stream_size;
	//LOCK_HANDLE lock;
	//THREAD_HANDLE stream_handle;
} usbcmd_context_t;

#ifdef __cplusplus
}
#endif