//-----------------------------------------------------------------------------
//
// Copyright (c) 2020 by Analog Devices, Inc. All Rights Reserved.
//
// This software is proprietary and confidential.  By possession or use of this
// software you agree to the terms of the associated Analog Devices License Agreement.
//
//-----------------------------------------------------------------------------
// File Name          : newton_control.c
// Description        : Newton Host API Software
//-----------------------------------------------------------------------------

/* -*- Mode: C; tab-width: 4 -*- */
#include "newton_control.h"
#include <regex.h>

//==============================================================================
// REVISION INFO
//==============================================================================
static int revisionID = 0; // 0 = revison A

//==============================================================================
// Pin Mux and Pin State
//==============================================================================
static adi_pin_modes_e pinMuxMode = PIN_MODE_FUNCTIONAL;

//==============================================================================
// SPI Driver Handle
//==============================================================================
int spi_handle   = -1;
//int spi_bit_rate = SPI_BIT_RATE_8M;
int spi_bit_rate = SPI_BIT_RATE_4M;

//==============================================================================
// FUNCTIONS
//==============================================================================

//==============================================================================
// Does base start with str function
//==============================================================================
bool startsWith( char* base, char* str ) {
    return( strstr( base, str ) - base ) == 0;
}

//==============================================================================
// spi_transfer function
//==============================================================================
static int adi_spi_transfer( int bytes_out, u08 *data_out, int bytes_in, u08 *data_in ) {
    int rc = ADI_NO_ERROR;
    int i = 0;
    if( bytes_in != bytes_out ) {
        return( ADI_UNEXPECTED_SPI_BYTE_COUNT );
    }

    global_tr.tx_buf = (unsigned long) data_out;
    global_tr.rx_buf = (unsigned long) data_in;
    global_tr.len = bytes_out;

    rc = ioctl( spi_handle, SPI_IOC_MESSAGE(1), &global_tr );
    if( rc == 1 ) {
        rc = ADI_SPI_XFER_ERROR;
    }
    else {
        rc = ADI_NO_ERROR;        
    }

#ifdef ENABLE_DEBUG_HIGH
    printf( "INFO: adi_spi_transfer: bytes_out = %d\n", bytes_out );
    for( i = 0; i < bytes_out; i++ ) {
        if( ((i % 8) == 0) && (i != 0) ) {
            printf( "\n" );
	}
	else {
            printf( "%02x ", data_out[i] );
	}
    }
    printf( "\n" );
    printf( "INFO: adi_spi_transfer: bytes_in = %d\n", bytes_in );
    for( i = 0; i < bytes_in; i++ ) {
        if( ((i % 8) == 0) && (i != 0 ) ) {
            printf( "\n" );
	}
	else {
            printf( "%02x ", data_in[i] );
	}
    }
    printf( "\n" );
#endif

    return( rc );

} // adi_spi_transfer

//=========================================================================
// Device- and host-independent spi write routine
//=========================================================================
int adi_spi_write( int bytes_out, u08 *data_out, int bytes_in, u08 *data_in ) {
    int  rc = ADI_ERROR_CODE_MISSING;
    bool called_adi_spi_open = false;
    int  i;

    // Perform byte swap for each 16-bit word.
    u08 *byteSwapOut = (u08 *) malloc( bytes_out );
    u08 *byteSwapIn  = (u08 *) malloc( bytes_in );

    for( i = 0; i < bytes_out; i++ ) {
        if( (i & 1) ) {
	    // Odd Byte
	    byteSwapOut[i-1] = data_out[i];
        }
	else {
	    // Even Byte
	    byteSwapOut[i+1] = data_out[i];
	}
    }

    if( spi_handle <= 0 ) {
        rc = adi_spi_open( spi_bit_rate );
        if( rc != ADI_NO_ERROR ) {
            return( ADI_SPI_DRIVER_ERROR );
        }
        called_adi_spi_open = true;
    }

    rc = adi_spi_transfer( bytes_out, byteSwapOut, bytes_in, byteSwapIn );
    if( rc != ADI_NO_ERROR ) {
        if( called_adi_spi_open ) {
            adi_spi_close( );
        }
        return( rc );
    }

    for( i = 0; i < bytes_in; i++ ) {
        if( (i & 1) ) {
	    // Odd Byte
	     data_in[i] = byteSwapIn[i-1];
        }
	else {
	    // Even Byte
	    data_in[i] = byteSwapIn[i+1];
	}
    }

    if( called_adi_spi_open ) {
        adi_spi_close( );
    }

    return( rc );

} // adi_spi_write

//=========================================================================
// Write 16-bit word at specified address
//=========================================================================
int adi_spi_write_word( u16 address, u16 data ) {
    int rc = ADI_ERROR_CODE_MISSING;
    u16 addr;
    int bytes_out = 0;
    int bytes_in  = 0;
    u08 data_out[4];
    u08 data_in[4];
    bool called_adi_spi_open = false;

    if( spi_handle <= 0 ) {
        rc = adi_spi_open( spi_bit_rate );
        if( rc != ADI_NO_ERROR ) {
            return( ADI_SPI_DRIVER_ERROR );
        }
        called_adi_spi_open = true;
    }

    // Set write bit.
    addr = address | 0x8000;
    
    data_out[bytes_out++] = (u08)  (addr & 0x00ff);
    data_out[bytes_out++] = (u08) ((addr & 0xff00 ) >>  8);
    
    data_out[bytes_out++] = (u08)  (data & 0xff);
    data_out[bytes_out++] = (u08) ((data & 0xff00) >> 8);

    bytes_in = bytes_out;
    
    rc = adi_spi_write( bytes_out, data_out, bytes_in, data_in );

#ifdef ENABLE_DEBUG_MEDIUM
    printf( "INFO: spi_write_word: address=0x%04x, data=0x%04x\n", address, data );
#endif

    if( called_adi_spi_open ) {
        adi_spi_close( );
    }

    return( rc );

} // adi_spi_write_word

//=========================================================================
// Write multiple words
//=========================================================================
int adi_spi_write_word_multiple( u16 address, int dataLength, u16 *dataPtr ) {
    int rc = ADI_ERROR_CODE_MISSING;
    u16 addr;
    int bytes_out = 0;
    int bytes_in = 0;
    int byteLength;
    int allocBytes;
    bool called_adi_spi_open = false;
    int i;

#ifdef ENABLE_DEBUG_MEDIUM
    printf( "INFO: adi_spi_write_word_multiple: address=0x%04x, dataLength=%d\n", address, dataLength );
    for( i = 0; i < dataLength; i++ ) {
        printf( "INFO: adi_spi_write_word_multiple: address=0x%04x, data=0x%04x\n", address, dataPtr[i] );
    }
#endif

    if( spi_handle <= 0 ) {
        rc = adi_spi_open( spi_bit_rate );
        if( rc != ADI_NO_ERROR ) {
            return( ADI_SPI_DRIVER_ERROR );
        }
        called_adi_spi_open = true;
    }

    // Set write bit.
    addr = address | 0x8000;
    
    byteLength = dataLength * 2;
    allocBytes = byteLength + 2;
    u08 *data_out = (u08 *) malloc( allocBytes );
    u08 *data_in  = (u08 *) malloc( allocBytes );

    data_out[bytes_out++] = (u08)  (addr & 0x00ff);
    data_out[bytes_out++] = (u08) ((addr & 0xff00 ) >>  8);

    memcpy( &data_out[bytes_out], dataPtr, byteLength );
    bytes_out += byteLength;
    bytes_in = bytes_out;

    rc = adi_spi_write( bytes_out, data_out, bytes_in, data_in );

    free( data_out );
    free( data_in );

    if( called_adi_spi_open ) {
        adi_spi_close( );
    }

    return( rc );

} // adi_spi_write_word_multiple

//=========================================================================
// Read 16-bit word at specified address
//=========================================================================
int adi_spi_read_word( u16 address, u16 *data ) {
    int rc = ADI_ERROR_CODE_MISSING;
    int i = 0;
    u16 addr;
    int bytes_out = 0;
    int bytes_in  = 0;
    u08 data_out[10];
    u08 data_in[10];
    u16 dataWord = 0;
    bool called_adi_spi_open = false;

    if( spi_handle <= 0 ) {
        rc = adi_spi_open( spi_bit_rate );
        if( rc != ADI_NO_ERROR ) {
            return( ADI_SPI_DRIVER_ERROR );
        }
        called_adi_spi_open = true;
    }

    addr = address;

    data_out[bytes_out++] = (u08)  (addr & 0x00ff);
    data_out[bytes_out++] = (u08) ((addr & 0xff00 ) >>  8);
    data_out[bytes_out++] = 0; // Dummy Byte
    data_out[bytes_out++] = 0; // Dummy Byte
    data_out[bytes_out++] = 0;
    data_out[bytes_out++] = 0;
    bytes_in = bytes_out;

    rc = adi_spi_write( bytes_out, data_out, bytes_in, data_in );
    if( rc != ADI_NO_ERROR ) {
        if( called_adi_spi_open ) {
            adi_spi_close( );
        }
        return( rc );
    }

    dataWord = ((u16) data_in[4]) | (((u16) data_in[5]) << 8);

#ifdef ENABLE_DEBUG_MEDIUM
    printf( "INFO: spi_read_word: address=0x%04x, data=0x%04x\n", address, dataWord );
#endif

    *data = dataWord;

    if( called_adi_spi_open ) {
        adi_spi_close( );
    }

    return( rc );

} // adi_spi_read_word

//=========================================================================
// Read 16-bit word at specified address
//=========================================================================
u16 adi_spi_read_word_py( u16 address ) {
    int rc = ADI_ERROR_CODE_MISSING;
    u16 read_data;
    
    rc = adi_spi_read_word( address, &read_data );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_spi_read_word_py: Error returned from adi_spi_read_word.\n" );
    }
    return( read_data );

} // adi_spi_read_word_py

//=========================================================================
// Read multiple words
//=========================================================================
int adi_spi_read_word_multiple( u16 address, int dataLength, u16 *dataPtr ) {
    int  rc = ADI_ERROR_CODE_MISSING;
    u16  addr;
    int  bytes_out = 0;
    int  bytes_in = 0;
    int  byteLength;
    int  allocBytes;
    bool called_adi_spi_open = false;
    int i;

    if( spi_handle <= 0 ) {
        rc = adi_spi_open( spi_bit_rate );
        if( rc != ADI_NO_ERROR ) {
            return( ADI_SPI_DRIVER_ERROR );
        }
        called_adi_spi_open = true;
    }

#ifdef ENABLE_DEBUG_MEDIUM
    printf( "INFO: spi_read_word_multiple: address=0x%04x, datalength=0x%04x\n", address, dataLength );
#endif

    byteLength = dataLength * 2;
    allocBytes = byteLength;
    u08 *data_out = (u08 *) malloc( allocBytes );
    u08 *data_in  = (u08 *) malloc( allocBytes );

    addr = address;

    data_out[bytes_out++] = (u08)  (addr & 0x00ff);
    data_out[bytes_out++] = (u08) ((addr & 0xff00 ) >>  8);
    data_out[bytes_out++] = 0; // Dummy Byte
    data_out[bytes_out++] = 0; // Dummy Byte

    bytes_out += byteLength;
    bytes_in = bytes_out;

    rc = adi_spi_write( bytes_out, data_out, bytes_in, data_in );

    memcpy( dataPtr, data_in+4, byteLength );
    /* FIXME:
    u16 *data_ptr = (u16 *) data_in+4;
    for( i = 0; i < dataLength; i++ ) {
       dataPtr[i] = (u16) data_in[i*2+4];
    }
    */
    free( data_out );
    //free( data_in );

    if( called_adi_spi_open ) {
        adi_spi_close( );
    }

    return( rc );

} // adi_spi_read_word_multiple

//====================================================================================
// Wait for HSP ready
//====================================================================================
int adi_wait_for_hsp_ready( ) {
    int rc = ADI_ERROR_CODE_MISSING;

    // FIXME: Not sure what to do for this.
    
    return( rc );

} // adi_wait_for_hsp_ready

//====================================================================================
// Send command to HSP
//====================================================================================
int adi_send_command( u16 command, u16 address, int word_count, adi_attribute_e attribute ) {
    int rc = ADI_ERROR_CODE_MISSING;
    ADI_HSP_REGS_ADI_S2H_MBX_STS_t S2H_MBX_STS;

    u16 mboxAddress;
    u16 writeBurstData[4];
    int byte_count = word_count * 2;
    int i = 0;

    mboxAddress = ADDR_HSP_REGS_ADI_S2H_MBX_FIFO_PUSH;
    mboxAddress = mboxAddress | 0x8000;
      
    writeBurstData[i++] = address;          // Address[15:0]
    writeBurstData[i++] = command;          // HSP Mailbox Write Command
    writeBurstData[i++] = (u16) attribute;  // Attribute
    writeBurstData[i++] = (u16) byte_count; // Data Size (bytes)

    rc = adi_spi_write_word_multiple( mboxAddress, i, writeBurstData );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_send_command: Error returned from adi_spi_write_word_multiple ...\n" );
        return( rc );
    }

    // Write Valid Bit
    mboxAddress = ADDR_HSP_REGS_ADI_S2H_MBX_STS;
    S2H_MBX_STS.VALUE16 = RSTVAL_HSP_REGS_ADI_S2H_MBX_STS;
    S2H_MBX_STS.S2H_FIFO_VALID = 1;
    mboxAddress = mboxAddress | 0x8000;

    rc = adi_spi_write_word( mboxAddress, S2H_MBX_STS.VALUE16 );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_send_command: Error returned from adi_spi_write_word ...\n" );
    }

    return( rc );

} // adi_send_command

//====================================================================================
// Clear HSP H2S valid
//====================================================================================
int adi_clear_h2s_valid( ) {
    int rc = ADI_ERROR_CODE_MISSING;
    ADI_HSP_REGS_ADI_H2S_MBX_INSTS_t H2S_MBX_INSTS;
    u16 mboxAddress;

    mboxAddress = ADDR_HSP_REGS_ADI_H2S_MBX_INSTS;
    mboxAddress = mboxAddress | 0x8000;
    H2S_MBX_INSTS.VALUE16 = RSTVAL_HSP_REGS_ADI_H2S_MBX_INSTS;
    H2S_MBX_INSTS.H2S_FIFO_VALID = 1;

    rc = adi_spi_write_word( mboxAddress, H2S_MBX_INSTS.VALUE16 );

    return( rc );

} // adi_clear_h2s_valid

//====================================================================================
// Check for H2S Error
//====================================================================================
int adi_check_for_h2s_error( ) {
    int rc = ADI_ERROR_CODE_MISSING;
    int rc2 = ADI_ERROR_CODE_MISSING;
    ADI_HSP_REGS_ADI_H2S_MBX_INSTS_t H2S_MBX_INSTS;
    
    rc = adi_spi_read_word( ADDR_HSP_REGS_ADI_H2S_MBX_INSTS, &H2S_MBX_INSTS.VALUE16 );
    if( H2S_MBX_INSTS.H2S_ERR_BIT == 1 ) {
        printf( "ERROR: ADI_H2S_MBX_INSTS.H2S_ERR_BIT == 1\n" );
	rc = ADI_H2S_ERROR;
	rc2 = adi_dump_error_log();
	if( rc2 != ADI_NO_ERROR ) {
            printf( "ERROR: Problem reading the error log.\n" );
	}
    }
    
    return( rc );

} // adi_check_for_h2s_error

//====================================================================================
// Check 1SP completion code
//====================================================================================
int adi_check_done_code( ) {
    int rc = ADI_ERROR_CODE_MISSING;
    int word_count;
    u16 readData[2];
    u32 completionCode;

    rc = adi_wait_for_h2s_valid();
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: Error returned from adi_wait_for_h2s_valid\n" );
        return( rc );
    }
    rc = adi_get_data( 2, readData );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: Error returned from adi_get_data\n" );
        return( rc );
    }
    rc = adi_clear_h2s_valid( );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: Error returned from adi_clear_h2s_valid\n" );
        return( rc );
    }

    completionCode = ((u32) readData[1] << 16) | readData[0];
    if( completionCode != 0x444f4e45 ) {
        printf( "ERROR: Completion code mistmatch: expect = 0x%08x, actual = 0x%08x.\n", completionCode, 0x444f4e45 );
    }
    
    return( rc );

} // adi_check_done_code

//====================================================================================
// Clear HSP H2S Error
//====================================================================================
int adi_clear_h2s_error( ) {
    int rc = ADI_ERROR_CODE_MISSING;
    ADI_HSP_REGS_ADI_H2S_MBX_INSTS_t H2S_MBX_INSTS;
    u16 mboxAddress;

#ifdef ENABLE_DEBUG_LOW
    printf( "INFO: adi_clear_h2s_error\n" );
#endif
    mboxAddress = ADDR_HSP_REGS_ADI_H2S_MBX_INSTS;
    mboxAddress = mboxAddress | 0x8000;
    H2S_MBX_INSTS.VALUE16 = RSTVAL_HSP_REGS_ADI_H2S_MBX_INSTS;
    H2S_MBX_INSTS.H2S_ERR_BIT = 1;

    rc = adi_spi_write_word( mboxAddress, H2S_MBX_INSTS.VALUE16 );

    return( rc );

} // adi_clear_h2s_error

//====================================================================================
// Wait for HSP S2H valid
//====================================================================================
int adi_wait_for_h2s_valid( ) {
    int rc = ADI_ERROR_CODE_MISSING;
    ADI_HSP_REGS_ADI_H2S_MBX_INSTS_t H2S_MBX_INSTS;

#ifdef ENABLE_DEBUG_MEDIUM
    printf( "INFO: adi_wait_for_h2s_valid\n" );
#endif

    H2S_MBX_INSTS.VALUE16 = RSTVAL_HSP_REGS_ADI_H2S_MBX_INSTS;

    while( H2S_MBX_INSTS.H2S_FIFO_VALID == 0 ) {
        rc = adi_spi_read_word( ADDR_HSP_REGS_ADI_H2S_MBX_INSTS, &H2S_MBX_INSTS.VALUE16 );
        if( rc != ADI_NO_ERROR ) {
            return( rc );
        }
        if( H2S_MBX_INSTS.H2S_ERR_BIT == 1 ) {
            printf( "ERROR: ADI_H2S_MBX_INSTS.H2S_ERR_BIT == 1\n" );
	    rc = ADI_H2S_ERROR;
            return( rc );
        }
    }

    return( rc );

} // adi_wait_for_h2s_valid

//====================================================================================
// Wait for HSP S2H not valid
//====================================================================================
int adi_wait_for_s2h_not_valid( ) {
    int rc = ADI_ERROR_CODE_MISSING;
    ADI_HSP_REGS_ADI_S2H_MBX_STS_t S2H_MBX_STS;
    ADI_HSP_REGS_ADI_H2S_MBX_INSTS_t H2S_MBX_INSTS;

#ifdef ENABLE_DEBUG_MEDIUM
    printf( "INFO: adi_wait_for_s2h_not_valid\n" );
#endif

    S2H_MBX_STS.VALUE16 = RSTVAL_HSP_REGS_ADI_S2H_MBX_STS;
    S2H_MBX_STS.S2H_FIFO_VALID = 1;

    while( S2H_MBX_STS.S2H_FIFO_VALID != 0 ) {
        rc = adi_spi_read_word( ADDR_HSP_REGS_ADI_S2H_MBX_STS, &S2H_MBX_STS.VALUE16 );
        if( rc != ADI_NO_ERROR ) {
            return( rc );
        }
        if( S2H_MBX_STS.S2H_FIFO_VALID != 0 ) {
	    rc = adi_check_for_h2s_error();
            if( rc != ADI_NO_ERROR ) {
                printf( "ERROR: adi_wait_for_s2h_not_valid: Error returned from adi_check_for_h2s_error.\n" );
                return( rc );
            }
        }
    }
    
    return( rc );

} // adi_wait_for_s2h_not_valid

//====================================================================================
// Read and Display Error Log
//====================================================================================
int adi_dump_error_log( ) {
    int rc = ADI_ERROR_CODE_MISSING;
    adi_ErrorLog_s hspErrorLog_actual;
    u16  readData[16];
    int burstSize = 16;

    printf( "INFO: Sending CMD_ERROR_LOG ...\n" );
    
#ifdef ENABLE_DEBUG_MEDIUM
    printf( "INFO: Calling adi_clear_h2s_error ...\n" );
#endif
    rc = adi_clear_h2s_error( );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_dump_error_log: Error returned from adi_clear_h2s_error.\n" );
        return( rc );
    }

#ifdef ENABLE_DEBUG_MEDIUM
    printf( "INFO: Calling adi_send_command ...\n" );
#endif
    rc = adi_send_command( CMD_ERROR_LOG, 0, burstSize, NO_ATTR );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_dump_error_log: Error returned from adi_send_command.\n" );
        return( rc );
    }

#ifdef ENABLE_DEBUG_MEDIUM
    printf( "INFO: Calling adi_wait_for_s2h_not_valid ...\n" );
#endif
    rc = adi_wait_for_s2h_not_valid( );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_dump_error_log: Error returned from adi_wait_for_s2h_not_valid.\n" );
        return( rc );
    }
#ifdef ENABLE_DEBUG_MEDIUM
    printf( "INFO: Calling adi_wait_for_h2s_valid ...\n" );
#endif
    rc = adi_wait_for_h2s_valid( );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_dump_error_log: Error returned from adi_wait_for_h2s_valid.\n" );
        return( rc );
    }
    // wait_for_mbox_interrupt();
#ifdef ENABLE_DEBUG_MEDIUM
    printf( "INFO: Calling adi_get_data ...\n" );
#endif
    rc = adi_get_data( burstSize, hspErrorLog_actual.data16 );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_dump_error_log: Error returned from adi_get_data.\n" );
        return( rc );
    }

#ifdef ENABLE_DEBUG_MEDIUM
    printf( "INFO: Calling adi_clear_h2s_valid ...\n" );
#endif
    rc = adi_clear_h2s_valid( );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_dump_error_log: Error returned from  adi_clear_h2s_valid.\n" );
        return( rc );
    }
    
    for( int i = 0; i < 4; i++ ) {
        printf( "INFO: MSFTDebugData0[%d] = 0x%08x\n", i, hspErrorLog_actual.MSFTDebugData0[i] );
    }
    printf( "INFO: errorStatus = 0x%08x\n", hspErrorLog_actual.errorStatus );
    for( int i = 0; i < 3; i++ ) {
        printf( "INFO: MSFTDebugData1[%d] = 0x%08x\n", i, hspErrorLog_actual.MSFTDebugData1[i] );
    }

    return( rc );

} // adi_dump_error_log

//====================================================================================
// Get H2S_MBX_INSTS
//====================================================================================
int adi_get_h2s_mbx_insts( u16 *rd_data ) {
    int rc = ADI_ERROR_CODE_MISSING;

    rc = adi_spi_read_word( ADDR_HSP_REGS_ADI_H2S_MBX_INSTS, rd_data );

#ifdef ENABLE_DEBUG_MEDIUM
    printf( "INFO: adi_get_h2s_mbx_insts: H2S_MBX_INSTS = 0x%04x\n", *rd_data );
#endif

    return( rc );

} // adi_get_h2s_mbx_insts

//====================================================================================
// Send write data to HSP
//====================================================================================
int adi_send_data( int word_count, u16 *wr_data ) {
    int rc = ADI_ERROR_CODE_MISSING;
    ADI_HSP_REGS_ADI_S2H_MBX_STS_t S2H_MBX_STS;
    u16 mboxAddress;

#ifdef ENABLE_DEBUG_MEDIUM
    printf( "INFO: adi_send_data: word_count=%d\n", word_count );
#endif

    mboxAddress = ADDR_HSP_REGS_ADI_S2H_MBX_FIFO_PUSH;
    mboxAddress = mboxAddress | 0x8000;

    rc = adi_spi_write_word_multiple( mboxAddress, word_count, wr_data );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }

    if( (word_count & 1) != 0 ) {
        rc = adi_spi_write_word( mboxAddress, 0 );
        if( rc != ADI_NO_ERROR ) {
            return( rc );
        }
    }

    // Write Valid Bit
    mboxAddress = ADDR_HSP_REGS_ADI_S2H_MBX_STS;
    mboxAddress = mboxAddress | 0x8000;
    S2H_MBX_STS.VALUE16 = RSTVAL_HSP_REGS_ADI_S2H_MBX_STS;
    S2H_MBX_STS.S2H_FIFO_VALID = 1;
      
    rc = adi_spi_write_word( mboxAddress, S2H_MBX_STS.VALUE16 );

    return( rc );

} // adi_send_data

//====================================================================================
// Send Write Register List to HSP
//====================================================================================
int adi_send_write_register_list( u16 *wr_addr, u16 *wr_data, int word_count ) {
    int rc = ADI_ERROR_CODE_MISSING;
    ADI_HSP_REGS_ADI_S2H_MBX_STS_t S2H_MBX_STS;
    u16 mboxAddress;
    u16 *writeData;
    int count = 0;
    int i;

#ifdef ENABLE_DEBUG_MEDIUM
    printf( "INFO: adi_send_write_register_list: word_count=%d\n", word_count );
#endif

    writeData = malloc (sizeof (u16) * word_count * 2 );

    for( i = 0; i < word_count; i++  ) {
       writeData[count++] = wr_data[i];
       writeData[count++] = wr_addr[i];
    }

    mboxAddress = ADDR_HSP_REGS_ADI_S2H_MBX_FIFO_PUSH;
    mboxAddress = mboxAddress | 0x8000;

    rc = adi_spi_write_word_multiple( mboxAddress, word_count, writeData );
    if( rc != ADI_NO_ERROR ) {
        free( writeData );
        return( rc );
    }

    // Write Valid Bit
    mboxAddress = ADDR_HSP_REGS_ADI_S2H_MBX_STS;
    mboxAddress = mboxAddress | 0x8000;
    S2H_MBX_STS.VALUE16 = RSTVAL_HSP_REGS_ADI_S2H_MBX_STS;
    S2H_MBX_STS.S2H_FIFO_VALID = 1;

    rc = adi_spi_write_word( mboxAddress, S2H_MBX_STS.VALUE16 );

    free( writeData );
    
    return( rc );

} // adi_send_write_register_list

//====================================================================================
// Send Read Register List to HSP
//====================================================================================
int adi_send_read_register_list( u16 *rd_addr, int word_count ) {
    int rc = ADI_ERROR_CODE_MISSING;
    ADI_HSP_REGS_ADI_S2H_MBX_STS_t S2H_MBX_STS;
    u16 mboxAddress;
    u16 *readAddr;
    int burstSize = word_count;
    int count = 0;
    int i;


#ifdef ENABLE_DEBUG_MEDIUM
    printf( "INFO: adi_send_read_register_list: word_count=%d\n", word_count );
#endif

    if( (word_count & 1) != 0 ) {
        burstSize++;
    }

    readAddr = malloc (sizeof (u16) * burstSize);

    for( i = 0; i < word_count; i++  ) {
        readAddr[count++] = rd_addr[i];
    }

    if( (word_count & 1) != 0 ) {
        readAddr[count++] = 0xffff;
    }

    mboxAddress = ADDR_HSP_REGS_ADI_S2H_MBX_FIFO_PUSH;
    mboxAddress = mboxAddress | 0x8000;

    rc = adi_spi_write_word_multiple( mboxAddress, burstSize, readAddr );
    if( rc != ADI_NO_ERROR ) {
        free( readAddr );
        return( rc );
    }

    // Write Valid Bit
    mboxAddress = ADDR_HSP_REGS_ADI_S2H_MBX_STS;
    mboxAddress = mboxAddress | 0x8000;
    S2H_MBX_STS.VALUE16 = RSTVAL_HSP_REGS_ADI_S2H_MBX_STS;
    S2H_MBX_STS.S2H_FIFO_VALID = 1;

    rc = adi_spi_write_word( mboxAddress, S2H_MBX_STS.VALUE16 );

    free( readAddr );

    return( rc );

} // adi_send_read_register_list

//====================================================================================
// Get read data from HSP
//====================================================================================
int adi_get_data( int word_count, u16 *rd_data ) {
    int rc = ADI_ERROR_CODE_MISSING;
    u16 readData;
    u16 mboxAddress;

#ifdef ENABLE_DEBUG_MEDIUM
    printf( "INFO: adi_get_data: word_count=%d\n", word_count );
#endif

    mboxAddress = ADDR_HSP_REGS_ADI_H2S_MBX_FIFO_POP;
      
    rc = adi_spi_read_word_multiple( mboxAddress, word_count, rd_data );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }

    return( rc );

} // adi_get_data

//====================================================================================
// Write Address Burst with incrementing addresses
//====================================================================================
int adi_write_burst( u16 addr, u16 word_count, u16 *wr_data ) {
    int rc = ADI_ERROR_CODE_MISSING;

    rc = adi_send_command( CMD_REGISTER_CFG, addr, word_count, WRITE_ATTR );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }
    rc = adi_wait_for_s2h_not_valid( );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }
    rc = adi_send_data( word_count, wr_data );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }
    rc = adi_wait_for_s2h_not_valid( );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }

    return( rc );

} // adi_write_burst

///====================================================================================
// Read Address Burst with incrementing addresses
//====================================================================================
int adi_read_burst( u16 addr, u16 word_count, u16 *rd_data ) {
    int rc = ADI_ERROR_CODE_MISSING;

    rc = adi_send_command( CMD_REGISTER_CFG, addr, word_count, NO_ATTR );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }
    rc = adi_wait_for_s2h_not_valid( );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }
    rc = adi_wait_for_h2s_valid( );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }
    rc = adi_get_data( word_count, rd_data );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }
    rc = adi_clear_h2s_valid( );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_read_register_list: Error returned adi_clear_h2s_valid.\n" );
        return( rc );
    }

    return( rc );

} // adi_read_burst

//====================================================================================
// Write Register
//====================================================================================
int adi_write_register( u16 addr, u16 wr_data ) {
    int rc = ADI_ERROR_CODE_MISSING;
    u16 writeAddr[1];
    u16 writeData[1];

    writeData[0] = wr_data;
    writeAddr[0] = addr;

    rc = adi_send_command( CMD_REGISTER_CFG, 0, 2, WRITE_ATTR );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }

    rc = adi_wait_for_s2h_not_valid( );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }

    rc = adi_send_write_register_list( writeAddr, writeData, 2 );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }

    rc = adi_wait_for_s2h_not_valid( );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }

    return( rc );

} // adi_write_register

//====================================================================================
// Write Register Backdoor
//====================================================================================
int adi_write_register_backdoor( u16 addr, u16 data ) {
    int rc = ADI_ERROR_CODE_MISSING;
    ADI_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS_t HSP_BYPASS;

    // Enable backdoor access bypassing the HSP. Only valid for FPGA.
    HSP_BYPASS.HSP_BYPASS_EN = 1;
    rc = adi_spi_write_word( ADDR_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS, HSP_BYPASS.VALUE16 );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }

    rc = adi_spi_write_word( addr, data );

    // Disable backdoor access bypassing the HSP. Only valid for FPGA.
    HSP_BYPASS.HSP_BYPASS_EN = 0;
    rc = adi_spi_write_word( ADDR_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS, HSP_BYPASS.VALUE16 );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }

    return( rc );

} // adi_write_register_backdoor

//====================================================================================
// Read Register
//====================================================================================
int adi_read_register( u16 addr, u16 *rd_data ) {
    int rc = ADI_ERROR_CODE_MISSING;
    u16 readAddr[1];
    u16 readData[1];

    readAddr[0] = addr;

    rc = adi_send_command( CMD_REGISTER_CFG, 0, 2, NO_ATTR );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_read_register: Error returned from adi_send_command.\n" );
        return( rc );
    }
    rc = adi_wait_for_s2h_not_valid( );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_read_register: Error returned from adi_wait_for_s2h_not_valid.\n" );
        return( rc );
    }
    rc = adi_send_read_register_list( readAddr, 1 );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_read_register: Error returned from adi_send_read_register_list.\n" );
        return( rc );
    }
    rc = adi_wait_for_s2h_not_valid( );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_read_register: Error returned from adi_wait_for_s2h_not_valid.\n" );
        return( rc );
    }
    rc = adi_get_data( 1, readData );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_read_register: Error returned from adi_get_data.\n" );
        return( rc );
    }
    rc = adi_clear_h2s_valid( );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_read_register: Error returned from adi_clear_h2s_valid.\n" );
    }

    *rd_data = readData[0];

    return( rc );

} // adi_read_register

//====================================================================================
// Read Register for python interface
//====================================================================================
u16 adi_read_register_py( u16 addr ) {
    int rc = ADI_ERROR_CODE_MISSING;
    u16 read_data;
    
    rc = adi_read_register( addr, &read_data );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_read_register_py: Error returned from adi_read_register.\n" );
    }
    return( read_data );
    
} // adi_read_register_py

//====================================================================================
// Check Register for python interface
//====================================================================================
u16 adi_check_register_py( u16 addr, u16 check_data  ) {
    int  rc = ADI_ERROR_CODE_MISSING;
    u16  read_data;
    bool result = false;
    
    rc = adi_read_register( addr, &read_data );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_check_register_py: Error returned from adi_read_register.\n" );
    }
    if( read_data != check_data ) {
        printf( "ERROR: adi_check_register_py: miscompare, expect = 0x%04x, actual = 0x%04x\n", check_data, read_data );
        result = true;
    }
    
    return( result );
    
} // adi_check_register_py

//====================================================================================
// Read Register Backdoor
//====================================================================================
int adi_read_register_backdoor( u16 addr, u16 *data ) {
    int rc = ADI_ERROR_CODE_MISSING;
    ADI_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS_t HSP_BYPASS;

    // Enable backdoor access bypassing the HSP. Only valid for FPGA.
    HSP_BYPASS.HSP_BYPASS_EN = 1;
    rc = adi_spi_write_word( ADDR_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS, HSP_BYPASS.VALUE16 );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }

    rc = adi_spi_read_word( addr, data );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }

    // Disable backdoor access bypassing the HSP. Only valid for FPGA.
    HSP_BYPASS.HSP_BYPASS_EN = 0;
    rc = adi_spi_write_word( ADDR_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS, HSP_BYPASS.VALUE16 );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }

    return( rc );

} // adi_read_register_backdoor

//====================================================================================
// Read Register Backdoor for python interface
//====================================================================================
u16 adi_read_register_backdoor_py( u16 addr ) {
    int rc = ADI_ERROR_CODE_MISSING;
    u16 read_data;
    
    rc = adi_read_register_backdoor( addr, &read_data );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_read_register_backdoor_py: Error returned from adi_read_register_backdoor.\n" );
    }
    return( read_data );
    
} // adi_read_register_backdoor_py

//====================================================================================
// Write list of registers
//====================================================================================
int adi_write_register_list( u16 *wr_addr, int burst_size, u16 *wr_data ) {
    int rc = ADI_ERROR_CODE_MISSING;

    rc = adi_send_command( CMD_REGISTER_CFG, 0, burst_size, WRITE_ATTR );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }
    rc = adi_wait_for_s2h_not_valid( );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }
    rc = adi_send_write_register_list( wr_addr, wr_data, burst_size );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }
    rc = adi_wait_for_s2h_not_valid( );

    return( rc );

} // adi_write_register_list

//====================================================================================
// Read list of registers
//====================================================================================
int adi_read_register_list( u16 *rd_addr, int burst_size, u16 *rd_data ) {
    int rc = ADI_ERROR_CODE_MISSING;

    rc = adi_send_command( CMD_REGISTER_CFG, 0,  burst_size, NO_ATTR );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_read_register_list: Error returned adi_send_command.\n" );
        return( rc );
    }
    rc = adi_wait_for_s2h_not_valid( );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_read_register_list: Error returned adi_wait_for_s2h_not_valid.\n" );
        return( rc );
    }
    rc = adi_send_read_register_list( rd_addr, burst_size );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_read_register_list: Error returned  adi_send_read_register_list.\n" );
        return( rc );
    }
    rc = adi_wait_for_s2h_not_valid( );
    // FIXME: rc = adi_wait_for_mbox_interrupt();
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_read_register_list: Error returned adi_wait_for_s2h_not_valid.\n" );
        return( rc );
    }
    rc = adi_get_data( burst_size, rd_data );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_read_register_list: Error returned from adi_get_data.\n" );
        return( rc );
    }
    rc = adi_clear_h2s_valid( );
    if( rc != ADI_NO_ERROR ) {
        printf( "ERROR: adi_read_register_list: Error returned adi_clear_h2s_valid.\n" );
    }

    return( rc );

} // adi_read_register_list

//====================================================================================
// returns number of words in str
//====================================================================================
#define OUT    0 
#define IN    1

unsigned countWords( char *str ) { 
    int state = OUT; 
    unsigned wc = 0;  // word count 
  
    // Scan all characters one by one 
    while (*str) { 
        // If next character is a separator, set the  
        // state as OUT 
        if (*str == ' ' || *str == '\n' || *str == '\t') {
            state = OUT; 
        }
        // If next character is not a word separator and  
        // state is OUT, then set the state as IN and  
        // increment word count 
        else if (state == OUT)  { 
            state = IN; 
            ++wc; 
        } 
  
        // Move to next character 
        ++str; 
    } 
  
    return wc; 
}

//====================================================================================
// Load HSP Memory
//====================================================================================
int adi_load_hsp( adi_loadTargets_e loadTarget, char *fileName ) {
    int  rc = ADI_ERROR_CODE_MISSING;
    bool called_adi_spi_open = false;
    FILE *file;
    int  lineNum = 0;
    u16  address = 0;
    u64  data64[4];
    u16  writeBurstData[3];
    u16  readBurstData[3];
    char line[128];
    int i, j;
    int lineWordCount;
    int err;

    ADI_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE_t backdoor_enable;

    if( spi_handle <= 0 ) {
        rc = adi_spi_open( spi_bit_rate );
        if( rc != ADI_NO_ERROR ) {
            return( ADI_SPI_DRIVER_ERROR );
        }
        called_adi_spi_open = true;
    }

    // Open the file
    file = fopen( fileName, "r" );
    if( !file ) {
        printf( "ERROR: Unable to open file '%s'\n", fileName );
        return( ADI_FILE_NOT_FOUND );
    }

    // Enable back door load of HSP ROM.
    backdoor_enable.VALUE16 = RSTVAL_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE;
    if( loadTarget == HSP_ROM ) {
        backdoor_enable.HSPROM_BDEN = 1;
    }
    else if( loadTarget == HSP_RAM ) {
        backdoor_enable.HSPRAM_BDEN = 1;
    }
    else if( loadTarget == EFUSE ) {
        backdoor_enable.FMCEFUSE_BDEN = 1;
    }
    else {
        return( ADI_UNEXPECTED_ARGS );
    }

    rc = adi_spi_write_word( ADDR_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE, backdoor_enable.VALUE16 );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }
    
    address = 0x1000;
    err = 0;

    while( fgets( line, 128, file ) != NULL ) {
        lineWordCount = countWords( line );
	if( lineWordCount == 1 ) {
            sscanf( line, "%08llx", &data64[0] );
	}
	else if( lineWordCount == 2 ) {
	    if( strstr( line, "//" ) != NULL ) {
	        lineWordCount = 1;
            }
            sscanf( line, "%010llx", &data64[0] );
	}
	else if( lineWordCount == 4 ) {
            sscanf( line, "%08llx %08llx %08llx %08llx", &data64[0], &data64[1], &data64[2], &data64[3] );
	}
	else {
	    printf( "ERROR: unexpected file format.\n" );
	    rc = ADI_FILE_FORMAT_ERROR;
            return( rc );
	}
	
	for( i = 0; i < lineWordCount; i++ ) {
            writeBurstData[0] = (u16) ((data64[i] >> 32) & 0xffff);
	    writeBurstData[1] = (u16) ((data64[i] >> 16) & 0xffff);
	    writeBurstData[2] = (u16) (data64[i] & 0xffff);
            rc = adi_spi_write_word_multiple( address, 3, writeBurstData );
            if( rc != ADI_NO_ERROR ) {
                return( rc );
            }

            rc = adi_spi_read_word_multiple( address, 3, readBurstData );
            if( rc != ADI_NO_ERROR ) {
                return( rc );
            }

            for( j = 0; j < 3; j++ ) {
                if(readBurstData[j] != writeBurstData[j]) {
	            printf("Load failed: writeBurstData[%d] = 0x%04x readBurstData[%d]: 0x%04x\n",j,writeBurstData[j],j,readBurstData[j]); 
	            err++;
                } 
#ifdef ENABLE_DEBUG_HIGH
                else  {
	            printf("Load: writeBurstData[%d] = 0x%04x readBurstData[%d]: 0x%04x\n",j,writeBurstData[j],j,readBurstData[j]); 
	        }
#endif
            }
	
            address++;
	}
        lineNum++;
    }

    fclose( file );

    if( err > 0 ) {
        printf( "ERROR: backdoor load failed\n" );
        printf( "ERROR: No. of Miscompares = %0d\n", err );
    }
    else {
        if( loadTarget == HSP_ROM ) {
            printf( "INFO: adi_load_hsp hsp_rom: PASS\n" );
        }
        else if( loadTarget == HSP_RAM ) {
            printf( "INFO: adi_load_hsp hsp_ram: PASS\n" );
        }
        else if( loadTarget == EFUSE ) {
            printf( "INFO: adi_load_hsp efuse: PASS\n" );
        }
        else {
            return( ADI_UNEXPECTED_ARGS );
        }
    }

    backdoor_enable.VALUE16 = RSTVAL_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE;
    rc = adi_spi_write_word( ADDR_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE, backdoor_enable.VALUE16 );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }
    return( rc );

} // adi_load_hsp

//====================================================================================
// Verifies HSP Memory
//====================================================================================
int adi_verify_hsp( adi_loadTargets_e verifyTarget, const char *fileName ) {
    int  rc = ADI_ERROR_CODE_MISSING;
    bool called_adi_spi_open = false;
    FILE *file;
    int  wordCount;
    int  lineWordCount;
    int  i = 0;
    int  lineNum = 0;
    u16  address = 0;
    u64  expect64[4];
    u64  actual64;
    u64  bitsDiffer64;
    u16  readBurstData[3];
    char line[128];
    ADI_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE_t backdoor_enable;
    int j;
    int err;

    if( spi_handle <= 0 ) {
        rc = adi_spi_open( spi_bit_rate );
        if( rc != ADI_NO_ERROR ) {
            return( ADI_SPI_DRIVER_ERROR );
        }
        called_adi_spi_open = true;
    }

    // Open the file
    file = fopen( fileName, "r" );
    if( !file ) {
        printf( "ERROR: Unable to open file '%s'\n", fileName );
        return( ADI_FILE_NOT_FOUND );
    }

    // Enable back door load of HSP ROM.
    backdoor_enable.VALUE16 = RSTVAL_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE;
    if( verifyTarget == HSP_ROM ) {
        backdoor_enable.HSPROM_BDEN = 1;
	wordCount = 4096;
    }
    else if( verifyTarget == HSP_RAM ) {
        backdoor_enable.HSPRAM_BDEN = 1;
	wordCount = 8192;
    }
    else if( verifyTarget == EFUSE ) {
        backdoor_enable.FMCEFUSE_BDEN = 1;
	wordCount = 64;
    }
    else {
        return( ADI_UNEXPECTED_ARGS );
    }

    rc = adi_spi_write_word( ADDR_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE, backdoor_enable.VALUE16 );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }

    address = 0x1000;
    err = 0;

    while( fgets( line, 128, file ) != NULL ) {
        lineWordCount = countWords( line );
	if( lineWordCount == 1 ) {
            sscanf( line, "%08llx", &expect64[0] );
	}
	else if( lineWordCount == 2 ) {
	    if( strstr( line, "//" ) != NULL ) {
	        lineWordCount = 1;
            }
            sscanf( line, "%010llx", &expect64[0] );
	}
	else if( lineWordCount == 4 ) {
            sscanf( line, "%08llx %08llx %08llx %08llx", &expect64[0], &expect64[1], &expect64[2], &expect64[3] );
	}
	else {
	    printf( "ERROR: unexpected file format.\n" );
	    rc = ADI_FILE_FORMAT_ERROR;
            return( rc );
	}
	
	for( i = 0; i < lineWordCount; i++ ) {
            rc = adi_spi_read_word_multiple( address, 3, readBurstData );
            if( rc != ADI_NO_ERROR ) {
                return( rc );
            }

#ifdef ENABLE_DEBUG_HIGH
            for(j = 0; j < 3; j++) {
                printf("readBurstData[%d]: 0x%04x\n",j,readBurstData[j]); 
            }
#endif

	    actual64 = 0;
	    actual64 = actual64 | ((u64) readBurstData[0] << 32);
	    actual64 = actual64 | ((u64) readBurstData[1] << 16);
	    actual64 = actual64 |  (u64) readBurstData[2];

#ifdef ENABLE_DEBUG_HIGH
            printf("actual64 = 0x%010llx\n",actual64);
#endif

	    if( expect64[i] != actual64 ) {
                err++;
	        bitsDiffer64 = expect64[i] ^ actual64;
                if( verifyTarget == HSP_ROM ) {
	            printf( "ERROR: Miscompare at line = %d, address = 0x%04x, expected = 0x%010llx, actual = 0x%010llx, bitsDiffer = 0x%010llx\n",
			    lineNum, (address - 0x1000), expect64[i], actual64, bitsDiffer64 );
                }
                else if( verifyTarget == HSP_RAM ) {
 	            printf( "ERROR: Miscompare at line = %d, address = 0x%04x, expected = 0x%010llx, actual = 0x%010llx, bitsDiffer = 0x%010llx\n",
			    lineNum, (address - 0x1000), expect64[i], actual64, bitsDiffer64 );
                }
                else if( verifyTarget == EFUSE ) {
 	            printf( "ERROR: Miscompare at line = %d, address = 0x%04x, expected = 0x%08x, actual = 0x%08x, bitsDiffer = 0x%08x\n",
			    lineNum, (address - 0x1000), expect64[i], actual64, bitsDiffer64 );
                }
                else {
                    return( ADI_UNEXPECTED_ARGS );
                }
	    }
	    else {
#ifdef ENABLE_DEBUG_HIGH
                printf( "INFO [verify]: lineNum = %d Address = 0x%04x Received = 0x%010llx Expected = 0x%010llx\n", lineNum, (address - 0x1000), actual64, expect64[i]);
#endif
            }

            if(lineNum == wordCount) {
                break;	
            }
            address++;
	}
        lineNum++;
    }

    fclose( file );
    
    backdoor_enable.VALUE16 = RSTVAL_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE;
    rc = adi_spi_write_word( ADDR_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE, backdoor_enable.VALUE16 );
    /*
    if( rc != ADI_NO_ERROR ) {
        if( called_adi_spi_open ) {
            adi_spi_close( );
        }
        return( rc );
    }    
    */

    if( err > 0 ) {
        printf("ERROR: FAILED\n");
        printf("ERROR: No. of Miscompares = %0d\n",err);
    }
    else {
        printf("INFO: PASS\n");
    }

    return( rc );

} // adi_verify_hsp

//====================================================================================
// Unload HSP Memory
//====================================================================================
int adi_unload_hsp( adi_loadTargets_e unloadTarget, const char *fileName ) {
    int  rc = ADI_ERROR_CODE_MISSING;
    bool called_adi_spi_open = false;
    FILE *file;
    int  wordCount;
    int  i = 0;
    u16  address = 0;
    u64  data64;
    u16  readBurstData[3];
    char line[128];
    ADI_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE_t backdoor_enable;

    if( spi_handle <= 0 ) {
        rc = adi_spi_open( spi_bit_rate );
        if( rc != ADI_NO_ERROR ) {
            return( ADI_SPI_DRIVER_ERROR );
        }
        called_adi_spi_open = true;
    }

    // Open the file
    file = fopen( fileName, "w" );
    if( !file ) {
        printf( "ERROR: Unable to open file '%s'\n", fileName );
        return( ADI_FILE_NOT_FOUND );
    }

    // Enable back door load of HSP ROM.
    backdoor_enable.VALUE16 = RSTVAL_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE;
    if( unloadTarget == HSP_ROM ) {
        backdoor_enable.HSPROM_BDEN = 1;
	wordCount = 4096;
    }
    else if( unloadTarget == HSP_RAM ) {
        backdoor_enable.HSPRAM_BDEN = 1;
	wordCount = 8192;
    }
    else if( unloadTarget == EFUSE ) {
        backdoor_enable.FMCEFUSE_BDEN = 1;
	wordCount = 64;
    }
    else {
        return( ADI_UNEXPECTED_ARGS );
    }

    rc = adi_spi_write_word( ADDR_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE, backdoor_enable.VALUE16 );
    if( rc != ADI_NO_ERROR ) {
        if( called_adi_spi_open ) {
            adi_spi_close( );
        }
        return( rc );
    }

    address = 0x1000;

    for( i = 0; i < wordCount; i++ ) {
        rc = adi_spi_read_word_multiple( address, 3, readBurstData );
        if( rc != ADI_NO_ERROR ) {
            if( called_adi_spi_open ) {
                adi_spi_close( );
            }
            return( rc );
        }
	data64 = 0;
	data64 = data64 | ((u64) readBurstData[0] << 32);
	data64 = data64 | ((u64) readBurstData[1] << 16);
	data64 = data64 |  (u64) readBurstData[2];

        if( unloadTarget == HSP_ROM ) {
            sprintf( line, "%010llx\n", data64 );
            fwrite( line, HSP_ROM_WIDTH/4+1, 1, file );
        }
        else if( unloadTarget == HSP_RAM ) {
            sprintf( line, "%010llx\n", data64 );
            fwrite( line, HSP_RAM_WIDTH/4+1, 1, file );
        }
        else if( unloadTarget == EFUSE ) {
            sprintf( line, "%08x\n", data64 );
            fwrite( line, EFUSE_WIDTH/4+1, 1, file );
        }
        else {
            return( ADI_UNEXPECTED_ARGS );
        }

        address++;
    }

    fclose( file );

    backdoor_enable.VALUE16 = RSTVAL_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE;
    rc = adi_spi_write_word( ADDR_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE, backdoor_enable.VALUE16 );
    if( rc != ADI_NO_ERROR ) {
        if( called_adi_spi_open ) {
            adi_spi_close( );
        }
        return( rc );
    }    

    return( rc );

} // adi_unload_hsp

//====================================================================================
// Load Command File
// - Read the file into an array.
// - Process command data in array.
//====================================================================================
int adi_load_command_file( const char *fileName ) {
    int  rc = ADI_ERROR_CODE_MISSING;
    bool called_adi_spi_open = false;
    FILE *file;
    int  numCommandWords;
    int  numDataWords;
    u16  byteCount;
    int  index;
    u16  command;
    u16  destAddress;
    u16  attributes;
    u16  address;
    u16  data16;
    u16  *writeData;
    u16  *commandData;
    u16  *readData;
    char line[128];
    int  i, j;
    int  isGroupedCommand = 0;
    adi_commandLoadStates_e state = CMD_LOAD_HEADER;

    if( spi_handle <= 0 ) {
        rc = adi_spi_open( spi_bit_rate );
        if( rc != ADI_NO_ERROR ) {
            return( ADI_SPI_DRIVER_ERROR );
        }
        called_adi_spi_open = true;
    }

    // Open the file
    file = fopen( fileName, "r" );
    if( !file ) {
        printf( "ERROR: Unable to open file '%s'\n", fileName );
        return( ADI_FILE_NOT_FOUND );
    }

    // Determine the number of words in the file.
    numCommandWords = 0;
    while( fgets( line, 128, file ) != NULL ) {
        numCommandWords++;
    }

    commandData = malloc (sizeof (u16) * numCommandWords);

    index = 0;

    // Read file and fill writeData array
    rewind( file );
    while( fgets( line, 128, file ) != NULL ) {
        sscanf( line, "%04x", &data16 );
	commandData[index++] = data16;
    }

    fclose( file );

    index = 0;
    
    while( index < numCommandWords ) {
        if( state == CMD_LOAD_HEADER ) {
	    destAddress  = commandData[index++];
	    command      = commandData[index++];
	    attributes   = commandData[index++];
	    byteCount    = commandData[index++];
	    numDataWords = byteCount / 2;

	    if( command == CMD_GROUPED_DATA ) {
	        printf( "INFO: Start of Grouped Command ...\n" );
	        isGroupedCommand = 1;
	        state = CMD_GROUPED_DATA_WRITE;
	    }
	    else if( command == CMD_SEQ_RAM ) {
	        printf( "INFO: Loading useq_seq_ram ...\n" );
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_WAVE_RAM ) {
	        printf( "INFO: Loading useq_wave_ram ...\n" );
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_MAP_RAM ) {
	        printf( "INFO: Loading useq_map_ram ...\n" );
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_DATAPATH_RAM ) {
	        printf( "INFO: Loading datapath_ram ...\n" );
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_DUMP_ENGINE_RAM ) {
	        printf( "INFO: Loading de_ram ...\n" );
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_LPS1_RAM ) {
	        printf( "INFO: Loading lps1_ram ...\n" );
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_LPS2_RAM ) {
	        printf( "INFO: Loading lps2_ram ...\n" );
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_REGISTER_CFG ) {
	        if( (attributes & 1) == 0 ) {
	            state = CMD_LOAD_REG_READ;
	        }
		else {
	            state = CMD_LOAD_REG_WRITE;
		}
	    }
	    else if( command == CMD_1SP_IMAGE ) {
	        printf( "INFO: Loading 1SP image ...\n" );
	        state = CMD_LOAD_1SP_IMAGE;
	    }
	    else if( command == CMD_PUBLIC_KEY ) {
	        printf( "INFO: Loading public Key ...\n" );
	        state = CMD_LOAD_PUBLIC_KEY;
	    }
	    else if( command == CMD_SIGNATURE ) {
	        printf( "INFO: Loading signature ...\n" );
	        state = CMD_LOAD_SIGNATURE;
	    }
	    else if( command == CMD_OPERATING_MODE ) {
	        printf( "INFO: Setting operating mode ...\n" );

#ifdef ENABLE_DEBUG_LOW
	        printf( "INFO: CMD_OPERATING_MODE destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		        destAddress, byteCount, attributes );
#endif

	        rc = adi_send_command( command, destAddress, numDataWords, attributes );
                if( rc != ADI_NO_ERROR ) {
                    return( rc );
                }
                rc = adi_wait_for_s2h_not_valid( );
                if( rc != ADI_NO_ERROR ) {
                    return( rc );
                }

	        state = CMD_LOAD_HEADER;
	    }
	    else if( command == CMD_ERROR_LOG ) {
	        state = CMD_LOAD_ERROR_LOG;
	    }
	    else {
	        printf( "ERROR: Bad command = 0x%04x\n", command );
	        state = CMD_LOAD_UNKNOWN;
	    }
        }
        else if( state == CMD_GROUPED_DATA_WRITE ) {
#ifdef ENABLE_DEBUG_LOW
	    printf( "INFO: CMD_GROUPED_DATA destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		    destAddress, byteCount, attributes );
#endif

            rc = adi_send_command( command, destAddress, numDataWords, attributes );
            if( rc != ADI_NO_ERROR ) {
	        printf( "ERROR: Error returned from adi_send_command.\n" );
                return( rc );
            }
            rc = adi_wait_for_s2h_not_valid( );
            if( rc != ADI_NO_ERROR ) {
	        printf( "ERROR: Error returned from adi_wait_for_s2h_not_valid.\n" );
                return( rc );
            }

	    state = CMD_LOAD_HEADER;
        }
        else if( state == CMD_LOAD_REG_WRITE ) {
#ifdef ENABLE_DEBUG_LOW
	    printf( "INFO: CMD_LOAD_REG destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		    destAddress, byteCount, attributes );
#endif
	    
	    writeData = malloc (sizeof (u16) * numDataWords);

            for( j = 0; j < numDataWords; j++ ) {
	        writeData[j] = commandData[index++];
	    }
	    
            rc = adi_send_command( command, destAddress, numDataWords, attributes );
            if( rc != ADI_NO_ERROR ) {
                return( rc );
            }
            rc = adi_wait_for_s2h_not_valid( );
            if( rc != ADI_NO_ERROR ) {
                return( rc );
            }
            rc = adi_send_data( numDataWords, writeData );
            if( rc != ADI_NO_ERROR ) {
                return( rc );
            }
            rc = adi_wait_for_s2h_not_valid( );
            if( rc != ADI_NO_ERROR ) {
                return( rc );
            }

            free( writeData );
	    state = CMD_LOAD_HEADER;
        }
        else if( state == CMD_LOAD_REG_READ ) {
#ifdef ENABLE_DEBUG_LOW
	    printf( "INFO: CMD_LOAD_REG destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		    destAddress, byteCount, attributes );
#endif
	    
	    writeData = malloc (sizeof (u16) * numDataWords);
	    readData  = malloc (sizeof (u16) * numDataWords);

            for( j = 0; j < numDataWords; j++ ) {
	        writeData[j] = commandData[index++];
	    }

            rc = adi_send_command( command, destAddress, numDataWords, attributes );
            if( rc != ADI_NO_ERROR ) {
                printf( "ERROR: adi_load_command_file: Error returned from adi_send_command.\n" );
                return( rc );
            }
            rc = adi_wait_for_s2h_not_valid( );
            if( rc != ADI_NO_ERROR ) {
                printf( "ERROR: adi_load_command_file: Error returned from adi_wait_for_s2h_not_valid.\n" );
                return( rc );
            }
            rc = adi_send_read_register_list( writeData, numDataWords );
            if( rc != ADI_NO_ERROR ) {
                printf( "ERROR: adi_load_command_file: Error returned from adi_send_read_register_list.\n" );
                return( rc );
            }
            rc = adi_wait_for_s2h_not_valid( );
            if( rc != ADI_NO_ERROR ) {
                printf( "ERROR: adi_load_command_file: Error returned from adi_wait_for_s2h_not_valid.\n" );
                return( rc );
            }
            rc = adi_get_data( numDataWords, readData );
            if( rc != ADI_NO_ERROR ) {
                printf( "ERROR: adi_load_command_file: Error returned from adi_get_data.\n" );
                return( rc );
            }
            rc = adi_clear_h2s_valid( );
            if( rc != ADI_NO_ERROR ) {
                printf( "ERROR: adi_load_command_file: Error returned from adi_clear_h2s_valid.\n" );
                return( rc );
            }

	    for( j = 0; j < numDataWords; j++ ) {
	        if( writeData[j] != 0xffff ) {
	            printf( "INFO: address 0x%04x readData = 0x%04x\n", writeData[j], readData[j] );
		}
	    }

            free( readData );
	    state = CMD_LOAD_HEADER;
        }
        else if( state == CMD_LOAD_RAM_WRITE ) {
#ifdef ENABLE_DEBUG_LOW
	    printf( "INFO: CMD_LOAD_RAM destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		    destAddress, byteCount, attributes );
#endif

	    writeData = malloc (sizeof (u16) * numDataWords);

            for( j = 0; j < numDataWords; j++ ) {
	        writeData[j] = commandData[index++];
	    }
	    
            rc = adi_send_command( command, destAddress, numDataWords, attributes );
            if( rc != ADI_NO_ERROR ) {
                printf( "ERROR: adi_load_command_file: Error returned from .\n" );
                return( rc );
            }
            rc = adi_wait_for_s2h_not_valid( );
            if( rc != ADI_NO_ERROR ) {
                printf( "ERROR: adi_load_command_file: Error returned from .\n" );
                return( rc );
            }
            rc = adi_send_data( numDataWords, writeData );
            if( rc != ADI_NO_ERROR ) {
                printf( "ERROR: adi_load_command_file: Error returned from .\n" );
                return( rc );
            }
            rc = adi_wait_for_s2h_not_valid( );
            if( rc != ADI_NO_ERROR ) {
                printf( "ERROR: adi_load_command_file: Error returned from .\n" );
                return( rc );
            }

            free( writeData );
	    state = CMD_LOAD_HEADER;
        }
        else if( state == CMD_LOAD_1SP_IMAGE ) {
#ifdef ENABLE_DEBUG_LOW
	    printf( "INFO: CMD_1SP_IMAGE destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		    destAddress, byteCount, attributes );
#endif

	    writeData = malloc (sizeof (u16) * numDataWords);

            for( j = 0; j < numDataWords; j++ ) {
	        writeData[j] = commandData[index++];
	    }
	    
            rc = adi_send_command( command, destAddress, numDataWords, attributes );
            if( rc != ADI_NO_ERROR ) {
                printf( "ERROR: adi_load_command_file: Error returned from adi_send_command.\n" );
                return( rc );
            }
            rc = adi_wait_for_s2h_not_valid( );
            if( rc != ADI_NO_ERROR ) {
                printf( "ERROR: adi_load_command_file: Error returned from adi_wait_for_s2h_not_valid.\n" );
                return( rc );
            }
            rc = adi_send_data( numDataWords, writeData );
            if( rc != ADI_NO_ERROR ) {
                printf( "ERROR: adi_load_command_file: Error returned from adi_send_data.\n" );
                return( rc );
            }
            rc = adi_wait_for_s2h_not_valid( );
            if( rc != ADI_NO_ERROR ) {
                printf( "ERROR: adi_load_command_file: Error returned from adi_wait_for_s2h_not_valid.\n" );
                return( rc );
            }

            free( writeData );
	    state = CMD_LOAD_HEADER;
        }
        else if( state == CMD_LOAD_UNKNOWN ) {
#ifdef ENABLE_DEBUG_LOW
	    printf( "INFO: CMD_UNKNOWN destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		    destAddress, byteCount, attributes );
#endif
	    
            rc = adi_send_command( command, destAddress, numDataWords, attributes );
            if( rc != ADI_NO_ERROR ) {
                printf( "ERROR: adi_load_command_file: Error returned from adi_send_command.\n" );
                return( rc );
            }
            rc = adi_wait_for_s2h_not_valid( );
            if( rc != ADI_NO_ERROR ) {
                printf( "ERROR: adi_load_command_file: Error returned from adi_wait_for_s2h_not_valid.\n" );
                return( rc );
            }
	    rc = adi_check_for_h2s_error( );
            if( rc != ADI_NO_ERROR ) {
                printf( "ERROR: adi_load_command_file: Error returned from adi_check_for_h2s_error.\n" );
                return( rc );
            }
	    rc = adi_clear_h2s_error( );
            if( rc != ADI_NO_ERROR ) {
                printf( "ERROR: adi_load_command_file: Error returned from adi_clear_h2s_error.\n" );
                return( rc );
            }
	    
	    state = CMD_LOAD_HEADER;
        }
	else if( state == CMD_LOAD_SIGNATURE ) { 
#ifdef ENABLE_DEBUG_LOW
	    printf( "INFO: CMD_LOAD_SIGNATURE destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		    destAddress, byteCount, attributes );
#endif

	    writeData = malloc (sizeof (u16) * numDataWords);

            for( j = 0; j < numDataWords; j++ ) {
	        writeData[j] = commandData[index++];
	    }

            rc = adi_send_command( command, destAddress, numDataWords, attributes );
            if( rc != ADI_NO_ERROR ) {
	        printf( "ERROR: Error returned from adi_send_command\n" );
                return( rc );
            }
            rc = adi_wait_for_s2h_not_valid( );
            if( rc != ADI_NO_ERROR ) {
	        printf( "ERROR: Error returned from adi_wait_for_s2h_not_valid\n" );
                return( rc );
            }
            
	    // send the 64 byte signature payload
            rc = adi_send_data( numDataWords, writeData );
            if( rc != ADI_NO_ERROR ) {
	        printf( "ERROR: Error returned from adi_send_data\n" );
                return( rc );
            }

	    rc = adi_check_for_h2s_error();
	    if( rc != ADI_NO_ERROR ) {
	        printf( "ERROR: Error returned from adi_check_for_h2s_error\n" );
                return( rc );
            }

	    // wait for the hsp to acknowledge the signature is received
            rc = adi_wait_for_s2h_not_valid( );
            if( rc != ADI_NO_ERROR ) {
	        printf( "ERROR: Error returned from adi_wait_for_s2h_not_valid\n" );
                return( rc );
            }

            free( writeData );
	    state = CMD_LOAD_HEADER;
	}
        else if( state == CMD_LOAD_PUBLIC_KEY ) {
#ifdef ENABLE_DEBUG_LOW
	    printf( "INFO: CMD_LOAD_PUBLIC_KEY destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		    destAddress, byteCount, attributes );
#endif
	    writeData = malloc (sizeof (u16) * numDataWords);

            for( j = 0; j < numDataWords; j++ ) {
	        writeData[j] = commandData[index++];
	    }

            rc = adi_send_command( command, destAddress, numDataWords, attributes );
            if( rc != ADI_NO_ERROR ) {
	        printf( "ERROR: Error returned from adi_send_command\n" );
                return( rc );
            }
            rc = adi_wait_for_s2h_not_valid( );
            if( rc != ADI_NO_ERROR ) {
	        printf( "ERROR: Error returned from adi_wait_for_s2h_not_valid\n" );
                return( rc );
            }
            
	    // Send the public key payload
            rc = adi_send_data( numDataWords, writeData );
            if( rc != ADI_NO_ERROR ) {
	        printf( "ERROR: Error returned from adi_send_data\n" );
                return( rc );
            }

	    rc = adi_check_for_h2s_error();
	    if( rc != ADI_NO_ERROR ) {
	        printf( "ERROR: Error returned from adi_check_for_h2s_error\n" );
                return( rc );
            }

	    // Wait for the hsp to acknowledge that the public key was received
            rc = adi_wait_for_s2h_not_valid( );
            if( rc != ADI_NO_ERROR ) {
	        printf( "ERROR: Error returned from adi_wait_for_s2h_not_valid\n" );
                return( rc );
            }
	    
            free( writeData );
	    state = CMD_LOAD_HEADER;
        }
        else if( state == CMD_LOAD_ERROR_LOG ) {
#ifdef ENABLE_DEBUG_LOW
	    printf( "INFO: CMD_ERROR_LOG destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		    destAddress, byteCount, attributes );
#endif

            readData = malloc ( 32 );
	    
            rc = adi_send_command( command, destAddress, numDataWords, attributes );
            if( rc != ADI_NO_ERROR ) {
                return( rc );
            }
            rc = adi_wait_for_s2h_not_valid( );
            if( rc != ADI_NO_ERROR ) {
                return( rc );
            }
	    rc = adi_wait_for_h2s_valid();
            if( rc != ADI_NO_ERROR ) {
     	        printf( "ERROR: adi_wait_for_h2s_valid returned rc = %d.\n", rc );
                return( rc );
            }
            rc = adi_get_data( 16, readData );
            if( rc != ADI_NO_ERROR ) {
     		printf( "ERROR: adi_get_data returned rc = %d.\n", rc );
                return( rc );
            }
            rc = adi_clear_h2s_valid();
            if( rc != ADI_NO_ERROR ) {
     		printf( "ERROR: adi_clear_h2s_valid returned rc = %d.\n", rc );
                return( rc );
            }

	    for( j = 0; j < 16; j++ ) {
	        printf( "INFO: ERROR_LOG[%d] = 0x%04x\n", j, readData[j] );
	    }
	    
            free( readData );
	    state = CMD_LOAD_HEADER;
	}
        else {
     	    printf( "ERROR: Unexpected state = %d.\n", state );
	    state = CMD_LOAD_HEADER;
	}
    }    

    return( rc );

} // adi_load_command_file

//====================================================================================
// Load Register File
//====================================================================================
int adi_load_register_file( const char *fileName ) {
    int  rc = ADI_ERROR_CODE_MISSING;
    bool called_adi_spi_open = false;
    FILE *file;
    int  numRegisters;
    int  numDataWords;
    int  index;
    u16  byteCount;
    u16  command;
    u16  destAddress;
    u16  attributes;
    u16  *writeAddr;
    u16  *writeData;
    u32  address;
    u32  data32;
    char line[128];
    int  i;

    if( spi_handle <= 0 ) {
        rc = adi_spi_open( spi_bit_rate );
        if( rc != ADI_NO_ERROR ) {
            return( ADI_SPI_DRIVER_ERROR );
        }
        called_adi_spi_open = true;
    }

    // Open the file
    file = fopen( fileName, "r" );
    if( !file ) {
        printf( "ERROR: Unable to open file '%s'\n", fileName );
        return( ADI_FILE_NOT_FOUND );
    }

    // Determine the number of words in the file.
    numRegisters = 0;
    while( fgets( line, 128, file ) != NULL ) {
        if( startsWith( line, "//" ) ) {
	    // Comment line
	}
	else {
            numRegisters++;
	}
    }

    writeAddr = malloc (sizeof (u16) * numRegisters);
    writeData = malloc (sizeof (u16) * numRegisters);

    numDataWords = numRegisters * 2;

    // Read file and fill writeData array
    index = 0;
    rewind( file );
    while( fgets( line, 128, file ) != NULL ) {
        if( startsWith( line, "//" ) ) {
	    // Comment line
	}
        else if( startsWith( line, "0x" ) ) {
	    sscanf( line, "0x%04x 0x%04x", &address, &data32 );
	    writeAddr[index]   = (u16) address;
	    writeData[index++] = (u16) data32;
	}
	else {
	    sscanf( line, "%04x %04x", &address, &data32 );
	    writeAddr[index]   = (u16) address;
	    writeData[index++] = (u16) data32;
	}
    }

    fclose( file );

    for( i = 0; i < numRegisters; i++ ) {
        rc = adi_write_register( writeAddr[i], writeData[i] );
        if( rc != ADI_NO_ERROR ) {
            return( rc );
        }
    }

    
    /*
    rc = adi_send_command( CMD_REGISTER_CFG, 0, 2, WRITE_ATTR );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }

    rc = adi_wait_for_s2h_not_valid( );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }

    rc = adi_send_write_register_list( writeAddr, writeData, numDataWords );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }

    rc = adi_wait_for_s2h_not_valid( );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }
    */
    
    free( writeAddr );
    free( writeData );

    return( rc );
    
} // load_register_file

//====================================================================================
// Verify Command File using backdoor accesses
// - Read the file into an array.
// - Process command data in array.
//====================================================================================
int adi_verify_command_file( const char *fileName ) {
    int  rc = ADI_ERROR_CODE_MISSING;
    bool called_adi_spi_open = false;
    FILE *file;
    int  lineNum = 0;
    int  numCommandWords;
    int  numDataWords;
    u16  byteCount;
    int  index;
    u16  command;
    u16  destAddress;
    u16  readAddress;
    u16  attributes;
    u16  address;
    u16  data16;
    u16  *expectData;
    u16  *actualData;
    u16  *writeData;
    u16  *readData;
    u16  *commandData;
    char line[128];
    int  i, j;
    int  isGroupedCommand = 0;
    int  dataMiscompare = 0;
    adi_commandLoadStates_e state = CMD_LOAD_HEADER;

    ADI_USEQ_REGS_MAP1_USEQRAMRDSTADDR_t USEQ_RAMRDADDR;

    ADI_DATAPATH_IA_SELECT_t DATAPATH_IA_SELECT;
    ADI_DATAPATH_IA_ADDR_REG_t DATAPATH_IA_ADDR;

    ADI_DE_REGS_YODA_DE_IA_SELECT_t DE_IA_SELECT;
    ADI_DE_REGS_YODA_DE_IA_ADDR_REG_t DE_IA_ADDR_REG;

    ADI_LPS_REGS_YODA_LPSRAMRDCMD_t LPS1_LPSRAMRDCMD;
    ADI_LPS_REGS_YODA_LPSRAMADDR_t LPS1_LPSRAMADDR;
    ADI_LPS_REGS_YODA_LPSRAMRDCMD_t LPS2_LPSRAMRDCMD;
    ADI_LPS_REGS_YODA_LPSRAMADDR_t LPS2_LPSRAMADDR;

    ADI_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS_t HSP_BYPASS;
    
    if( spi_handle <= 0 ) {
        rc = adi_spi_open( spi_bit_rate );
        if( rc != ADI_NO_ERROR ) {
            return( ADI_SPI_DRIVER_ERROR );
        }
        called_adi_spi_open = true;
    }

    // Open the file
    file = fopen( fileName, "r" );
    if( !file ) {
        printf( "ERROR: Unable to open file '%s'\n", fileName );
        return( ADI_FILE_NOT_FOUND );
    }

    // Determine the number of words in the file.
    numCommandWords = 0;
    while( fgets( line, 128, file ) != NULL ) {
        numCommandWords++;
    }

    commandData = malloc (sizeof (u16) * numCommandWords);

    index = 0;

    // Read file and fill writeData array
    rewind( file );
    while( fgets( line, 128, file ) != NULL ) {
        sscanf( line, "%04x", &data16 );
	commandData[index++] = data16;
    }

    fclose( file );

    // Read the data back bypassing the HSP. Only valid for FPGA.
    HSP_BYPASS.HSP_BYPASS_EN = 1;
    rc = adi_spi_write_word( ADDR_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS, HSP_BYPASS.VALUE16 );
    if( rc != ADI_NO_ERROR ) {
        if( called_adi_spi_open ) {
            adi_spi_close( );
        }
        return( rc );
    }

    index = 0;

    for( i = 0; i < numCommandWords; i++ ) {
        if( index > numCommandWords ) {
	    break;
        }
        if( state == CMD_LOAD_HEADER ) {
	    destAddress  = commandData[index++];
	    command      = commandData[index++];
	    attributes   = commandData[index++];
	    byteCount    = commandData[index++];
	    numDataWords = byteCount / 2;

	    if( command == CMD_GROUPED_DATA ) {
	        isGroupedCommand = 1;
#ifdef ENABLE_DEBUG_LOW
	        printf( "INFO: CMD_GROUPED_DATA destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		        destAddress, byteCount, attributes );
#endif
	    }
	    else if( command == CMD_SEQ_RAM ) {
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_WAVE_RAM ) {
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_MAP_RAM ) {
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_DATAPATH_RAM ) {
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_DUMP_ENGINE_RAM ) {
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_LPS1_RAM ) {
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_LPS2_RAM ) {
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_REGISTER_CFG ) {
	        if( (attributes & 1) == 0 ) {
	            state = CMD_LOAD_REG_READ;
	        }
		else {
	            state = CMD_LOAD_REG_WRITE;
		}
	    }
	    else if( command == CMD_1SP_IMAGE ) {
	        state = CMD_LOAD_1SP_IMAGE;
	    }
	    else if( command == CMD_PUBLIC_KEY ) {
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_SIGNATURE ) {
	        state = CMD_LOAD_SIGNATURE;
	    }
	    else if( command == CMD_OPERATING_MODE ) {
	        state = CMD_LOAD_HEADER;
	    }
        }
        else if( state == CMD_LOAD_REG_READ ) {
#ifdef ENABLE_DEBUG_LOW
	    printf( "INFO: CMD_LOAD_REG destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		    destAddress, byteCount, attributes );
#endif
	    
	    writeData = malloc (sizeof (u16) * numDataWords);
	    readData  = malloc (sizeof (u16) * numDataWords);

            for( j = 0; j < numDataWords; j++ ) {
	        writeData[j] = commandData[index++];
	    }

	    for( j = 0; j < numDataWords; j++ ) {
	        adi_spi_read_word( writeData[j], &readData[j] );
	    }

	    for( j = 0; j < numDataWords; j++ ) {
	        if( writeData[j] != 0xffff ) {
	            printf( "INFO: address 0x%04x readData = 0x%04x\n", writeData[j], readData[j] );
		}
	    }

            free( readData );
	    state = CMD_LOAD_HEADER;
        }
        else if( state == CMD_LOAD_REG_WRITE ) {
#ifdef ENABLE_DEBUG_LOW
	    printf( "INFO: CMD_LOAD_REG destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		    destAddress, byteCount, attributes );
	    printf( "INFO: numDataWords = %d\n", numDataWords );
#endif
	    
	    expectData = malloc (sizeof (u16) * numDataWords);

            for( j = 0; j < numDataWords; j++ ) {
	        expectData[j] = commandData[index++];
	    }

            free( expectData );
	    state = CMD_LOAD_HEADER;
        }
        else if( state == CMD_LOAD_RAM_WRITE ) {
#ifdef ENABLE_DEBUG_LOW
	    printf( "INFO: CMD_SEQ_RAM destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
          	    destAddress, byteCount, attributes );
	    printf( "INFO: numDataWords = %d\n", numDataWords );
#endif
	    
            address = 0;
	    expectData = malloc (sizeof (u16) * numDataWords);
	    actualData = malloc (sizeof (u16) * numDataWords);

            for( j = 0; j < numDataWords; j++ ) {
	        expectData[j] = commandData[index++];
	    }

            if( command == CMD_SEQ_RAM ) {
                printf( "INFO: Reading useq_seq_ram ...\n" );

    	        USEQ_RAMRDADDR.VALUE16 = 0;
	        USEQ_RAMRDADDR.RD_RAM_SEL = 0;

    	        USEQ_RAMRDADDR.RD_ADDR = address;
	        adi_spi_write_word( ADDR_USEQ_REGS_USEQRAMRDSTADDR, USEQ_RAMRDADDR.VALUE16 );

		adi_spi_read_word_multiple( ADDR_USEQ_REGS_USEQRAMRDDATA, numDataWords, actualData );
            }
            else if( command == CMD_WAVE_RAM ) {
#ifdef ENABLE_DEBUG_LOW
	        printf( "INFO: CMD_WAVE_RAM destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		        destAddress, byteCount, attributes );
	        printf( "INFO: numDataWords = %d\n", numDataWords );
#endif
	    
                printf( "INFO: Reading useq_wave_ram ...\n" );

    	        USEQ_RAMRDADDR.VALUE16 = 0;
                USEQ_RAMRDADDR.RD_RAM_SEL = 1;

    	        USEQ_RAMRDADDR.RD_ADDR = address;
	        adi_spi_write_word( ADDR_USEQ_REGS_USEQRAMRDSTADDR, USEQ_RAMRDADDR.VALUE16 );

		adi_spi_read_word_multiple( ADDR_USEQ_REGS_USEQRAMRDDATA, numDataWords, actualData );
            }
            else if( command == CMD_MAP_RAM ) {
#ifdef ENABLE_DEBUG_LOW
	        printf( "INFO: CMD_SEQ_RAM destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		        destAddress, byteCount, attributes );
	        printf( "INFO: numDataWords = %d\n", numDataWords );
#endif
	    
                printf( "INFO: Reading useq_map_ram ...\n" );

    	        USEQ_RAMRDADDR.VALUE16 = 0;
	        USEQ_RAMRDADDR.RD_RAM_SEL = 2;

    	        USEQ_RAMRDADDR.RD_ADDR = address;
	        adi_spi_write_word( ADDR_USEQ_REGS_USEQRAMRDSTADDR, USEQ_RAMRDADDR.VALUE16 );

		adi_spi_read_word_multiple( ADDR_USEQ_REGS_USEQRAMRDDATA, numDataWords, actualData );
            }
            else if( command == CMD_DATAPATH_RAM ) {
#ifdef ENABLE_DEBUG_LOW
	        printf( "INFO: CMD_DATAPATH_RAM destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		        destAddress, byteCount, attributes );
	        printf( "INFO: numDataWords = %d\n", numDataWords );
#endif
	    
                printf( "INFO: Reading datapath_ram ...\n" );

	        DATAPATH_IA_SELECT.IA_ENA = 1;
                adi_spi_write_word( ADDR_DATAPATH_REGS_IA_SELECT, DATAPATH_IA_SELECT.VALUE16 );

	        DATAPATH_IA_ADDR.IA_START_ADDR = address;
                adi_spi_write_word( ADDR_DATAPATH_REGS_IA_ADDR_REG, DATAPATH_IA_ADDR.VALUE16 );
		
		adi_spi_read_word_multiple( ADDR_DATAPATH_REGS_IA_RDDATA_REG, numDataWords, actualData );
            }
            else if( command == CMD_DUMP_ENGINE_RAM ) {
#ifdef ENABLE_DEBUG_LOW
	        printf( "INFO: CMD_DUMP_ENGINE_RAM destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		        destAddress, byteCount, attributes );
	        printf( "INFO: numDataWords = %d\n", numDataWords );
#endif
	    
                printf( "INFO: Reading de_ram ...\n" );

	        DE_IA_SELECT.RAM = 1;
                adi_spi_write_word( ADDR_DE_REGS_DE_IA_SELECT, DE_IA_SELECT.VALUE16 );

	        DE_IA_ADDR_REG.RAM_ADDR = address;
                adi_spi_write_word( ADDR_DE_REGS_DE_IA_ADDR_REG, DE_IA_ADDR_REG.VALUE16 );

                adi_spi_read_word_multiple( ADDR_DE_REGS_DE_IA_RDDATA_REG, numDataWords, actualData );
            }
            else if( command == CMD_LPS1_RAM ) {
#ifdef ENABLE_DEBUG_LOW
	        printf( "INFO: CMD_LPS1_RAM destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		        destAddress, byteCount, attributes );
	        printf( "INFO: numDataWords = %d\n", numDataWords );
#endif
	    
                printf( "INFO: Reading lps1_ram ...\n" );

	        LPS1_LPSRAMRDCMD.LPS_RAM_READ_EN = 1;
                LPS1_LPSRAMRDCMD.LPS_RAM_READ_RDY = 0;
                adi_spi_write_word( ADDR_LPS1_REGS_LPSRAMRDCMD, LPS1_LPSRAMRDCMD.VALUE16 );
	        LPS1_LPSRAMADDR.LPS_RAM_ADDR = address;
                adi_spi_write_word( ADDR_LPS1_REGS_LPSRAMADDR, LPS1_LPSRAMADDR.VALUE16 );

                adi_spi_read_word_multiple( ADDR_LPS1_REGS_LPSRAMDATA, numDataWords, actualData );
            }
            else if( command == CMD_LPS2_RAM ) {
#ifdef ENABLE_DEBUG_LOW
	        printf( "INFO: CMD_LPS2_RAM destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		        destAddress, byteCount, attributes );
	        printf( "INFO: numDataWords = %d\n", numDataWords );
#endif
	    
                printf( "INFO: Reading lps2_ram ...\n" );
        	LPS2_LPSRAMRDCMD.LPS_RAM_READ_EN = 1;
        	LPS2_LPSRAMRDCMD.LPS_RAM_READ_RDY = 0;
                adi_spi_write_word( ADDR_LPS2_REGS_LPSRAMRDCMD, LPS2_LPSRAMRDCMD.VALUE16 );
	        LPS2_LPSRAMADDR.LPS_RAM_ADDR = address;
	
                adi_spi_write_word( ADDR_LPS2_REGS_LPSRAMADDR, LPS2_LPSRAMADDR.VALUE16 );

                adi_spi_read_word_multiple( ADDR_LPS2_REGS_LPSRAMDATA, numDataWords, actualData );
            }
            else {
                printf( "ERROR: command = 0x%04x\n", command );
            }

            if( (command == CMD_LPS1_RAM) || (command == CMD_LPS2_RAM) ) {
	        // Compare Data.
                for( i = 0; i < (numDataWords - 1); i++ ) {
                    if( actualData[i + 1] != expectData[i] ) {
	                printf( "ERROR: actualData[%d] = 0x%04x,  expectData[%d] = 0x%04x\n", i, actualData[i+1], i, expectData[i] );
                         dataMiscompare = 1;
                   }
                }
            }
            else {
	        // Compare Data.
                for( i = 0; i < numDataWords; i++ ) {
                    if( actualData[i] != expectData[i] ) {
	                printf( "ERROR: actualData[%d] = 0x%04x,  expectData[%d] = 0x%04x\n", i, actualData[i], i, expectData[i] );
                        dataMiscompare = 1;
                    }
                }
            }

            free( actualData );
            free( expectData );
	    state = CMD_LOAD_HEADER;
        }
        else if( state == CMD_LOAD_1SP_IMAGE ) {
#ifdef ENABLE_DEBUG_LOW
	    printf( "INFO: CMD_1SP_IMAGE destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		    destAddress, byteCount, attributes );
#endif

	    writeData = malloc (sizeof (u16) * numDataWords);

            for( j = 0; j < numDataWords; j++ ) {
	        writeData[j] = commandData[index++];
	    }
	    
            free( writeData );
	    state = CMD_LOAD_HEADER;
        }
	else if( state == CMD_LOAD_SIGNATURE ) { 
#ifdef ENABLE_DEBUG_LOW
	    printf( "INFO: CMD_LOAD_SIGNATURE destAddress = 0x%04x, byteCount = %d, attributes = 0x%04x\n",
		    destAddress, byteCount, attributes );
#endif

            // send signature
            writeData = malloc (sizeof (u16) * 64/2);
	    
            for( j = 0; j < 64/2; j++ ) {
	        writeData[j] = commandData[index++];
	    }

            free( writeData );
	    state = CMD_LOAD_HEADER;
	}
    }    

    HSP_BYPASS.HSP_BYPASS_EN = 0;
    rc = adi_spi_write_word( ADDR_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS, HSP_BYPASS.VALUE16 );
    if( rc != ADI_NO_ERROR ) {
        if( called_adi_spi_open ) {
            adi_spi_close( );
        }
        return( rc );
    }

    if( dataMiscompare == 1 ) {
        rc = ADI_MISCOMPARE_ERROR;
    }

    return( rc );

} // adi_verify_command_file

//====================================================================================
// Verify Command File using accesses through HSP
// - Read the file into an array.
// - Process command data in array.
//====================================================================================
int adi_verify_command_file_hsp( const char *fileName ) {
    int  rc = ADI_ERROR_CODE_MISSING;
    bool called_adi_spi_open = false;
    FILE *file;
    int  numCommandWords;
    int  numDataWords;
    u16  byteCount;
    int  commandIndex;
    int  dataIndex;
    u16  command;
    u16  destAddress;
    u16  readAddress;
    u16  attributes;
    u16  address;
    u16  data16;
    u16  *expectData;
    u16  *actualData;
    u16  *writeData;
    u16  *readData;
    u16  *commandData;
    char line[128];
    int  i, j, k, n;
    int  isGroupedCommand = 0;
    int  dataMiscompare = 0;
    adi_commandLoadStates_e state = CMD_LOAD_HEADER;

    ADI_USEQ_REGS_MAP1_USEQRAMRDSTADDR_t USEQ_RAMRDADDR;

    ADI_DATAPATH_IA_SELECT_t DATAPATH_IA_SELECT;
    ADI_DATAPATH_IA_ADDR_REG_t DATAPATH_IA_ADDR;

    ADI_DE_REGS_YODA_DE_IA_SELECT_t DE_IA_SELECT;
    ADI_DE_REGS_YODA_DE_IA_ADDR_REG_t DE_IA_ADDR_REG;

    ADI_LPS_REGS_YODA_LPSRAMRDCMD_t LPS1_LPSRAMRDCMD;
    ADI_LPS_REGS_YODA_LPSRAMADDR_t LPS1_LPSRAMADDR;
    ADI_LPS_REGS_YODA_LPSRAMRDCMD_t LPS2_LPSRAMRDCMD;
    ADI_LPS_REGS_YODA_LPSRAMADDR_t LPS2_LPSRAMADDR;

    ADI_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS_t HSP_BYPASS;
  
    if( spi_handle <= 0 ) {
        rc = adi_spi_open( spi_bit_rate );
        if( rc != ADI_NO_ERROR ) {
            return( ADI_SPI_DRIVER_ERROR );
        }
        called_adi_spi_open = true;
    }

    // Open the file
    file = fopen( fileName, "r" );
    if( !file ) {
        printf( "ERROR: Unable to open file '%s'\n", fileName );
        return( ADI_FILE_NOT_FOUND );
    }

    // Determine the number of words in the file.
    numCommandWords = 0;
    while( fgets( line, 128, file ) != NULL ) {
        numCommandWords++;
    }

    commandData = malloc (sizeof (u16) * numCommandWords);

    commandIndex = 0;

    // Read file and fill writeData array
    rewind( file );
    while( fgets( line, 128, file ) != NULL ) {
        sscanf( line, "%04x", &data16 );
	commandData[commandIndex++] = data16;
    }

    fclose( file );

    commandIndex = 0;

    for( i = 0; i < numCommandWords; i++ ) {
        if( commandIndex > numCommandWords ) {
	    break;
        }
        if( state == CMD_LOAD_HEADER ) {
	    destAddress  = commandData[commandIndex++];
	    command      = commandData[commandIndex++];
	    attributes   = commandData[commandIndex++];
	    byteCount    = commandData[commandIndex++];
	    numDataWords = byteCount / 2;

	    if( command == CMD_GROUPED_DATA ) {
	        isGroupedCommand = 1;
	    }
	    else if( command == CMD_SEQ_RAM ) {
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_WAVE_RAM ) {
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_MAP_RAM ) {
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_DATAPATH_RAM ) {
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_DUMP_ENGINE_RAM ) {
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_LPS1_RAM ) {
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_LPS2_RAM ) {
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_REGISTER_CFG ) {
	        if( (attributes & 1) == 0 ) {
	            state = CMD_LOAD_REG_READ;
	        }
		else {
	            state = CMD_LOAD_REG_WRITE;
		}
	    }
	    else if( command == CMD_1SP_IMAGE ) {
	        state = CMD_LOAD_1SP_IMAGE;
	    }
	    else if( command == CMD_PUBLIC_KEY ) {
	        state = CMD_LOAD_RAM_WRITE;
	    }
	    else if( command == CMD_SIGNATURE ) {
	        state = CMD_LOAD_SIGNATURE;
	    }
	    else if( command == CMD_OPERATING_MODE ) {
	        state = CMD_LOAD_HEADER;
	    }
        }
        else if( state == CMD_LOAD_REG_READ ) {	    
	    writeData = malloc (sizeof (u16) * numDataWords);
	    readData  = malloc (sizeof (u16) * numDataWords);

            for( j = 0; j < numDataWords; j++ ) {
	        writeData[j] = commandData[commandIndex++];
	    }

            rc = adi_send_command( command, 0, numDataWords, attributes );
            if( rc != ADI_NO_ERROR ) {
                return( rc );
            }
            rc = adi_wait_for_s2h_not_valid( );
            if( rc != ADI_NO_ERROR ) {
                return( rc );
            }
            rc = adi_send_read_register_list( writeData, numDataWords );
            if( rc != ADI_NO_ERROR ) {
                return( rc );
            }
            rc = adi_wait_for_s2h_not_valid( );
            if( rc != ADI_NO_ERROR ) {
                return( rc );
            }
            rc = adi_get_data( numDataWords, readData );
            if( rc != ADI_NO_ERROR ) {
                return( rc );
            }
            rc = adi_clear_h2s_valid( );
            if( rc != ADI_NO_ERROR ) {
                return( rc );
            }
	    
	    for( j = 0; j < numDataWords; j++ ) {
	        if( writeData[j] != 0xffff ) {
	            printf( "INFO: address 0x%04x readData = 0x%04x\n", writeData[j], readData[j] );
		}
	    }

            free( readData );
	    state = CMD_LOAD_HEADER;
        }
        else if( state == CMD_LOAD_REG_WRITE ) {
	    expectData = malloc (sizeof (u16) * numDataWords);

            for( j = 0; j < numDataWords; j++ ) {
	        expectData[j] = commandData[commandIndex++];
	    }

            free( expectData );
	    state = CMD_LOAD_HEADER;
        }
        else if( state == CMD_LOAD_RAM_WRITE ) {
            address = 0;
	    dataIndex = 0;
	    expectData = malloc (sizeof (u16) * numDataWords);
	    actualData = malloc (sizeof (u16) * numDataWords);
	    readData   = malloc (sizeof (u16) * 4);

            for( j = 0; j < numDataWords; j++ ) {
	        expectData[j] = commandData[commandIndex++];
	    }

            if( command == CMD_SEQ_RAM ) {
                printf( "INFO: Reading useq_seq_ram ...\n" );
    	        USEQ_RAMRDADDR.VALUE16 = 0;
	        USEQ_RAMRDADDR.RD_RAM_SEL = 0;

    	        USEQ_RAMRDADDR.RD_ADDR = address;
	        rc = adi_write_register( ADDR_USEQ_REGS_USEQRAMRDSTADDR, USEQ_RAMRDADDR.VALUE16 );
                if( rc != ADI_NO_ERROR ) {
		    printf( "ERROR: adi_write_register returned rc = %d.\n", rc );
                    return( rc );
                }

		readAddress = ADDR_USEQ_REGS_USEQRAMRDDATA;

                for( j = 0; j < numDataWords; j += 4 ) {
                    rc = adi_send_command( CMD_SEQ_RAM, readAddress, 4, NO_ATTR );
                    if( rc != ADI_NO_ERROR ) {
		        printf( "ERROR: adi_send_command returned rc = %d.\n", rc );
                        return( rc );
                    }
                    rc = adi_wait_for_s2h_not_valid();
                    if( rc != ADI_NO_ERROR ) {
     		        printf( "ERROR: adi_wait_for_s2h_not_valid returned rc = %d.\n", rc );
                        return( rc );
                    }
		    rc = adi_wait_for_h2s_valid();
                    if( rc != ADI_NO_ERROR ) {
     		        printf( "ERROR: adi_wait_for_h2s_valid returned rc = %d.\n", rc );
                        return( rc );
                    }
                    rc = adi_get_data( 4, readData );
                    if( rc != ADI_NO_ERROR ) {
     		        printf( "ERROR: adi_get_data returned rc = %d.\n", rc );
                        return( rc );
                    }
                    rc = adi_clear_h2s_valid();
                    for( k = 0; k < 4; k++ ) {
                        actualData[dataIndex++] = readData[k];
                    }
		}
            }
            else if( command == CMD_WAVE_RAM ) {
                printf( "INFO: Reading useq_wave_ram ...\n" );
    	        USEQ_RAMRDADDR.VALUE16 = 0;
                USEQ_RAMRDADDR.RD_RAM_SEL = 1;

    	        USEQ_RAMRDADDR.RD_ADDR = address;
	        adi_write_register( ADDR_USEQ_REGS_USEQRAMRDSTADDR, USEQ_RAMRDADDR.VALUE16 );

		readAddress = ADDR_USEQ_REGS_USEQRAMRDDATA;

                for( j = 0; j < numDataWords; j += 4 ) {
                    rc = adi_send_command( CMD_WAVE_RAM, readAddress, 4, NO_ATTR );
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    rc = adi_wait_for_s2h_not_valid();
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
		    rc = adi_wait_for_h2s_valid();
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    rc = adi_get_data( 4, readData );
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    rc = adi_clear_h2s_valid();
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    for( k = 0; k < 4; k++ ) {
                        actualData[dataIndex++] = readData[k];
                   }
		}
            }
            else if( command == CMD_MAP_RAM ) {
                printf( "INFO: Reading useq_map_ram ...\n" );
    	        USEQ_RAMRDADDR.VALUE16 = 0;
	        USEQ_RAMRDADDR.RD_RAM_SEL = 2;

    	        USEQ_RAMRDADDR.RD_ADDR = address;
	        adi_write_register( ADDR_USEQ_REGS_USEQRAMRDSTADDR, USEQ_RAMRDADDR.VALUE16 );

		readAddress = ADDR_USEQ_REGS_USEQRAMRDDATA;

                for( j = 0; j < numDataWords; j += 4 ) {
                    rc = adi_send_command( CMD_MAP_RAM, readAddress, 4, NO_ATTR );
                    if( rc != ADI_NO_ERROR ) {
		        printf( "ERROR: adi_send_command returned rc = %d.\n", rc );
                        return( rc );
                    }
                    rc = adi_wait_for_s2h_not_valid();
                    if( rc != ADI_NO_ERROR ) {
     		        printf( "ERROR: adi_wait_for_s2h_not_valid returned rc = %d.\n", rc );
                        return( rc );
                    }
		    rc = adi_wait_for_h2s_valid();
                    if( rc != ADI_NO_ERROR ) {
     		        printf( "ERROR: adi_wait_for_h2s_valid returned rc = %d.\n", rc );
                        return( rc );
                    }
                    rc = adi_get_data( 4, readData );
                    if( rc != ADI_NO_ERROR ) {
     		        printf( "ERROR: adi_get_data returned rc = %d.\n", rc );
                        return( rc );
                    }
                    rc = adi_clear_h2s_valid();
                    for( k = 0; k < 4; k++ ) {
                        actualData[dataIndex++] = readData[k];
                   }
		}
            }
            else if( command == CMD_DATAPATH_RAM ) {
                printf( "INFO: Reading datapath_ram ...\n" );
	        DATAPATH_IA_SELECT.IA_ENA = 1;
                adi_write_register( ADDR_DATAPATH_REGS_IA_SELECT, DATAPATH_IA_SELECT.VALUE16 );

	        DATAPATH_IA_ADDR.IA_START_ADDR = address;
                adi_write_register( ADDR_DATAPATH_REGS_IA_ADDR_REG, DATAPATH_IA_ADDR.VALUE16 );
		
 		readAddress = ADDR_DATAPATH_REGS_IA_RDDATA_REG;

                for( j = 0; j < numDataWords; j += 4 ) {
                    rc = adi_send_command( CMD_DATAPATH_RAM, readAddress, 4, NO_ATTR );
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    rc = adi_wait_for_s2h_not_valid();
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
		    rc = adi_wait_for_h2s_valid();
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    rc = adi_get_data( 4, readData );
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    rc = adi_clear_h2s_valid();
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    for( k = 0; k < 4; k++ ) {
                        actualData[dataIndex++] = readData[k];
                   }
		}
            }
            else if( command == CMD_DUMP_ENGINE_RAM ) {
                printf( "INFO: Reading de_ram ...\n" );
	        DE_IA_SELECT.RAM = 1;
                adi_write_register( ADDR_DE_REGS_DE_IA_SELECT, DE_IA_SELECT.VALUE16 );

	        DE_IA_ADDR_REG.RAM_ADDR = address;
                adi_write_register( ADDR_DE_REGS_DE_IA_ADDR_REG, DE_IA_ADDR_REG.VALUE16 );

 		readAddress = ADDR_DE_REGS_DE_IA_RDDATA_REG;

                for( j = 0; j < numDataWords; j += 4 ) {
                    rc = adi_send_command( CMD_DUMP_ENGINE_RAM, readAddress, 4, NO_ATTR );
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    rc = adi_wait_for_s2h_not_valid();
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
		    rc = adi_wait_for_h2s_valid();
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    rc = adi_get_data( 4, readData );
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    rc = adi_clear_h2s_valid();
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    for( k = 0; k < 4; k++ ) {
                        actualData[dataIndex++] = readData[k];
                   }
		}
            }
            else if( command == CMD_LPS1_RAM ) {
                printf( "INFO: Reading lps1_ram ...\n" );
	        LPS1_LPSRAMRDCMD.LPS_RAM_READ_EN = 1;
                LPS1_LPSRAMRDCMD.LPS_RAM_READ_RDY = 0;
                adi_write_register( ADDR_LPS1_REGS_LPSRAMRDCMD, LPS1_LPSRAMRDCMD.VALUE16 );
	        LPS1_LPSRAMADDR.LPS_RAM_ADDR = address;
                adi_write_register( ADDR_LPS1_REGS_LPSRAMADDR, LPS1_LPSRAMADDR.VALUE16 );

 		readAddress = ADDR_LPS1_REGS_LPSRAMDATA;

                for( j = 0; j < numDataWords; j += 4 ) {
                    rc = adi_send_command( CMD_LPS1_RAM, readAddress, 4, NO_ATTR );
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    rc = adi_wait_for_s2h_not_valid();
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
		    rc = adi_wait_for_h2s_valid();
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    rc = adi_get_data( 4, readData );
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    rc = adi_clear_h2s_valid();
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    for( k = 0; k < 4; k++ ) {
                        actualData[dataIndex++] = readData[k];
                   }
		}
            }
            else if( command == CMD_LPS2_RAM ) {
                printf( "INFO: Reading lps2_ram ...\n" );
        	LPS2_LPSRAMRDCMD.LPS_RAM_READ_EN = 1;
        	LPS2_LPSRAMRDCMD.LPS_RAM_READ_RDY = 0;
                adi_write_register( ADDR_LPS2_REGS_LPSRAMRDCMD, LPS2_LPSRAMRDCMD.VALUE16 );
	        LPS2_LPSRAMADDR.LPS_RAM_ADDR = address;
                adi_write_register( ADDR_LPS2_REGS_LPSRAMADDR, LPS2_LPSRAMADDR.VALUE16 );

 		readAddress = ADDR_LPS2_REGS_LPSRAMDATA;

                for( j = 0; j < numDataWords; j += 4 ) {
                    rc = adi_send_command( CMD_LPS2_RAM, readAddress, 4, NO_ATTR );
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    rc = adi_wait_for_s2h_not_valid();
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
 		    rc = adi_wait_for_h2s_valid();
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    rc = adi_get_data( 4, readData );
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    rc = adi_clear_h2s_valid();
                    if( rc != ADI_NO_ERROR ) {
                        return( rc );
                    }
                    for( k = 0; k < 4; k++ ) {
                        actualData[dataIndex++] = readData[k];
                   }
		}
            }
            else {
                printf( "ERROR: command = 0x%04x\n", command );
            }

            if( (command == CMD_LPS1_RAM) || (command == CMD_LPS2_RAM) ) {
	        // Compare Data.
                for( n = 0; n < (numDataWords - 1); n++ ) {
                    if( actualData[n + 1] != expectData[n] ) {
	                printf( "ERROR: actualData[%d] = 0x%04x,  expectData[%d] = 0x%04x\n", n, actualData[n+1], n, expectData[n] );
                        dataMiscompare = 1;
                    }
                }
            }
            else {
	        // Compare Data.
                for( n = 0; n < numDataWords; n++ ) {
                    if( actualData[n] != expectData[n] ) {
	                printf( "ERROR: actualData[%d] = 0x%04x,  expectData[%d] = 0x%04x\n", n, actualData[n], n, expectData[n] );
                        dataMiscompare = 1;
                    }
                }
            }

            free( actualData );
            free( expectData );
	    state = CMD_LOAD_HEADER;
        }
        else if( state == CMD_LOAD_1SP_IMAGE ) {
	    writeData = malloc (sizeof (u16) * numDataWords);

            for( j = 0; j < numDataWords; j++ ) {
	        writeData[j] = commandData[commandIndex++];
	    }
	    
            free( writeData );
	    state = CMD_LOAD_HEADER;
        }
	else if( state == CMD_LOAD_SIGNATURE ) { 
            writeData = malloc (sizeof (u16) * 64/2);
	    
            for( j = 0; j < 64/2; j++ ) {
	        writeData[j] = commandData[commandIndex++];
	    }

            free( writeData );
	    state = CMD_LOAD_HEADER;
	}
    }    

    if( dataMiscompare == 1 ) {
        rc = ADI_MISCOMPARE_ERROR;
    }
    
    return( rc );

} // adi_verify_command_file_hsp

//====================================================================================
// Unload Newton RAM
//====================================================================================
int adi_unload_newton_ram( adi_loadTargets_e unloadTarget, const char *fileName ) {
    int  rc = ADI_ERROR_CODE_MISSING;
    bool called_adi_spi_open = false;
    FILE *file;
    int  numRamWords;
    int  numDataWords;
    int  index;
    u16  address = 0;
    u64  data64;
    u16  *actualData;
    char line[128];
    int  i;

    ADI_USEQ_REGS_MAP1_USEQRAMRDSTADDR_t USEQ_RAMRDADDR;
    ADI_DATAPATH_IA_SELECT_t DATAPATH_IA_SELECT;
    ADI_DATAPATH_IA_ADDR_REG_t DATAPATH_IA_ADDR;

    ADI_DE_REGS_YODA_DE_IA_SELECT_t DE_IA_SELECT;
    ADI_DE_REGS_YODA_DE_IA_ADDR_REG_t DE_IA_ADDR_REG;

    ADI_LPS_REGS_YODA_LPSRAMRDCMD_t LPS1_LPSRAMRDCMD;
    ADI_LPS_REGS_YODA_LPSRAMADDR_t LPS1_LPSRAMADDR;
    ADI_LPS_REGS_YODA_LPSRAMRDCMD_t LPS2_LPSRAMRDCMD;
    ADI_LPS_REGS_YODA_LPSRAMADDR_t LPS2_LPSRAMADDR;
    
    ADI_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS_t HSP_BYPASS;

    if( spi_handle <= 0 ) {
        rc = adi_spi_open( spi_bit_rate );
        if( rc != ADI_NO_ERROR ) {
            return( ADI_SPI_DRIVER_ERROR );
        }
        called_adi_spi_open = true;
    }

    // Open the file
    file = fopen( fileName, "w" );
    if( !file ) {
        printf( "ERROR: Unable to open file '%s'\n", fileName );
        return( ADI_FILE_NOT_FOUND );
    }

    if( unloadTarget == USEQ_SEQ_RAM ) {
        numDataWords = USEQ_SEQ_RAM_DEPTH;
    }
    else if( unloadTarget == USEQ_MAP_RAM ) {
        numDataWords = USEQ_MAP_RAM_DEPTH;
    }
    else if( unloadTarget == USEQ_WAVE_RAM ) {
        numDataWords = USEQ_WAVE_RAM_DEPTH;
    }
    else if( unloadTarget == DATAPATH_RAM ) {
        numDataWords = DATAPATH_RAM_DEPTH;
    }
    else if( unloadTarget == DE_RAM ) {
        numDataWords = DE_RAM_DEPTH * 4;
    }
    else if( unloadTarget == LPS1_RAM ) {
        numDataWords = LPS1_RAM_DEPTH * 2;
    }
    else if( unloadTarget == LPS2_RAM ) {
        numDataWords = LPS2_RAM_DEPTH * 2;
    }

    actualData = malloc (sizeof (u16) * numDataWords);

    // Read the data back bypassing the HSP. Only valid for FPGA.
    HSP_BYPASS.HSP_BYPASS_EN = 1;
    rc = adi_spi_write_word( ADDR_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS, HSP_BYPASS.VALUE16 );
    if( rc != ADI_NO_ERROR ) {
        if( called_adi_spi_open ) {
            adi_spi_close( );
        }
        return( rc );
    }

    // Perform the data transfer.
    if( unloadTarget == USEQ_SEQ_RAM ) {
        printf( "INFO: Reading useq_seq_ram ..." );
	USEQ_RAMRDADDR.RD_RAM_SEL = 0;
	USEQ_RAMRDADDR.RD_ADDR = 0;

	adi_write_register( ADDR_USEQ_REGS_USEQRAMRDSTADDR, USEQ_RAMRDADDR.VALUE16 );

	address = ADDR_USEQ_REGS_USEQRAMRDDATA;

        adi_spi_read_word_multiple( address, numDataWords, actualData );
    }
    else if( unloadTarget == USEQ_WAVE_RAM ) {
        printf( "INFO: Reading useq_wave_ram ..." );
	USEQ_RAMRDADDR.RD_RAM_SEL = 1;
	USEQ_RAMRDADDR.RD_ADDR = 0;
	
	adi_write_register( ADDR_USEQ_REGS_USEQRAMRDSTADDR, USEQ_RAMRDADDR.VALUE16 );

	address = ADDR_USEQ_REGS_USEQRAMRDDATA;

        adi_spi_read_word_multiple( address, numDataWords, actualData );
    }
    else if( unloadTarget == USEQ_MAP_RAM ) {
        printf( "INFO: Reading useq_map_ram ..." );
	USEQ_RAMRDADDR.RD_RAM_SEL = 2;
	USEQ_RAMRDADDR.RD_ADDR = 0;
	
	adi_write_register( ADDR_USEQ_REGS_USEQRAMRDSTADDR, USEQ_RAMRDADDR.VALUE16 );

	address = ADDR_USEQ_REGS_USEQRAMRDDATA;

        adi_spi_read_word_multiple( address, numDataWords, actualData );
    }
    else if( unloadTarget == DATAPATH_RAM ) {
        printf( "INFO: Reading datapath_ram ..." );
	DATAPATH_IA_SELECT.IA_ENA = 1;
        adi_write_register( ADDR_DATAPATH_REGS_IA_SELECT, DATAPATH_IA_SELECT.VALUE16 );
	DATAPATH_IA_ADDR.IA_START_ADDR = 0;

        adi_write_register( ADDR_DATAPATH_REGS_IA_ADDR_REG, DATAPATH_IA_ADDR.VALUE16 );

	address = ADDR_DATAPATH_REGS_IA_RDDATA_REG;

        adi_spi_read_word_multiple( address, numDataWords, actualData );
    }
    else if( unloadTarget == DE_RAM ) {
        printf( "INFO: Reading de_ram ..." );
        DE_IA_SELECT.RAM = 1;
        adi_write_register( ADDR_DE_REGS_DE_IA_SELECT, DE_IA_SELECT.VALUE16 );
	DE_IA_ADDR_REG.RAM_ADDR = 0;
	
        adi_write_register( ADDR_DE_REGS_DE_IA_ADDR_REG, DE_IA_ADDR_REG.VALUE16 );

        address = ADDR_DE_REGS_DE_IA_RDDATA_REG;

        adi_spi_read_word_multiple( address, numDataWords, actualData );
    }
    else if( unloadTarget == LPS1_RAM ) {
        printf( "INFO: Reading lps1_ram ..." );
	LPS1_LPSRAMRDCMD.LPS_RAM_READ_EN = 0;
	LPS1_LPSRAMRDCMD.LPS_RAM_READ_RDY = 0;
        adi_write_register( ADDR_LPS1_REGS_LPSRAMRDCMD, LPS1_LPSRAMRDCMD.VALUE16 );
	LPS1_LPSRAMADDR.LPS_RAM_ADDR = 0;
	
        adi_write_register( ADDR_LPS1_REGS_LPSRAMADDR, LPS1_LPSRAMADDR.VALUE16 );

        address = ADDR_LPS1_REGS_LPSRAMDATA;

        adi_spi_read_word_multiple( address, numDataWords, actualData );
    }
    else if( unloadTarget == LPS2_RAM ) {
        printf( "INFO: Reading lps2_ram ..." );
	LPS1_LPSRAMRDCMD.LPS_RAM_READ_EN = 0;
	LPS1_LPSRAMRDCMD.LPS_RAM_READ_RDY = 0;
        adi_write_register( ADDR_LPS2_REGS_LPSRAMRDCMD, LPS2_LPSRAMRDCMD.VALUE16 );
	LPS1_LPSRAMADDR.LPS_RAM_ADDR = 0;
	
        adi_write_register( ADDR_LPS2_REGS_LPSRAMADDR, LPS2_LPSRAMADDR.VALUE16 );

        address = ADDR_LPS2_REGS_LPSRAMDATA;

        adi_spi_read_word_multiple( address, numDataWords, actualData );
    }
    else {
        printf( "ERROR: unloadTarget = %d", unloadTarget );
    }

    data64 = 0;

    // Save data to file.
    for( i = 0; i < numDataWords; i++ ) {
        if( unloadTarget == USEQ_SEQ_RAM ) {
	    data64 = data64 |  (u64) actualData[0];
            sprintf( line, "%04x\n", data64 );
            fwrite( line, HSP_ROM_WIDTH/4+1, 1, file );
	}
	else if( unloadTarget == USEQ_MAP_RAM ) {
	    data64 = data64 |  (u64) actualData[0];
            sprintf( line, "%04x\n", data64 );
            fwrite( line, HSP_ROM_WIDTH/4+1, 1, file );
	}
	else if( unloadTarget == USEQ_WAVE_RAM ) {
	    data64 = data64 |  (u64) actualData[0];
            sprintf( line, "%04x\n", data64 );
            fwrite( line, HSP_ROM_WIDTH/4+1, 1, file );
	}
	else if( unloadTarget == DATAPATH_RAM ) {
	    data64 = data64 |  (u64) actualData[0];
            sprintf( line, "%04x\n", data64 );
            fwrite( line, HSP_ROM_WIDTH/4+1, 1, file );
	}
	else if( unloadTarget == DE_RAM ) {
	    data64 = data64 |  (u64) actualData[0];
	    data64 = data64 | ((u64) actualData[1] << 16);
	    data64 = data64 | ((u64) actualData[2] << 32);
	    data64 = data64 | ((u64) actualData[3] << 48);
            sprintf( line, "%010llx\n", data64 );
            fwrite( line, HSP_ROM_WIDTH/4+1, 1, file );
	}
	else if( unloadTarget == LPS1_RAM ) {
	    data64 = data64 |  (u64) actualData[0];
	    data64 = data64 | ((u64) actualData[1] << 16);
            sprintf( line, "%08x\n", data64 );
            fwrite( line, HSP_ROM_WIDTH/4+1, 1, file );
	}
	else if( unloadTarget == LPS2_RAM ) {
	    data64 = data64 |  (u64) actualData[0];
	    data64 = data64 | ((u64) actualData[1] << 16);
            sprintf( line, "%08x\n", data64 );
            fwrite( line, HSP_ROM_WIDTH/4+1, 1, file );
	}
    }

    fclose( file );

    free( actualData );

    HSP_BYPASS.HSP_BYPASS_EN = 0;
    rc = adi_spi_write_word( ADDR_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS, HSP_BYPASS.VALUE16 );
    if( rc != ADI_NO_ERROR ) {
        if( called_adi_spi_open ) {
            adi_spi_close( );
        }
        return( rc );
    }

    return( rc );

} // adi_unload_newton_ram

//=========================================================================
// Issue Soft Reset
//=========================================================================
int adi_soft_reset( ) {
    int rc = ADI_ERROR_CODE_MISSING;
    bool called_adi_spi_open = false;

    if( spi_handle <= 0 ) {
        rc = adi_spi_open( spi_bit_rate );
        if( rc != ADI_NO_ERROR ) {
            return( ADI_SPI_DRIVER_ERROR );
        }
        called_adi_spi_open = true;
    }

    // FIXME: Does noewton have a soft reset function?

    return( rc );

} // adi_soft_reset

//=========================================================================
// Issue Soft Reset to HSP
//=========================================================================
int adi_reset_hsp( ) {
    int rc = ADI_ERROR_CODE_MISSING;
    bool called_adi_spi_open = false;

    if( spi_handle <= 0 ) {
        rc = adi_spi_open( spi_bit_rate );
        if( rc != ADI_NO_ERROR ) {
            return( ADI_SPI_DRIVER_ERROR );
        }
        called_adi_spi_open = true;
    }
    
    rc = adi_spi_write_word( ADDR_FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET, 1 );
    if( rc != ADI_NO_ERROR ) {
        if( called_adi_spi_open ) {
            adi_spi_close( );
        }
        return( rc );
    }

    rc = adi_spi_write_word( ADDR_FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET, 0 );
    if( rc != ADI_NO_ERROR ) {
        if( called_adi_spi_open ) {
            adi_spi_close( );
        }
        return( rc );
    }

    return( rc );

} // adi_reset_hsp

//=========================================================================
// Issue hard reset to newton
//=========================================================================
int adi_reset_newton( adi_pin_modes_e pin_mode ) {
    int rc = ADI_NO_ERROR;

    adi_spi_close( );

#ifdef WIRINGPI
    switch( pin_mode ) {
        case PIN_MODE_FUNCTIONAL:
            printf( "INFO: adi_reset_newton: Enterring Functional pinmux mode.\n" );
            pinMuxMode = PIN_MODE_FUNCTIONAL;
    	    pinMode( PIN_I2C_SDA, OUTPUT );
    	    pinMode( PIN_GPIO0,   OUTPUT );
    	    pinMode( PIN_GPIO5,   OUTPUT );
    	    pinMode( PIN_GPIO6,   OUTPUT );
    	    pinMode( PIN_GPIO10,  OUTPUT );
	    digitalWrite( PIN_I2C_SDA, PIN_LOW  ); // Pull low for SPI Mode
	    digitalWrite( PIN_GPIO0,   PIN_LOW  ); // Pull low for I2C slave ID of 0x3C
	    digitalWrite( PIN_GPIO5,   PIN_LOW  ); // Pull low for not DFT JTAG Mode
	    digitalWrite( PIN_GPIO6,   PIN_LOW  ); // Pull low for Functional Mode
	    digitalWrite( PIN_GPIO10,  PIN_LOW  ); // Pull low to disable parametric test mode

	    digitalWrite( PIN_RSTN, PIN_LOW );
    	    delay( 10 );
    	    digitalWrite( PIN_RSTN, PIN_HIGH );
    	    delay( 10 );

    	    // Revert I2C_SCL,I2C_SDA and GPIO to ALT4 functionality
    	    pinModeAlt( PIN_I2C_SCL, 4 );
    	    pinModeAlt( PIN_I2C_SDA, 4 );
    	    pinModeAlt( PIN_GPIO0,   4 );

    	    pinMode( PIN_GPIO2,  OUTPUT );    // GPIO2
    	    pinMode( PIN_GPIO3,  OUTPUT );    // GPIO3
    	    pinMode( PIN_GPIO4,  INPUT  );    // MBOX_OUT
    	    pinMode( PIN_GPIO5,  OUTPUT );    // FSYNC
    	    pinMode( PIN_GPIO6,  OUTPUT );    // LIGHT_EN
    	    pinMode( PIN_GPIO7,  OUTPUT );    // SPIM_SCLK
    	    pinMode( PIN_GPIO8,  OUTPUT );    // SPIM_MOSI
    	    pinMode( PIN_GPIO9,  OUTPUT );    // SPIM_SCS
    	    pinMode( PIN_GPIO10, INPUT  );    // SPIM_MISO
	    pullUpDnControl( PIN_GPIO6,  PUD_OFF );
	    pullUpDnControl( PIN_GPIO7,  PUD_OFF );
	    pullUpDnControl( PIN_GPIO8,  PUD_OFF );
	    pullUpDnControl( PIN_GPIO9,  PUD_OFF );
	    pullUpDnControl( PIN_GPIO10, PUD_OFF );
            break;

        case PIN_MODE_HSP_DEBUG:
            printf( "INFO: adi_reset_newton: Enterring HSP Debug pinmux mode.\n" );
            pinMuxMode = PIN_MODE_HSP_DEBUG;
    	    pinMode( PIN_I2C_SDA, OUTPUT );
    	    pinMode( PIN_GPIO0,   OUTPUT );
    	    pinMode( PIN_GPIO5,   OUTPUT );
    	    pinMode( PIN_GPIO6,   OUTPUT );
    	    pinMode( PIN_GPIO10,  OUTPUT );
	    digitalWrite( PIN_I2C_SDA, PIN_LOW  ); // Pull low for SPI Mode
	    digitalWrite( PIN_GPIO0,   PIN_LOW  ); // Pull low for I2C slave ID of 0x3C
	    digitalWrite( PIN_GPIO5,   PIN_LOW  ); // Pull low for not DFT JTAG Mode
	    digitalWrite( PIN_GPIO6,   PIN_HIGH ); // Pull high for HSP Debug Mode
	    digitalWrite( PIN_GPIO10,  PIN_LOW  ); // Pull low to disable parametric test mode

	    digitalWrite( PIN_RSTN, PIN_LOW );
    	    delay( 10 );
    	    digitalWrite( PIN_RSTN, PIN_HIGH );
    	    delay( 10 );

    	    // Revert I2C_SCL,I2C_SDA and GPIO to ALT4 functionality
    	    pinModeAlt( PIN_I2C_SCL, 4 );
    	    pinModeAlt( PIN_I2C_SDA, 4 );
    	    pinModeAlt( PIN_GPIO0,   4 );

    	    pinMode( PIN_GPIO4,  INPUT  );    // MBOX_OUT
    	    pinMode( PIN_GPIO5,  INPUT  );    // FSYNC
#ifdef OLIMEX
    	    pinMode( PIN_GPIO6,  INPUT );     // TRST_HSP
#else
    	    pinMode( PIN_GPIO6,  OUTPUT );    // TRST_HSP
#endif
    	    pinMode( PIN_GPIO7,  OUTPUT );    // TDI_HSP
    	    pinMode( PIN_GPIO8,  OUTPUT );    // TMS_HSP
    	    pinMode( PIN_GPIO9,  OUTPUT );    // TCK
    	    pinMode( PIN_GPIO10, INPUT  );    // TDO_HSP
	    pullUpDnControl( PIN_GPIO6,  PUD_UP );
	    pullUpDnControl( PIN_GPIO7,  PUD_UP );
	    pullUpDnControl( PIN_GPIO8,  PUD_UP );
	    pullUpDnControl( PIN_GPIO9,  PUD_DOWN );
	    pullUpDnControl( PIN_GPIO10, PUD_UP );
	    digitalWrite( PIN_GPIO6,  PIN_HIGH ); // TRST_HSP
	    digitalWrite( PIN_GPIO7,  PIN_LOW  ); // TDI_HSP
	    digitalWrite( PIN_GPIO8,  PIN_HIGH ); // TMS_HSP
	    digitalWrite( PIN_GPIO9,  PIN_LOW  ); // TCK
	    break;

        case PIN_MODE_DFT_JTAG:
            printf( "INFO: adi_reset_newton: Enterring DFT JTAG pinmux mode.\n" );
            pinMuxMode = PIN_MODE_DFT_JTAG;
    	    pinMode( PIN_I2C_SDA, OUTPUT );
    	    pinMode( PIN_GPIO0,   OUTPUT );
    	    pinMode( PIN_GPIO5,   OUTPUT );
    	    pinMode( PIN_GPIO6,   OUTPUT );
    	    pinMode( PIN_GPIO10,  OUTPUT );
	    digitalWrite( PIN_I2C_SDA, PIN_LOW  );  // Pull low for SPI Mode
	    digitalWrite( PIN_GPIO0,   PIN_LOW  );  // Pull low for I2C slave ID of 0x3C
	    digitalWrite( PIN_GPIO5,   PIN_HIGH );  // Pull low for DFT JTAG Mode
	    digitalWrite( PIN_GPIO6,   PIN_LOW  );  // Pull low for not HSP Debug Mode
	    digitalWrite( PIN_GPIO10,  PIN_LOW  );  // Pull low to disable parametric test mode

	    digitalWrite( PIN_RSTN, PIN_LOW );
    	    delay( 10 );
    	    digitalWrite( PIN_RSTN, PIN_HIGH );
    	    delay( 10 );

    	    pinMode( PIN_I2C_SCL, OUTPUT );    // TCK
    	    pinMode( PIN_I2C_SDA, OUTPUT );    // TDI_DFT
    	    pinMode( PIN_GPIO0,   OUTPUT );    // TRST_DFT
    	    pinMode( PIN_GPIO1,   INPUT  );    // 
    	    pinMode( PIN_GPIO2,   INPUT  );    // 
    	    pinMode( PIN_GPIO3,   INPUT  );    // 
    	    pinMode( PIN_GPIO4,   INPUT  );    // MBOX_OUT
    	    pinMode( PIN_GPIO5,   OUTPUT );    // TMS_DFT
    	    pinMode( PIN_GPIO6,   OUTPUT );    // TRST_HSP
    	    pinMode( PIN_GPIO7,   INPUT  );    // GPIO7
    	    pinMode( PIN_GPIO8,   INPUT  );    // GPIO8
    	    pinMode( PIN_GPIO9,   INPUT  );    // GPIO9
    	    pinMode( PIN_GPIO10,  INPUT  );    // GPIO10
            break;

        case PIN_MODE_PROD_SCAN:
            printf( "ERROR: adi_reset_newton: Pin mode PIN_MODE_PROD_SCAN pinmux is not supported.\n" );
            return( ADI_UNEXPECTED_PIN_MODE );
            break;

        case PIN_MODE_FIELD_RETURN_SCAN:
            return( ADI_UNEXPECTED_PIN_MODE );
            break;

        case PIN_MODE_KEEP_CURRENT_MODE:
            printf( "INFO: adi_reset_newton: Keeping current pinmux mode..\n" );
            break;

	default:
            printf( "ERROR: adi_reset_newton: Unknown pinmux  mode.\n" );
            return( ADI_UNEXPECTED_PIN_MODE );
            break;
    }
 
#endif
    
    rc = adi_spi_open( spi_bit_rate );
    if( rc != ADI_NO_ERROR ) {
        return( ADI_SPI_DRIVER_ERROR );
    }

    return( rc );

} // adi_reset_newton

//====================================================================================
// Backdoor load newton ram
// - Read the file into an array.
// - Process command data in array.
//====================================================================================
int adi_test_useq_ram ( ) {
    int  rc = ADI_ERROR_CODE_MISSING;
    bool called_adi_spi_open = false;
    FILE *file;
    u16  byteCount;
    u16  destAddress;
    u16  attributes;
    u16  address = 0;
    u16  data16;
    u16  *expectData;
    u16  *actualData;
    u16  *recData;
    int  i, j;
    int mem_size;

    ADI_USEQ_REGS_MAP1_USEQRAMRDSTADDR_t USEQ_RAMRDADDR;
    ADI_USEQ_REGS_MAP1_USEQRAMLOADADDR_t USEQ_RAMLOADADDR;


    ADI_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS_t HSP_BYPASS;

    mem_size = 128;
    actualData = malloc (mem_size*2);
    expectData = malloc (mem_size*2);
    recData = malloc (mem_size*2);


    // Enable HSP Bypass path
    HSP_BYPASS.HSP_BYPASS_EN = 1;
    rc = adi_spi_write_word( ADDR_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS, HSP_BYPASS.VALUE16 );
    if( rc != ADI_NO_ERROR ) {
        if( called_adi_spi_open ) {
            adi_spi_close( );
        }
        return( rc );
    }


    for( i = 0; i < mem_size; i++ ) {
        actualData[i] = i;
	expectData[i] = actualData[i];
	recData[i] = 0xffff;
    	printf("teset_useq_ram: actualData[%0d] = 0x%04x expectData[%0d] = 0x%04x recData[%0d] = 0x%04x\n",
	       i,actualData[i],i,expectData[i],i,recData[i]); 
    }
   
    
    printf( "INFO: Loading useq_map_ram ...\n" );
    USEQ_RAMLOADADDR.VALUE16 = 0;
    USEQ_RAMLOADADDR.LD_RAM_SEL = 2;
    USEQ_RAMLOADADDR.LD_ADDR = 0;
    adi_spi_write_word( ADDR_USEQ_REGS_USEQRAMLOADADDR, USEQ_RAMLOADADDR.VALUE16 );

    adi_spi_write_word_multiple( ADDR_USEQ_REGS_USEQRAMLOADDATA, mem_size, actualData );
    
    printf( "INFO: Reading useq_map_ram ...\n" );
    USEQ_RAMRDADDR.VALUE16 = 0;
    USEQ_RAMRDADDR.RD_RAM_SEL = 2;
    USEQ_RAMRDADDR.RD_ADDR = 0;
    adi_spi_write_word( ADDR_USEQ_REGS_USEQRAMRDSTADDR, USEQ_RAMRDADDR.VALUE16 );

    adi_spi_read_word_multiple( ADDR_USEQ_REGS_USEQRAMRDDATA, mem_size, recData );

    i = 0;
    for( i = 0; i < mem_size; i++) {
        printf("expectData[%0d] = 0x%04x recData[%0d] = 0x%04x\n",i,expectData[i],i,recData[i]);
    }

    // Disabling HSP bypass path
    HSP_BYPASS.HSP_BYPASS_EN = 0;
    rc = adi_spi_write_word( ADDR_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS, HSP_BYPASS.VALUE16 );
    if( rc != ADI_NO_ERROR ) {
        if( called_adi_spi_open ) {
            adi_spi_close( );
        }
        return( rc );
    }

    return( rc );

} // adi_test_useq_ram 

//=========================================================================
// Configure the newton control program
//=========================================================================
int adi_newton_config( int bitRateOverride ) {
    int rc = ADI_NO_ERROR;
    int foundBitRate = 0;
    int bitRate = spi_bit_rate;
    int i;

    if( bitRateOverride != 0 ) {
        bitRate = bitRateOverride;
    }

    for( i = 0; i < BIT_RATE_COUNT; i++ ) {
        if( bitRate == bitRate_map[i] ) {
	    foundBitRate = 1;
	}
    }

    if( !foundBitRate ) {
        printf( "ERROR: SPI bit rate error.\n" );
        return( ADI_SPI_BIT_RATE_ERROR );
    }
    else {
        spi_bit_rate = bitRate;
    }

#ifdef WIRINGPI
    if( wiringPiSetup() == -1 ) {
        printf( "ERROR: Failed to setup Wiring Pi!\n" );
	return( ADI_WIRINGPI_ERROR );	
    }

    pinMode( PIN_RSTN, OUTPUT );
#endif
    
    return( ADI_NO_ERROR );

} // adi_newton_config

//=========================================================================
// Setup the SPI Interface
//=========================================================================
int adi_spi_open( int bitRate ) {
    int rc     = ADI_ERROR_CODE_MISSING;
    int port   = 0;
    int myMode = 3; // CPOL = 1, CPHA = 1
    int bits   = 8;
    int speed  = bitRate * 1000;

#ifdef BEAGLE_BONE
    char *linux_devname = "/dev/spidev1.0";
#else
    char *linux_devname = "/dev/spidev0.0";
#endif

    port = open( linux_devname, O_RDWR );

    if( port < 0 ) {
        return( ADI_SPI_DRIVER_ERROR );
    }

    rc = ioctl( port, SPI_IOC_WR_MODE, &myMode );
    if( rc != ADI_NO_ERROR ) {
        return( ADI_SPI_DRIVER_ERROR );
    }
    rc = ioctl( port, SPI_IOC_WR_BITS_PER_WORD, &bits );
    if( rc != ADI_NO_ERROR ) {
        return( ADI_SPI_DRIVER_ERROR );
    }
    rc = ioctl( port, SPI_IOC_WR_MAX_SPEED_HZ, &speed );
    if( rc != ADI_NO_ERROR ) {
        return( ADI_SPI_DRIVER_ERROR );
    }

    spi_handle = port;

    return( ADI_NO_ERROR );

} // adi_spi_open

//=========================================================================
// Setup the SPI Interface
//=========================================================================
int adi_spi_close( ) {
    int rc = ADI_ERROR_CODE_MISSING;

    rc = close( spi_handle );
    if( rc != ADI_NO_ERROR ) {
        rc = ADI_SPI_DRIVER_ERROR;
    }

    spi_handle = -1;

    return( ADI_NO_ERROR );

} // adi_spi_close

#ifndef PYTHON
// These are here to satisfy log calls in str_parms.c
void adi_logi( char* LOG_INFO, char* LOG_TAG, char* LOG_ARGS ) {}
void adi_logv( char* LOG_INFO, char* LOG_FUNC ) {}
#endif

//====================================================================================
// Get ADI error message
//====================================================================================
char* adi_error_msg( int returnCode ) {
    char* msg = (char*) adi_errorMessages[returnCode-ERROR_CODE_BASE];
    return( msg );    
}

//====================================================================================
// Configure Newton GPIO
//====================================================================================
int  adi_configure_newton_gpio( int gpio, int direction ) {
    int rc = ADI_NO_ERROR;

#ifdef WIRINGPI

    switch( pinMuxMode ) {
        case PIN_MODE_FUNCTIONAL:
	    if( gpio == 0 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO0 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 1 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO1 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 2 ) {
	        if( direction == 1 ) {
    	            pinMode( PIN_GPIO2, OUTPUT );
		}
		else {
    	            pinMode( PIN_GPIO2, INPUT );
		}
	    }
	    else if( gpio == 3 ) {
	        if( direction == 1 ) {
    	            pinMode( PIN_GPIO3, OUTPUT );
		}
		else {
    	            pinMode( PIN_GPIO3, INPUT );
		}
	    }
	    else if( gpio == 4 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO4 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 5 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO5 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 6 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO6 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 7 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO7 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 8 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO8 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 9 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO9 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 10 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO10 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
            break;

        case PIN_MODE_HSP_DEBUG:
	    if( gpio == 0 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO0 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 1 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO1 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 2 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO2 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 3 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO3 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 4 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO4 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 5 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO5 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 6 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO6 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 7 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO7 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 8 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO8 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 9 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO9 is not allowed inHSP Debug  Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 10 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO10 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    break;

        case PIN_MODE_DFT_JTAG:
	    if( gpio == 0 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO0 is not allowed in DFT JTAG Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 1 ) {
	        if( direction == 1 ) {
    	            pinMode( PIN_GPIO1, OUTPUT );
		}
		else {
    	            pinMode( PIN_GPIO1, INPUT );
		}
	    }
	    else if( gpio == 2 ) {
	        if( direction == 1 ) {
    	            pinMode( PIN_GPIO2, OUTPUT );
		}
		else {
    	            pinMode( PIN_GPIO2, INPUT );
		}
	    }
	    else if( gpio == 3 ) {
	        if( direction == 1 ) {
    	            pinMode( PIN_GPIO3, OUTPUT );
		}
		else {
    	            pinMode( PIN_GPIO3, INPUT );
		}
	    }
	    else if( gpio == 4 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO4 is not allowed in  DFT JTAG Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 5 ) {
                printf( "ERROR: adi_configure_newton_gpio: configuring GPIO5 is not allowed in  DFT JTAG Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 6 ) {
	        if( direction == 1 ) {
    	            pinMode( PIN_GPIO6, OUTPUT );
		}
		else {
    	            pinMode( PIN_GPIO6, INPUT );
		}
	    }
	    else if( gpio == 7 ) {
	        if( direction == 1 ) {
    	            pinMode( PIN_GPIO7, OUTPUT );
		}
		else {
    	            pinMode( PIN_GPIO7, INPUT );
		}
	    }
	    else if( gpio == 8 ) {
	        if( direction == 1 ) {
    	            pinMode( PIN_GPIO8, OUTPUT );
		}
		else {
    	            pinMode( PIN_GPIO8, INPUT );
		}
	    }
	    else if( gpio == 9 ) {
	        if( direction == 1 ) {
    	            pinMode( PIN_GPIO9, OUTPUT );
		}
		else {
    	            pinMode( PIN_GPIO9, INPUT );
		}
	    }
	    else if( gpio == 10 ) {
	        if( direction == 1 ) {
    	            pinMode( PIN_GPIO10, OUTPUT );
		}
		else {
    	            pinMode( PIN_GPIO10, INPUT );
		}
	    }
	    break;

        case PIN_MODE_PROD_SCAN:
            printf( "ERROR: adi_configure_newton_gpio: Pin mode PIN_MODE_PROD_SCAN pinmux is not supported.\n" );
            return( ADI_UNEXPECTED_PIN_MODE );
            break;

        case PIN_MODE_FIELD_RETURN_SCAN:
            printf( "ERROR: adi_configure_newton_gpio: Pin mode PIN_MODE_FIELD_RETURN_SCAN pinmux is not supported.\n" );
            return( ADI_UNEXPECTED_PIN_MODE );
            break;

	default:
            printf( "ERROR: adi_configure_newton_gpio: Unknown pinmux  mode.\n" );
            return( ADI_UNEXPECTED_PIN_MODE );
            break;
    }
#endif
    
    return( rc );

} // adi_configure_newton_gpio

//====================================================================================
// Set Newton GPIO
//====================================================================================
int  adi_set_newton_gpio( int gpio, int value ) {
    int rc = ADI_NO_ERROR;

#ifdef WIRINGPI

    switch( pinMuxMode ) {
        case PIN_MODE_FUNCTIONAL:
	    if( gpio == 0 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO0 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 1 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO1 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 2 ) {
    	        pinMode( PIN_GPIO2, OUTPUT );
	        if( value == 1 ) {
	            digitalWrite( PIN_GPIO2, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO2, PIN_LOW );
		}
	    }
	    else if( gpio == 3 ) {
    	        pinMode( PIN_GPIO3, OUTPUT );
	        if( value == 1 ) {
	            digitalWrite( PIN_GPIO3, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO3, PIN_LOW );
		}
	    }
	    else if( gpio == 4 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO4 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 5 ) {
    	        pinMode( PIN_GPIO5, OUTPUT );
	        if( value == 1 ) {
	            digitalWrite( PIN_GPIO5, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO5, PIN_LOW );
		}
	    }
	    else if( gpio == 6 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO6 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 7 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO7 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 8 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO8 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 9 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO9 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 10 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO10 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
            break;

        case PIN_MODE_HSP_DEBUG:
            printf( "INFO: adi_reset_newton: Enterring HSP Debug pinmux mode.\n" );
	    if( gpio == 0 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO0 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 1 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO1 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 2 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO2 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 3 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO3 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 4 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO4 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 5 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO5 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 6 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO6 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 7 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO7 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 8 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO8 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 9 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO9 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 10 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO10 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    break;

        case PIN_MODE_DFT_JTAG:
	    if( gpio == 0 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO0 is not allowed in DFT JTAG Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 1 ) {
    	        pinMode( PIN_GPIO1, OUTPUT );
	        if( value == 1 ) {
	            digitalWrite( PIN_GPIO1, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO1, PIN_LOW );
		}
	    }
	    else if( gpio == 2 ) {
    	        pinMode( PIN_GPIO2, OUTPUT );
	        if( value == 1 ) {
	            digitalWrite( PIN_GPIO2, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO2, PIN_LOW );
		}
	    }
	    else if( gpio == 3 ) {
    	        pinMode( PIN_GPIO3, OUTPUT );
	        if( value == 1 ) {
	            digitalWrite( PIN_GPIO3, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO3, PIN_LOW );
		}
	    }
	    else if( gpio == 4 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO4 is not allowed in DFT JTAG Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 5 ) {
                printf( "ERROR: adi_set_newton_gpio: setting GPIO5 is not allowed in DFT JTAG Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 6 ) {
    	        pinMode( PIN_GPIO6, OUTPUT );
	        if( value == 1 ) {
	            digitalWrite( PIN_GPIO6, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO6, PIN_LOW );
		}
	    }
	    else if( gpio == 7 ) {
    	        pinMode( PIN_GPIO7, OUTPUT );
	        if( value == 1 ) {
	            digitalWrite( PIN_GPIO7, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO7, PIN_LOW );
		}
	    }
	    else if( gpio == 8 ) {
    	        pinMode( PIN_GPIO8, OUTPUT );
	        if( value == 1 ) {
	            digitalWrite( PIN_GPIO8, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO8, PIN_LOW );
		}
	    }
	    else if( gpio == 9 ) {
    	        pinMode( PIN_GPIO9, OUTPUT );
	        if( value == 1 ) {
	            digitalWrite( PIN_GPIO9, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO9, PIN_LOW );
		}
	    }
	    else if( gpio == 10 ) {
    	        pinMode( PIN_GPIO10, OUTPUT );
	        if( value == 1 ) {
	            digitalWrite( PIN_GPIO10, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO10, PIN_LOW );
		}
	    }
	    break;

        case PIN_MODE_PROD_SCAN:
            printf( "ERROR: adi_set_newton_gpio: Pin mode PIN_MODE_PROD_SCAN pinmux is not supported.\n" );
            return( ADI_UNEXPECTED_PIN_MODE );
            break;

        case PIN_MODE_FIELD_RETURN_SCAN:
            printf( "ERROR: adi_set_newton_gpio: Pin mode PIN_MODE_FIELD_RETURN_SCAN pinmux is not supported.\n" );
            return( ADI_UNEXPECTED_PIN_MODE );
            break;

	default:
            printf( "ERROR: adi_set_newton_gpio: Unknown pinmux mode.\n" );
            return( ADI_UNEXPECTED_PIN_MODE );
            break;
    }
#endif
    
    return( rc );

} // adi_set_newton_gpio

//====================================================================================
// Toggle Newton GPIO
//====================================================================================
int  adi_toggle_newton_gpio( int gpio ) {
    int rc = ADI_NO_ERROR;
    int value;

#ifdef WIRINGPI

    switch( pinMuxMode ) {
        case PIN_MODE_FUNCTIONAL:
	    if( gpio == 0 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO0 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 1 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO1 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 2 ) {
    	        pinMode( PIN_GPIO2, OUTPUT );
                value = digitalRead( PIN_GPIO2 );
	        if( value == 0 ) {
	            digitalWrite( PIN_GPIO2, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO2, PIN_LOW );
		}
	    }
	    else if( gpio == 3 ) {
    	        pinMode( PIN_GPIO3, OUTPUT );
                value = digitalRead( PIN_GPIO3 );
	        if( value == 0 ) {
	            digitalWrite( PIN_GPIO3, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO3, PIN_LOW );
		}
	    }
	    else if( gpio == 4 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO4 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 5 ) {
    	        pinMode( PIN_GPIO5, OUTPUT );
                value = digitalRead( PIN_GPIO5 );
	        if( value == 0 ) {
	            digitalWrite( PIN_GPIO5, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO5, PIN_LOW );
		}
	    }
	    else if( gpio == 6 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO6 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 7 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO7 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 8 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO8 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 9 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO9 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 10 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO10 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
            break;

        case PIN_MODE_HSP_DEBUG:
            printf( "INFO: adi_reset_newton: Enterring HSP Debug pinmux mode.\n" );
	    if( gpio == 0 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO0 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 1 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO1 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 2 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO2 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 3 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO3 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 4 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO4 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 5 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO5 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 6 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO6 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 7 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO7 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 8 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO8 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 9 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO9 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 10 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO10 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    break;

        case PIN_MODE_DFT_JTAG:
	    if( gpio == 0 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO0 is not allowed in DFT JTAG Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 1 ) {
    	        pinMode( PIN_GPIO1, OUTPUT );
                value = digitalRead( PIN_GPIO1 );
	        if( value == 0 ) {
	            digitalWrite( PIN_GPIO1, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO1, PIN_LOW );
		}
	    }
	    else if( gpio == 2 ) {
    	        pinMode( PIN_GPIO2, OUTPUT );
                value = digitalRead( PIN_GPIO2 );
	        if( value == 0 ) {
	            digitalWrite( PIN_GPIO2, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO2, PIN_LOW );
		}
	    }
	    else if( gpio == 3 ) {
    	        pinMode( PIN_GPIO3, OUTPUT );
                value = digitalRead( PIN_GPIO3 );
	        if( value == 0 ) {
	            digitalWrite( PIN_GPIO3, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO3, PIN_LOW );
		}
	    }
	    else if( gpio == 4 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO4 is not allowed in DFT JTAG Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 5 ) {
                printf( "ERROR: adi_toggle_newton_gpio: setting GPIO5 is not allowed in DFT JTAG Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 6 ) {
    	        pinMode( PIN_GPIO6, OUTPUT );
                value = digitalRead( PIN_GPIO6 );
	        if( value == 0 ) {
	            digitalWrite( PIN_GPIO6, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO6, PIN_LOW );
		}
	    }
	    else if( gpio == 7 ) {
    	        pinMode( PIN_GPIO7, OUTPUT );
                value = digitalRead( PIN_GPIO7 );
	        if( value == 0 ) {
	            digitalWrite( PIN_GPIO7, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO7, PIN_LOW );
		}
	    }
	    else if( gpio == 8 ) {
    	        pinMode( PIN_GPIO8, OUTPUT );
                value = digitalRead( PIN_GPIO8 );
	        if( value == 0 ) {
	            digitalWrite( PIN_GPIO8, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO8, PIN_LOW );
		}
	    }
	    else if( gpio == 9 ) {
    	        pinMode( PIN_GPIO9, OUTPUT );
                value = digitalRead( PIN_GPIO9 );
	        if( value == 0 ) {
	            digitalWrite( PIN_GPIO9, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO9, PIN_LOW );
		}
	    }
	    else if( gpio == 10 ) {
    	        pinMode( PIN_GPIO10, OUTPUT );
                value = digitalRead( PIN_GPIO10 );
	        if( value == 0 ) {
	            digitalWrite( PIN_GPIO10, PIN_HIGH );
		}
		else {
	            digitalWrite( PIN_GPIO10, PIN_LOW );
		}
	    }
	    break;

        case PIN_MODE_PROD_SCAN:
            printf( "ERROR: adi_toggle_newton_gpio: Pin mode PIN_MODE_PROD_SCAN pinmux is not supported.\n" );
            return( ADI_UNEXPECTED_PIN_MODE );
            break;

        case PIN_MODE_FIELD_RETURN_SCAN:
            printf( "ERROR: adi_toggle_newton_gpio: Pin mode PIN_MODE_FIELD_RETURN_SCAN pinmux is not supported.\n" );
            return( ADI_UNEXPECTED_PIN_MODE );
            break;

	default:
            printf( "ERROR: adi_toggle_newton_gpio: Unknown pinmux mode.\n" );
            return( ADI_UNEXPECTED_PIN_MODE );
            break;
    }
#endif
    
    return( rc );

} // adi_toggle_newton_gpio

//====================================================================================
// Pulse Newton GPIO
//====================================================================================
int  adi_pulse_newton_gpio( int gpio, int width ) {
    int rc = ADI_NO_ERROR;
    int value;

#ifdef WIRINGPI

    switch( pinMuxMode ) {
        case PIN_MODE_FUNCTIONAL:
	    if( gpio == 0 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO0 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 1 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO1 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 2 ) {
    	        pinMode( PIN_GPIO2, OUTPUT );
                value = digitalRead( PIN_GPIO2 );
	        if( value == PIN_LOW ) {
	            digitalWrite( PIN_GPIO2, PIN_HIGH );
		    usleep( width );
	            digitalWrite( PIN_GPIO2, PIN_LOW );
		}
		else {
	            digitalWrite( PIN_GPIO2, PIN_LOW );
		    usleep( width );
	            digitalWrite( PIN_GPIO2, PIN_HIGH );
		}
	    }
	    else if( gpio == 3 ) {
    	        pinMode( PIN_GPIO3, OUTPUT );
                value = digitalRead( PIN_GPIO3 );
	        if( value == PIN_LOW ) {
	            digitalWrite( PIN_GPIO3, PIN_HIGH );
		    usleep( width );
	            digitalWrite( PIN_GPIO3, PIN_LOW );
		}
		else {
	            digitalWrite( PIN_GPIO3, PIN_LOW );
		    usleep( width );
	            digitalWrite( PIN_GPIO3, PIN_HIGH );
		}
	    }
	    else if( gpio == 4 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO4 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 5 ) {
    	        pinMode( PIN_GPIO5, OUTPUT );
                value = digitalRead( PIN_GPIO5 );
	        if( value == PIN_LOW ) {
	            digitalWrite( PIN_GPIO5, PIN_HIGH );
		    usleep( width );
	            digitalWrite( PIN_GPIO5, PIN_LOW );
		}
		else {
	            digitalWrite( PIN_GPIO5, PIN_LOW );
		    usleep( width );
	            digitalWrite( PIN_GPIO5, PIN_HIGH );
		}
	    }
	    else if( gpio == 6 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO6 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 7 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO7 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 8 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO8 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 9 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO9 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 10 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO10 is not allowed in Functional Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
            break;

        case PIN_MODE_HSP_DEBUG:
            printf( "INFO: adi_reset_newton: Enterring HSP Debug pinmux mode.\n" );
	    if( gpio == 0 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO0 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 1 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO1 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 2 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO2 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 3 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO3 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 4 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO4 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 5 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO5 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 6 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO6 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 7 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO7 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 8 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO8 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 9 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO9 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 10 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO10 is not allowed in HSP Debug Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    break;

        case PIN_MODE_DFT_JTAG:
	    if( gpio == 0 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO0 is not allowed in DFT JTAG Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 1 ) {
    	        pinMode( PIN_GPIO1, OUTPUT );
                value = digitalRead( PIN_GPIO1 );
	        if( value == PIN_LOW ) {
	            digitalWrite( PIN_GPIO1, PIN_HIGH );
		    usleep( width );
	            digitalWrite( PIN_GPIO1, PIN_LOW );
		}
		else {
	            digitalWrite( PIN_GPIO1, PIN_LOW );
		    usleep( width );
	            digitalWrite( PIN_GPIO1, PIN_HIGH );
		}
	    }
	    else if( gpio == 2 ) {
    	        pinMode( PIN_GPIO2, OUTPUT );
                value = digitalRead( PIN_GPIO2 );
	        if( value == PIN_LOW ) {
	            digitalWrite( PIN_GPIO2, PIN_HIGH );
		    usleep( width );
	            digitalWrite( PIN_GPIO2, PIN_LOW );
		}
		else {
	            digitalWrite( PIN_GPIO2, PIN_LOW );
		    usleep( width );
	            digitalWrite( PIN_GPIO2, PIN_HIGH );
		}
	    }
	    else if( gpio == 3 ) {
    	        pinMode( PIN_GPIO3, OUTPUT );
                value = digitalRead( PIN_GPIO3 );
	        if( value == PIN_LOW ) {
	            digitalWrite( PIN_GPIO3, PIN_HIGH );
		    usleep( width );
	            digitalWrite( PIN_GPIO3, PIN_LOW );
		}
		else {
	            digitalWrite( PIN_GPIO3, PIN_LOW );
		    usleep( width );
	            digitalWrite( PIN_GPIO3, PIN_HIGH );
		}
	    }
	    else if( gpio == 4 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO4 is not allowed in DFT JTAG Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 5 ) {
                printf( "ERROR: adi_pulse_newton_gpio: setting GPIO5 is not allowed in DFT JTAG Mode.\n" );
		rc = ADI_UNEXPECTED_ARGS;
	    }
	    else if( gpio == 6 ) {
    	        pinMode( PIN_GPIO6, OUTPUT );
                value = digitalRead( PIN_GPIO6 );
	        if( value == PIN_LOW ) {
	            digitalWrite( PIN_GPIO6, PIN_HIGH );
		    usleep( width );
	            digitalWrite( PIN_GPIO6, PIN_LOW );
		}
		else {
	            digitalWrite( PIN_GPIO6, PIN_LOW );
		    usleep( width );
	            digitalWrite( PIN_GPIO6, PIN_HIGH );
		}
	    }
	    else if( gpio == 7 ) {
    	        pinMode( PIN_GPIO7, OUTPUT );
                value = digitalRead( PIN_GPIO7 );
	        if( value == PIN_LOW ) {
	            digitalWrite( PIN_GPIO7, PIN_HIGH );
		    usleep( width );
	            digitalWrite( PIN_GPIO7, PIN_LOW );
		}
		else {
	            digitalWrite( PIN_GPIO7, PIN_LOW );
		    usleep( width );
	            digitalWrite( PIN_GPIO7, PIN_HIGH );
		}
	    }
	    else if( gpio == 8 ) {
    	        pinMode( PIN_GPIO8, OUTPUT );
                value = digitalRead( PIN_GPIO8 );
	        if( value == PIN_LOW ) {
	            digitalWrite( PIN_GPIO8, PIN_HIGH );
		    usleep( width );
	            digitalWrite( PIN_GPIO8, PIN_LOW );
		}
		else {
	            digitalWrite( PIN_GPIO8, PIN_LOW );
		    usleep( width );
	            digitalWrite( PIN_GPIO8, PIN_HIGH );
		}
	    }
	    else if( gpio == 9 ) {
    	        pinMode( PIN_GPIO9, OUTPUT );
                value = digitalRead( PIN_GPIO9 );
	        if( value == PIN_LOW ) {
	            digitalWrite( PIN_GPIO9, PIN_HIGH );
		    usleep( width );
	            digitalWrite( PIN_GPIO9, PIN_LOW );
		}
		else {
	            digitalWrite( PIN_GPIO9, PIN_LOW );
		    usleep( width );
	            digitalWrite( PIN_GPIO9, PIN_HIGH );
		}
	    }
	    else if( gpio == 10 ) {
    	        pinMode( PIN_GPIO10, OUTPUT );
                value = digitalRead( PIN_GPIO10 );
	        if( value == PIN_LOW ) {
	            digitalWrite( PIN_GPIO10, PIN_HIGH );
		    usleep( width );
	            digitalWrite( PIN_GPIO10, PIN_LOW );
		}
		else {
	            digitalWrite( PIN_GPIO10, PIN_LOW );
		    usleep( width );
	            digitalWrite( PIN_GPIO10, PIN_HIGH );
		}
	    }
	    break;

        case PIN_MODE_PROD_SCAN:
            printf( "ERROR: adi_pulse_newton_gpio: Pin mode PIN_MODE_PROD_SCAN pinmux is not supported.\n" );
            return( ADI_UNEXPECTED_PIN_MODE );
            break;

        case PIN_MODE_FIELD_RETURN_SCAN:
            printf( "ERROR: adi_pulse_newton_gpio: Pin mode PIN_MODE_FIELD_RETURN_SCAN pinmux is not supported.\n" );
            return( ADI_UNEXPECTED_PIN_MODE );
            break;

	default:
            printf( "ERROR: adi_pulse_newton_gpio: Unknown pinmux mode.\n" );
            return( ADI_UNEXPECTED_PIN_MODE );
            break;
    }
#endif
    
    return( rc );

} // adi_pulse_newton_gpio

//====================================================================================
// Get Newton GPIO value
//====================================================================================
int  adi_get_newton_gpio( int gpio ) {
    int value;
    
    value = digitalRead( PIN_GPIO2 );

#ifdef WIRINGPI
    switch( gpio ) {
        case GPIO0:  value = digitalRead( PIN_GPIO0  ); break;
        case GPIO1:  value = digitalRead( PIN_GPIO1  ); break;
        case GPIO2:  value = digitalRead( PIN_GPIO2  ); break;
        case GPIO3:  value = digitalRead( PIN_GPIO3  ); break;
        case GPIO4:  value = digitalRead( PIN_GPIO4  ); break;
        case GPIO5:  value = digitalRead( PIN_GPIO5  ); break;
        case GPIO6:  value = digitalRead( PIN_GPIO6  ); break;
        case GPIO7:  value = digitalRead( PIN_GPIO7  ); break;
        case GPIO8:  value = digitalRead( PIN_GPIO8  ); break;
        case GPIO9:  value = digitalRead( PIN_GPIO9  ); break;
        case GPIO10: value = digitalRead( PIN_GPIO10 ); break;
	default:
	    printf( "ERROR: adi_set_newton_gpio: Unknown gpio (%d) specified.\n", gpio );
            return( ADI_UNEXPECTED_PIN_MODE );
            break;
    }
#endif

} // adi_set_newton_gpio
