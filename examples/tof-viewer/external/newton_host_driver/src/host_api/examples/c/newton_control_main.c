/* -*- Mode: C; tab-width: 4 -*- */
//-----------------------------------------------------------------------------
// Project            : newton
// File Name          : newton_control_main.c
// Original Author    : wpeet
// Description        : Utility for controlling newton.
//-----------------------------------------------------------------------------
// Copyright (c) 2020 by ADI. This code is the confidential and
// proprietary property of ADI and the possession or use of this
// file requires a written license from ADI.
//------------------------------------------------------------------------------

/**
 * @file newton_control_main.h
 * @brief Newton Control API
 *
 * @mainpage Newton Control Console Application and Newton Control API
 *
 * @section intro_sec Newton Control Console Application
 *
 * The purpose of the Newton Control Console Application is to provide a a program for
 * conrtolling and debugging Newton operation via the SPI slave interface.
 * 
 * Also provided is an API for building C programs to control Newton
 * 
 * @section toc Table of Contents
 * 
 * - @ref command_line_usage_sec
 * - @ref console_operation_sec
 * - @ref command_line_examples_sec
 *     - @ref newton_control_api_sec
 *         - @ref api_intro_sec
 *         - @ref functions_sec
 *         - @ref examples_sec
 *         - @ref load_firmware_sec
 * 
 * @section command_line_usage_sec Command Line Usage
 *
 * Usage:
 *
 *     - newton_control.exe load load_target file_name
 *     - newton_control.exe verify verify_target file_name (FPGA only)
 *     - newton_control.exe verify_frontdoor verify_target file_name (FPGA only)
 *     - newton_control.exe unload unload_target file_name (FPGA only)
 *     - newton_control.exe spi_write address data
 *     - newton_control.exe spi_read address
 *     - newton_control.exe reg_write address data
 *     - newton_control.exe reg_write_backdoor address data (FPGA only)
 *     - newton_control.exe reg_read address
 *     - newton_control.exe reg_read_backdoor address (FPGA only)
 *     - newton_control.exe reg_dump address word_count
 *     - newton_control.exe reg_fill address data word_count
 *     - newton_control.exe console
 *     - newton_control.exe reset_hsp (FPGA only)
 *     - newton_control.exe reset_newton
 *     - newton_control.exe check_done_code
 *
 * Where:
 *
 *     - load             : Loads the contents of the specified file into the target memory or executes a command file.
 *     - verify           : Compares the contents of the specified file to the contents of the target memory using backdoor accesses. (FPGA only)
 *     - verify_frontdoor : Compares the contents of the specified file to the contents of the target memory using frontdoor accesses.
 *     - unload           : Dumps the content of the target memory to the specified file.
 *
 *     - load_target is one of the following:
 *       - cmd_file      : HSP command file 
 *       - reg_file      : Registe address / data pair file 
 *       - hsp_rom       : HSP ROM (FPGA only)
 *       - hsp_ram       : HSP RAM (FPGA only)
 *       - efuse         : eFuse   (FPGA only)
 *
 *     - unload_target is one of the following:
 *       - hsp_rom       : HSP ROM (FPGA only)
 *       - hsp_ram       : HSP RAM (FPGA only)
 *       - efuse         : eFuse   (FPGA only)
 *
 *     - verify_target is one of the following:
 *       - hsp_rom       : HSP ROM (FPGA only)
 *       - hsp_ram       : HSP RAM (FPGA only)
 *       - efuse         : eFuse   (FPGA only)
 *
 *     - spi_write          : Write data to the address. For accessing HSP MBOX registers.
 *     - spi_read           : Read data at address. For accessing HSP MBOX registers.
 *     - reg_write          : Write data to the Newton register at address
 *     - reg_write_backdoor : Write data to the Newton register at address bypassing the HSP (FPGA only)
 *     - reg_read           : Read the Newton register at address
 *     - reg_read_backdoor  : Read the Newton register at address bypassing the HSP (FPGA only)
 *     - reg_dump           : Read and display word_count Newton registers starting at address.
 *     - reg_fill           : Write word_count word_count Newton registers with data starting at address.
 *     - console            : Enter console mode
 *     - reset_hsp          : Reset the HSP hardware (FPGA only)
 *     - reset_newton       : Reset the HSP hardware
 *     - check_done_code    : Check the completion code return by some 1SP code.
 *
 * Options:
 *
 *     --help Shows this help message.
 *
 * @section command_line_examples_sec Command Line Examples
 *
 * Here are a few examples of the command line interface:
 * 
 * - Loading new HSP ROM and eFuse data (FPGA only):
 *     - newton_control.exe load hsp_rom ../../firmware/ADI_Firmware_Drop_0.4/ADI_HSP_ROM_0.4.4.vhx
 *     - newton_control.exe load efuse ../../efuse/hsp_fuse_file.vhx
 *
 *     - Writing the HSP mailbox S2H_MBX_VALID register:
 *         - spi_write 2 4
 *
 *     - Reading the HSP mailbox S2H_MBX_VALID register:
 *         - spi_read 2
 *
 *     - Writing the newton microsequencer gprR0 register:
 *         - reg_write 60 01234
 *
 *     - Reading the newton microsequencer gprR0 register:
 *         - reg_read 60
 *
 * @section console_operation_sec Console Operation
 *
 * Console mode supports the same commands and arguments at the command line interface.
 *
 * \code{.c}
 * FIXME: need new code example
 * \endcode
 *
 * See newton_control.h for detailed documentation.
 *
 */

//=========================================================================
// INCLUDES
//=========================================================================
#include "newton_control.h"
//#include <wiringPi.h>
//#include <stdio.h>

//==============================================================================
// SPI Driver Handle
//==============================================================================
extern int spi_handle;

//=========================================================================
// Event Handler Flags
//=========================================================================
extern int event_flag[32];

//=========================================================================
// Dump specified number of 16-bit words
//=========================================================================
int adi_reg_dump( u16 address, int wordCount ) {
    int rc = ADI_ERROR_CODE_MISSING;
    int i = 0;
    u16 data16;
    u16 addr = address;
    u16 rd_data[256];

    if( wordCount > 256 ) {
        printf( "ERROR: Word count (%d) greater than 256 is not supported.\n", wordCount );
        return( ADI_UNEXPECTED_ARGS );
    }

    rc = adi_read_burst( address, wordCount, rd_data );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }

    for( i = 0; i < wordCount; i++ ) {
        if( i == 0 ) {
            printf( "%08x : ", addr );
        }
        else if( (i % 4) == 0 ) {
            printf( "\n%04x : ", addr );
        }

        printf( "%04x", rd_data[i] );

        printf( " " );
        addr += 2;
    }
    printf( "\n" );
    
    return( ADI_NO_ERROR );

} // adi_reg_dump
  
//=========================================================================
// Fill specified number of 16-bit words with 16-bit pattern
//=========================================================================
int adi_reg_fill( u16 address, u16 pattern, int wordCount ) {
    int rc = ADI_ERROR_CODE_MISSING;
    int i = 0;
    u16 addr = address;
    u16 wr_data[256];

    if( wordCount > 256 ) {
        printf( "ERROR: Word count (%d) greater than 256 is not supported.\n", wordCount );
        return( ADI_UNEXPECTED_ARGS );
    }

    for( i = 0; i < wordCount; i++ ) {
        wr_data[i] = pattern + i;
    }

    rc = adi_write_burst( address, wordCount, wr_data );
    if( rc != ADI_NO_ERROR ) {
        return( rc );
    }

    return( ADI_NO_ERROR );

} // adi_reg_fill

//=========================================================================
// Print usage message
//=========================================================================
void usage() {
    printf( "Usage:\n" );
    printf( "    newton_control.exe load target file_name : Loads the specified target (hsp_rom, hsp_ram, efuse, reg_file or cmd_file) memory with the contents of the specified file.\n" );
    printf( "    newton_control.exe spi_write address data : Writes the specfied address with data.\n" );
    printf( "    newton_control.exe spi_read address : Prints out the contents address\n" );
    printf( "    newton_control.exe reg_write address data : Writes the specified newton register (address) with data.\n" );
    printf( "    newton_control.exe reg_read address : Prints out the contents the newton register (address)\n" );
    printf( "    newton_control.exe reg_dump address word_count : Prints out the contents of word_count words starting at address.\n" );
    printf( "    newton_control.exe reg_fill address data word_count : Fills address with word_count words of the specified pattern.\n" );
    printf( "    newton_control.exe console : enter console mode\n" );
    printf( "    newton_control.exe check_done_code : Check the completion code return by some 1SP code.\n" );
    printf( "    newton_control.exe reset_newton pin_mode : Resets Newton with the specified Pin Mode.\n" );

#ifdef FPGA
    printf( "    newton_control.exe verify target file_name : Verifies the specified target (hsp_rom, hsp_ram, efuse, reg_file or cmd_file) memory with the contents of the specified file.\n" );
    printf( "    newton_control.exe verify_frontdoor target file_name : Verifies the specified target (cmd_file) memory with the contents of the specified file.\n" );
    printf( "    newton_control.exe unload target file_name : Unloads the specified target (hsp_rom, hsp_ram or efuse) memory to the specified file.\n" );
    printf( "    newton_control.exe reg_write_backdoor address data : Writes the specified newton register (address) with data. Bypasses HSP.\n" );
    printf( "    newton_control.exe reg_read_backdoor address : Prints out the contents the newton register (address). Bypasses HSP.\n" );
    printf( "    newton_control.exe reset_hsp : Resets the HSP hardware.\n" );
#endif

    printf( "Options:\n" );
    printf( "    --help Shows this help message.\n" );
}

//=========================================================================
// Help message
//=========================================================================
void help() {
    printf( "load target file_name : Loads the specified target (hsp_rom, hsp_ram or efuse) memory with the contents of the specified file or loads a command file.\n" );
    printf( "spi_write address data : Writes the specfied address with data.\n" );
    printf( "spi_read address : Prints out the contents address\n" );
    printf( "reg_write address data : Writes the specified newton register (address) with data.\n" );
    printf( "reg_write_backdoor address data : Writes the specified newton register (address) with data. Bypasses HSP.\n" );
    printf( "reg_read address : Prints out the contents the newton register (address)\n" );
    printf( "reg_read_backdoor address : Prints out the contents the newton register (address). Bypasses HSP.\n" );
    printf( "reg_dump address word_count : Prints out the contents of word_count words starting at address.\n" );
    printf( "reg_fill address pattern word_count : Fills address with word_count words of the specifie pattern.\n" );
    printf( "help : Prints this message.\n" );
    printf( "check_done_code : Check the completion code return by some 1SP code.\n" );
    printf( "reset_newton pin_mode : Resets newton with the specified Pin Mode.\n" );

#ifdef FPGA
    printf( "verify target file_name : Verifies the specified target (hsp_rom, hsp_ram, efuse, reg_file of cmd_file) memory with the contents of the specified file.\n" );
    printf( "verify_frontdoor target file_name : Verifies the specified target (cmd_file) memory with the contents of the specified file.\n" );
    printf( "unload target file_name : Unloads the specified target (hsp_rom, hsp_ram or efuse) memory to the specified file.\n" );
    printf( "reset_hsp : Resets the HSP hardware.\n" );
#endif

}

enum commands {
    HELP = 0,
    LOAD,
    VERIFY,
    VERIFY_FRONTDOOR,
    UNLOAD,
    SPI_WRITE,
    SPI_READ,
    REG_WRITE,
    REG_READ,
    REG_READ_BACKDOOR,
    REG_WRITE_BACKDOOR,
    LOOP_TEST,
    RESET_HSP,
    CHECK_DONE_CODE,
    REG_DUMP,
    REG_FILL,
	TEST_USEQ_RAM,
	RESET_NEWTON,
	CONSOLE,
	QUIT
} commands_e;

//=========================================================================
// parse command from console or script
//=========================================================================
int parseCommand( char *buff ) {
    int  rc = ADI_ERROR_CODE_MISSING;
    u16  address16 = 0;
    u32  address32 = 0;
    u16  writeData16 = 0;
    u32  writeData32 = 0;
    u16  readData  = 0;
    u16  pattern16 = 0;
    u32  pattern32 = 0;
    u16  increment16 = 0;
    u32  increment32 = 0;
    int  wordCount = 0;
    u08  data8[8];
    char command[32] = {'\0'};
    char target[32] = {'\0'};
    char cmd = HELP;
    char fileName[128];

	if( buff[0] == '\0' ) {
        return 1;
    }

    sscanf( buff, "%s", command );

    if( strcmp( "spi_read", command) == 0 ) {
        cmd = SPI_READ;
        sscanf( buff, "%s %4x", command, &address32 );
		address16 = address32;
    }
    else if( strcmp( "reg_read", command) == 0 ) {
        cmd = REG_READ;
        sscanf( buff, "%s %4x", command, &address32 );
		address16 = address32;
    }
    else if( strcmp( "reg_read_backdoor", command) == 0 ) {
        cmd = REG_READ_BACKDOOR;
        sscanf( buff, "%s %4x", command, &address32 );
		address16 = address32;
    }
    else if( strcmp( "help", command) == 0 ) {
        cmd = HELP;
    }
    else if ( strcmp( "load", command) == 0 ) {
        cmd = LOAD;
        sscanf( buff, "%s %s %s", command, target, fileName );
    }
    else if ( strcmp( "verify", command) == 0 ) {
        cmd = VERIFY;
        sscanf( buff, "%s %s %s", command, target, fileName );
    }
    else if ( strcmp( "verify_frontdoor", command) == 0 ) {
        cmd = VERIFY;
        sscanf( buff, "%s %s %s", command, target, fileName );
    }
    else if ( strcmp( "unload", command) == 0 ) {
        cmd = UNLOAD;
        sscanf( buff, "%s %s %s", command, target, fileName );
    }
    else if ( strcmp( "spi_write", command) == 0 ) {
        cmd = SPI_WRITE;
        sscanf( buff, "%s %4x %4x", command, &address32, &writeData32 );
		address16 = address32;
		writeData16 = writeData32;
    }
    else if ( strcmp( "reg_write", command) == 0 ) {
        cmd = REG_WRITE;
        sscanf( buff, "%s %4x %4x", command, &address32, &writeData32 );
		address16 = address32;
		writeData16 = writeData32;
    }
    else if ( strcmp( "reg_write_backdoor", command) == 0 ) {
        cmd = REG_WRITE_BACKDOOR;
        sscanf( buff, "%s %4x %4x", command, &address32, &writeData32 );
		address16 = address32;
		writeData16 = writeData32;
    }
    else if ( strcmp( "reg_dump", command) == 0 ) {
        cmd = REG_DUMP;
        sscanf( buff, "%s %4x %d", command, &address32, &wordCount );
		address16 = address32;
    }
    else if ( strcmp( "reg_fill", command) == 0 ) {
        cmd = REG_FILL;
        sscanf( buff, "%s %4x %4x %d", command, &address32, &writeData32, &wordCount );
		address16 = address32;
		writeData16 = writeData32;
    }
    else if ( strcmp( "exit", command) == 0 ) {
        cmd = QUIT;
	}
    else if ( strcmp( "quit", command) == 0 ) {
        cmd = QUIT;
	}
    else if ( strcmp( "reset_hsp", command) == 0 ) {
        cmd = RESET_HSP;
	}
    else if ( strcmp( "reset_newton", command) == 0 ) {
        cmd = RESET_NEWTON;
        sscanf( buff, "%s %s", command, target );
	}
    else if ( strcmp( "check_done_code", command) == 0 ) {
        cmd = CHECK_DONE_CODE;
	}
    else if ( strcmp( "test_useq_ram", command) == 0 ) {
	    cmd = TEST_USEQ_RAM;
	}

    switch( cmd ) {
        case HELP:
		    help();
            break;

        case LOAD:
            if( strcmp( "cmd_file", target ) == 0 ) {
				rc = adi_load_command_file( fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_load_command_file.\n", adi_error_msg( rc ) );
                }
			}
            if( strcmp( "reg_file", target ) == 0 ) {
				rc = adi_load_register_file( fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_load_register_file.\n", adi_error_msg( rc ) );
                }
			}
            else if( strcmp( "hsp_rom", target ) == 0 ) {
				rc = adi_load_hsp( HSP_ROM, fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_load_hsp.\n", adi_error_msg( rc ) );
                }
			}
            else if( strcmp( "hsp_ram", target ) == 0 ) {
				rc = adi_load_hsp( HSP_RAM, fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_load_hsp.\n", adi_error_msg( rc ) );
                }
			}
            else if( strcmp( "efuse", target ) == 0 ) {
		        rc = adi_load_hsp( EFUSE, fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_load_hsp.\n", adi_error_msg( rc ) );
                }
			}
            break;

        case VERIFY:
            if( strcmp( "cmd_file", target ) == 0 ) {
				rc = adi_verify_command_file( fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_verify_command_file.\n", adi_error_msg( rc ) );
                }
			}
            else if( strcmp( "hsp_rom", target ) == 0 ) {
				rc = adi_verify_hsp( HSP_ROM, fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_verify_hsp.\n", adi_error_msg( rc ) );
                }
			}
            else if( strcmp( "hsp_ram", target ) == 0 ) {
				rc = adi_verify_hsp( HSP_RAM, fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_verify_hsp.\n", adi_error_msg( rc ) );
                }
			}
            else if( strcmp( "efuse", target ) == 0 ) {
		        rc = adi_verify_hsp( EFUSE, fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_verify_hsp.\n", adi_error_msg( rc ) );
                }
			}
			break;

        case VERIFY_FRONTDOOR:
            if( strcmp( "cmd_file", target ) == 0 ) {
				rc = adi_verify_command_file_hsp( fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_verify_command_file_hsp.\n", adi_error_msg( rc ) );
                }
			}
			break;

        case UNLOAD:
            if( strcmp( "hsp_rom", target ) == 0 ) {
				rc = adi_unload_hsp( HSP_ROM, fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_unload_hsp.\n", adi_error_msg( rc ) );
                }
			}
            else if( strcmp( "hsp_ram", target ) == 0 ) {
				rc = adi_unload_hsp( HSP_RAM, fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_unload_hsp.\n", adi_error_msg( rc ) );
                }
			}
            else if( strcmp( "efuse", target ) == 0 ) {
		        rc = adi_unload_hsp( EFUSE, fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_unload_hsp.\n", adi_error_msg( rc ) );
                }
			}
            break;

        case SPI_WRITE:
	        rc = adi_spi_write_word( address16, writeData16 );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_spi_write_word.\n", adi_error_msg( rc ) );
            }
            break;

        case REG_WRITE:
	        rc = adi_write_register( address16, writeData16 );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_write_register.\n", adi_error_msg( rc ) );
            }
            break;

        case REG_WRITE_BACKDOOR:
	        rc = adi_write_register_backdoor( address16, writeData16 );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_write_register_backdoor.\n", adi_error_msg( rc ) );
            }
            break;

        case SPI_READ:
	        rc = adi_spi_read_word( address16, &readData );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_spi_read_word.\n", adi_error_msg( rc ) );
            }
            else {
	            printf( "0x%04x\n", readData );
            }
            break;

        case REG_READ:
	        rc = adi_read_register( address16, &readData );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_read_register.\n", adi_error_msg( rc ) );
            }
            else {
	            printf( "0x%04x\n", readData );
            }
            break;

        case REG_READ_BACKDOOR:
	        rc = adi_read_register_backdoor( address16, &readData );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_read_register_backdoor.\n", adi_error_msg( rc ) );
            }
            else {
	            printf( "0x%04x\n", readData );
            }
            break;

        case LOOP_TEST:
            // Not implemented.
            break;

        case REG_DUMP:
            rc = adi_reg_dump( address16, wordCount );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_reg_dump.\n", adi_error_msg( rc ) );
            }
            break;

        case REG_FILL:
            rc = adi_reg_fill( address16, writeData16, wordCount );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_reg_fill.\n", adi_error_msg( rc ) );
            }
            break;

        case RESET_HSP:
            rc = adi_reset_hsp( );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_reset_hsp.\n", adi_error_msg( rc ) );
            }
            break;

        case CHECK_DONE_CODE:
            rc = adi_check_done_code( );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_check_done_code.\n", adi_error_msg( rc ) );
            }
            break;

        case TEST_USEQ_RAM:
	        rc = adi_test_useq_ram();
	        if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_test_useq_ram.\n", adi_error_msg( rc ) );
	        }
	        break;

		case RESET_NEWTON:
            if( strcmp( "hsp_debug", target ) == 0 ) {
                rc = adi_reset_newton( PIN_MODE_HSP_DEBUG );
			}
            else if( strcmp( "dft_jtag", target ) == 0 ) {
                rc = adi_reset_newton( PIN_MODE_DFT_JTAG );
			}
            else if( strcmp( "keep_mode", target ) == 0 ) {
                rc = adi_reset_newton( PIN_MODE_KEEP_CURRENT_MODE );
			}
			else {
	            printf( "ERROR: Error \"%s\" is not supported by reset_newton command.\n", target );
			}
			
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_reset_newton.\n", adi_error_msg( rc ) );
            }
            break;
			
	    case QUIT:
            exit( 1 );
            break;

        default:
            break;
    }

    return rc;

} // parseCommand

//=========================================================================
// Command Shell
//=========================================================================
int readFromConsole (  ) {
    int rc = ADI_ERROR_CODE_MISSING;
    char buffer[512];
	char *buff = buffer;

    size_t bufsize = 64;
    size_t characters;
	
    while( 1 ) {
		printf( "console >  ");
        characters = getline( &buff, &bufsize, stdin );
        rc = parseCommand( buffer );
        if( rc != ADI_NO_ERROR ) {
            continue;
		}
    }
	
    return rc;

} // readFromConsole

//=========================================================================
// MAIN PROGRAM
//=========================================================================
int main( int argc, char **argv ) {
    int  rc = ADI_NO_ERROR;
    char target[32];
    char fileName[128];
    int  command = HELP;
    int  valueLen;
    int  param_code = 0;
    int  time_interval = 0;
    u16  address16 = 0;
    u32  address32 = 0;
    u16  writeData16 = 0;
    u32  writeData32 = 0;
    u16  readData;
    int  wordCount = 0;
    u16  increment16 = 0;
    u32  increment32 = 0;
	int  maxSpiBytes = 256;
    int  i;

    if( argc <= 1 ) {
        usage();
        exit( -1 );
    }
    
    if( argc > 1 ) {
        if ( !strcmp( argv[1], "--help" ) ) {
            usage();
            exit( -1 );
        }
        else if ( !strcmp( argv[1], "load" ) ) {
            command = LOAD;
            if( argc > 3 ) {
                strcpy( target, argv[2] );
                strcpy( fileName, argv[3] );
            }
			else {
	            printf( "ERROR: Missing arguments for load command.\n" );
                usage();
                exit( -1 );
			}
        }
        else if ( !strcmp( argv[1], "verify" ) ) {
            command = VERIFY;
            if( argc > 3 ) {
                strcpy( target, argv[2] );
                strcpy( fileName, argv[3] );
            }
			else {
	            printf( "ERROR: Missing arguments for verify command.\n" );
                usage();
                exit( -1 );
			}
        }
        else if ( !strcmp( argv[1], "verify_frontdoor" ) ) {
            command = VERIFY_FRONTDOOR;
            if( argc > 3 ) {
                strcpy( target, argv[2] );
                strcpy( fileName, argv[3] );
            }
			else {
	            printf( "ERROR: Missing arguments for verify_frontdoor command.\n" );
                usage();
                exit( -1 );
			}
        }
        else if ( !strcmp( argv[1], "unload" ) ) {
            command = UNLOAD;
            if( argc > 3 ) {
                strcpy( target, argv[2] );
                strcpy( fileName, argv[3] );
            }
			else {
	            printf( "ERROR: Missing arguments for unload command.\n" );
                usage();
                exit( -1 );
			}
        }
        else if ( !strcmp( argv[1], "spi_write" ) ) {
            command = SPI_WRITE;
            if( argc > 2 ) {
                sscanf( argv[2], "%4x", &address32 );
				address16 = address32;
                if( argc > 3 ) {
                    sscanf( argv[3], "%4x", &writeData32 );
				    writeData16 = writeData32;
                }
			    else {
	                printf( "ERROR: Missing address and data for spi_write command.\n" );
                    usage();
                    exit( -1 );
			    }
            }
			else {
	            printf( "ERROR: Missing data for spi_write command.\n" );
                usage();
                exit( -1 );
			}
        }
        else if ( !strcmp( argv[1], "reg_write" ) ) {
            command = REG_WRITE;
            if( argc > 2 ) {
                sscanf( argv[2], "%4x", &address32 );
				address16 = address32;
                if( argc > 3 ) {
                    sscanf( argv[3], "%4x", &writeData32 );
				    writeData16 = writeData32;
                }
			    else {
	                printf( "ERROR: Missing address and data for reg_write command.\n" );
                    usage();
                    exit( -1 );
			    }
            }
			else {
	            printf( "ERROR: Missing data for reg_write command.\n" );
                usage();
                exit( -1 );
			}
        }
        else if ( !strcmp( argv[1], "reg_write_backdoor" ) ) {
            command = REG_WRITE_BACKDOOR;
            if( argc > 2 ) {
                sscanf( argv[2], "%4x", &address32 );
				address16 = address32;
                if( argc > 3 ) {
                    sscanf( argv[3], "%4x", &writeData32 );
				    writeData16 = writeData32;
                }
			    else {
	                printf( "ERROR: Missing address and data for reg_write_backdoor command.\n" );
                    usage();
                    exit( -1 );
			    }
            }
			else {
	            printf( "ERROR: Missing data for reg_write_backdoor command.\n" );
                usage();
                exit( -1 );
			}
        }
        else if ( !strcmp( argv[1], "spi_read" ) ) {
            command = SPI_READ;
            if( argc > 2 ) {
                sscanf( argv[2], "%4x", &address32 );
				address16 = address32;
            }
			else {
	            printf( "ERROR: Missing address for spi_read command.\n" );
                usage();
                exit( -1 );
			}
        }
        else if ( !strcmp( argv[1], "reg_read" ) ) {
            command = REG_READ;
            if( argc > 2 ) {
                sscanf( argv[2], "%4x", &address32 );
				address16 = address32;
            }
			else {
	            printf( "ERROR: Missing address for reg_read command.\n" );
                usage();
                exit( -1 );
			}
        }
        else if ( !strcmp( argv[1], "reg_read_backdoor" ) ) {
            command = REG_READ_BACKDOOR;
            if( argc > 2 ) {
                sscanf( argv[2], "%4x", &address32 );
				address16 = address32;
            }
			else {
	            printf( "ERROR: Missing address for reg_read_backdoor command.\n" );
                usage();
                exit( -1 );
			}
        }
        else if ( !strcmp( argv[1], "reg_dump" ) ) {
            command = REG_DUMP;
            if( argc > 2 ) {
	            sscanf( argv[2], "%4x", &address32 );
				address16 = address32;
	            printf( "address = 0x%04x\n", address16 );
	            sscanf( argv[3], "%d", &wordCount );
	            printf( "wordCount = %d\n", wordCount );
            }
			else {
	            printf( "ERROR: Missing param_name for reg_dump command.\n" );
                usage();
                exit( -1 );
			}
        }
        else if ( !strcmp( argv[1], "reg_fill" ) ) {
            command = REG_FILL;
            if( argc > 2 ) {
	            sscanf( argv[2], "%4x", &address32 );
				address16 = address32;
	            sscanf( argv[3], "%4x", &writeData32 );
				writeData16 = writeData32;
	            sscanf( argv[4], "%d", &wordCount );
            }
			else {
	            printf( "ERROR: Missing param_name for reg_fill command.\n" );
                usage();
                exit( -1 );
			}
        }
        else if ( !strcmp( argv[1], "console" ) ) {
            command = CONSOLE;
		}
        else if ( !strcmp( argv[1], "reset_hsp" ) ) {
            command = RESET_HSP;
		}
        else if ( !strcmp( argv[1], "check_done_code" ) ) {
            command = CHECK_DONE_CODE;
		}
	    else if ( !strcmp( argv[1], "test_useq_ram") ) {
	        command = TEST_USEQ_RAM;
	    }
		else if ( !strcmp( argv[1], "reset_newton") ){
		    command = RESET_NEWTON;
            if( argc > 2 ) {
                strcpy( target, argv[2] );
            }
			else {
	            printf( "ERROR: Missing arguments for load command.\n" );
                usage();
                exit( -1 );
			}
	    }

    }

    rc = adi_newton_config( 0 );
    if( rc != ADI_NO_ERROR ) {
	    printf( "ERROR: Error \"%s\" returned from adi_newton_config.\n", adi_error_msg( rc ) );
        exit( rc );
    }
    
    switch( command ) {
        case HELP:
            usage();
            exit( -1 );
            break;

        case LOAD:
            if( strcmp( "cmd_file", target ) == 0 ) {
				rc = adi_load_command_file( fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_load_command_file.\n", adi_error_msg( rc ) );
                }
			}
            else if( strcmp( "reg_file", target ) == 0 ) {
				rc = adi_load_register_file( fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_load_register_file.\n", adi_error_msg( rc ) );
                }
			}
            else if( strcmp( "hsp_rom", target ) == 0 ) {
				rc = adi_load_hsp( HSP_ROM, fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_load_hsp.\n", adi_error_msg( rc ) );
                }
			}
            else if( strcmp( "hsp_ram", target ) == 0 ) {
				rc = adi_load_hsp( HSP_RAM, fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_load_hsp.\n", adi_error_msg( rc ) );
                }
			}
            else if( strcmp( "efuse", target ) == 0 ) {
		        rc = adi_load_hsp( EFUSE, fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_load_hsp.\n", adi_error_msg( rc ) );
                }
			}
            break;

        case VERIFY:
            if( strcmp( "cmd_file", target ) == 0 ) {
				rc = adi_verify_command_file( fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_verify_command_file.\n", adi_error_msg( rc ) );
                }
			}
            else if( strcmp( "hsp_rom", target ) == 0 ) {
				rc = adi_verify_hsp( HSP_ROM, fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_verify_hsp.\n", adi_error_msg( rc ) );
                }
			}
            else if( strcmp( "hsp_ram", target ) == 0 ) {
				rc = adi_verify_hsp( HSP_RAM, fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_verify_hsp.\n", adi_error_msg( rc ) );
                }
			}
            else if( strcmp( "efuse", target ) == 0 ) {
		        rc = adi_verify_hsp( EFUSE, fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_verify_hsp.\n", adi_error_msg( rc ) );
                }
			}
            break;

        case VERIFY_FRONTDOOR:
            if( strcmp( "cmd_file", target ) == 0 ) {
				rc = adi_verify_command_file_hsp( fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_verify_command_file_hsp.\n", adi_error_msg( rc ) );
                }
			}
            break;

        case UNLOAD:
            if( strcmp( "hsp_rom", target ) == 0 ) {
				rc = adi_unload_hsp( HSP_ROM, fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_unload_hsp.\n", adi_error_msg( rc ) );
                }
			}
            else if( strcmp( "hsp_ram", target ) == 0 ) {
				rc = adi_unload_hsp( HSP_RAM, fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_unload_hsp.\n", adi_error_msg( rc ) );
                }
			}
            else if( strcmp( "efuse", target ) == 0 ) {
		        rc = adi_unload_hsp( EFUSE, fileName );
                if( rc != ADI_NO_ERROR ) {
	                printf( "ERROR: Error \"%s\" returned from adi_unload_hsp.\n", adi_error_msg( rc ) );
                }
			}
            break;

        case SPI_WRITE:
	        rc = adi_spi_write_word( address16, writeData16 );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_spi_write_word.\n", adi_error_msg( rc ) );
                exit( rc );
            }
            break;

        case REG_WRITE:
	        rc = adi_write_register( address16, writeData16 );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_spi_write_word.\n", adi_error_msg( rc ) );
                exit( rc );
            }
            break;

        case REG_WRITE_BACKDOOR:
	        rc = adi_write_register_backdoor( address16, writeData16 );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_spi_write_word_backdoor.\n", adi_error_msg( rc ) );
                exit( rc );
            }
            break;

        case SPI_READ:
	        rc = adi_spi_read_word( address16, &readData );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_spi_read_word.\n", adi_error_msg( rc ) );
                exit( rc );
            }
            else {
	            printf( "0x%04x\n", readData );
            }
            break;

        case REG_READ:
	        rc = adi_read_register( address16, &readData );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_spi_read_register.\n", adi_error_msg( rc ) );
                exit( rc );
            }
            else {
	            printf( "0x%04x\n", readData );
            }
            break;

         case REG_READ_BACKDOOR:
	        rc = adi_read_register_backdoor( address16, &readData );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_spi_read_register_backdoor.\n", adi_error_msg( rc ) );
                exit( rc );
            }
            else {
	            printf( "0x%04x\n", readData );
            }
            break;

       case REG_DUMP:
            rc = adi_reg_dump( address16, wordCount );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_reg_dump.\n", adi_error_msg( rc ) );
                exit( rc );
            }
            break;

        case REG_FILL:
            rc = adi_reg_fill( address16, writeData16, wordCount );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_reg_fill.\n", adi_error_msg( rc ) );
                exit( rc );
            }
            break;

        case CONSOLE:
            rc = readFromConsole( );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from console.\n", adi_error_msg( rc ) );
                exit( rc );
            }
            break;

        case RESET_HSP:
            rc = adi_reset_hsp( );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_reset_hsp.\n", adi_error_msg( rc ) );
            }
            break;

        case CHECK_DONE_CODE:
            rc = adi_check_done_code( );
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_check_done_code.\n", adi_error_msg( rc ) );
            }
            break;

        case TEST_USEQ_RAM:
	        rc = adi_test_useq_ram();
	        if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_test_useq_ram.\n", adi_error_msg( rc ) );
	        }
	        break;
 
		case RESET_NEWTON:
            if( strcmp( "hsp_debug", target ) == 0 ) {
                rc = adi_reset_newton( PIN_MODE_HSP_DEBUG );
			}
            else if( strcmp( "dft_jtag", target ) == 0 ) {
                rc = adi_reset_newton( PIN_MODE_DFT_JTAG );
			}
            else if( strcmp( "keep_mode", target ) == 0 ) {
                rc = adi_reset_newton( PIN_MODE_KEEP_CURRENT_MODE );
			}
			else {
	            printf( "ERROR: Error \"%s\" is not supported by reset_newton command.\n", target );
			}
			
            if( rc != ADI_NO_ERROR ) {
	            printf( "ERROR: Error \"%s\" returned from adi_reset_newton.\n", adi_error_msg( rc ) );
            }
            break;
			
		default:
            printf( "ERROR: Unknown command.\n" );
            usage();
            exit( -1 );
            break;
    }

	exit( rc );
	
} // main
