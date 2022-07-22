
#include "jtagmailbox_hostside.h"
#include <cstring>
#include <stdio.h>
#include <stdlib.h>

//=========================================================================
// Read IDCODE
//=========================================================================
void adi_read_idcode( ) {
  
     JtagSetup( false ); //For non-TEST states                                                      

}

//=========================================================================
// Change the security state from Blank to Test using the JTAG mailbox
//=========================================================================
void adi_blank_to_test( ) {
  
     JtagSetup( false ); //For non-TEST states                                                      
     DoTransitionFlow( JTAG_MAILBOX_COMMAND_CHANGE_TO_TEST_STATE );

}

//=========================================================================
// Change the security state from Test to Retest using the JTAG mailbox
//=========================================================================
void adi_test_to_retest( ) {

     JtagSetup( true ); //For TEST state                                                            
     DoTransitionFlow( JTAG_MAILBOX_COMMAND_CHANGE_TO_RETEST_OR_EOL_STATE );

}

//=========================================================================
// Change the security state from Test to Production using the JTAG mailbox
//=========================================================================
void adi_test_to_production( ) {

     JtagSetup( true ); //For TEST state                                                            
     DoTransitionFlow( JTAG_MAILBOX_COMMAND_CHANGE_TO_PROD_STATE );

}

//=========================================================================
// Read the security state using the JTAG mailbox
//=========================================================================
void adi_read_security_state ( ) {

    JtagSetup( true ); //For TEST state
    DoReadSSFlow();

}

//=========================================================================
// Read IDCODEs
//=========================================================================
void adi_read_idcodes ( ) {

    JtagSetup( false ); //For not TEST state

}

//=========================================================================
// Print usage message
//=========================================================================
void usage() {
    printf( "Usage:\n" );
    printf( "    jtag_mailbox.exe blank2test  : Change the security state from Blank to Test using the JTAG mailbox.\n" );
    printf( "    jtag_mailbox.exe test2prod   : Change the security state from Test to Production using the JTAG mailbox\n" );
    printf( "    jtag_mailbox.exe test2retest : Change the security state from Test to Retest using the JTAG mailbox.\n" );
    printf( "    jtag_mailbox.exe read_state  : Read security state using the JTAG mailbox.\n" );
    printf( "    jtag_mailbox.exe idcode      : Read IDCODEs.\n" );

    printf( "Options:\n" );
    printf( "    --help Shows this help message.\n" );
}

int main( int argc, char **argv ) {

    if( argc <= 1 ) {
        usage();
        exit( -1 );
    }
    
    if( argc > 1 ) {
        if ( !strcmp( argv[1], "--help" ) ) {
            usage();
            exit( -1 );
        }
        else if ( !strcmp( argv[1], "blank2test" ) ) {
	    adi_blank_to_test( );
        }
        else if ( !strcmp( argv[1], "test2prod" ) ) {
	    adi_test_to_production( );
        }
        else if ( !strcmp( argv[1], "test2retest" ) ) {
	    adi_test_to_retest( );
        }
        else if ( !strcmp( argv[1], "read_state" ) ) {
	    adi_read_security_state( );
        }
        else if ( !strcmp( argv[1], "idcode" ) ) {
	    adi_read_security_state( );
        }
	else {
	    printf( "ERROR: Unknown command \"%s\"\n", argv[1] );
	}
    }
    
}
