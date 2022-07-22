
#include "jtagmailbox_hostside.h"
int main()
{
  JtagSetup(false); //For non-TEST states                                                      
  DoTransitionFlow(JTAG_MAILBOX_COMMAND_CHANGE_TO_TEST_STATE);
}
