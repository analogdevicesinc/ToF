
#include "jtagmailbox_hostside.h"
int main()
{
  JtagSetup(true); //For TEST state                                                            
  DoTransitionFlow(JTAG_MAILBOX_COMMAND_CHANGE_TO_PROD_STATE);
}
