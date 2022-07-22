
#include "jtagmailbox_hostside.h"
int main()
{
  //JtagSetupAndReset(true); //For TEST state reset
  //JtagSetupAndReset(false); ////For non-TEST states reset

  JtagSetup(true); //For TEST state
  //JtagSetup(false); //For non-TEST states
  DoReadSSFlow();
  //DoTransitionFlow(JTAG_MAILBOX_COMMAND_CHANGE_TO_TEST_STATE);
  //DoTransitionFlow(JTAG_MAILBOX_COMMAND_CHANGE_TO_PROD_STATE);
  //DoTransitionFlow(JTAG_MAILBOX_COMMAND_CHANGE_TO_RETEST_OR_EOL_STATE);
}
