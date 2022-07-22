
#include "jtagmailbox_hostside.h"
int main()
{
  JtagSetup(true); //For TEST state
  DoReadSSFlow();
}
