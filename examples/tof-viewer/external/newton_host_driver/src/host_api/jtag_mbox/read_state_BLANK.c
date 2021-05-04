
#include "jtagmailbox_hostside.h"
int main()
{
  JtagSetup(false); //For non-TEST states                                                      
  DoReadSSFlow();
}
