#include <iostream>
#include <errno.h>
#include <wiringPiI2C.h>
#include "Ezo_i2c.h"

using std::cin;
using std::cout;
using std::endl;

//
Ezo_board TP = Ezo_board(0x65, "TP");

#define	LED	8

int main()
{
TP.send_read_cmd();

  /*
  int fd, fd2, result;

  fd = wiringPiI2CSetup(0x64);
  cout << "Init result: "<< fd << endl;


  fd2 = wiringPiI2CSetup(0x65);
  cout << "2 result: "<< fd2 << endl;
  result = wiringPiI2CWrite(fd, 'R');

      if(result == -1)
      {
         cout << "Error.  Errno is: " << errno << endl;
      }

*/
 return 0;
}
