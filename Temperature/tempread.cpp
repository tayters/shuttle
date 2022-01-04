
extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

#include <stdio.h>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <iostream>
#include <errno.h>
#include <unistd.h>
//#include <wiringPiI2C.h>
//#include <wiringPi.h>
//#include "Ezo_i2c.h"

#define	DEVICE_ID_0 0x64
#define DEVICE_ID_1 0x65

using std::cin;
using std::cout;
using std::endl;
using namespace std;



int main()
{
  int fd;
  int result;
  int wbuf[1] = {'R'};
  int data = 2;
  int len = 16;
  uint8_t buf[len];


  if ((fd = open("/dev/i2c-1", O_RDWR)) < 0)
  {
    cout << strerror (errno) << endl;
  }


  if (ioctl (fd, I2C_SLAVE, 0x64) < 0)
  {
    cout << strerror (errno) << endl;
  }

  write(fd, wbuf, 1);

  usleep(1000000);

  read(fd,buf,len);

  for(int i=0; i<len; i++)
  {
    cout << buf[i] ;
  }

  cout<<endl;

  if(result == -1)
  {
    cout << "Error.  Errno is: " << errno << endl;
  }


  return 0;
}
