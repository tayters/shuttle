
extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

#include <stdio.h>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <iostream>
#include <errno.h>
#include <unistd.h>

using std::cin;
using std::cout;
using std::endl;
using namespace std;

#define	DEVICE_ADD_0 0x64
#define DEVICE_ADD_1 0x65
#define DEVICE_ADD_2 0x66
#define DEVICE_ADD_3 0x67
#define DEVICE_ADD_4 0x68

#define READ_DELAY 700000
#define READ_BUF_LENGTH 16
const int READ_CMD[1] = {'R'};


int i2c_device_init(int fd, int Addr)
{
  if ((fd = open("/dev/i2c-1", O_RDWR)) < 0)
    cout << strerror (errno) << endl;

  if (ioctl (fd, I2C_SLAVE, Addr) < 0)
    cout << strerror (errno) << endl;

  return fd;
}

int main()
{
  uint8_t buf[READ_BUF_LENGTH];
  int fd0, fd1, fd2, fd3, fd4;

  //Initialize I2C temperature channels
  fd0 = i2c_device_init(fd0, DEVICE_ADD_0);
  fd1 = i2c_device_init(fd1, DEVICE_ADD_1);
  fd2 = i2c_device_init(fd2, DEVICE_ADD_2);
  fd3 = i2c_device_init(fd3, DEVICE_ADD_3);
  fd4 = i2c_device_init(fd4, DEVICE_ADD_4);

  //Create array of I2C channel file descriptors
  int fdArr[5] = {fd0, fd1, fd2, fd3, fd4};

  for(;;)
  {
    //Send read command to all sensors
    for (int i = 0; i < 5; i++)
    {
      write(fdArr[i], READ_CMD, 1);
    }

    usleep(READ_DELAY);

    //Read all sensors
    for (int i = 0; i < 5; i++)
    {
      read(fdArr[i], buf, READ_BUF_LENGTH);
      cout << (char*)buf << endl;
    }

    cout<<endl;

  }

  return 0;
}
