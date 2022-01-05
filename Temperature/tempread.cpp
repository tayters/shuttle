
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


#define	DEVICE_ADD_0 0x64
#define DEVICE_ADD_1 0x65
#define DEVICE_ADD_2 0x66

#define READ_DELAY 700000

using std::cin;
using std::cout;
using std::endl;
using namespace std;

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
  int r;
  int result;
  int wbuf[1] = {'R'};
  int len = 16;
  uint8_t buf[len];
  int fd0, fd1, fd2;

  fd0 = i2c_device_init(fd0, DEVICE_ADD_0);
  fd1 = i2c_device_init(fd1, DEVICE_ADD_1);
  fd2 = i2c_device_init(fd1, DEVICE_ADD_2);

  for(;;)
  {
    write(fd0, READ_CMD, 1);
    write(fd1, READ_CMD, 1);
    write(fd2, READ_CMD, 1);

    usleep(READ_DELAY);

    read(fd0, buf, len);
    cout << (char*)buf << endl;

    read(fd1, buf, len);
    cout << (char*)buf << endl;

    read(fd2, buf, len);
    cout << (char*)buf << endl;

}

  return 0;
}
