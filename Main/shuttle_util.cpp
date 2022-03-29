// shuttle_util.cpp

#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <stdio.h>
#include <time.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <errno.h>
#include <unistd.h>
#include <thread>
#include <wiringPi.h>
#include <vector>
#include "shuttle_util.h"

using namespace cv;
using namespace std;

#define READ_DELAY 700000
#define READ_BUF_LENGTH 16

#define NONE 5

//constructor
Thermocouple::Thermocouple(int addr)
{
    //open i2cdevice
    if ((fd = open("/dev/i2c-1", O_RDWR)) < 0)
    cout << strerror (errno) << endl;

    if (ioctl (fd, I2C_SLAVE, addr) < 0)
    cout << strerror (errno) << endl;
}

void Thermocouple::update()
{
    static uint8_t buf[READ_BUF_LENGTH];
    static int READ_CMD[1] = {'R'};

    write(fd, READ_CMD, 1);
    usleep(READ_DELAY);
    read(fd, buf, READ_BUF_LENGTH);

    deg_string = (char*)buf;
    deg_string.erase(0,1);
    deg = stod(deg_string);
}

//Constructor
Pump::Pump(int p)
{
    if(p != NONE)
    {
        outpin = p;
        pinMode(outpin, OUTPUT);
        digitalWrite(outpin, LOW);
        active = false;
    }
}

void Pump::turnOn()
{
    if(!active)
    {
        //digitalWrite(outpin, HIGH);
        active = true;
    }
}

void Pump::turnOff()
    {
    if(active)
    {
        digitalWrite(outpin, LOW);
        active = false;
    }
}




  //Constructor
  Tank::Tank(int addr, int h, int c)
  : tC(addr), hotPump(h), coldPump(c)
  {
  }

  //Destructor
  Tank::~Tank()
  {
    hotPump.turnOff();
    coldPump.turnOff();
  }



