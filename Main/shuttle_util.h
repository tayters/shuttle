// shuttle_util.h
#ifndef SHUTTLE_UTIL_H
#define SHUTTLE_UTIL_H

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

class Thermocouple
{
public:
  int fd;
  double deg = 0, deg_prev = 0;
  string deg_string = "0";

  //constructor
  Thermocouple(int addr); 
    
  void update();
  
};

class Pump
{
public:
  bool active;
  int outpin;

  //Constructor
  Pump(int p);
  void turnOn();
  void turnOff();
  
};

class Tank
{
public:
  Thermocouple tC;
  Pump hotPump, coldPump;

  //Constructor
  Tank(int addr, int h, int c);
  

  //Destructor
  ~Tank();
  
  
};


#endif