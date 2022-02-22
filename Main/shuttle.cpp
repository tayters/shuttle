#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <stdio.h>
#include <time.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
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


using namespace cv;
using namespace std;

#define	DEVICE_ADD_0 0x64
#define DEVICE_ADD_1 0x65
#define DEVICE_ADD_2 0x66
#define DEVICE_ADD_3 0x67
#define DEVICE_ADD_4 0x68

#define COLD_L 0
#define COLD_R 1
#define HOT_L 2
#define HOT_R 3

#define READ_DELAY 700000
#define READ_BUF_LENGTH 16
const int READ_CMD[1] = {'R'};

#define BLACK Scalar(0,0,0)
#define WHITE Scalar(255,255,255)

int fdArr[5];
bool read_complete = true;
string values[5];

/*Function to initialise a I2C device returning a filehandle(fd)*/
int i2c_device_init(int fd, int Addr)
{
  if ((fd = open("/dev/i2c-1", O_RDWR)) < 0)
    cout << strerror (errno) << endl;

  if (ioctl (fd, I2C_SLAVE, Addr) < 0)
    cout << strerror (errno) << endl;

  return fd;
}

/*Function to read from PT1000 temperature I2C device*/
string temp_read(int fd)
{
  static uint8_t buf[READ_BUF_LENGTH];

  write(fd, READ_CMD, 1);
  usleep(READ_DELAY);
  read(fd, buf, READ_BUF_LENGTH);

  return (char*)buf;
}

/*Separate thread for reading tempeartures*/
 void temp_read_thr(void)
{
  for(;;)
  {
    for (int i=0; i<5; i++)
      values[i] = temp_read(fdArr[i]);

    read_complete = true;
  }
}

string generate_filename(time_t in_time)
{
  struct tm * time_struct;
  char fn_str[80];
  string out_str;

  time_struct = localtime(&in_time);
  strftime(fn_str, 80,"shuttleVid_%H-%M-%S_%d-%m-%Y", time_struct);
  out_str = fn_str;

  return out_str;
}


int main(int argc, char** argv)
{
    Rect arena, arena_left, arena_right;
    Point arena_centre;
    Mat frame, thr, gray, src, src_crop;
    time_t rawtime, start_time;
    struct tm * timeinfo;
    double time_elapsed;


    //Initialise relay(pump) control
    wiringPiSetup () ;
    pinMode(COLD_L, OUTPUT) ;
    pinMode(COLD_R, OUTPUT) ;
    pinMode(HOT_L, OUTPUT) ;
    pinMode(HOT_R, OUTPUT) ;

    // Intialise temp probes and start temperature read thread
    for (int i =0; i<5; i++)
      fdArr[i] = i2c_device_init(fdArr[i], DEVICE_ADD_0 + i);

    thread th1(temp_read_thr);

    //Get start time
    time(&rawtime);
    time(&start_time);
    timeinfo = localtime(&rawtime);
    printf ( "The current date/time is: %s", asctime(timeinfo));

    /*Setup Hilook IPcamera (substream /102), will need to be changed depending
    on network*/
    string vidAddress =
    "rtsp://admin:Snapper1@10.45.100.72:554/Streaming/Channels/101";
    VideoCapture cap1(vidAddress);

    if (!cap1.isOpened())
    {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    //Capture single frame and define arena
    cap1 >> frame;
    arena = selectROI("Arena", frame, 0);
    arena_left = Rect(arena.tl(), Size(arena.width*0.5, arena.height));
    arena_centre = (arena.br() + arena.tl()) * 0.5;
    destroyWindow("Arena");
    cout << "Arena: " << arena << endl;

    // Setup videocapture
    int frame_width = cap1.get(CAP_PROP_FRAME_WIDTH);
    int frame_height = cap1.get(CAP_PROP_FRAME_HEIGHT);
    string vidfilepath = "/home/pi/Videos/";
    string vidfilename = generate_filename(start_time);
    VideoWriter vw((vidfilepath + vidfilename + ".mp4"),
                      VideoWriter::fourcc('M','P','4','V'),
                      10, Size(frame_width, frame_height));


    //Refresh video stream
    cap1.release();
    VideoCapture cap(vidAddress);

    for (;;)
    {
        // get frame from the video and crate cropped area
        cap >> src;
        src_crop = src(arena);

        //Draw arenas
        rectangle(src, arena, Scalar(0,255,0), 2);
        rectangle(src, arena_left, Scalar(100,0,0), 2);

        //Grayscale
        cvtColor(src_crop, gray, COLOR_BGR2GRAY);

        //Threshold and invert
        threshold(gray, thr, 10,255,THRESH_BINARY);
        thr = ~(thr);

        //Find centre of object
        Moments m = moments(thr, true);
        Point p(m.m10/m.m00, m.m01/m.m00);

        //Print circle to centre of object
        circle(src_crop, p, 5, Scalar(0,0,255), -1);

        //Show x,y coordinates
        rectangle(src, Rect(Point(0,0), Size(300,75)), BLACK,
                  FILLED);

        putText(src,"x:"+to_string(p.x)+" y:"+to_string(p.y), Point(10, 30),
                FONT_HERSHEY_PLAIN, 1, WHITE, 1, 1);

        putText(src, ((p.x > arena.width*0.5)?"LEFT":"RIGHT"), Point(10, 45),
                FONT_HERSHEY_PLAIN, 1, WHITE, 1, 1);

        if (p.x > arena.width*0.5)
        {
          digitalWrite(HOT_L, LOW);
          digitalWrite(COLD_L, LOW);
          digitalWrite(COLD_R, LOW);
          digitalWrite(HOT_R, LOW);
        }
        else
        {
          digitalWrite(COLD_L, HIGH);
          digitalWrite(HOT_L, HIGH);
          digitalWrite(COLD_R, HIGH);
          digitalWrite(HOT_R, HIGH);
        }

        //Display time
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        putText(src, asctime (timeinfo), Point(10, 15), FONT_HERSHEY_PLAIN,
                1, WHITE, 1, 1);


        // show image with the tracked object
        imshow("LIVE", src);

        //Write to file
        vw.write(src);

        if(read_complete)
        {
          time(&rawtime);
          timeinfo = localtime(&rawtime);
          printf ("%s ", asctime (timeinfo));
          time_elapsed = difftime(rawtime, start_time);
          cout << time_elapsed << endl;

          if(time_elapsed > 1200)
          {
            time(&start_time);
            vw.release();
            vidfilename = generate_filename(start_time);
            vw.open((vidfilepath + vidfilename + ".mp4"),
                              VideoWriter::fourcc('M','P','4','V'),
                              10, Size(frame_width, frame_height));

          }



          for (int i=0; i<5; i++)
          {
            cout << values[i] <<" ";
          }
          cout << endl;
          read_complete = false;
        }

        //quit on ESC button
        if (waitKey(1) == 27)
        {
          vw.release();
          cap.release();
          digitalWrite(HOT_L, LOW);
          digitalWrite(COLD_L, LOW);
          digitalWrite(COLD_R, LOW);
          digitalWrite(HOT_R, LOW);
          break;
        }
    }

    return 0;
}
