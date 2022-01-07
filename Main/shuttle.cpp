extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}
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

using std::cin;
using std::cout;
using std::endl;
using std::vector;
using namespace cv;
using namespace std;

#define	DEVICE_ADD_0 0x64
#define DEVICE_ADD_1 0x65
#define DEVICE_ADD_2 0x66
#define DEVICE_ADD_3 0x67
#define DEVICE_ADD_4 0x68

#define READ_DELAY 700000
#define READ_BUF_LENGTH 16
const int READ_CMD[1] = {'R'};
//int fd0, fd1, fd2, fd3, fd4;
int fdArr[5];
bool read_complete = true;
string value;
string values[5];


int i2c_device_init(int fd, int Addr)
{
  if ((fd = open("/dev/i2c-1", O_RDWR)) < 0)
    cout << strerror (errno) << endl;

  if (ioctl (fd, I2C_SLAVE, Addr) < 0)
    cout << strerror (errno) << endl;

  return fd;
}

string temp_read(int fd)
{
  static uint8_t buf[READ_BUF_LENGTH];
  write(fd, READ_CMD, 1);
  usleep(READ_DELAY);
  read(fd, buf, READ_BUF_LENGTH);

  return (char*)buf;

}

 void read_thr(void)
{
  for(;;)
  {
    for (int i=0; i<5; i++)
    {
      values[i] = temp_read(fdArr[i]);
    }
    read_complete = true;
  }
}


int main(int argc, char** argv)
{
    Rect arena, arena_left, arena_right;
    Point arena_centre;
    Mat frame, thr, gray, src, src_crop;
    time_t rawtime;

    //Initialize I2C temperature channels
    /*fd0 = i2c_device_init(fd0, DEVICE_ADD_0);
    fd1 = i2c_device_init(fd1, DEVICE_ADD_1);
    fd2 = i2c_device_init(fd2, DEVICE_ADD_2);
    fd3 = i2c_device_init(fd3, DEVICE_ADD_3);
    fd4 = i2c_device_init(fd4, DEVICE_ADD_4);
    */
    for (int i =0; i<5; i++)
    {
      fdArr[i] = i2c_device_init(fdArr[i], DEVICE_ADD_0+i);
    }

      //start read thread
    thread th1(read_thr);
    cout << value << endl;

    struct tm * timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    printf ( "The current date/time is: %s", asctime (timeinfo) );

    //Hilook IPcamera (substream /102)
    string vidAddress = "rtsp://admin:Snapper1@192.168.137.100:554/Streaming/Channels/101";
    VideoCapture cap(vidAddress);

    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    // Default resolutions of the frame are obtained.The default resolutions are system dependent.
    //int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    //int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    //VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, Size(frame_width,frame_height));



    //Capture single frame and define arena
    cap >> frame;

    arena = selectROI("Arena", frame, 0);
    arena_left = Rect(arena.tl(), Size(arena.width*0.5, arena.height));
    arena_centre = (arena.br() + arena.tl()) * 0.5;


    cout << "Arena: " << arena << endl;
    cout << "Arena_centre: " << arena_centre <<endl;

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
        circle(src_crop, p, 5, Scalar(255,0,0), -1);

        //Show x,y coordinates
        putText(src,"x:"+to_string(p.x)+" y:"+to_string(p.y),
                Point(1000, 600), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 200, 130),
                2, 1);




        putText(src, ((p.x > arena.width*0.5)?"LEFT":"RIGHT"),
                Point(100, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 200, 130),
                2, 1);


        time(&rawtime);
        timeinfo = localtime(&rawtime);

        putText(src, asctime (timeinfo), Point(800, 650), FONT_HERSHEY_SIMPLEX,
                1, Scalar(0, 200, 130), 1, 1);


      //video.write(src);

        // show image with the tracked object
        imshow("LIVE", src);

        if(read_complete)
        {
          for (int i=0; i<5; i++)
          {
            cout << values[i] <<" ";
          }
          cout<<endl;
          read_complete = false;
        }

        //quit on ESC button
        if (waitKey(1) == 27)break;
    }

    return 0;
}
