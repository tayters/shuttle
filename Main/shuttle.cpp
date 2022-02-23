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

#define READ_DELAY 700000
#define READ_BUF_LENGTH 16
const int READ_CMD[1] = {'R'};

#define BLACK Scalar(0,0,0)
#define WHITE Scalar(255,255,255)

enum pumpcontrol {COLD_L, COLD_R, HOT_L, HOT_R, NONE};
bool read_complete = true;

class Thermocouple
{
  public:
  int fd;
  double deg;
  string deg_string;

  //constructor
  Thermocouple(int addr)
  {
    //open i2cdevice
    if ((fd = open("/dev/i2c-1", O_RDWR)) < 0)
      cout << strerror (errno) << endl;

    if (ioctl (fd, I2C_SLAVE, addr) < 0)
      cout << strerror (errno) << endl;
  }

  void update()
  {
    static uint8_t buf[READ_BUF_LENGTH];

    write(fd, READ_CMD, 1);
    usleep(READ_DELAY);
    read(fd, buf, READ_BUF_LENGTH);

    deg_string = (char*)buf;
    deg_string.erase(0,1);
    deg = stod(deg_string);
  }
};

class Pump
{
  public:
    bool active;
    pumpcontrol outpin;

    //Constructor
    Pump(pumpcontrol p)
    {
      if(p != NONE)
      {
      outpin = p;
      pinMode(outpin, OUTPUT);
      digitalWrite(outpin, LOW);
      active = false;
      }
    }

    void turnOn()
    {
      if(!active)
      {
        digitalWrite(outpin, HIGH);
        active = true;
      }
    }

    void turnOff()
    {
      if(active)
      {
        digitalWrite(outpin, LOW);
        active = false;
      }
    }
};

class Tank
{
  public:
    Thermocouple tC;
    Pump hotPump, coldPump;

    //Constructor
    Tank(int addr, pumpcontrol h, pumpcontrol c)
      : tC(addr), hotPump(h), coldPump(c)
      {
      }

    //Destructor
    ~Tank()
    {
      hotPump.turnOff();
      coldPump.turnOff();
    }

};

/*Separate thread for reading temperatures*/
void read_thr(Tank *t1, Tank *t2, Tank *t3, Tank *t4)
{
  for(;;)
  {
   t1->tC.update();
   t2->tC.update();
   t3->tC.update();
   t4->tC.update();
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
    ofstream logfile;
    char tmstr[80];
    string tstr;




    /*Setup Tanks pumps and thermocouples */
    wiringPiSetup () ;
    Tank rightTank(DEVICE_ADD_1, HOT_R, COLD_R);
    Tank leftTank(DEVICE_ADD_2, HOT_L, COLD_L);
    Tank coldTank(DEVICE_ADD_3, NONE, NONE);
    Tank hotTank(DEVICE_ADD_0, NONE, NONE);

    /*start temperature reading thread */
    thread th1(read_thr, &leftTank, &rightTank, &hotTank, &coldTank);

    //Get start time
    time(&rawtime);
    time(&start_time);
    timeinfo = localtime(&rawtime);
    printf ( "The current date/time is: %s", asctime(timeinfo));

    /*Setup Hilook IPcamera (substream /101), will need to be changed depending
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

    logfile.open(vidfilepath + vidfilename + "_log.txt");
    logfile << "Time Xpos Ypos rightTankTemp leftTankTemp" <<endl;
    //Refresh video stream
    cap1.release();
    VideoCapture cap(vidAddress);


    for (;;)
    {
        // get frame from the video and create cropped area
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

        /*if (p.x > arena.width*0.5){}
        else{}*/

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
          printf ("%s ", asctime(timeinfo));
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


          cout << leftTank.tC.deg_string + " " + rightTank.tC.deg_string + " ";
          cout << hotTank.tC.deg_string + " " + coldTank.tC.deg_string << endl;



          strftime(tmstr, 80,"%H:%M:%S %d/%m/%Y", timeinfo);
          tstr=tmstr;
          logfile << tstr+" "+to_string(p.x)+" "+to_string(p.y)+" "+
          rightTank.tC.deg_string+" "+leftTank.tC.deg_string<<endl;

          read_complete = false;
        }

        //quit on ESC button
        if (waitKey(1) == 27)
        {
          vw.release();
          cap.release();

          logfile.close();

          break;
        }
    }

    return 0;
}
