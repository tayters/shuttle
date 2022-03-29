// shuttle.cpp
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
#include "MiniPID.h"

using namespace cv;
using namespace std;

#define	DEVICE_ADD_0 0x64
#define DEVICE_ADD_1 0x65
#define DEVICE_ADD_2 0x66
#define DEVICE_ADD_3 0x67
#define DEVICE_ADD_4 0x68

#define READ_DELAY 700000
#define READ_BUF_LENGTH 16

#define BLACK Scalar(0,0,0)
#define WHITE Scalar(255,255,255)
#define GREEN Scalar(0,255,0)
#define BLUE Scalar(255,0,0)
#define RED Scalar(0,0,255)
#define ORANGE Scalar(0,165,255)
#define RED2 Scalar(0,20,255)

#define FRAME_HEIGHT 720
#define FRAME_WIDTH 1280

#define TRAIL_SIZE 200

#define RATE 0.00833

#define FONT FONT_HERSHEY_PLAIN

#define VIDEO_LENGTH 1200

#define COLD_L 0
#define COLD_R 1
#define HOT_L 2
#define HOT_R 3
#define NONE 4

enum Mode {Heat, Cool, Top, Bottom};
bool read_complete = true;
double duty = 0.1;

//Separate thread for reading temperatures
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

//Generate filename string based on current time
string generate_filename(time_t in_time)
{
  struct tm * time_struct;
  char temp[80];
  string out_str;

  time_struct = localtime(&in_time);
  strftime(temp, 80,"shuttleVid_%H-%M-%S_%d-%m-%Y", time_struct);
  out_str = temp;

  return out_str;
}

//Get string of surrent time
string get_time_string(void)
{
    time_t rawtime;
    struct tm * time_struct;
    char temp[40];
    string out_str;

    time(&rawtime);
    time_struct = localtime(&rawtime);
    strftime(temp, 40,"%d/%m/%Y %H:%M:%S", time_struct);
    out_str = temp;

    return out_str;
}


int median(vector<int> v)
{
    size_t n = v.size() / 2;
    nth_element(v.begin(), v.begin()+n, v.end());
    
    return v[n];
}


//Heating/cooling control 
void update_control(const vector<int> &x_vec, Tank *ht, Tank *ct, MiniPID *pidh, MiniPID *pidc)
{ 
  static double dutyc, dutyh;
  static double targeth = 15, targetc = 15, target_diff = 2;
  static int count, control_count = 20, h_count, c_count, tar_count;

  //Update control settings every control_count*2.8 seconds
  if(count >= control_count)
  {
    cout <<get_time_string()<<"  Time up"<<endl;  
    int x_median = median(x_vec);
    cout << "x_median: "<<x_median<<"  "<<x_vec.size()<<endl;

    if(tar_count >= 6)
    {
      if(targetc >= 15)
      {
        //targetc = 0.1;
      }

      targeth = ct->tC.deg + target_diff;
      tar_count = 0;
    }
    
    cout<< "Hot target: " << targeth << endl;
    cout<< "Cold target: " << targetc << endl;

    //Update hot chamber duty cycle   
    dutyh = pidh->getOutput(ht->tC.deg, targeth);
    cout << "Pid hot output: " << dutyh <<endl;
    //cout << "Hot on count: " << h_count << endl;

    if(dutyh > 0.1)
    {
      h_count = abs((int)(control_count*dutyh));
      ht->hotPump.turnOn();
    }
    else if(dutyh < -0.1)
    {
      h_count = 2*abs((int)(control_count*dutyh));
      ht->coldPump.turnOn();
    }

    //Update right chamber duty cycle    
    dutyc = pidc->getOutput(ct->tC.deg, targetc);
    cout << "Pid cold output: " << dutyc <<endl;
    //cout << "Left on count: " << h_count << endl;
    
    if(dutyc > 0.1)
    {
      c_count = abs((int)(control_count*dutyc));
      ct->hotPump.turnOn();
    }
    else if(dutyc < -0.1)
    {
      c_count = 2*abs((int)(control_count*dutyc));
      ct->coldPump.turnOn();
    }

      count = 0;
      tar_count++;
  }

  if(count == h_count)
  {
    //cout << get_time_string()<< " Turn off left pumps" << endl;
    ht->hotPump.turnOff();
    ht->coldPump.turnOff();
  }

  if(count == c_count)
  {
    //cout << get_time_string()<< " Turn off right pumps" << endl;
    ct->hotPump.turnOff();
    ct->coldPump.turnOff();
  }

 count++;

}


//Main
int main(int argc, char** argv)
{
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  vector<int> x_vec(600,0);
  Mat src, src_crop, thr, gray, can, fgmask;
  Rect boundrect;
  static Point p;
  time_t now_time, start_time, control_time;
  double time_elapsed, control_time_elapsed, min_area = 150;
  int key = 0, big_cont_index = 0;
  string hstring;
  bool found = false;
  
  //Setup Tanks pumps and thermocouples
  wiringPiSetup();
  Tank leftTank(DEVICE_ADD_1, HOT_L, COLD_L);
  Tank rightTank(DEVICE_ADD_0, HOT_R, COLD_R);
  Tank coldTank(DEVICE_ADD_2, NONE, NONE);
  Tank hotTank(DEVICE_ADD_3, NONE, NONE);

  //start temperature reading thread
  thread th1(read_thr, &leftTank, &rightTank, &hotTank, &coldTank);

  //Get start time
  time(&start_time);
  time(&control_time);
  cout << "Start time: " + get_time_string() << endl;

  //Setup arena and info
  Rect arena = Rect(Point(65,170),Size(1130,540));
  Rect arena_left = Rect(arena.tl(), Size(arena.width*0.5, arena.height));
  Point arena_centre = (arena.br() + arena.tl()) * 0.5;
  Rect info_left = Rect((arena.tl()-Point(0,51)),Size(200,50));
  Rect info_right = Rect((arena.tl()+Point(arena.width/2,-51)),Size(200,50));

  // Setup videocapture
  string vidfilepath = "/home/pi/Videos/";
  string vidfilename = generate_filename(start_time);
  VideoWriter vw((vidfilepath + vidfilename + ".mp4"),
                  VideoWriter::fourcc('m','p','4','v'), 10,
                  Size(FRAME_WIDTH, FRAME_HEIGHT));

  //Open logfile for writing to
  ofstream logfile;
  hstring = "Date Time X Y RTemp1 RTemp2 LTemp1 LTemp2 RHot RCold LHot LCold";
  logfile.open(vidfilepath + vidfilename + "_log.txt");
  logfile << hstring << endl;


  //Setup Hilook IPcamera (substream /101), will need to be changed depending on
  //network
  string vidAddress =
  "rtsp://admin:Snapper1@10.45.100.72:554/Streaming/Channels/101";
  //Start video stream
  VideoCapture cap(vidAddress);

  //Init Background subtractor
  Ptr<BackgroundSubtractor> pBackSub;
  pBackSub = createBackgroundSubtractorKNN();

  MiniPID pidh = MiniPID(2,0.1,0.1);
  pidh.setOutputLimits(-1,1);
  MiniPID pidc = MiniPID(2,0.1,0.1);
  pidc.setOutputLimits(-1,1);

   
  //MAIN LOOP*******************************************************************
  for (;;)
  {
    //Get frame from the video and create cropped area
    cap >> src;
    src_crop = src(arena);
    rectangle(src_crop, Rect(Point((arena.width/2)-40,190), Size(80,80)), WHITE,
              FILLED);

     //update the background model
    pBackSub->apply(src_crop, fgmask);
    
    //find contours
    findContours(fgmask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE,
                 Point(0, 0));

    for(int i = 0; i < contours.size(); i++)
    {
      double area = contourArea(contours[i]);

      if(area > min_area)
      {
        big_cont_index = i;
        found = true;
      }
    }

    if(found)
    {
      drawContours(src_crop, contours, big_cont_index, ORANGE, 2, 8, hierarchy, 
                   0, Point(0,0));
      Moments m = moments(contours[big_cont_index],true);
      p = Point(m.m10/m.m00, m.m01/m.m00);
           
      found = false;
    }

    x_vec.insert(x_vec.begin(), p.x);
    x_vec.pop_back();
    
    //Mark position (found or not found)
    circle(src_crop, p, 5, RED, -1);    

    //Show x,y coordinates, display time, display coordinates
    rectangle(src, Rect(Point(0,0), Size(300,100)), BLACK, FILLED);
    putText(src, get_time_string(), Point(10, 15), FONT, 1, WHITE,
            1, 1);
    putText(src, "x:"+to_string(p.x)+" y:"+to_string(p.y), Point(10, 30),
            FONT, 1, WHITE, 1, 1);
    //Left or right chamber
    putText(src, ((p.x > arena.width*0.5)?"RIGHT":"LEFT"), Point(10, 45),
            FONT, 1, WHITE, 1, 1);
    putText(src, "COLD RES: "+coldTank.tC.deg_string, Point(10, 60),
            FONT, 1, WHITE, 1, 1);
    putText(src, "HOT RES: "+hotTank.tC.deg_string, Point(10, 75),
            FONT, 1, WHITE, 1, 1);

    //Draw arenas
    rectangle(src, arena, BLUE, 2);
    rectangle(src, arena_left, BLUE, 2);
    rectangle(src, info_left, BLACK, FILLED);
    rectangle(src, info_right, BLACK, FILLED);
    //Chamber temperatures and pump activity
    putText(src, "LEFT "+leftTank.tC.deg_string+" (H)", info_left.tl()+Point(10,15),
            FONT, 1, WHITE, 1, 1);
    putText(src, leftTank.hotPump.active?"HOT PUMP ON":"HOT PUMP OFF",
            info_left.tl()+Point(10,30), FONT, 1, WHITE, 1, 1);
    putText(src, leftTank.coldPump.active?"COLD PUMP ON":"COLD PUMP OFF",
            info_left.tl()+Point(10,45), FONT, 1, WHITE, 1, 1);
    putText(src, "RIGHT "+rightTank.tC.deg_string+" (C)", info_right.tl()+Point(10,15),
            FONT, 1, WHITE, 1, 1);
    putText(src, rightTank.hotPump.active?"HOT PUMP ON":"HOT PUMP OFF",
            info_right.tl()+Point(10,30), FONT, 1, WHITE, 1, 1);
    putText(src, rightTank.coldPump.active?"COLD PUMP ON":"COLD PUMP OFF",
            info_right.tl()+Point(10,45), FONT, 1, WHITE, 1, 1);

    
    //imshow("src_crop", src_crop);
    //imshow("fgmask", fgmask);
    namedWindow("LIVE", 1);
    imshow("LIVE", src);

    //Write video to file
    vw.write(src);

    //After every temperature read (~2.8s)
    if(read_complete)
    {
      //Get time elapsed in seconds
      time(&now_time);
      time_elapsed = difftime(now_time, start_time);

      update_control(x_vec, &leftTank, &rightTank, &pidh, &pidc);
      
      //New files
      if(time_elapsed > VIDEO_LENGTH)
      {
        vw.release();
        time(&start_time);
        vidfilename = generate_filename(start_time);
        vw.open((vidfilepath + vidfilename + ".mp4"),
                VideoWriter::fourcc('m','p','4','v'), 10,
                Size(FRAME_WIDTH, FRAME_HEIGHT));
        logfile.close();
        logfile.open(vidfilepath + vidfilename + "_log.txt");
        logfile << hstring << endl;
      }

      logfile << get_time_string()+" "+to_string(p.x)+" "+ to_string(p.y);
      logfile <<" "<< rightTank.tC.deg_string<<" "<<hotTank.tC.deg_string
              <<" "<< leftTank.tC.deg_string <<" "<<coldTank.tC.deg_string
              <<" "<<rightTank.hotPump.active<<" "<<rightTank.coldPump.active
              <<" "<< leftTank.hotPump.active<<" "<<leftTank.coldPump.active
              << endl;

      read_complete = false;
    }

    switch((char)waitKey(1))
    {
      case 'o':
        if(!leftTank.hotPump.active){
          leftTank.hotPump.turnOn();
          cout<<"Left Hot Pump On"<<endl;
        }else{
          leftTank.hotPump.turnOff();
          cout<<"Left Hot Pump Off"<<endl;
        }
        break;

      case 'p':
        if(!leftTank.coldPump.active){
          leftTank.coldPump.turnOn();
          cout<<"Left Cold Pump On"<<endl;
        }else{
          leftTank.coldPump.turnOff();
          cout<<"Left Cold Pump Off"<<endl;
        }
        break;

      case 'k':
        if(!rightTank.hotPump.active){
          rightTank.hotPump.turnOn();
          cout<<"Right Hot Pump On"<<endl;
        }else{
          rightTank.hotPump.turnOff();
          cout<<"Right Hot Pump Off"<<endl;
        }
        break;

      case 'l':
        if(!rightTank.coldPump.active){
          rightTank.coldPump.turnOn();
          cout<<"Right Cold Pump On"<<endl;
        }else{
          rightTank.coldPump.turnOff();
          cout<<"Right Cold Pump Off"<<endl;
        }
        break;
      
      case 'm':
        min_area++;
        cout<<"Min area: "<<min_area<<endl;
        break;

      case 'n':
        min_area--;
        cout<<"Min area: "<<min_area<<endl;
        break;
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
