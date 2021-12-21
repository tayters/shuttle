#include <stdio.h>
#include <time.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

using std::cin;
using std::cout;
using std::endl;
using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    Rect arena;
    Point arena_centre;
    Mat frame, thr, gray, src, src_crop;
    time_t rawtime;

    struct tm * timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    printf ( "The current ddate/time is: %s", asctime (timeinfo) );

    //Hilook IPcamera (substream /102)
    string vidAddress = "rtsp://admin:Snapper1@192.168.137.100:554/Streaming/Channels/101";
    VideoCapture cap(vidAddress);

    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    //Capture single frame and define arena
    cap >> frame;
    arena = selectROI("Arena", frame, 0);
    arena_centre = (arena.br() + arena.tl()) * 0.5;


    for (;;)
    {
        // get frame from the video and crate cropped area
        cap >> src;
        src_crop = src(arena);

        //Draw arena
        rectangle(src, arena, Scalar(0,255,0), 2);

        //Grayscale
        cvtColor(src_crop, gray, COLOR_BGR2GRAY);
        //Threshold and invert
        threshold(gray, thr, 10,255,THRESH_BINARY);
        thr = ~(thr);

        //Find centre of object
        Moments m = moments(thr, true);
        Point p(m.m10/m.m00, m.m01/m.m00);

        //Print circle to centre of object
        circle(src_crop, p, 3, Scalar(0,255,255), -1);

        //Show x,y coordinates
        putText(src,"x:"+to_string(p.x)+" y:"+to_string(p.y),
                Point(1000, 600), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 200, 130),
                1, 1);

        time(&rawtime);
        timeinfo = localtime(&rawtime);


        putText(src, asctime (timeinfo), Point(800, 650), FONT_HERSHEY_SIMPLEX,
                1, Scalar(0, 200, 130), 1, 1);


        // show image with the tracked object
        imshow("LIVE", src);

        //quit on ESC button
        if (waitKey(1) == 27)break;
    }
    return 0;
}
