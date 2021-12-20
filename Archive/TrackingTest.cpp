#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

using std::cin;
using std::cout;
using std::endl;
using namespace cv;
using namespace std;

void MyFilledCircle(Mat img, Point center);

int main(int argc, char** argv)
{
    Rect roi, arena;
    Point roi_center, arena_centre;
    Mat frame, thr, gray, src, src_crop;

    string vidAddress = "rtsp://admin:Snapper1@192.168.137.100:554/Streaming/Channels/101";

    VideoCapture cap(vidAddress);

    // Create a tracker object
    Ptr<Tracker> tracker = TrackerCSRT::create();


    cap >> frame;

    arena = selectROI("tracker", frame);
    //roi = selectROI("tracker", frame);
    arena_centre = (arena.br() + arena.tl()) * 0.5;

    /*if (roi.width == 0 || roi.height == 0)
        return 0;

        */
    // initialize the tracker
    //tracker->init(frame, roi);
    // perform the tracking process
    printf("Start the tracking process, press ESC to quit.\n");



    for (;;)
    {
        // get frame from the video
        cap >> src;
        src_crop = src(arena);



        cvtColor(src_crop, gray, COLOR_BGR2GRAY);

        threshold(gray,thr, 50,255,THRESH_BINARY);

        thr = ~(thr);

        Moments m = moments(thr,true);
        Point p(m.m10/m.m00, m.m01/m.m00);

        circle(src_crop, p, 5, Scalar(0,255,255), -1);

        //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

        /*

        // stop the program if no more images
        if (frame.rows == 0 || frame.cols == 0)
            break;
        // update the tracking result
        tracker->update(frame, roi);


        roi_center = (roi.br() + roi.tl()) * 0.5;
        // draw the tracked object
        rectangle(frame, roi, Scalar(255, 0, 0), 2, 1);
        /*
        if(roi_center.y>=280)
        {
               circle(frame, roi_center, 5, Scalar(0, 255, 255), FILLED, LINE_8);
        }
        else
        {
            circle(frame, roi_center, 5, Scalar(0, 255, 0), FILLED, LINE_8);
        }
        putText(frame,
            "x:"+to_string(roi_center.x)+" y:"+to_string(roi_center.y),
            Point(1000, 700),
            FONT_HERSHEY_SIMPLEX,
            1,
            Scalar(0, 255, 0),
            1,
            1);

            */
        // show image with the tracked object

        imshow("tracker", src);
        //quit on ESC button
        if (waitKey(1) == 27)break;
    }
    return 0;
}
