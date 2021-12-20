#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
int main()
{
    Mat frame;
    namedWindow("video", 1);
    VideoCapture cap("rtsp://admin:Snapper1@192.168.137.100/");
    
    if(!cap.isOpened())
    {
			printf("Camera not found\n");
			getchar();
			return -1;
	}
	
	while(cap.isOpened())
	{
		cap>>frame;
		if(frame.empty()) break;
		
		imshow("video", frame);
		if(waitKey(30) >= 0) break;
	}
	
	return 0;
    
    
}
