// see website for more advanced method:
// http://docs.opencv.org/3.0-beta/modules/videoio/doc/reading_and_writing_video.html
// compile with
// g++ -I/usr/local/include/opencv -I/usr/local/include/opencv2 -L/usr/local/lib/ -g -o webcam1Function  webcam1Function.cpp -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_stitching

#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <unistd.h>

using namespace cv;
using namespace std;


void takeImages(VideoCapture& cap, VideoCapture& cap1, int imagecount){
	cap.set(CV_CAP_PROP_FRAME_WIDTH,1920);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
    if(!cap.isOpened()) {  // check if we succeeded
        return;
	}
	
	//sleep(2);
    Mat edges;
    namedWindow("edges",1);
    for(int x = 0;x < imagecount; x++)
    {
        Mat frame, frame2;
		if( (!cap.grab()) || (!cap1.grab()) )
        {
            cout << "Can not grab images." << endl;
            return;
		}

		if( (cap.retrieve(frame,3))  && (cap1.retrieve(frame2,3)) ){
			imshow("edges", frame);
						
			stringstream s;
			s << x;
			string index = s.str();

			string filename = "images/[CAM1]image" + index + ".jpg";
			imwrite(filename,frame);
			string filename2 = "images/[CAM2]image" + index + ".jpg";
			imwrite(filename2,frame2);
		} else {
			cout << "Can not retrieve images." << endl;
            return;
		}
        if(waitKey(30) >= 0) break;
	}

}
int main(int, char**)
{
    VideoCapture cap(0); // open the default camera
    VideoCapture cap1(1); // open the default camera
	
	takeImages(cap, cap1, 3);
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
