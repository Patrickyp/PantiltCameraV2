/*
*  Source: http://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c/38318768#comment71422395_38318768
*
*
*  Compile: g++ -I/usr/local/include/opencv -I/usr/local/include/opencv2 -L/usr/local/lib/ -g -o PantiltV1  PantiltV1.cpp -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_stitching
*  Run: ./PantiltV1
*
*/

#include "opencv2/opencv.hpp"
#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <ctime>
#include <sys/time.h>
#include <time.h>
#include <fstream>
#include <iomanip>

using namespace cv;
using namespace std;

/*
* Serial port settings.
*/
int set_interface_attribs(int fd, int speed) {
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Er#include <fstream>ror from tcgetattr: %s\n", strerror(errno));
        return -1;
    }
 
    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void set_mincount(int fd, int mcount) {
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}

/*
*
*/
int open_serial(const char * portname) {
	const char *port = portname;
	int fd;
	// to access serial porprintf ldt on linux, you need to open file.
    fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B115200);
    //set_mincount(fd, 0);                /* set to pure timed read */
	return fd;
}



/*
*	Write the given c-string data to serial port.
*/
int write_serial(int fd, const char *data) {
	int datalen = strlen(data);
	//printf("Length = %d", datalen);
	int wrlen = write(fd, data, datalen);
    if (wrlen != datalen) {
        printf("Error from write: %d, %d\n", wrlen, errno);
		return -1;
    }
    tcdrain(fd);    /* delay for output */
	return 0;
}
/*
*	Combine the 3 integers with commas creating "p1,p2,p3" as c++ string.
*/
std::string createCommand(int p1, int p2, int p3){
	std::stringstream sstm;
	std::string comma = ",";
	sstm << p1 << comma << p2 << comma << p3;
	std::string str = sstm.str();
	return str;
}

/*
*	Get the current date/time and return as string.
*/
std::string getCurrentTime(){
	//time_t now;
	struct tm *now_tm;
    struct timeval tv;

    time_t currtime;
    gettimeofday(&tv, NULL); 

    currtime=tv.tv_sec;
	long int usec = tv.tv_usec;

	now_tm = localtime(&currtime);
	int hour = now_tm->tm_hour;
	int sec = now_tm->tm_sec;
	int min = now_tm->tm_min;
	int day = now_tm->tm_yday;
	
	double dhour = (double)hour / 24.0;
	double dmin = (double)min / (60 * 24.0);
	//double dsec = (double)sec / (3600 * 24.0);
	double dusec = (double)usec / 1000000;
	double dsec = (double)sec + dusec;
	dsec = dsec / (3600.0 * 24.0);
	double overallDate = (double) day + dhour + dmin + dsec;

	std::ostringstream strs;
	strs << std::setprecision(9) << overallDate;
	std::string timeStr = strs.str();
	//cout <<"final overall:"<<str<<endl;
	return timeStr;
}

/*
*	Read file containing longitude, latitude, height data (file contains 3 lines) and set the passed strings
*	to their respective data.  Note that the 3 strings are passed by reference so any changes made
*	to them within this method will be saved when this function exits.
*/
void readGPSFile(string fileName, string& longitude, string& latitude, string& height) {
	std::ifstream file("current_data.txt");
	std::string str;
	int currentLine = 0;
	while (std::getline(file, str))
    {
		//line 1 is latitude data
        if(currentLine == 0){
			latitude = str;
		}
		else if(currentLine == 1){
			longitude = str;
		}
		else if(currentLine == 2){
			height = str;
		}
		else {
			break;
		}
		currentLine++;
    }
}

/*
*	Take imagecount number of images for each camera, create filenames, and save them with the created names.
*/
void takeImages(VideoCapture& cap, VideoCapture& cap1, int imagecount){
	
	string frameTime1, frameTime2;

	printf("Taking %d images per camera\n.",imagecount);	
    // Set capture resolution.
	cap.set(CV_CAP_PROP_FRAME_WIDTH,1920);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
	cap1.set(CV_CAP_PROP_FRAME_WIDTH,1920);
	cap1.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
	
    //Mat edges;
    //namedWindow("edges",1);

	// Loop both cameras imagecount times and grab frame from each camera
    for(int x = 0;x < imagecount; x++)
    {
        Mat frame, frame2;
		if(!cap.grab())
        {
            cout << "Can not grab camera1." << endl;
            return;
		}
		string frameTime1 = getCurrentTime();
  
		if(!cap1.grab())
        {
            cout << "Can not grab camera2." << endl;
            return;
		}
		string frameTime2 = getCurrentTime();

		if( (cap.retrieve(frame,3))  && (cap1.retrieve(frame2,3)) ){
			//get current image index
			stringstream s;
			s << x;
			string index = s.str();

			//Get current time.
			string currtime = getCurrentTime();

			//Get longitude,latitude,height from file.
			string lon, lat, height;
			readGPSFile("current_data.txt", lon,lat,height);


			string filename = "images/[C1]i" + index + "-" + lon + "-" + lat + "-" + height + "-" + frameTime1 +  ".jpg";
			imwrite(filename,frame);
			string filename2 = "images/[C2]i" + index + "-" + lon + "-" + lat + "-" + height + "-" + frameTime2 +  ".jpg";
			imwrite(filename2,frame2);
		} else {
			cout << "Can not retrieve images." << endl;
            return;
		}
        //if(waitKey(30) >= 0) break;
	}

}

int main() {

	string str = getCurrentTime();
	int asize = 4;
	int fd;
	int pan_angles[asize] = {50,100,233,300};
	int tilt_angles[asize] = {65,90,133,280};

	VideoCapture cap(0); // open the default camera
    VideoCapture cap1(1); // open the default camera2
	if((!cap.isOpened()) || (!cap1.isOpened()) ) {  // check if we succeeded
        return -1;
	}
	
	const char *port_name = "/dev/ttyACM0";
	fd = open_serial(port_name);
	

	for (int x=0;x<1;x++){
		for (int p=0;p<asize;p++){

			printf("********Moving to Pan %d Tilt %d********\n", pan_angles[p], tilt_angles[p]);
			// Move to Pan angle.  First create command string (3 comma separated numbers).
			const char * pcommand = createCommand(1,0,pan_angles[p]).c_str();
			printf("sending command %s\n",pcommand);
			write_serial(fd, pcommand);
	
			unsigned char buf[10];
			int n = read (fd, buf, sizeof buf);
			//printf("Data = %s length = %d\n", buf, n);
		
			// Move to tilt angle.
			pcommand = createCommand(2,0,tilt_angles[p]).c_str();
			printf("sending command %s\n",pcommand);
			write_serial(fd, pcommand);
	
			n = read (fd, buf, sizeof buf);
			//printf("Data = %s length = %d\n", buf, n);

			//Take images for each camera
			takeImages(cap, cap1, 3);

		}	
	}


}
