#include "stdafx.h"
#include <string>
#include <iostream>
#include <vector>

#include <opencv2/core/utility.hpp>
#include "opencv2/core.hpp"     // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/tracking.hpp>
#include <opencv2/imgproc.hpp>  // Gaussian Blur
#include <opencv2/opencv_modules.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>  // OpenCV window I/O

#include "Joint.h"


using namespace std;
using namespace cv;

const std::string VIDEO1 = "E:\\Studia\\@PD\\green1.mp4";
const std::string VIDEO2 = "E:\\Studia\\@PD\\VID_1.avi";


// ------------------------------------
// Global Variables
// -----------------------------------

int iLowH = 30;
int iHighH = 65;

int iLowS = 80;
int iHighS = 255;

int iLowV = 50;
int iHighV = 255;

const int MIN_OBJECT_AREA = 25 * 25;

string intToString(int number) {


	stringstream ss;
	ss << number;
	return ss.str();
}

void morphOperations(Mat &thresh)
{
	// RECT is much faster than ELIPSE

	Mat erodeElem = getStructuringElement(MORPH_RECT, Size(3, 3));
	Mat dilateElem = getStructuringElement(MORPH_RECT, Size(8, 8));

	erode(thresh, thresh, erodeElem); // size matters for frame speed
	erode(thresh, thresh, erodeElem);

	dilate(thresh, thresh, dilateElem);
	dilate(thresh, thresh, dilateElem);

	// there are circles in good shape after two times of erode and dilates, but also small ones in between frames
	// slow performance
}

void drawJoints(vector<Joint> joints, Mat &frame)
{

	for (int i = 0; i < joints.size()-1; i++)
	{
		circle(frame, Point(joints.at(i).getX(), joints.at(i).getY()), 3, Scalar(255, 0, 0), 3, 8, 0);
		putText(frame, intToString(joints.at(i).getX()) + " , " + intToString(joints.at(i).getY()), Point(joints.at(i).getX() + 25, joints.at(i).getY() + 10), 1, 1, Scalar(0, 255, 0));
		line(frame, Point(joints.at(i).getX(), joints.at(i).getY()), Point(joints.at(i+1).getX(), joints.at(i+1).getY()), Scalar(255, 0, 0), 2);
	}

	circle(frame, Point(joints.back().getX(), joints.back().getY()), 3, Scalar(255, 0, 0), 3, 8, 0);
	putText(frame, intToString(joints.back().getX()) + " , " + intToString(joints.back().getY()), Point(joints.back().getX() + 25, joints.back().getY() + 10), 1, 1, Scalar(0, 255, 0));
}

void trackJoints(Mat &threshold, Mat &draw)
{
	vector< Joint> joints;
	Mat temp;
	threshold.copyTo(temp);
	// vectors for output
	vector< vector<Point> > contours;
	vector< Vec4i> hierarchy;

	// find contours
	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	// moments method to find filtered joints
	if (hierarchy.size() > 0)
	{
		int numOfObjects = hierarchy.size();
		for (int i = 0; i >= 0; i = hierarchy[i][0])
		{
			Moments moment = moments((Mat)contours[i]);
			double area = moment.m00;

			if (area > MIN_OBJECT_AREA)
			{
				Joint joint;
				joint.setX(moment.m10 / area);
				joint.setY(moment.m01 / area);

				joints.push_back(joint);
			}
		}
		drawJoints(joints, draw);
	}
}

int main()
{
	int frameNum = -1;
	VideoCapture video(VIDEO1);
	if (!video.isOpened())
	{
		cout << "Cant open file\n";
		return -1;
	}
	namedWindow("Threshold", CV_WINDOW_NORMAL);
	resizeWindow("Threshold", Size(640, 480));

	namedWindow("Joints", CV_WINDOW_NORMAL);
	resizeWindow("Joints", Size(640, 480));

	// create variables
	Mat frame; // to hold each frame from video
	Mat HSV; // to hold the converted frame
	Mat threshold; // to hold binary represetation of converted frame

	while (true) // read video frame by frame and make operations
	{
		video.read(frame);
		if (frame.empty())
		{
			cout << "The End\n";
			break;
		}

		// convert frame from BGR to HSV color representation
		cvtColor(frame, HSV, COLOR_BGR2HSV);
		// treshold converted frame
		inRange(HSV,Scalar(iLowH, iLowS, iLowV),Scalar(iHighH, iHighS, iHighV), threshold);

		// morphological operations to eliminate the background noise
		morphOperations(threshold);

		// findObjects
		trackJoints(threshold, frame);

		imshow("Threshold", threshold);
		imshow("Joints", frame);



		// break by "ESC"
		char c = (char)waitKey(25);
		if (c == 27)
			break;
		
	}

	return 0;
}