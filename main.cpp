#include "stdafx.h"
#include <string>
#include <iostream>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>

#include <opencv2/core/utility.hpp>
#include "opencv2/core.hpp"     // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/core/ocl.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/imgproc.hpp>  // Gaussian Blur
#include <opencv2/opencv_modules.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include "Joint.h"
#include "TrackingJoints.h"
#include "SquatVerify.h"
#include "main.h"


using namespace std;
using namespace cv;

const std::string VIDEO1 = "E:\\Studia\\@PD\\green1.mp4";
const std::string VIDEO2 = "E:\\Studia\\@PD\\squatVerify.avi";


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

vector<Point> previousSteps;

TrackingJoints tj; // figure out the name

string ToString(int number) {
	stringstream ss;
	ss << number;
	return ss.str();
}

string ToString(double number) {
	stringstream ss;
	ss << number;
	return ss.str();
}

void morphOperations(Mat &thresh)
{
	// RECT is much faster than ELIPSE

	Mat erodeElem = getStructuringElement(MORPH_RECT, Size(4, 4));
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
		putText(frame, ToString(joints.at(i).identifier), Point(joints.at(i).getX() + 10, joints.at(i).getY() + 10), 1, 1, Scalar(0, 255, 0));
		//putText(frame, ToString(joints.at(i).getX()) + " , " + ToString(joints.at(i).getY()), Point(joints.at(i).getX() + 25, joints.at(i).getY() + 10), 1, 1, Scalar(0, 255, 0));
		line(frame, Point(joints.at(i).getX(), joints.at(i).getY()), Point(joints.at(i+1).getX(), joints.at(i+1).getY()), Scalar(255, 0, 0), 2);
	}

	circle(frame, Point(joints.back().getX(), joints.back().getY()), 3, Scalar(255, 0, 0), 3, 8, 0);
	putText(frame, ToString(joints.back().getX()) + " , " + ToString(joints.back().getY()), Point(joints.back().getX() + 25, joints.back().getY() + 10), 1, 1, Scalar(0, 255, 0));
}

void drawCOG(Mat &frame, int p)
{
	arrowedLine(frame, Point(p + 20, 0), Point(p + 20, frame.cols), Scalar(255, 255, 0), 2);
}

void drawPath(vector<Joint> joints, Mat &frame)
{
	for (size_t i = 0; i < joints.size(); i++)
	{
		line(frame, Point(joints.at(i).getX(), joints.at(i).getY()), Point(joints.at(i).prevXpos, joints.at(i).prevYpos), Scalar(0, 0, 255));
	}
}

double calculateAngle(Joint j1, Joint j2, Joint j3)
{
	int x1 = j1.getX() - j2.getX();
	int y1 = j1.getY() - j2.getY();

	int x2 = j3.getX() - j2.getX();
	int y2 = j3.getY() - j2.getY();

	int dot = (x1 * x2) + (y1 * y2);
	int det = (x1 * y2) - (y1 * x2);

	return atan2(det, dot);
}

void trackJoints(Mat &threshold, Mat &draw)
{
	vector< Joint> joints;
	int detectedJoints = 0;
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

				joint.identifier = ++detectedJoints;

				joints.push_back(joint);
			}
		}
		//tj.initialize(joints, draw);
		if (detectedJoints == 5)
		{
			drawJoints(joints, draw);
			int kneeAngle = abs(calculateAngle(joints[0], joints[1], joints[2]) * 180 / M_PI);
			int hipAngle = abs(calculateAngle(joints[1], joints[2], joints[3]) * 180 / M_PI);
			putText(draw, ToString(kneeAngle), Point(joints[1].getX() + 25, joints[1].getY() + 15), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255),4,8,false);
			putText(draw, ToString(hipAngle), Point(joints[2].getX() - 35, joints[2].getY() + 15), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 4, 8, false);

			//drawCOG(draw, joints[0].getX());
		}
		//if (joints[0].prevXpos != 0 && joints[0].prevYpos != 0)
			//drawPath(joints, draw);

		for (auto &joint : joints)
		{
			joint.prevXpos = joint.getX();
			joint.prevYpos = joint.getY();
		}
	}
}

void initWindows();

int main()
{
	int frameNum = -1;
	VideoCapture video(VIDEO1);
	if (!video.isOpened())
	{
		cout << "Cant open file\n";
		return -1;
	}
	Size S = Size((int)video.get(CAP_PROP_FRAME_WIDTH),
		(int)video.get(CAP_PROP_FRAME_HEIGHT));
	SquatVerify squatVerify(VideoWriter(VIDEO2, CV_FOURCC('M', 'J', 'P', 'G'), video.get(CAP_PROP_FPS), S, true));
	
	//initialize the windows width and height
	initWindows();

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

		Mat skeleton = Mat::zeros(frame.size(), CV_8UC3);
		// convert frame from BGR to HSV color representation
		cvtColor(frame, HSV, COLOR_BGR2HSV);
		// treshold converted frame
		inRange(HSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), threshold);

		// morphological operations to eliminate the background noise
		morphOperations(threshold);

		// findObjects
		trackJoints(threshold, skeleton);

		// tracking algorithm update and drawing
		//tj.trackers.update(frame);
		//for (unsigned i = 0; i<tj.trackers.getObjects().size(); i++)
			//rectangle(frame, tj.trackers.getObjects()[i], Scalar(255, 0, 0), 2, 1);

		imshow("Threshold", threshold);
		//imshow("Joints", frame);

		imshow("Tracking", skeleton);

		// break by "ESC"
		char c = (char)waitKey(25);
		if (c == 27)
			break;
		++frameNum;
	}

	return 0;
}

void initWindows()
{
	namedWindow("Threshold", CV_WINDOW_NORMAL);
	resizeWindow("Threshold", Size(640, 480));

	namedWindow("Joints", CV_WINDOW_NORMAL);
	resizeWindow("Joints", Size(640, 480));

	namedWindow("Tracking", CV_WINDOW_NORMAL);
	resizeWindow("Tracking", Size(640, 480));
}
