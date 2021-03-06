// opencv.cpp : Defines the entry point for the console application.
//

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

vector<Mat> spl;
Mat3b bgr, hsv;
Mat mask1, mask2, mask;
Mat frame, frame2;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

/// Video Tracking variables
Mat trackFrame;

void findObj();
Mat thresh_callback(int, void*);
void houghTransform();

int bam()
{

	int frameNum = -1;
	VideoCapture vc(VIDEO1);
	if (!vc.isOpened())
	{
		cout << "Cant open file\n";
		return -1;
	}
	Mat tmp;
	vc.read(tmp);
	Mat imgLines = Mat::zeros(tmp.size(), CV_8UC3);;
#pragma region Control Window

	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

	int iLowH = 30;
	int iHighH = 65;

	int iLowS = 80;
	int iHighS = 255;

	int iLowV = 50;
	int iHighV = 255;


	//Create trackbars in "Control" window
	createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &iHighH, 179);

	createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &iHighS, 255);

	createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
	createTrackbar("HighV", "Control", &iHighV, 255);
#pragma endregion

	int iLastX = -1;
	int iLastY = -1;

	int ex = (int)vc.get(CAP_PROP_FOURCC);

	Size S = Size((int)vc.get(CAP_PROP_FRAME_WIDTH),
		(int)vc.get(CAP_PROP_FRAME_HEIGHT));
	cout << "Frame resolution : Width :" << S.width << " Height :" << S.height << " of nr#: " << vc.get(CAP_PROP_FRAME_COUNT) << endl;

	namedWindow("this", WINDOW_NORMAL);
	resizeWindow("this", 800, 800);
	namedWindow("mask", WINDOW_NORMAL);
	resizeWindow("mask", 600, 600);
	VideoWriter vw(VIDEO2, CV_FOURCC('M', 'J', 'P', 'G'), vc.get(CAP_PROP_FPS), S, true);
	if (!vw.isOpened())
	{
		cout << "Cant open file to write\n";
		return -1;
	}
#pragma region Trackers
	/* one  tracker
	// create a tracker object
	Ptr<Tracker> tracker = TrackerKCF::create();

	/// get bounding box
	vc >> trackFrame;
	Rect2d ROIs;
	ROIs = selectROI("tracker", frame);

	/// quit when the tracked object(s) is not provided
	//quit if ROI was not selected
	if (ROIs.width == 0 || ROIs.height == 0)
		return 0;

	// initialize the tracker
	tracker->init(frame, ROIs);
	*/
	/*
	string trackingAlg = "KCF";
	MultiTracker trackers;

	/// container of the tracked objects
	vector<Rect2d> objects;

	/// get bounding box
	vc >> trackFrame;
	vector<Rect> ROIs;
	selectROIs("tracker", frame, ROIs);

	/// quit when the tracked object(s) is not provided
	if (ROIs.size()<1)
		return 0;

	/// initialize the tracker
	std::vector<Ptr<Tracker> > algorithms;
	for (size_t i = 0; i < ROIs.size(); i++)
	{
		algorithms.push_back(TrackerKCF::create());
		objects.push_back(ROIs[i]);
	}
	
	
	trackers.add(algorithms, frame, objects);
	*/
#pragma endregion
	while (true)
	{
		vc.read(frame);
		if (frame.empty())
		{
			cout << "The End\n";
			break;
		}

		cvtColor(frame, hsv, CV_BGR2HSV);
		inRange(hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), mask2);

		///morphological opening (remove small objects from the foreground)
		erode(mask2, mask2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(mask2, mask2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		dilate(mask2, mask2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(mask2, mask2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
		//morphologyEx(mask2, mask2, cv::MORPH_OPEN, str_el);
		//morphologyEx(mask2, mask2, cv::MORPH_CLOSE, str_el);

		/*
		//Calculate the moments of the thresholded image
		Moments oMoments = moments(mask2);

		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;

		// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
		if (dArea > 10000)
		{
			//calculate the position of the ball
			int posX = dM10 / dArea;
			int posY = dM01 / dArea;

			if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
			{
				//Draw a red line from the previous point to the current point
				line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0, 0, 255), 2);
			}

			iLastX = posX;
			iLastY = posY;
		}
		*/ 

		//imshow("mask", mask2);

		//frame = frame + imgLines;
		//imshow("this", frame);
		
		

		//imshow("this", frame);
		Mat drawing = thresh_callback(0, 0);
		vw.write(drawing);

		imshow("this", frame + drawing);

		char c = (char)waitKey(25);
		if (c == 27)
			break;
		cout << "Frame #" << ++frameNum << endl;
	}
	
    return 0;
}

Mat thresh_callback(int, void *)
{
	Mat canny_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	/// Detect edges using canny
	Canny(mask2, canny_output, thresh, thresh * 2, 3);
	/// Find contours
	findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	/// Get the moments
	vector<Moments> mu(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		mu[i] = moments(contours[i], false);
	}

	///  Get the mass centers:
	vector<Point2f> mc(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}

	/// Draw contours
	Mat drawing = Mat::zeros(mask2.size(), CV_8UC3);
	for (int i = 0; i< contours.size()-1; i++)
	{
		cv::Scalar red(255, 0, 0);
		//drawContours(drawing, contours, i, red, 2, 8, hierarchy, 0, Point());
		circle(drawing, mc[i], 12, red, 3, 8, 0);
		cv::line(drawing, mc[i], mc[i+1], Scalar(255, 0, 0), 2);
	}

	/// Show in a window
	namedWindow("Contours", CV_WINDOW_NORMAL);
	resizeWindow("Contours", 800, 800);
	imshow("Contours", drawing);
	return drawing;
}

void houghTransform()
{
	/// Convert it to gray
	cvtColor(frame, frame2, CV_BGR2GRAY);

	/// Reduce the noise so we avoid false circle detection
	GaussianBlur(frame2, frame2, Size(9, 9), 2, 2);

	vector<Vec3f> circles;

	/// Apply the Hough Transform to find the circles
	HoughCircles(frame2, circles, CV_HOUGH_GRADIENT, 1, frame2.rows / 8, 200, 100, 0, 0);

	/// Draw the circles detected
	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// circle center
		circle(frame, center, 3, Scalar(0, 255, 0), -1, 8, 0);
		// circle outline
		circle(frame, center, radius, Scalar(0, 0, 255), 3, 8, 0);
	}

	/// Show your results
	namedWindow("Hough Circle Transform Demo", CV_WINDOW_NORMAL);
	resizeWindow("Hough Circle Transform Demo", 800, 800);
	imshow("Hough Circle Transform Demo", frame);
}

/*void findObj()
{
vector<vector<cv::Point> > contours;
vector<cv::Vec4i> heirarchy;
vector<cv::Point2i> center;
vector<int> radius;

cv::findContours(mask2.clone(), contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

size_t count = contours.size();

for (int i = 0; i<count; i++)
{
cv::Point2f c;
float r;
cv::minEnclosingCircle(contours[i], c, r);

if (r >= 3)
{
center.push_back(c);
radius.push_back(r);
}
}
size_t count = center.size();
cv::Scalar red(255, 0, 0);

for (int i = 0; i < count; i++)
{
cv::circle(mask2, center[i], radius[i], red, 3);
}
}*/
