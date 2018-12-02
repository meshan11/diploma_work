#pragma once

#include "Joint.h"	 
#include <vector>
#include <opencv2/tracking.hpp>


using namespace std;
using namespace cv;

class TrackingJoints
{
public:
	TrackingJoints();
	~TrackingJoints();

	bool isInitialized = false;
	MultiTracker trackers;
	vector<Ptr<Tracker>> algorithms;
	vector<Rect2d> objects;

	void initialize(vector<Joint>, Mat &frame);
};

