#include "stdafx.h"
#include "TrackingJoints.h"


TrackingJoints::TrackingJoints()
{
}


TrackingJoints::~TrackingJoints()
{
}

void TrackingJoints::initialize(vector<Joint> joints, Mat &frame)
{
	if (!isInitialized)
	{
		for (auto &joint : joints)
		{
			algorithms.push_back(TrackerKCF::create());
			objects.push_back(Rect2d(joint.getX()-20, joint.getY()-20, 40, 40));
		}
		trackers.add(algorithms, frame, objects);
		isInitialized = true;
	}
}
