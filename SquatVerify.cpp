#include "stdafx.h"
#include <opencv2\videoio.hpp>

#include "SquatVerify.h"
#include "Joint.h"
#include "TrackingJoints.h"

using namespace cv;

SquatVerify::SquatVerify()
{
}

SquatVerify::SquatVerify(VideoWriter vw)
{
	videoWriter = vw;
}


SquatVerify::~SquatVerify()
{
}



void SquatVerify::verify(Mat frame)
{
	Mat image = frame;
	Scalar wrong(0, 0, 255);
	Scalar good(0, 255, 0);
	Mat mask,result;

	if (1)
		mask = (image.rows, image.cols, CV_8UC3, good);
	else
		mask = (image.rows, image.cols, CV_8UC3, wrong);

	addWeighted(image, 0.5, mask, 0.5, 0, result, CV_8UC3);

	VideoWrite(result);
}

int SquatVerify::VideoWrite(Mat frame)
{

	if (!videoWriter.isOpened())
	{
		cout << "Cant open file to write\n";
		return -1;
	}
	this->videoWriter.write(frame);
	return 0;
}
