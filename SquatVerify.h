#pragma once
class SquatVerify
{
public:
	SquatVerify();
	SquatVerify(VideoWriter);
	~SquatVerify();


	VideoWriter videoWriter;
	int minKneeAngle, maxKneeAngle;
	int minHipAngle, maxHipAngle;

	const std::string VIDEO2 = "E:\\Studia\\@PD\\squatVerify.avi";

	void verify(Mat);
	int VideoWrite(Mat);
};

