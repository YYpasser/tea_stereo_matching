#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include "./include/calib.h"
#include "./include/stereo.h"
#include "./include/stereo_utils.h"
#include "./include/utils.h"
#include "./include/timer.h"
#include "./include/camera.h"
#include "./include/camera_utils.h"
#include <chrono>

using namespace std::literals;

int main()
{	
	stereo::TensorRTInference trt;
	trt.loadModel("../models/RTFFLONetDynamic.trt");
	std::vector<std::string> leftImages, rightImages;
	cv::glob("../demo-imgs/*Left.png", leftImages, false);
	cv::glob("../demo-imgs/*Right.png", rightImages, false);
	if (leftImages.size() != rightImages.size())
	{
		std::cerr << "The number of left and right images must be the same." << std::endl;
		return -1;
	}
	auto leftIt = leftImages.begin();
	auto rightIt = rightImages.begin();
	for (; leftIt != leftImages.end(); ++leftIt, ++rightIt)
	{
		cv::Mat leftImage = cv::imread(*leftIt);
		cv::Mat rightImage = cv::imread(*rightIt);
		cv::Mat disparity;
		trt.compute(leftImage, rightImage, disparity);
		cv::Mat disparityColor;
		stereo::applyColorMap(disparity, disparityColor, stereo::JETColorMap());
		cv::imshow("disparity", disparityColor);
		cv::waitKey(0);
	}
    return 0;
}
