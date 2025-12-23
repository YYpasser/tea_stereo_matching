#include <iostream>
#include "../include/stereo.h"
#include "../include/calib.h"
#include "../include/utils.h"
#include "../include/camera.h"
#include "../include/logger.h"
#include <opencv2/opencv.hpp>

int main()
{
    cv::Mat leftImage = cv::imread("../demo-imgs/0600-Left.bmp");
    cv::Mat rightImage = cv::imread("../demo-imgs/0600-Right.bmp");
    stereo::StereoMatchingTensorRT smtrt;
    smtrt.loadEngine("../models/FFLO_it32.trt");
    auto disparity = smtrt.compute(leftImage, rightImage);
    auto disparity_color = stereo::applyMapping(disparity);
    cv::imshow("disparity", disparity_color);
    cv::waitKey(0);
    cv::imwrite("../demo-output/0600_FFLO.png", disparity_color);
	return 0;
}
