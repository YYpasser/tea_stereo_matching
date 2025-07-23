#include "../include/stereo.h"
#include "../include/logger.h"
#include "../include/utils.h"
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

class stereo::EpipolarRectify::ERImpl
{
public:
	ERImpl();
	~ERImpl();
public:
	EpipolarRectifyMap m_rectifyMap;
	cv::Size m_imgsz;
};

stereo::EpipolarRectify::EpipolarRectify()
{
	this->impl = std::make_unique<EpipolarRectify::ERImpl>();
}

stereo::EpipolarRectify::EpipolarRectify(const EpipolarRectifyMap& rectifyMap, const cv::Size& imgsz)
{
	this->impl = std::make_unique<EpipolarRectify::ERImpl>();
	loadEpipolarRectifyMap(rectifyMap, imgsz);
}

stereo::EpipolarRectify::~EpipolarRectify()
{
}

void stereo::EpipolarRectify::loadEpipolarRectifyMap(const EpipolarRectifyMap& rectifyMap, const cv::Size& imgsz)
{
	LOG_INFO("Loading stereo epipolar rectify params...");
	if (rectifyMap.empty())
	{
		std::string msg = "stereo params is empty, please load it first";
		LOG_ERROR(msg);
		throw std::runtime_error(msg);
	}
	this->impl->m_rectifyMap = rectifyMap;
	this->impl->m_imgsz = imgsz;
	LOG_INFO("Loaded stereo epipolar rectify params!");
}

StereoPair<cv::Mat> stereo::EpipolarRectify::rectify(const cv::Mat& stereoImage)
{
	if (this->impl->m_rectifyMap.empty())
	{
		LOG_ERROR("Stereo epipolar rectify params is empty, please load it first.");
		return StereoPair<cv::Mat>();
	}
	if (stereoImage.empty())
	{
		LOG_ERROR("Stereo image is empty.");
		return StereoPair<cv::Mat>();
	}

	cv::Mat left = stereoImage(cv::Rect(0, 0, this->impl->m_imgsz.width, this->impl->m_imgsz.height)).clone();
	cv::Mat right = stereoImage(cv::Rect(this->impl->m_imgsz.width, 0, this->impl->m_imgsz.width, this->impl->m_imgsz.height)).clone();
	return this->rectify(left, right);
}

StereoPair<cv::Mat> stereo::EpipolarRectify::rectify(const cv::Mat& leftImage, const cv::Mat& rightImage)
{
	if (this->impl->m_rectifyMap.empty())
	{
		LOG_ERROR("Stereo epipolar rectify params is empty, please load it first.");
		return StereoPair<cv::Mat>();
	}
	if (leftImage.empty() || rightImage.empty())
	{
		LOG_ERROR("Left or Right image is empty.");
	}

	cv::Mat rectifiedLeft, rectifiedRight;
	cv::remap(leftImage, rectifiedLeft, this->impl->m_rectifyMap.map00, this->impl->m_rectifyMap.map01, cv::INTER_LINEAR);
	cv::remap(rightImage, rectifiedRight, this->impl->m_rectifyMap.map10, this->impl->m_rectifyMap.map11, cv::INTER_LINEAR);
	return StereoPair<cv::Mat>(rectifiedLeft, rectifiedRight);
}

StereoPair<cv::Mat> stereo::EpipolarRectify::rectify(const StereoPair<cv::Mat>& stereoImagePair)
{
	if (this->impl->m_rectifyMap.empty())
	{
		LOG_ERROR("Stereo epipolar rectify params is empty, please load it first.");
		return StereoPair<cv::Mat>();
	}
	if (stereoImagePair.left.empty() || stereoImagePair.right.empty())
	{
		LOG_ERROR("Left or Right image is empty.");
	}
	return this->rectify(stereoImagePair.left, stereoImagePair.right);
}

void stereo::EpipolarRectify::rectify(const std::string& stereoPattern, const bool& recursive)
{
	if (this->impl->m_rectifyMap.empty())
	{
		LOG_ERROR("Stereo epipolar rectify params is empty, please load it first.");
		return;
	}
	if (stereoPattern.empty())
	{
		LOG_ERROR("Stereo image path is empty.");
		return;
	}

	auto stereoPathList = utils::glob(stereoPattern, recursive);
	size_t cnt = stereoPathList.size();
	LOG_INFO("Find stereo images = " + std::to_string(cnt) + ".");
	if (cnt == 0)
		return;
	//-- 设置保存路径
	size_t upperDirSNum = std::string::npos;
	size_t pos_slash = stereoPathList[0].rfind('/');
	size_t pos_backslash = stereoPathList[0].rfind('\\');

	// 处理正斜杠情况
	if (pos_slash != std::string::npos) {
		upperDirSNum = pos_slash;
	}

	// 处理反斜杠情况（如果反斜杠位置更靠后，则更新结果）
	if (pos_backslash != std::string::npos) {
		if (upperDirSNum == std::string::npos || pos_backslash > upperDirSNum) {
			upperDirSNum = pos_backslash;
		}
	}
	std::string upperDir = stereoPathList[0].substr(0, upperDirSNum);//图像保存路径上一级
	size_t dirLength = upperDir.length();//图像保存路径
	std::string leftRectifyDir = upperDir + "/rectify/left";
	std::string rightRectifyDir = upperDir + "/rectify/right";
	utils::generateNewFolder(leftRectifyDir);
	utils::generateNewFolder(rightRectifyDir);

	LOG_INFO("Rectifying stereo images...");
	for (auto it = stereoPathList.begin(); it != stereoPathList.end(); ++it)
	{
		auto i = std::distance(stereoPathList.begin(), it);
		LOG_INFO("Processing [" + std::to_string(i + 1) + " / " + std::to_string(cnt) + "] ...");

		size_t imageDirLength = it->length();//图像完整路径
		std::string nameWithFormat = it->substr(dirLength + 1, imageDirLength);//图像名
		size_t pos = nameWithFormat.rfind(".");
		std::string name = nameWithFormat.substr(0, pos);
		std::string format = nameWithFormat.substr(pos + 1, nameWithFormat.length());
		name = name.substr(0, name.find('-'));

		auto result = rectify(cv::imread(*it));
		std::string leftFullDir = leftRectifyDir + "/" + name + "." + format;
		std::string rightFullDir = rightRectifyDir + "/" + name + "." + format;
		cv::imwrite(leftFullDir, result.left);
		cv::imwrite(rightFullDir, result.right);
	}
	LOG_INFO("Rectified stereo images!");
}

stereo::EpipolarRectify::ERImpl::ERImpl()
{
}

stereo::EpipolarRectify::ERImpl::~ERImpl()
{
}
