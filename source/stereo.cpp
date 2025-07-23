#include "../include/stereo.h"
#include "../include/logger.h"
#include "../include/utils.h"
#include <limits>
#include <fstream>
#include <charconv>
#include <opencv2/opencv.hpp>

cv::Mat stereo::reprojectTo3D(const cv::Mat& disparity, const cv::Mat& QMatrix)
{
	cv::Mat u = cv::Mat::zeros(disparity.size(), CV_32FC1);
	cv::Mat v = cv::Mat::zeros(disparity.size(), CV_32FC1);
	auto uIt = u.begin<float>();
	auto vIt = v.begin<float>();
	for (; uIt != u.end<float>(); ++uIt, ++vIt)
	{
		*uIt = static_cast<float>(uIt.pos().x);
		*vIt = static_cast<float>(vIt.pos().y);
	}
	cv::Mat u_flat = u.reshape(0, 1);
	cv::Mat v_flat = v.reshape(0, 1);
	cv::Mat disparity_flat = disparity.reshape(0, 1);
	cv::Mat ones = cv::Mat::ones(disparity.size(), CV_32FC1);
	cv::Mat ones_flat = ones.reshape(0, 1);
	std::vector<cv::Mat> pixel_flat_vec = { u_flat, v_flat, disparity_flat, ones_flat };
	cv::Mat pixel_flat;
	cv::vconcat(pixel_flat_vec, pixel_flat);
	cv::Mat Q_float32;
	QMatrix.convertTo(Q_float32, CV_32F);
	cv::Mat xyzw_flat = Q_float32 * pixel_flat;
	cv::Mat xyz_flat = xyzw_flat(cv::Rect(0, 0, xyzw_flat.cols, 3)).clone();
	cv::Mat w_flat = xyzw_flat(cv::Rect(0, 3, xyzw_flat.cols, 1)).clone();
	xyz_flat.row(0) /= w_flat;
	xyz_flat.row(1) /= w_flat;
	xyz_flat.row(2) /= w_flat;
	cv::Mat x = xyz_flat.row(0).reshape(0, disparity.rows);
	cv::Mat y = xyz_flat.row(1).reshape(0, disparity.rows);
	cv::Mat z = xyz_flat.row(2).reshape(0, disparity.rows);
	cv::Mat xyz;
	cv::merge(std::vector<cv::Mat>{x, y, z}, xyz);
	return xyz;
}

std::string stereo::mapTypeToString(MapType mapType)
{
	switch (mapType)
	{
	case MapType::JET: return "JET";
	default:           return "UNKNOWN";
	}
}

cv::Mat stereo::JETMap()
{
	cv::Mat colorMap(1, 256, CV_8UC3);
	for (int i = 0; i < 32; ++i)
		colorMap.at<cv::Vec3b>(i) = cv::Vec3b(128 + 4 * i, 0, 0);
	colorMap.at<cv::Vec3b>(32) = cv::Vec3b(255, 0, 0);
	for (int i = 0; i < 63; ++i)
		colorMap.at<cv::Vec3b>(33 + i) = cv::Vec3b(255, 4 + 4 * i, 0);
	colorMap.at<cv::Vec3b>(96) = cv::Vec3b(254, 255, 2);
	for (int i = 0; i < 62; ++i)
		colorMap.at<cv::Vec3b>(97 + i) = cv::Vec3b(250 - 4 * i, 255, 6 + 4 * i);
	colorMap.at<cv::Vec3b>(159) = cv::Vec3b(1, 255, 254);
	for (int i = 0; i < 64; ++i)
		colorMap.at<cv::Vec3b>(160 + i) = cv::Vec3b(0, 252 - 4 * i, 255);
	for (int i = 0; i < 32; ++i)
		colorMap.at<cv::Vec3b>(224 + i) = cv::Vec3b(0, 0, 252 - 4 * i);
	return colorMap;
}

cv::Mat stereo::applyMapping(const cv::Mat& image, const MapType& mapType)
{
	cv::Mat map;
	switch (mapType)
	{
	case MapType::JET:
		map = JETMap();
		LOG_INFO("Applying MINMAX_LINEAR + " + mapTypeToString(mapType) + " mapping to image...");
		break;
	default:
		LOG_ERROR("Unknown mapping type: " + mapTypeToString(mapType));
		return cv::Mat();
	}

	auto min_val = std::numeric_limits<float>::infinity();
	auto max_val = -std::numeric_limits<float>::infinity();
	for (auto imageIt = image.begin<float>(); imageIt != image.end<float>(); ++imageIt)
	{
		if (*imageIt < 0 || std::isinf(*imageIt))
			continue;
		min_val = std::min(min_val, *imageIt);
		max_val = std::max(max_val, *imageIt);
	}

	cv::Mat image_uint8 = cv::Mat::zeros(image.size(), CV_8UC3);
	auto imageIt_uint8 = image_uint8.begin<cv::Vec3b>();
	for (auto imageIt = image.begin<float>(); imageIt != image.end<float>(); ++imageIt, ++imageIt_uint8)
	{
		if (*imageIt < 0)
		{
			*imageIt_uint8 = cv::Vec3b(0, 0, 0);
			continue;
		}
		*imageIt_uint8 = map.at<cv::Vec3b>(static_cast<unsigned char>(
			((*imageIt - min_val) / (max_val - min_val)) * 255));
	}
	LOG_INFO("Mapping applied successfully!");
	return image_uint8;
}

cv::Mat stereo::applyMapping(const cv::Mat& image, const float& minValue, const float& maxValue, const MapType& mapType)
{
	if (minValue >= maxValue || minValue * maxValue < 0)
	{
		LOG_ERROR("Invalid min/max value range: " + std::to_string(minValue) + " - " + std::to_string(maxValue));
		return cv::Mat();
	}

	cv::Mat map;
	switch (mapType)
	{
	case MapType::JET:
		map = JETMap();
		LOG_INFO("Applying MINMAX_LINEAR + " + mapTypeToString(mapType) + " mapping to image...");
		break;
	default:
		LOG_ERROR("Unknown mapping type: " + mapTypeToString(mapType));
		return cv::Mat();
	}

	cv::Mat image_uint8 = cv::Mat::zeros(image.size(), CV_8UC3);
	auto imageIt_uint8 = image_uint8.begin<cv::Vec3b>();
	for (auto imageIt = image.begin<float>(); imageIt != image.end<float>(); ++imageIt, ++imageIt_uint8)
	{
		if (*imageIt < minValue || *imageIt > maxValue)
		{
			*imageIt_uint8 = cv::Vec3b(0, 0, 0);
			continue;
		}
		*imageIt_uint8 = map.at<cv::Vec3b>(static_cast<unsigned char>(
			((*imageIt - minValue) / (maxValue - minValue)) * 255));
	}
	LOG_INFO("Mapping applied successfully!");
	return image_uint8;
}

static const std::vector<cv::Scalar> colorMap
{
	cv::Scalar(0, 255,   0),
	cv::Scalar(255,   0,   0),
	cv::Scalar(0, 255, 255),
	cv::Scalar(255,   0, 255),
	cv::Scalar(255, 255,   0),
	cv::Scalar(0,   0, 255)
};

cv::Mat stereo::drawHorizontalLines(const cv::Mat& stereoImage)
{
	auto lineGap = 20;
	auto height = stereoImage.rows;
	auto width = stereoImage.cols;
	cv::Mat srcc = stereoImage.clone();
	for (int i = 0; i < height / lineGap; ++i)
	{
		cv::Scalar color = colorMap[i % colorMap.size()];
		cv::line(srcc, cv::Point(0, i * lineGap), cv::Point(width, i * lineGap), color, 1, cv::LINE_AA);
	}
	return srcc;
}

cv::Mat stereo::drawHorizontalLines(const cv::Mat& leftImage, const cv::Mat& rightImage)
{
	cv::Mat stereoImage = concatStereoImage(leftImage, rightImage);
	cv::hconcat(leftImage, rightImage, stereoImage);
	auto lineGap = 20;
	auto height = stereoImage.rows;
	auto width = stereoImage.cols;
	for (int i = 0; i < height / lineGap; ++i)
	{
		cv::Scalar color = colorMap[i % colorMap.size()];
		cv::line(stereoImage, cv::Point(0, i * lineGap), cv::Point(width, i * lineGap), color, 1, cv::LINE_AA);
	}
	return stereoImage;
}

cv::Mat stereo::drawHorizontalLines(const StereoPair<cv::Mat>& stereoImagePair)
{
	cv::Mat stereoImage = concatStereoImage(stereoImagePair.left, stereoImagePair.right);
	auto lineGap = 20;
	auto height = stereoImage.rows;
	auto width = stereoImage.cols;
	for (int i = 0; i < height / lineGap; ++i)
	{
		cv::Scalar color = colorMap[i % colorMap.size()];
		cv::line(stereoImage, cv::Point(0, i * lineGap), cv::Point(width, i * lineGap), color, 1, cv::LINE_AA);
	}
	return stereoImage;
}

cv::Mat stereo::drawVerticalLines(const cv::Mat& stereoImage)
{
	auto height = stereoImage.rows;
	auto width = stereoImage.cols;
	cv::Mat left = stereoImage(cv::Rect(0, 0, width / 2, height)).clone();
    cv::Mat right = stereoImage(cv::Rect(width / 2, 0, width / 2, height)).clone();
	cv::Mat concat;
    cv::vconcat(left, right, concat);
	height = concat.rows;
    width = concat.cols;
	auto lineGap = 20;
	for (int i = 0; i < width / lineGap; ++i)
	{
		cv::Scalar color = colorMap[i % colorMap.size()];
		cv::line(concat, cv::Point(i * lineGap, 0), cv::Point(i * lineGap, height), color, 1, cv::LINE_AA);
	}
	return concat;
}

cv::Mat stereo::drawVerticalLines(const cv::Mat& leftImage, const cv::Mat& rightImage)
{
	cv::Mat concat;
	cv::vconcat(leftImage, rightImage, concat);
	auto height = concat.rows;
	auto width = concat.cols;
	auto lineGap = 20;
	for (int i = 0; i < width / lineGap; ++i)
	{
		cv::Scalar color = colorMap[i % colorMap.size()];
		cv::line(concat, cv::Point(i * lineGap, 0), cv::Point(i * lineGap, height), color, 1, cv::LINE_AA);
	}
	return concat;
}

cv::Mat stereo::drawVerticalLines(const StereoPair<cv::Mat>& stereoImagePair)
{
	cv::Mat concat;
	cv::vconcat(stereoImagePair.left, stereoImagePair.right, concat);
	auto height = concat.rows;
	auto width = concat.cols;
	auto lineGap = 20;
	for (int i = 0; i < width / lineGap; ++i)
	{
		cv::Scalar color = colorMap[i % colorMap.size()];
		cv::line(concat, cv::Point(i * lineGap, 0), cv::Point(i * lineGap, height), color, 1, cv::LINE_AA);
	}
	return concat;
}

cv::Mat stereo::concatStereoImage(const cv::Mat& leftImage, const cv::Mat& rightImage)
{
	cv::Mat stereoImage;
	cv::hconcat(leftImage, rightImage, stereoImage);
	return stereoImage;
}

StereoPair<cv::Mat> stereo::splitStereoImage(const cv::Mat& stereoImage)
{
	cv::Mat leftImage = stereoImage(cv::Rect(0, 0, stereoImage.cols / 2, stereoImage.rows)).clone();
	cv::Mat rightImage = stereoImage(cv::Rect(stereoImage.cols / 2, 0, stereoImage.cols / 2, stereoImage.rows)).clone();
	return StereoPair<cv::Mat>(leftImage, rightImage);
}

static void writePCD(const std::vector<cv::Point3f>& p, const std::vector<cv::Vec3b>& c, const std::string& pcdPath)
{
	std::ofstream pc(pcdPath);
	pc << "# .PCD v0.7 - Point Cloud Data file format\n";
	pc << "VERSION 0.7\n";
	pc << "FIELDS x y z rgb\n";
	pc << "SIZE 4 4 4 4\n";
	pc << "TYPE F F F U\n";
	pc << "COUNT 1 1 1 1\n";
	pc << "WIDTH " << p.size() << "\n";
	pc << "HEIGHT 1\n";
	pc << "VIEWPOINT 0 0 0 1 0 0 0\n";
	pc << "POINTS " << p.size() << "\n";
	pc << "DATA ascii\n";

	size_t len = p.size();
	std::unique_ptr<char[]> buffer{ new char[64 * len] };
	char* curr = buffer.get();
	char* end = buffer.get() + 64 * len;
	auto rgbIt = c.begin();
	auto xyzIt = p.begin();
	for (; xyzIt != p.end(); ++rgbIt, ++xyzIt)
	{
		curr = std::to_chars(curr, end, xyzIt->x).ptr; //>=C++17
		*curr++ = ' ';
		curr = std::to_chars(curr, end, xyzIt->y).ptr;
		*curr++ = ' ';
		curr = std::to_chars(curr, end, xyzIt->z).ptr;
		*curr++ = ' ';
		uint rgb = (*rgbIt)[2] << 16 | (*rgbIt)[1] << 8 | (*rgbIt)[0] | 1 << 24;
		curr = std::to_chars(curr, end, rgb).ptr;
		*curr++ = '\n';
	}
	pc.write(buffer.get(), curr - buffer.get());
	pc.close();
}

void stereo::writePointCloudToPCD(const std::string& pcdPath, const cv::Mat& rgbImage, cv::Mat& pointCloud)
{
	if (rgbImage.empty() or pointCloud.empty() or pcdPath.empty())
	{
		std::string errstr = "Empty input.";
		LOG_ERROR(errstr);
		throw std::runtime_error(errstr);
	}

	LOG_INFO("Writing point cloud to PCD file...");
	std::vector<cv::Point3f> p;
	std::vector<cv::Vec3b> c;
	auto xyzIt = pointCloud.begin<cv::Vec3f>();
	auto rgbIt = rgbImage.begin<cv::Vec3b>();
	for (; xyzIt != pointCloud.end<cv::Vec3f>(); ++xyzIt, ++rgbIt)
	{
		if ((*xyzIt)[0] == std::numeric_limits<float>::infinity() or (*xyzIt)[1] == std::numeric_limits<float>::infinity()
			or (*xyzIt)[2] == std::numeric_limits<float>::infinity())
			continue;
		p.push_back(*xyzIt);
		c.push_back(*rgbIt);
	}

	auto start = std::chrono::steady_clock::now();
	writePCD(p, c, pcdPath);
	auto end = std::chrono::steady_clock::now();
	auto tt = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	LOG_INFO("Write Done. Points: " + std::to_string(p.size()) + ". Timing: " + utils::formatMilliseconds(tt.count() / 1000.0) + " ms.");
}
