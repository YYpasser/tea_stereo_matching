#include "../include/stereo.h"
#include "../include/utils.h"
#include "../include/logger.h"
#include <opencv2/opencv.hpp>
#include <limits>
#include <algorithm>
#include <fstream>
#include <charconv>


void stereo::hconcat(const cv::Mat& leftImage, const cv::Mat& rightImage, cv::Mat& stereoImage)
{
    cv::hconcat(leftImage, rightImage, stereoImage);
}

void stereo::vconcat(const cv::Mat& topImage, const cv::Mat& bottomImage, cv::Mat& stereoImage)
{
    cv::vconcat(topImage, bottomImage, stereoImage);
}

void stereo::hsplit(const cv::Mat& stereoImage, cv::Mat& leftImage, cv::Mat& rightImage)
{
    int rows = stereoImage.rows;
    int cols = stereoImage.cols / 2;
    leftImage = stereoImage(cv::Rect(0, 0, cols, rows)).clone();
    rightImage = stereoImage(cv::Rect(cols, 0, cols, rows)).clone();
}

void stereo::vsplit(const cv::Mat& stereoImage, cv::Mat& topImage, cv::Mat& bottomImage)
{
    int rows = stereoImage.rows / 2;
    int cols = stereoImage.cols;
    topImage = stereoImage(cv::Rect(0, 0, cols, rows)).clone();
    bottomImage = stereoImage(cv::Rect(0, rows, cols, rows)).clone();
}

static const std::vector<cv::Scalar> colorMap
{
    cv::Scalar(  0, 255,   0),
    cv::Scalar(255,   0,   0),
    cv::Scalar(  0, 255, 255),
    cv::Scalar(255,   0, 255),
    cv::Scalar(255, 255,   0),
    cv::Scalar(  0,   0, 255)
};

cv::Mat stereo::drawHorizontalLine(const cv::Mat& stereoImage)
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

cv::Mat stereo::drawVerticalLine(const cv::Mat& stereoImage)
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

cv::Mat stereo::JETColorMap()
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

void stereo::applyColorMap(const cv::Mat& src, cv::Mat& dst, const cv::Mat& colorMap)
{
    auto minVal = std::numeric_limits<float>::infinity();
    auto maxVal = -std::numeric_limits<float>::infinity();
    for (auto imageIt = src.begin<float>(); imageIt != src.end<float>(); ++imageIt)
    {
        if (*imageIt < 0 || std::isinf(*imageIt))
            continue;
        minVal = std::min(minVal, *imageIt);
        maxVal = std::max(maxVal, *imageIt);
    }

    dst = cv::Mat::zeros(src.size(), CV_8UC3);
    auto imageIt_uint8 = dst.begin<cv::Vec3b>();
    for (auto imageIt = src.begin<float>(); imageIt != src.end<float>(); ++imageIt, ++imageIt_uint8)
    {
        if (*imageIt < 0)
        {
            *imageIt_uint8 = cv::Vec3b(0, 0, 0);
            continue;
        }
        *imageIt_uint8 = colorMap.at<cv::Vec3b>(static_cast<unsigned char>(
            ((*imageIt - minVal) / (maxVal - minVal)) * 255));
    }
}

void stereo::applyColorMap(const cv::Mat& src, cv::Mat& dst, float minVal, float maxVal, const cv::Mat& colorMap)
{
    dst = cv::Mat::zeros(src.size(), CV_8UC3);
    auto imageIt_uint8 = dst.begin<cv::Vec3b>();
    for (auto imageIt = src.begin<float>(); imageIt != src.end<float>(); ++imageIt, ++imageIt_uint8)
    {
        if (*imageIt < minVal || *imageIt > maxVal)
        {
            *imageIt_uint8 = cv::Vec3b(0, 0, 0);
            continue;
        }
        *imageIt_uint8 = colorMap.at<cv::Vec3b>(static_cast<unsigned char>(
            ((*imageIt - minVal) / (maxVal - minVal)) * 255));
    }
}

void stereo::reprojectToDepth(const cv::Mat& disparity, float focalLength, float baseline, cv::Mat& depth)
{
    depth = cv::Mat::zeros(disparity.size(), CV_32FC1);
    auto dispIt = disparity.begin<float>();
    auto depthIt = depth.begin<float>();
    auto fb = focalLength * baseline;
    for (; dispIt != disparity.end<float>(); ++dispIt, ++depthIt)
    {
        if (*dispIt < 0 || std::isinf(*dispIt))
            continue;
        *depthIt = fb / *dispIt;
    }
}

void stereo::reprojectTo3D(const cv::Mat& disparity, float focalLength, float baseline, float cx, float cy, cv::Mat& XYZPoints)
{
    XYZPoints = cv::Mat::zeros(disparity.size(), CV_32FC3);
    auto fb = focalLength * baseline;
    for (auto v = 0; v < disparity.rows; ++v)
    {
        auto XYZPointsRow = XYZPoints.ptr<cv::Vec3f>(v);
        for (auto u = 0; u < disparity.cols; ++u)
        {
            auto d = disparity.at<float>(v, u);
            if (d < 0 || std::isinf(d))
                continue;
            auto Z = fb / d;
            auto Z_div_f = Z / focalLength;
            auto X = (u - cx) * Z_div_f;
            auto Y = (v - cy) * Z_div_f;
            XYZPointsRow[u] = cv::Vec3f(X, Y, Z);
        }
    }
}

void stereo::reprojectTo3D(const cv::Mat& disparity, const cv::Mat& Q, cv::Mat& XYZPoints)
{
    cv::Mat u = cv::Mat::zeros(disparity.size(), CV_32FC1);
    cv::Mat v = cv::Mat::zeros(disparity.size(), CV_32FC1);
    for (auto uIt = u.begin<float>(), vIt = v.begin<float>();
        uIt != u.end<float>(); ++uIt, ++vIt)
    {
        *uIt = static_cast<float>(uIt.pos().x);
        *vIt = static_cast<float>(vIt.pos().y);
    }

    cv::Mat u_flat = u.reshape(0, 1);
    cv::Mat v_flat = v.reshape(0, 1);
    cv::Mat disparity_flat = disparity.reshape(0, 1);
    cv::Mat ones_flat = cv::Mat::ones(disparity_flat.size(), CV_32FC1);
    cv::Mat pixel_flat;
    cv::vconcat(std::move(std::vector<cv::Mat>{ u_flat, v_flat, disparity_flat, ones_flat }), pixel_flat);
    
    cv::Mat Q_float32;
    Q.convertTo(Q_float32, CV_32F);
    cv::Mat xyzw_flat = Q_float32 * pixel_flat;
    cv::Mat xyz_flat = xyzw_flat(cv::Rect(0, 0, xyzw_flat.cols, 3)).clone();
    cv::Mat w_flat = xyzw_flat(cv::Rect(0, 3, xyzw_flat.cols, 1)).clone();
    xyz_flat.row(0) /= w_flat;
    xyz_flat.row(1) /= w_flat;
    xyz_flat.row(2) /= w_flat;
    
    cv::Mat x = xyz_flat.row(0).reshape(0, disparity.rows);
    cv::Mat y = xyz_flat.row(1).reshape(0, disparity.rows);
    cv::Mat z = xyz_flat.row(2).reshape(0, disparity.rows);
    cv::merge(std::move(std::vector<cv::Mat>{x, y, z}), XYZPoints);
}

static void writePCD(const std::vector<cv::Point3f>& p, const std::vector<cv::Vec3b>& c, const std::string& pcdPath)
{
    std::string header;
    header += "# .PCD v0.7 - Point Cloud Data file format\n";
    header += "VERSION 0.7\n";
    header += "FIELDS x y z rgb\n";
    header += "SIZE 4 4 4 4\n";
    header += "TYPE F F F U\n";
    header += "COUNT 1 1 1 1\n";
    header += "WIDTH " + std::to_string(p.size()) + "\n";
    header += "HEIGHT 1\n";
    header += "VIEWPOINT 0 0 0 1 0 0 0\n";
    header += "POINTS " + std::to_string(p.size()) + "\n";
    header += "DATA ascii\n";

    const size_t headerLen = header.size();
    const size_t pointBufLen = 64 * p.size();
    const size_t totalBufLen = headerLen + pointBufLen;

    std::unique_ptr<char[]> buffer{ new char[totalBufLen] };
    char* curr = buffer.get();

    memcpy(curr, header.c_str(), headerLen);
    curr += headerLen;
    char* end = buffer.get() + totalBufLen;

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

    std::ofstream pc(pcdPath, std::ios::binary); // 二进制模式避免换行符转换
    pc.write(buffer.get(), curr - buffer.get());
    pc.close();
}

void stereo::writePointCloudToPCD(const cv::Mat& RGBImage, const cv::Mat& XYZPoints, const std::string& pcdPath)
{
    if (RGBImage.empty() || XYZPoints.empty() || pcdPath.empty())
    {
        LOG_ERROR("Empty input.");
        return;
    }

    LOG_INFO("Writing point cloud to PCD file...");
    auto start = std::chrono::steady_clock::now();
    std::vector<cv::Point3f> p;
    std::vector<cv::Vec3b> c;
    p.reserve(XYZPoints.rows * XYZPoints.cols); //预分配内存提高效率
    c.reserve(RGBImage.rows * RGBImage.cols);   //预分配内存提高效率
    auto xyzIt = XYZPoints.begin<cv::Vec3f>();
    auto rgbIt = RGBImage.begin<cv::Vec3b>();
    for (; xyzIt != XYZPoints.end<cv::Vec3f>(); ++xyzIt, ++rgbIt)
    {
        if ((*xyzIt)[0] == std::numeric_limits<float>::infinity() or (*xyzIt)[1] == std::numeric_limits<float>::infinity()
            or (*xyzIt)[2] == std::numeric_limits<float>::infinity())
            continue;
        p.push_back(*xyzIt);
        c.push_back(*rgbIt);
    }
    writePCD(p, c, pcdPath);
    auto end = std::chrono::steady_clock::now();
    auto tt = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    LOG_INFO("Write Done. Points: " + std::to_string(p.size()) + ". Timing: " + utils::formatMilliseconds(tt.count() / 1000.0) + " ms.");
}

static void writePLY(const std::vector<cv::Point3f>& p, const std::vector<cv::Vec3b>& c, const std::string& plyPath)
{
    std::string header;
    header += "ply\n";
    header += "format ascii 1.0\n";
    header += "element vertex " + std::to_string(p.size()) + "\n";
    header += "property float x\n";
    header += "property float y\n";
    header += "property float z\n";
    header += "property uchar red\n";
    header += "property uchar green\n";
    header += "property uchar blue\n";
    header += "end_header\n";

    const size_t headerLen = header.size();
    const size_t pointBufLen = 128 * p.size();
    const size_t totalBufLen = headerLen + pointBufLen;

    std::unique_ptr<char[]> buffer{ new char[totalBufLen] };
    char* curr = buffer.get();

    memcpy(curr, header.c_str(), headerLen);
    curr += headerLen;
    char* end = buffer.get() + totalBufLen;

    auto rgbIt = c.begin();
    auto xyzIt = p.begin();
    for (; xyzIt != p.end() && rgbIt != c.end(); ++rgbIt, ++xyzIt)
    {
        curr = std::to_chars(curr, end, xyzIt->x).ptr;
        *curr++ = ' ';
        curr = std::to_chars(curr, end, xyzIt->y).ptr;
        *curr++ = ' ';
        curr = std::to_chars(curr, end, xyzIt->z).ptr;
        *curr++ = ' ';
        curr = std::to_chars(curr, end, static_cast<int>((*rgbIt)[2])).ptr;
        *curr++ = ' ';
        curr = std::to_chars(curr, end, static_cast<int>((*rgbIt)[1])).ptr;
        *curr++ = ' ';
        curr = std::to_chars(curr, end, static_cast<int>((*rgbIt)[0])).ptr;
        *curr++ = '\n';
    }

    std::ofstream plyFile(plyPath, std::ios::binary);
    plyFile.write(buffer.get(), curr - buffer.get());
    plyFile.close();
}

void stereo::writePointCloudToPLY(const cv::Mat& RGBImage, const cv::Mat& XYZPoints, const std::string& plyPath)
{
    if (RGBImage.empty() || XYZPoints.empty() || plyPath.empty())
    {
        LOG_ERROR("Empty input.");
        return;
    }

    LOG_INFO("Writing point cloud to PLY file...");
    auto start = std::chrono::steady_clock::now();
    std::vector<cv::Point3f> p;
    std::vector<cv::Vec3b> c;
    p.reserve(XYZPoints.rows * XYZPoints.cols); //预分配内存提高效率
    c.reserve(RGBImage.rows * RGBImage.cols);   //预分配内存提高效率
    auto xyzIt = XYZPoints.begin<cv::Vec3f>();
    auto rgbIt = RGBImage.begin<cv::Vec3b>();
    for (; xyzIt != XYZPoints.end<cv::Vec3f>(); ++xyzIt, ++rgbIt)
    {
        if ((*xyzIt)[0] == std::numeric_limits<float>::infinity() or (*xyzIt)[1] == std::numeric_limits<float>::infinity()
            or (*xyzIt)[2] == std::numeric_limits<float>::infinity())
            continue;
        p.push_back(*xyzIt);
        c.push_back(*rgbIt);
    }
    writePLY(p, c, plyPath);
    auto end = std::chrono::steady_clock::now();
    auto tt = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    LOG_INFO("Write Done. Points: " + std::to_string(p.size()) + ". Timing: " + utils::formatMilliseconds(tt.count() / 1000.0) + " ms.");
}

class stereo::InputPadder::InputPadderImpl
{
public:
    InputPadderImpl() {}
    ~InputPadderImpl() {}

    const int m_dividedBy = 32;
    std::vector<int> m_pad; // {left, right, top, bottom}
    cv::Rect m_unpad;       // {x, y, width, height}
};


stereo::InputPadder::InputPadder()
{
    this->impl = std::make_unique<InputPadderImpl>();
}

stereo::InputPadder::~InputPadder() = default;

std::vector<cv::Mat> stereo::InputPadder::pad(const std::vector<cv::Mat>& images)
{
    int imageHeight = images[0].rows;
    int imageWidth = images[0].cols;
    int padHeight = (((imageHeight / this->impl->m_dividedBy) + 1) * this->impl->m_dividedBy - imageHeight) % this->impl->m_dividedBy;
    int padWidth = (((imageWidth / this->impl->m_dividedBy) + 1) * this->impl->m_dividedBy - imageWidth) % this->impl->m_dividedBy;

    this->impl->m_pad = {
        padWidth / 2,             // left
		padWidth - padWidth / 2,  // right
        padHeight / 2,            // top
		padHeight - padHeight / 2 // bottom
    };
    this->impl->m_unpad = cv::Rect(
        this->impl->m_pad[0],
        this->impl->m_pad[2],
        imageWidth,
        imageHeight
    );

    std::vector<cv::Mat> paddedInputs;
    for (auto& img : images)
    {
        cv::Mat paddedImage;
        cv::copyMakeBorder(img, paddedImage, this->impl->m_pad[2], this->impl->m_pad[3],
            this->impl->m_pad[0], this->impl->m_pad[1], cv::BORDER_REPLICATE);
        paddedInputs.push_back(std::move(paddedImage));
    }
    return paddedInputs;
}

cv::Mat stereo::InputPadder::unpad(const cv::Mat& disparity)
{
    return disparity(this->impl->m_unpad);
}

stereo::StereoMatching::~StereoMatching() {};
