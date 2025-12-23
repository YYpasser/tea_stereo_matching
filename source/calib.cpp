#include "../include/calib.h"
#include <opencv2/opencv.hpp>

std::vector<cv::Point3f> calib::generateWorldPoints(const int& rows, const int& cols, const float& squareSize)
{
    std::vector<cv::Point3f> points;
    points.resize(0);
    for (int u = 0; u < rows; ++u)
    {
        for (int v = 0; v < cols; ++v)
        {
            points.push_back(cv::Point3f(
                static_cast<float>(v * squareSize),
                static_cast<float>(u * squareSize),
                0.f));
        }
    }
    return points;
}

double calib::computeReprojectionErrors(const std::vector<std::vector<cv::Point3f>>& worldPoints, const std::vector<std::vector<cv::Point2f>>& pixelPoints, const std::vector<cv::Mat>& RVecs, const std::vector<cv::Mat>& tVecs, const cv::Mat& intrinsicMatrix, const cv::Mat& distortionCoefficients, std::vector<double>& perViewErrors)
{
    int totalPoints = 0;
    double totalErr = 0;
    perViewErrors.resize(worldPoints.size());
    for (int i = 0; i < (int)worldPoints.size(); ++i)
    {
        std::vector<cv::Point2f> reprojectPoints;
        cv::projectPoints(cv::Mat(worldPoints[i]), RVecs[i], tVecs[i],
            intrinsicMatrix, distortionCoefficients, reprojectPoints);
        double err = cv::norm(cv::Mat(pixelPoints[i]), cv::Mat(reprojectPoints), cv::NORM_L2);
        int n = (int)worldPoints[i].size();
        perViewErrors[i] = std::sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
    }
    return std::sqrt(totalErr / totalPoints);
}
