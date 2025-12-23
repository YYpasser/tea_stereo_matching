#include "../include/calib.h"
#include "../include/utils.h"
#include "../include/logger.h"
#include <opencv2/opencv.hpp>

class calib::StereoCalibrate::SCImpl
{
public:
    SCImpl();
    ~SCImpl();
public:
    ChessboardParams m_chessboardParams; /*!< 棋盘格参数 */
    std::string m_path;
    std::vector<std::string> m_stereoPathList; /*!< 双目棋盘格图像路径 */
    std::vector<std::string> m_stereoNameExt;  /*!< 双目棋盘格图像名称和扩展名 */
    std::vector<stereo::StereoPair<cv::Mat>> m_imagesList; /*!< 双目棋盘格图像 */
    std::vector<stereo::StereoPair<GoodChessboardImage>> m_goodImagesList; /*!< 优质棋盘格图像 */
    stereo::StereoPair<ChessboardCorners> m_corners;  /*!< 棋盘格角点坐标 */
    stereo::StereoPair<std::vector<cv::Mat>> m_RVecs; /*!< 相机相对于棋盘格标定板的旋转矩阵 */
    stereo::StereoPair<std::vector<cv::Mat>> m_TVecs; /*!< 相机相对于棋盘格标定板的平移矩阵 */
    stereo::StereoPair<std::vector<double>> m_perViewErrors;
    cv::Mat m_stereoPerViewErrors;  /*!< 双目每张图像平均重投影均方根误差 */
    stereo::StereoParams  m_stereoParams;   /*!< 双目相机参数 */
};

calib::StereoCalibrate::StereoCalibrate()
{
	this->impl = std::make_unique<calib::StereoCalibrate::SCImpl>();
}

calib::StereoCalibrate::~StereoCalibrate()
{
}

void calib::StereoCalibrate::setChessboardParams(const ChessboardParams& chessboardParams)
{
    this->impl->m_chessboardParams = chessboardParams;
}

void calib::StereoCalibrate::loadChessboardImages(const std::string& pattern, const bool& recursive)
{
    if (pattern.empty())
    {
        std::string msg = "Invalid path";
        LOG_INFO(msg);
        throw std::invalid_argument(msg);
    }

    size_t upperDirSNum = std::string::npos;
    size_t pos_slash = pattern.rfind('/');
    size_t pos_backslash = pattern.rfind('\\');
    if (pos_slash != std::string::npos) {
        upperDirSNum = pos_slash;
    }
    if (pos_backslash != std::string::npos) {
        if (upperDirSNum == std::string::npos || pos_backslash > upperDirSNum) {
            upperDirSNum = pos_backslash;
        }
    }
    std::string upperDir = pattern.substr(0, upperDirSNum);
    size_t dirLength = upperDir.length();
    this->impl->m_path = upperDir;
    this->impl->m_stereoPathList = utils::glob(pattern);

    //-- 读取图像
    for (auto it = this->impl->m_stereoPathList.begin(); it != this->impl->m_stereoPathList.end(); )
    {
        cv::Mat stereoImage = cv::imread(*it);
        if (stereoImage.empty())
        {
            LOG_INFO("Failed to load image: " + *it);
            it = this->impl->m_stereoPathList.erase(it);
        }
        else
        {
            LOG_INFO("Loaded image: " + *it);
            this->impl->m_imagesList.push_back(stereo::StereoPair<cv::Mat>
                (stereoImage(cv::Rect(0, 0, stereoImage.cols / 2, stereoImage.rows)).clone(),
                    stereoImage(cv::Rect(stereoImage.cols / 2, 0, stereoImage.cols / 2, stereoImage.rows)).clone()));
            this->impl->m_stereoNameExt.push_back(it->substr(dirLength + 1, it->length()));
            ++it;
        }
    }

    // 计算相机内参 K 矩阵需要 >= 3张棋盘格图像 (K: 5个自由度, 每张图像的单应矩阵 H 提供2个约束条件)
    if (this->impl->m_imagesList.size() < 3)
    {
        std::string msg = "Compute camera intrinsic matrix need >= 3 images";
        LOG_INFO(msg);
        throw std::invalid_argument(msg);
    }

    std::string leftPath = this->impl->m_path + "/left";
    std::string rightPath = this->impl->m_path + "/right";
    utils::generateNewFolder(leftPath);
    utils::generateNewFolder(rightPath);

    auto nameIt = this->impl->m_stereoNameExt.begin();
    auto imageIt = this->impl->m_imagesList.begin();
    for (; nameIt != this->impl->m_stereoNameExt.end(); ++nameIt, ++imageIt)
    {
        cv::imwrite(leftPath + "/" + (*nameIt), imageIt->left);
        cv::imwrite(rightPath + "/" + (*nameIt), imageIt->right);
    }
    this->impl->m_stereoParams.imgsz = this->impl->m_imagesList[0].left.size();
}

/**
 * @brief 极线误差评估结果结构体
 */
struct EpipolarErrorResult {
    double averageError;          // 平均极线误差
    double maxError;              // 最大极线误差
    double minError;              // 最小极线误差
    std::vector<double> allErrors; // 所有点对的极线误差
};

/**
 * @brief 计算双目相机的极线误差
 * @param F 基础矩阵 (3x3)
 * @param imagePoints1 左图中的特征点集合
 * @param imagePoints2 右图中对应的特征点集合
 * @return 极线误差评估结果
 */
EpipolarErrorResult calculateEpipolarError(const cv::Mat& F,
    const std::vector<std::vector<cv::Point2f>>& imagePoints1,
    const std::vector<std::vector<cv::Point2f>>& imagePoints2) {
    EpipolarErrorResult result;
    result.averageError = 0.0;
    result.maxError = -1.0;
    result.minError = 1e9;  // 初始化为一个较大的值

    // 检查输入有效性
    if (F.empty() || F.rows != 3 || F.cols != 3) {
        std::cerr << "错误: 基础矩阵F必须是3x3的矩阵!" << std::endl;
        return result;
    }

    if (imagePoints1.size() != imagePoints2.size()) {
        std::cerr << "错误: 左右图像的点集数量不匹配!" << std::endl;
        return result;
    }

    int totalPoints = 0;

    // 遍历所有图像对
    for (size_t i = 0; i < imagePoints1.size(); ++i) {
        // 检查单张图像中的点数量是否匹配
        if (imagePoints1[i].size() != imagePoints2[i].size()) {
            std::cerr << "警告: 第" << i << "组图像的点数量不匹配，已跳过该组!" << std::endl;
            continue;
        }

        // 遍历该图像中的所有点对
        for (size_t j = 0; j < imagePoints1[i].size(); ++j) {
            cv::Point2f p1 = imagePoints1[i][j];  // 左图点
            cv::Point2f p2 = imagePoints2[i][j];  // 右图对应点

            // 构造左图点的齐次坐标 [u1, v1, 1]
            cv::Mat p1_hom = (cv::Mat_<double>(3, 1) << p1.x, p1.y, 1.0);

            // 计算极线 l2 = F * p1_hom (极线方程: ax + by + c = 0)
            cv::Mat l2 = F * p1_hom;
            double a = l2.at<double>(0);
            double b = l2.at<double>(1);
            double c = l2.at<double>(2);

            // 计算点p2到极线l2的距离 (极线误差)
            double error = std::abs(a * p2.x + b * p2.y + c) / std::sqrt(a * a + b * b);

            // 更新结果统计
            result.allErrors.push_back(error);
            result.averageError += error;
            totalPoints++;

            if (error > result.maxError) {
                result.maxError = error;
            }
            if (error < result.minError) {
                result.minError = error;
            }
        }
    }

    // 计算平均误差
    if (totalPoints > 0) {
        result.averageError /= totalPoints;
    }
    else {
        result.averageError = 0.0;
        result.maxError = 0.0;
        result.minError = 0.0;
        std::cerr << "警告: 没有有效的点对用于计算极线误差!" << std::endl;
    }

    return result;
}

/**
 * @brief 打印极线误差评估结果
 * @param result 极线误差评估结果
 * @param precision 输出精度 (小数位数)
 */
void printEpipolarErrorResult(const EpipolarErrorResult& result, int precision = 4) {
    LOG_INFO("极线误差评估结果:");
    LOG_INFO("---------------------");
    LOG_INFO("平均极线误差: " +  std::to_string(result.averageError));
    LOG_INFO("最大极线误差: " +  std::to_string(result.maxError));
    LOG_INFO("最小极线误差: " +  std::to_string(result.minError));
    LOG_INFO("总点数: " +  std::to_string(result.allErrors.size()));
    LOG_INFO("---------------------");
}

// 使用示例
/*
int main() {
    // 示例: 假设已经通过stereoCalibrate获取了基础矩阵F
    // 以及左右图像的角点imagePoints1和imagePoints2

    // cv::Mat F;  // 3x3基础矩阵
    // std::vector<std::vector<cv::Point2f>> imagePoints1, imagePoints2;  // 左右图像的角点集合

    // 计算极线误差
    // EpipolarErrorResult result = calculateEpipolarError(F, imagePoints1, imagePoints2);

    // 打印结果
    // printEpipolarErrorResult(result);

    return 0;
}
*/


double calib::StereoCalibrate::calibrate(const float& error, const bool& showCorners, const bool& writeCorners)
{
    cv::Size patternSize = cv::Size(this->impl->m_chessboardParams.board_width, this->impl->m_chessboardParams.board_height);
    // 计算单应矩阵 H 需要 >= 4个角点约束 (8个自由度)
    if (patternSize.area() < 4)
    {
        std::string msg = "Compute Homography matrix need >= 4 points";
        LOG_INFO(msg);
        throw std::invalid_argument(msg);
    }
    // 计算相机内参 K 矩阵需要 >= 3张棋盘格图像 (K: 5个自由度, 每张图像的单应矩阵 H 提供2个约束条件)
    if (this->impl->m_imagesList.size() < 3)
    {
        std::string msg = "Compute camera intrinsic matrix need >= 3 images";
        LOG_INFO(msg);
        throw std::invalid_argument(msg);
    }

    std::string leftCornersPath = this->impl->m_path + "/left/corners";
    std::string rightCornersPath = this->impl->m_path + "/right/corners";
    if (writeCorners)
    {
        utils::generateNewFolder(leftCornersPath);
        utils::generateNewFolder(rightCornersPath);
    }
    if (showCorners)
        cv::namedWindow("corners", cv::WINDOW_NORMAL);

    //-- 棋盘格角点检测
    LOG_INFO("Detecting corners...");
    auto imageIt = this->impl->m_imagesList.begin();
    auto pathIt = this->impl->m_stereoPathList.begin();
    auto nameextIt = this->impl->m_stereoNameExt.begin();
    for (; imageIt != this->impl->m_imagesList.end(); ++imageIt, ++pathIt, ++nameextIt)
    {
        int i = std::distance(this->impl->m_imagesList.begin(), imageIt);
        cv::Mat leftc = imageIt->left.clone();
        cv::Mat rightc = imageIt->right.clone();
        std::vector<cv::Point2f> leftCorners;
        std::vector<cv::Point2f> rightCorners;
        //bool leftFound = cv::findChessboardCornersSB(leftc, patternSize, leftCorners,
        //    cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);
        //bool rightFound = cv::findChessboardCornersSB(rightc, patternSize, rightCorners,
        //    cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);
        bool leftFound = cv::findChessboardCorners(leftc, patternSize, leftCorners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        bool rightFound = cv::findChessboardCorners(rightc, patternSize, rightCorners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (!leftFound or !rightFound)
        {
            LOG_INFO("Bad chessboard corners of image: " + *pathIt);
            continue;
        }
        cv::Mat leftcc = imageIt->left.clone();
        cv::Mat rightcc = imageIt->right.clone();
        cv::Mat leftGray, rightGray;
        cv::cvtColor(leftcc, leftGray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(rightcc, rightGray, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(leftGray, leftCorners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
        cv::cornerSubPix(rightGray, rightCorners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
        if (showCorners)
        {
            cv::drawChessboardCorners(leftcc, patternSize, leftCorners, leftFound);
            cv::drawChessboardCorners(rightcc, patternSize, rightCorners, rightFound);
            cv::Mat stereo;
            cv::hconcat(leftcc, rightcc, stereo);
            cv::imshow("corners", stereo);
            cv::waitKey(5);
        }
        this->impl->m_goodImagesList.emplace_back(stereo::StereoPair<GoodChessboardImage>
            (GoodChessboardImage(*pathIt, imageIt->left, i),
                GoodChessboardImage(*pathIt, imageIt->right, i)));

        this->impl->m_corners.left.pixel_points.emplace_back(leftCorners);
        this->impl->m_corners.right.pixel_points.emplace_back(rightCorners);

        if (writeCorners)
        {
            cv::imwrite(leftCornersPath + "/" + *nameextIt, leftcc);
            cv::imwrite(rightCornersPath + "/" + *nameextIt, rightcc);
        }
    }

    if (showCorners)
        cv::destroyWindow("corners");

    LOG_INFO("Detecting corners done.");
    auto goodCount = this->impl->m_goodImagesList.size();
    LOG_INFO("Number of good corner images = " + std::to_string(goodCount) + ".");

    if (goodCount < 3)
    {
        std::string msg = "Number of good corner images < 3, can't compute camera intrinsic matrix.";
        LOG_INFO(msg);
        throw std::runtime_error(msg);
    }

    //-- 相机标定, 计算 K, D, R, t
    LOG_INFO("Calibrating camera...");
    // 生成世界坐标系中的棋盘格角点坐标
    auto worldPoints = calib::generateWorldPoints(this->impl->m_chessboardParams.board_height, this->impl->m_chessboardParams.board_width, this->impl->m_chessboardParams.square_size);
    this->impl->m_corners.left.world_points.resize(goodCount, worldPoints);
    this->impl->m_corners.right.world_points.resize(goodCount, worldPoints);

    std::vector<double> stdDeviationsIntrinsics;
    std::vector<double> stdDeviationsExtrinsics;
    double leftRMS = cv::calibrateCamera(this->impl->m_corners.left.world_points, this->impl->m_corners.left.pixel_points, this->impl->m_stereoParams.imgsz,
        this->impl->m_stereoParams.intrinsic.left.intrinsic_matrix, this->impl->m_stereoParams.intrinsic.left.distortion_coefficients,
        this->impl->m_RVecs.left, this->impl->m_TVecs.left,
        stdDeviationsIntrinsics, stdDeviationsExtrinsics, this->impl->m_perViewErrors.left);
    double rightRMS = cv::calibrateCamera(this->impl->m_corners.right.world_points, this->impl->m_corners.right.pixel_points, this->impl->m_stereoParams.imgsz,
        this->impl->m_stereoParams.intrinsic.right.intrinsic_matrix, this->impl->m_stereoParams.intrinsic.right.distortion_coefficients,
        this->impl->m_RVecs.right, this->impl->m_TVecs.right,
        stdDeviationsIntrinsics, stdDeviationsExtrinsics, this->impl->m_perViewErrors.right);
    LOG_INFO("Calibrating camera done.");

    LOG_INFO_MSG("Left Camera param as follow:");
    LOG_INFO_MSG_MAT("Camera Matrix:", this->impl->m_stereoParams.intrinsic.left.intrinsic_matrix);
    LOG_INFO_MSG_MAT("Distortion Matrix:", this->impl->m_stereoParams.intrinsic.left.distortion_coefficients);
    LOG_INFO_MSG("Overall images reproject RMS error = " + std::to_string(leftRMS) + " px.");

    calib::computeReprojectionErrors(this->impl->m_corners.left.world_points, this->impl->m_corners.left.pixel_points, this->impl->m_RVecs.left, this->impl->m_TVecs.left, this->impl->m_stereoParams.intrinsic.left.intrinsic_matrix, this->impl->m_stereoParams.intrinsic.left.distortion_coefficients, this->impl->m_perViewErrors.left);
    for (int i = 0; i < this->impl->m_perViewErrors.left.size(); i++)
        LOG_INFO_MSG("Image " + this->impl->m_goodImagesList[i].left.file_name + " reproject RMS error = " + std::to_string(this->impl->m_perViewErrors.left[i]) + " px.");

    LOG_INFO_MSG("Right Camera param as follow:");
    LOG_INFO_MSG_MAT("Camera Matrix:", this->impl->m_stereoParams.intrinsic.right.intrinsic_matrix);
    LOG_INFO_MSG_MAT("Distortion Matrix:", this->impl->m_stereoParams.intrinsic.right.distortion_coefficients);
    LOG_INFO_MSG("Overall images reproject RMS error = " + std::to_string(rightRMS) + " px.");

    calib::computeReprojectionErrors(this->impl->m_corners.right.world_points, this->impl->m_corners.right.pixel_points, this->impl->m_RVecs.right, this->impl->m_TVecs.right, this->impl->m_stereoParams.intrinsic.right.intrinsic_matrix, this->impl->m_stereoParams.intrinsic.right.distortion_coefficients, this->impl->m_perViewErrors.right);
    for (int i = 0; i < this->impl->m_perViewErrors.right.size(); i++)
        LOG_INFO_MSG("Image " + this->impl->m_goodImagesList[i].right.file_name + " reproject RMS error = " + std::to_string(this->impl->m_perViewErrors.right[i]) + " px.");
    LOG_INFO_MSG("Overall images reproject RMS error = " + std::to_string(rightRMS) + " px.");

    if (writeCorners)
    {
        LOG_INFO("Detected corners image save in folder: \"" + leftCornersPath + "\".");
        LOG_INFO("Detected corners image save in folder: \"" + rightCornersPath + "\".");
    }

    if (error > 0)
    {
        //-- 检查重投影误差
        LOG_INFO("Checking reproject RMS error...");
        auto bSize = this->impl->m_corners.left.pixel_points.size();
        auto lpIt = this->impl->m_corners.left.pixel_points.begin();
        auto rpIt = this->impl->m_corners.right.pixel_points.begin();
        auto olIt = this->impl->m_corners.left.world_points.begin();
        auto orIt = this->impl->m_corners.right.world_points.begin();
        auto leIt = this->impl->m_perViewErrors.left.begin();
        auto reIt = this->impl->m_perViewErrors.right.begin();
        auto giIt = this->impl->m_goodImagesList.begin();

        for (; lpIt != this->impl->m_corners.left.pixel_points.end(); )
        {
            if (*leIt <= error and *reIt <= error)
            {
                ++lpIt, ++rpIt, ++olIt, ++orIt, ++leIt, ++reIt, ++giIt;
                continue;
            }
            LOG_INFO("Removing image " + giIt->left.file_name + " which reproject RMS error > " + std::to_string(error) + " px...");

            lpIt = this->impl->m_corners.left.pixel_points.erase(lpIt);
            rpIt = this->impl->m_corners.right.pixel_points.erase(rpIt);
            olIt = this->impl->m_corners.left.world_points.erase(olIt);
            orIt = this->impl->m_corners.right.world_points.erase(orIt);
            leIt = this->impl->m_perViewErrors.left.erase(leIt);
            reIt = this->impl->m_perViewErrors.right.erase(reIt);
            giIt = this->impl->m_goodImagesList.erase(giIt);
        }
        if (bSize != this->impl->m_corners.left.pixel_points.size() and this->impl->m_corners.left.pixel_points.size() >= 3)
        {
            LOG_INFO("ReCalibrating...");
            leftRMS = cv::calibrateCamera(this->impl->m_corners.left.world_points, this->impl->m_corners.left.pixel_points, this->impl->m_stereoParams.imgsz,
                this->impl->m_stereoParams.intrinsic.left.intrinsic_matrix, this->impl->m_stereoParams.intrinsic.left.distortion_coefficients,
                this->impl->m_RVecs.left, this->impl->m_TVecs.left,
                stdDeviationsIntrinsics, stdDeviationsExtrinsics, this->impl->m_perViewErrors.left);
            rightRMS = cv::calibrateCamera(this->impl->m_corners.right.world_points, this->impl->m_corners.right.pixel_points, this->impl->m_stereoParams.imgsz,
                this->impl->m_stereoParams.intrinsic.right.intrinsic_matrix, this->impl->m_stereoParams.intrinsic.right.distortion_coefficients,
                this->impl->m_RVecs.right, this->impl->m_TVecs.right,
                stdDeviationsIntrinsics, stdDeviationsExtrinsics, this->impl->m_perViewErrors.right);

            LOG_INFO("Calibrating camera done.");
            LOG_INFO_MSG("Left Camera param as follow:");
            LOG_INFO_MSG_MAT("Camera Matrix:", this->impl->m_stereoParams.intrinsic.left.intrinsic_matrix);
            LOG_INFO_MSG_MAT("Distortion Matrix:", this->impl->m_stereoParams.intrinsic.left.distortion_coefficients);
            LOG_INFO_MSG("Overall images reproject RMS error = " + std::to_string(leftRMS) + " px.");

            calib::computeReprojectionErrors(this->impl->m_corners.left.world_points, this->impl->m_corners.left.pixel_points, this->impl->m_RVecs.left, this->impl->m_TVecs.left, this->impl->m_stereoParams.intrinsic.left.intrinsic_matrix, this->impl->m_stereoParams.intrinsic.left.distortion_coefficients, this->impl->m_perViewErrors.left);
            for (int i = 0; i < this->impl->m_perViewErrors.left.size(); i++)
                LOG_INFO_MSG("Image " + this->impl->m_goodImagesList[i].left.file_name + " reproject RMS error = " + std::to_string(this->impl->m_perViewErrors.left[i]) + " px.");

            LOG_INFO_MSG("Right Camera param as follow:");
            LOG_INFO_MSG_MAT("Camera Matrix:", this->impl->m_stereoParams.intrinsic.right.intrinsic_matrix);
            LOG_INFO_MSG_MAT("Distortion Matrix:", this->impl->m_stereoParams.intrinsic.right.distortion_coefficients);
            LOG_INFO_MSG("Overall images reproject RMS error = " + std::to_string(rightRMS) + " px.");

            calib::computeReprojectionErrors(this->impl->m_corners.right.world_points, this->impl->m_corners.right.pixel_points, this->impl->m_RVecs.right, this->impl->m_TVecs.right, this->impl->m_stereoParams.intrinsic.right.intrinsic_matrix, this->impl->m_stereoParams.intrinsic.right.distortion_coefficients, this->impl->m_perViewErrors.right);
            for (int i = 0; i < this->impl->m_perViewErrors.right.size(); i++)
                LOG_INFO_MSG("Image " + this->impl->m_goodImagesList[i].right.file_name + " reproject RMS error = " + std::to_string(this->impl->m_perViewErrors.right[i]) + " px.");
            LOG_INFO_MSG("Overall images reproject RMS error = " + std::to_string(rightRMS) + " px.");
        }
    }

    LOG_INFO("Calibrating stereo camera...");
    double stereoOverallRMS = cv::stereoCalibrate(this->impl->m_corners.right.world_points, this->impl->m_corners.left.pixel_points, this->impl->m_corners.right.pixel_points,
        this->impl->m_stereoParams.intrinsic.left.intrinsic_matrix, this->impl->m_stereoParams.intrinsic.left.distortion_coefficients,
        this->impl->m_stereoParams.intrinsic.right.intrinsic_matrix, this->impl->m_stereoParams.intrinsic.right.distortion_coefficients,
        this->impl->m_stereoParams.imgsz, this->impl->m_stereoParams.extrinsic.R, this->impl->m_stereoParams.extrinsic.T,
        this->impl->m_stereoParams.extrinsic.E, this->impl->m_stereoParams.extrinsic.F, this->impl->m_stereoPerViewErrors, cv::CALIB_USE_INTRINSIC_GUESS);
    LOG_INFO("Stereo Calibrate done.");
    LOG_INFO("Overall stereo images reproject RMS error = " + std::to_string(stereoOverallRMS) + " px.");


    for (auto iter = this->impl->m_stereoPerViewErrors.begin<double>(); iter != this->impl->m_stereoPerViewErrors.end<double>(); ++iter)
    {
        int i = std::distance(this->impl->m_stereoPerViewErrors.begin<double>(), iter);
        if (i % 2 == 0)
            LOG_INFO_MSG("Stereo image " + this->impl->m_goodImagesList[i / 2].left.file_name + " reproject RMS error = " + std::to_string(*iter) + " px, ");
        else
            LOG_INFO_MSG(std::to_string(*iter) + " px.");
    }
    LOG_INFO_MSG_MAT("Matrix R(rotation matrix):", this->impl->m_stereoParams.extrinsic.R);
    LOG_INFO_MSG_MAT("Matrix T(translation vector):", this->impl->m_stereoParams.extrinsic.T);
    LOG_INFO_MSG_MAT("Matrix E(essential matrix):", this->impl->m_stereoParams.extrinsic.E);
    LOG_INFO_MSG_MAT("Matrix F(fundamental matrix):", this->impl->m_stereoParams.extrinsic.F);


    EpipolarErrorResult result = calculateEpipolarError(this->impl->m_stereoParams.extrinsic.F,
        this->impl->m_corners.left.pixel_points, this->impl->m_corners.right.pixel_points);

    printEpipolarErrorResult(result);

    LOG_INFO("Calculating rectify param...");
    cv::Rect validRoi[2];
    cv::stereoRectify(this->impl->m_stereoParams.intrinsic.left.intrinsic_matrix, this->impl->m_stereoParams.intrinsic.left.distortion_coefficients,
        this->impl->m_stereoParams.intrinsic.right.intrinsic_matrix, this->impl->m_stereoParams.intrinsic.right.distortion_coefficients,
        this->impl->m_stereoParams.imgsz, this->impl->m_stereoParams.extrinsic.R, this->impl->m_stereoParams.extrinsic.T,
        this->impl->m_stereoParams.map.R1, this->impl->m_stereoParams.map.R2, this->impl->m_stereoParams.map.P1, this->impl->m_stereoParams.map.P2,
        this->impl->m_stereoParams.Q, cv::CALIB_ZERO_DISPARITY, 0, this->impl->m_stereoParams.imgsz, &validRoi[0], &validRoi[1]);
    LOG_INFO("Stereo rectify done.");
    LOG_INFO_MSG_MAT("Matrix R1(rotation matrix(camera 1)):", this->impl->m_stereoParams.map.R1);
    LOG_INFO_MSG_MAT("Matrix R2(rotation matrix(camera 2)):", this->impl->m_stereoParams.map.R2);
    LOG_INFO_MSG_MAT("Matrix P1(projection matrix(camera 1)):", this->impl->m_stereoParams.map.P1);
    LOG_INFO_MSG_MAT("Matrix P2(projection matrix(camera 2)):", this->impl->m_stereoParams.map.P2);
    LOG_INFO_MSG_MAT("Matrix Q(disparity-to-depth mapping matrix):", this->impl->m_stereoParams.Q);
    cv::initUndistortRectifyMap(this->impl->m_stereoParams.intrinsic.right.intrinsic_matrix, this->impl->m_stereoParams.intrinsic.left.distortion_coefficients,
        this->impl->m_stereoParams.map.R1, this->impl->m_stereoParams.map.P1, this->impl->m_stereoParams.imgsz, CV_16SC2, this->impl->m_stereoParams.map.map00, this->impl->m_stereoParams.map.map01);
    cv::initUndistortRectifyMap(this->impl->m_stereoParams.intrinsic.right.intrinsic_matrix, this->impl->m_stereoParams.intrinsic.right.distortion_coefficients,
        this->impl->m_stereoParams.map.R2, this->impl->m_stereoParams.map.P2, this->impl->m_stereoParams.imgsz, CV_16SC2, this->impl->m_stereoParams.map.map10, this->impl->m_stereoParams.map.map11);
    this->impl->m_stereoParams.rectified_f = static_cast<float>(impl->m_stereoParams.Q.at<double>(2, 3));
    this->impl->m_stereoParams.rectified_cx = static_cast<float>(-impl->m_stereoParams.Q.at<double>(0, 3));
    this->impl->m_stereoParams.rectified_cy = static_cast<float>(-impl->m_stereoParams.Q.at<double>(1, 3));
    this->impl->m_stereoParams.baseline = 1.f / static_cast<float>(impl->m_stereoParams.Q.at<double>(3, 2));
    return stereoOverallRMS;
}

void calib::StereoCalibrate::writeYAMLFile(const std::string& path) const
{
    std::string stereoPath = std::string();
    if (path.empty())
        stereoPath = this->impl->m_path + "/yml/stereo.yml";
    else
        stereoPath = path;
    utils::generateNewFolder(stereoPath);
    cv::FileStorage fs;
    fs.open(stereoPath, cv::FileStorage::WRITE);
    if (fs.isOpened())
    {
        fs << "leftK" << this->impl->m_stereoParams.intrinsic.left.intrinsic_matrix
            << "leftD" << this->impl->m_stereoParams.intrinsic.left.distortion_coefficients
            << "rightK" << this->impl->m_stereoParams.intrinsic.right.intrinsic_matrix
            << "rightD" << this->impl->m_stereoParams.intrinsic.right.distortion_coefficients;

        fs << "R" << this->impl->m_stereoParams.extrinsic.R
            << "T" << this->impl->m_stereoParams.extrinsic.T
            << "E" << this->impl->m_stereoParams.extrinsic.E
            << "F" << this->impl->m_stereoParams.extrinsic.F
            << "R1" << this->impl->m_stereoParams.map.R1
            << "R2" << this->impl->m_stereoParams.map.R2
            << "P1" << this->impl->m_stereoParams.map.P1
            << "P2" << this->impl->m_stereoParams.map.P2
            << "Q" << this->impl->m_stereoParams.Q
            << "fx" << this->impl->m_stereoParams.map.P2.ptr<double>(0)[0]
            << "fy" << this->impl->m_stereoParams.map.P2.ptr<double>(1)[1]
            << "cx" << this->impl->m_stereoParams.map.P2.ptr<double>(0)[2]
            << "cy" << this->impl->m_stereoParams.map.P2.ptr<double>(1)[2]
            << "baseline" << -this->impl->m_stereoParams.map.P2.ptr<double>(0)[3] / this->impl->m_stereoParams.map.P2.ptr<double>(0)[0]
            << "imgsz" << this->impl->m_stereoParams.imgsz;
        LOG_INFO("Stereo YML saved in floder: \"" + stereoPath + "\".");
    }
    else
    {
        fs.release();
        throw std::runtime_error("Failed to save the stereo YML.");
    }
    fs.release();
}

stereo::StereoParams calib::StereoCalibrate::getStereoParams() const
{
    return this->impl->m_stereoParams;
}

stereo::StereoPair<std::vector<cv::Mat>> calib::StereoCalibrate::getCameraRotations() const
{
    return this->impl->m_RVecs;
}

stereo::StereoPair<std::vector<cv::Mat>> calib::StereoCalibrate::getCameraTranslations() const
{
    return this->impl->m_TVecs;
}

stereo::StereoPair<ChessboardCorners> calib::StereoCalibrate::getChessboardCorners() const
{
    return this->impl->m_corners;
}

std::vector<stereo::StereoPair<GoodChessboardImage>> calib::StereoCalibrate::getGoodChessboardImages() const
{
    return this->impl->m_goodImagesList;
}

stereo::StereoPair<std::vector<double>> calib::StereoCalibrate::getPerViewErrors() const
{
    return this->impl->m_perViewErrors;
}

calib::StereoCalibrate::SCImpl::SCImpl()
{
}

calib::StereoCalibrate::SCImpl::~SCImpl()
{
}
