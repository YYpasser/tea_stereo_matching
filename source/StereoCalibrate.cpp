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
    ChessboardParams m_chessboardParams; /*!< ���̸���� */
    std::string m_path;
    std::vector<std::string> m_stereoPathList; /*!< ˫Ŀ���̸�ͼ��·�� */
    std::vector<std::string> m_stereoNameExt;  /*!< ˫Ŀ���̸�ͼ�����ƺ���չ�� */
    std::vector<StereoPair<cv::Mat>> m_imagesList; /*!< ˫Ŀ���̸�ͼ�� */
    std::vector<StereoPair<GoodChessboardImage>> m_goodImagesList; /*!< �������̸�ͼ�� */
    StereoPair<ChessboardCorners> m_corners;  /*!< ���̸�ǵ����� */
    StereoPair<std::vector<cv::Mat>> m_RVecs; /*!< �����������̸�궨�����ת���� */
    StereoPair<std::vector<cv::Mat>> m_TVecs; /*!< �����������̸�궨���ƽ�ƾ��� */
    StereoPair<std::vector<double>> m_perViewErrors; 
    cv::Mat m_stereoPerViewErrors;  /*!< ˫Ŀÿ��ͼ��ƽ����ͶӰ��������� */
    StereoParams  m_stereoParams;   /*!< ˫Ŀ������� */
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

    //-- ��ȡͼ��
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
            this->impl->m_imagesList.push_back(StereoPair<cv::Mat>
                (stereoImage(cv::Rect(0, 0, stereoImage.cols / 2, stereoImage.rows)).clone(),
                    stereoImage(cv::Rect(stereoImage.cols / 2, 0, stereoImage.cols / 2, stereoImage.rows)).clone()));
            this->impl->m_stereoNameExt.push_back(it->substr(dirLength + 1, it->length()));
            ++it;
        }
    }

    // ��������ڲ� K ������Ҫ >= 3�����̸�ͼ�� (K: 5�����ɶ�, ÿ��ͼ��ĵ�Ӧ���� H �ṩ2��Լ������)
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

double calib::StereoCalibrate::calibrate(const float& error, const bool& showCorners, const bool& writeCorners)
{
    cv::Size patternSize = cv::Size(this->impl->m_chessboardParams.board_width, this->impl->m_chessboardParams.board_height);
    // ���㵥Ӧ���� H ��Ҫ >= 4���ǵ�Լ�� (8�����ɶ�)
    if (patternSize.area() < 4)
    {
        std::string msg = "Compute Homography matrix need >= 4 points";
        LOG_INFO(msg);
        throw std::invalid_argument(msg);
    }
    // ��������ڲ� K ������Ҫ >= 3�����̸�ͼ�� (K: 5�����ɶ�, ÿ��ͼ��ĵ�Ӧ���� H �ṩ2��Լ������)
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

    //-- ���̸�ǵ���
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
        this->impl->m_goodImagesList.emplace_back(StereoPair<GoodChessboardImage>
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

    //-- ����궨, ���� K, D, R, t
    LOG_INFO("Calibrating camera...");
    // ������������ϵ�е����̸�ǵ�����
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
        //-- �����ͶӰ���
        LOG_INFO("Checking reproject RMS error...");
        auto bSize = this->impl->m_corners.left.pixel_points.size();
        auto lpIt = this->impl->m_corners.left.pixel_points.begin();
        auto rpIt = this->impl->m_corners.right.pixel_points.begin();
        auto oIt = this->impl->m_corners.left.world_points.begin();
        auto leIt = this->impl->m_perViewErrors.left.begin();
        auto reIt = this->impl->m_perViewErrors.right.begin();
        auto giIt = this->impl->m_goodImagesList.begin();

        for (; lpIt != this->impl->m_corners.left.pixel_points.end(); )
        {
            if (*leIt <= error and *reIt <= error)
            {
                ++lpIt, ++rpIt, ++oIt, ++leIt, ++reIt, ++giIt;
                continue;
            }
            LOG_INFO("Removing image " + giIt->left.file_name + " which reproject RMS error > " + std::to_string(error) + " px...");

            lpIt = this->impl->m_corners.left.pixel_points.erase(lpIt);
            rpIt = this->impl->m_corners.right.pixel_points.erase(rpIt);
            oIt = this->impl->m_corners.left.world_points.erase(oIt);
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

StereoParams calib::StereoCalibrate::getStereoParams() const
{
    return this->impl->m_stereoParams;
}

StereoPair<std::vector<cv::Mat>> calib::StereoCalibrate::getCameraRotations() const
{
    return this->impl->m_RVecs;
}

StereoPair<std::vector<cv::Mat>> calib::StereoCalibrate::getCameraTranslations() const
{
    return this->impl->m_TVecs;
}

StereoPair<ChessboardCorners> calib::StereoCalibrate::getChessboardCorners() const
{
    return this->impl->m_corners;
}

std::vector<StereoPair<GoodChessboardImage>> calib::StereoCalibrate::getGoodChessboardImages() const
{
    return this->impl->m_goodImagesList;
}

StereoPair<std::vector<double>> calib::StereoCalibrate::getPerViewErrors() const
{
    return this->impl->m_perViewErrors;
}

calib::StereoCalibrate::SCImpl::SCImpl()
{
}

calib::StereoCalibrate::SCImpl::~SCImpl()
{
}
