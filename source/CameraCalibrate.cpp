#include "../include/calib.h"
#include "../include/utils.h"
#include "../include/logger.h"
#include <opencv2/opencv.hpp>

class calib::CameraCalibrate::CCImpl
{
public:
	CCImpl();
	~CCImpl();
public:
    ChessboardParams m_chessboardParams; /*!< ���̸���� */
    std::string m_path;   /*!< ���̸�ͼ���ļ���·�� */
    cv::Size m_imageSize; /*!< ���̸�ͼ��ߴ� */
    std::vector<std::string> m_imagesPathList; /*!< ���̸�ͼ������·���� */
    std::vector<cv::Mat> m_imagesList; /*!< ���̸�ͼ�� */
    std::vector<GoodChessboardImage> m_goodImages; /*!< �������̸�ͼ����Ϣ */
    ChessboardCorners m_corners; /*!< ���̸�ǵ���Ϣ */
    std::vector<cv::Mat> m_RVecs; /*!< ������(����任) T -- ��ת���� */
    std::vector<cv::Mat> m_tVecs; /*!< ������(����任) T -- ƽ������ */
    cv::Mat m_KMatrix; /*!< ����ڲ�, (fx, s(0), cx; 0, fy, cy; 0, 0, 1) */
    cv::Mat m_DCoeffs; /*!< �������, (k1, k2, r1, r2, k3, ...) */
    std::vector<double> m_perViewErrors; /*!< ÿ��ͼ������̸�ǵ���ͶӰ��� */
};

calib::CameraCalibrate::CameraCalibrate()
{
	this->impl = std::make_unique<CCImpl>();
}

calib::CameraCalibrate::~CameraCalibrate()
{
}

void calib::CameraCalibrate::setChessboardParams(const ChessboardParams& chessboardParams)
{
    this->impl->m_chessboardParams = chessboardParams;
}

void calib::CameraCalibrate::loadChessboardImages(const std::string& pattern, const bool& recursive)
{
    if (pattern.empty())
    {
        std::string msg = "Invalid path";
        LOG_ERROR(msg);
        throw std::invalid_argument(msg);
    }


    size_t lastSlashPos = pattern.find_last_of("/\\");
    std::string directory = (lastSlashPos != std::string::npos) ? pattern.substr(0, lastSlashPos) : ".";
    this->impl->m_path = directory;

    //-- ��ȡ����ͼ��·��
    this->impl->m_imagesPathList = utils::glob(pattern);

    //-- ��ȡͼ��
    for (auto it = this->impl->m_imagesPathList.begin(); it != this->impl->m_imagesPathList.end(); )
    {
        cv::Mat image = cv::imread(*it);
        if (image.empty())
        {
            LOG_INFO("Failed to load image: " + *it);
            //std::remove(it->c_str());
            it = this->impl->m_imagesPathList.erase(it);
        }
        else
        {
            LOG_INFO("Loaded image: " + *it);
            this->impl->m_imagesList.push_back(image);
            ++it;
        }
    }

    this->impl->m_imageSize = this->impl->m_imagesList[0].size();
}

double calib::CameraCalibrate::calibrate(const float& error, const bool& showCorners, const bool& writeCorners)
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

    std::string cornersPath = this->impl->m_path + "/corners";
    if (writeCorners)
        utils::generateNewFolder(cornersPath);
    if (showCorners)
        cv::namedWindow("corners", cv::WINDOW_NORMAL);

    //-- ���̸�ǵ���
    LOG_INFO("Detecting corners...");
    auto imgIt = this->impl->m_imagesList.begin();
    auto pathIt = this->impl->m_imagesPathList.begin();
    for (; imgIt != this->impl->m_imagesList.end(); ++imgIt, ++pathIt)
    {
        int i = std::distance(this->impl->m_imagesList.begin(), imgIt);
        cv::Mat imgc = imgIt->clone();
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(imgc, patternSize, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (!found)
        {
            LOG_INFO("Bad chessboard corners of image: " + *pathIt);
            continue;
        }
        cv::Mat gray;
        cv::Mat imgcc = imgIt->clone();
        cv::cvtColor(imgcc, gray, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
        if (showCorners)
        {
            cv::drawChessboardCorners(imgcc, patternSize, corners, found);
            cv::imshow("corners", imgcc);
            cv::waitKey(5);
        }

        this->impl->m_goodImages.emplace_back(*pathIt, *imgIt, i);
        this->impl->m_corners.pixel_points.emplace_back(corners);
        if (writeCorners)
            cv::imwrite(cornersPath + "/" + pathIt->substr(pathIt->find_last_of("/\\") + 1), imgcc);
    }

    if (showCorners)
        cv::destroyWindow("corners");

    LOG_INFO("Detecting corners done.");
    auto goodCount = this->impl->m_goodImages.size();
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
    this->impl->m_corners.world_points.resize(goodCount, worldPoints);

    std::vector<double> stdDeviationsIntrinsics;
    std::vector<double> stdDeviationsExtrinsics;
    double rms = cv::calibrateCamera(this->impl->m_corners.world_points, this->impl->m_corners.pixel_points, this->impl->m_imageSize,
        this->impl->m_KMatrix, this->impl->m_DCoeffs, this->impl->m_RVecs, this->impl->m_tVecs,
        stdDeviationsIntrinsics, stdDeviationsExtrinsics, this->impl->m_perViewErrors);
    LOG_INFO("Calibrating camera done.");
    LOG_INFO_MSG_MAT("Camera Matrix:", this->impl->m_KMatrix);
    LOG_INFO_MSG_MAT("Distortion Matrix:", this->impl->m_DCoeffs);
    LOG_INFO_MSG("Overall images reproject RMS error = " + std::to_string(rms) + " px.");

    if (writeCorners)
    {
        LOG_INFO("Detected corners image save in folder: \"" + cornersPath + "\".");
    }

    if (error > 0)
    {
        //-- �����ͶӰ���
        LOG_INFO("Checking reproject RMS error...");
        auto bSize = this->impl->m_corners.pixel_points.size();
        auto pIt = this->impl->m_corners.pixel_points.begin();
        auto oIt = this->impl->m_corners.world_points.begin();
        auto eIt = this->impl->m_perViewErrors.begin();
        auto gIt = this->impl->m_goodImages.begin();
        for (; pIt != this->impl->m_corners.pixel_points.end(); )
        {
            if (*eIt <= error)
            {
                ++pIt, ++oIt, ++eIt, ++gIt;
                continue;
            }
            LOG_INFO("Removing image " + gIt->file_name + " which reproject RMS error > " + std::to_string(error) + " px...");
            pIt = this->impl->m_corners.pixel_points.erase(pIt);
            oIt = this->impl->m_corners.world_points.erase(oIt);
            eIt = this->impl->m_perViewErrors.erase(eIt);
            gIt = this->impl->m_goodImages.erase(gIt);
        }

        if (bSize != this->impl->m_corners.pixel_points.size() and this->impl->m_corners.pixel_points.size() >= 3)
        {
            LOG_INFO("ReCalibrating...");
            rms = cv::calibrateCamera(this->impl->m_corners.world_points, this->impl->m_corners.pixel_points, this->impl->m_imageSize,
                this->impl->m_KMatrix, this->impl->m_DCoeffs, this->impl->m_RVecs, this->impl->m_tVecs,
                stdDeviationsIntrinsics, stdDeviationsExtrinsics, this->impl->m_perViewErrors);
            LOG_INFO("Calibrating camera done. Camera param as follow: ");
            LOG_INFO_MSG_MAT("Camera Matrix:", this->impl->m_KMatrix);
            LOG_INFO_MSG_MAT("Distortion Matrix:", this->impl->m_DCoeffs);
            LOG_INFO_MSG("Overall images reproject RMS error = " + std::to_string(rms) + " px.");
        }
        eIt = this->impl->m_perViewErrors.begin();
        gIt = this->impl->m_goodImages.begin();
        for (; eIt != this->impl->m_perViewErrors.end(); ++eIt, ++gIt)
            LOG_INFO_MSG("Image " + gIt->file_name + " reproject RMS error = " + std::to_string(*eIt) + " px.");

    }    
    return rms;
}

void calib::CameraCalibrate::writeYAMLFile(const std::string& path) const
{
    std::string monoPath = std::string();
    if (path.empty())
        monoPath = this->impl->m_path + "/mono.yml";
    else
        monoPath = path;
    cv::FileStorage fs1(monoPath, cv::FileStorage::WRITE);
    if (fs1.isOpened())
    {
        fs1 << "K" << this->impl->m_KMatrix << "D" << this->impl->m_DCoeffs;
        LOG_INFO("Mono YML saved in floder: \"" + monoPath + "\".");
    }
    else
    {
        fs1.release();
        std::string msg = "Failed to save the mono YML.";
        LOG_ERROR(msg);
        throw std::runtime_error(msg);
    }
    fs1.release();
}

calib::CameraCalibrate::CCImpl::CCImpl()
{
}

calib::CameraCalibrate::CCImpl::~CCImpl()
{
}
