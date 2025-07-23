#include "../include/calib.h"
#include "../include/utils.h"
#include "../include/logger.h"
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>
#include <corecrt_math_defines.h>

static double rad2deg = 180.0 / M_PI;
static double deg2rad = M_PI / 180.0;

class calib::StereoHandEyeAXXB::SHEAXXBImpl
{
public:
	SHEAXXBImpl();
	~SHEAXXBImpl();
public:
    void tr_rpy2R(const cv::Vec3d& rpy, cv::Mat& R) const;

    void tr_R2rpy(const cv::Mat& R, cv::Vec3d& rpy) const;

    void tr_xyz2t(const cv::Vec3d& xyz, cv::Mat& t) const;

    void tr_Rt2T(const cv::Mat& R, const cv::Mat& t, cv::Mat& T) const;

    void tr_T2Rt(const cv::Mat& T, cv::Mat& R, cv::Mat& t) const;

    void computeBase2End(const std::vector<cv::Vec6d>& xyzrpy, std::vector<cv::Mat>& R_base2end, std::vector<cv::Mat>& t_base2end) const;

    void computeEnd2Base(const std::vector<cv::Vec6d>& xyzrpy, std::vector<cv::Mat>& R_end2base, std::vector<cv::Mat>& t_end2base) const;

    void computeObject2Camera(const std::vector<cv::Point3f>& objectCorners, const std::vector<std::vector<cv::Point2f>>& imagesCorners,
        const cv::Mat& K, const cv::Mat& D, std::vector<cv::Mat>& R_obj2cam, std::vector<cv::Mat>& t_obj2cam) const;

    void findCorners(const std::vector<cv::Mat>& images, std::vector<std::vector<cv::Point2f>>& imagesCorners, std::vector<cv::Point3f>& objectCorners);

    void rectifyImage(const cv::Mat& stereo, cv::Mat& left, cv::Mat& right) const;

    void computeEIHSSD(const std::vector<cv::Mat>& Rs_end2base, const std::vector<cv::Mat>& ts_end2base,
        const std::vector<cv::Mat>& Rs_obj2cam, const std::vector<cv::Mat>& ts_obj2cam, cv::Mat& T);

    void computeETHSSD(const std::vector<cv::Mat>& Rs_base2end, const std::vector<cv::Mat>& ts_base2end,
        const std::vector<cv::Mat>& Rs_obj2cam, const std::vector<cv::Mat>& ts_obj2cam, const cv::Mat T);

public:
    ChessboardParams m_chessboardParams;
    cv::Size m_imageSize;
    std::string m_pattern;
    std::vector<cv::Mat> m_imageList;
    std::vector<StereoPair<GoodChessboardImage>> m_goodImagesList;
    std::vector<cv::Vec6d> m_xyzrpy;
    CameraIntrinsic m_stereoIntrinsic; /*!< 双目内参 */
    EpipolarRectifyMap m_rectifyMap;   /*!< 双目极线校正映射 */
    std::vector<cv::Mat> m_TMatrix;    /*!< 齐次变换矩阵 */
    calib::StereoCalibrate* m_stereoCalib;
};

calib::StereoHandEyeAXXB::StereoHandEyeAXXB()
{
	this->impl = std::make_unique<SHEAXXBImpl>();
}

calib::StereoHandEyeAXXB::~StereoHandEyeAXXB()
{
}

void calib::StereoHandEyeAXXB::setChessboardParams(const ChessboardParams& chessboardParams)
{
    if (chessboardParams.board_height <= 0 or chessboardParams.board_width <= 0 or chessboardParams.square_size <= 0.f)
    {
        std::string msg = "Check input parameters";
        LOG_ERROR(msg);
        throw std::invalid_argument(msg);
    }
    this->impl->m_chessboardParams = chessboardParams;
    this->impl->m_stereoCalib->setChessboardParams(chessboardParams);
}

void calib::StereoHandEyeAXXB::loadChessboardImages(const std::string& pattern, const bool& recursive)
{
    this->impl->m_pattern = pattern;
    this->impl->m_stereoCalib->loadChessboardImages(pattern, recursive);
}

void calib::StereoHandEyeAXXB::loadXYZRPYFile(const std::string& csvFilePath)
{
    LOG_INFO("Loading end to base xyzrpy data...");
    std::ifstream xyzrpyData(csvFilePath, std::ios::in);
    if (!xyzrpyData.is_open())
    {
        xyzrpyData.close();
        LOG_ERROR("Failed to open csv file");
        throw std::invalid_argument("Invalid csv file");
    }
    std::string lineStr;
    std::vector<std::vector<double>> strArray;
    while (getline(xyzrpyData, lineStr))
    {
        std::stringstream ss(lineStr);
        std::string str;
        std::vector<double> lineArray;
        while (getline(ss, str, ','))
        {
            double strDouble;
            std::istringstream istr(str);
            istr >> strDouble;
            lineArray.push_back(strDouble);
        }
        strArray.push_back(lineArray);
    }
    xyzrpyData.close();
    auto it = std::next(strArray.begin());
    for (; it != strArray.end(); ++it)//从第2行开始
    {
        cv::Vec6d data;
        for (int j = 1; j <= 6; ++j)//从第2列开始
            data[j - 1] = (*it)[j];
        this->impl->m_xyzrpy.push_back(data);
    }
    LOG_INFO("Loaded end to base xyzrpy data.");
}

void calib::StereoHandEyeAXXB::calibrateEyeInHand(const bool& showCorners, const bool& writeCorners)
{
    LOG_INFO("Eye-in-Hand calibrating...");
    try
    {
        this->impl->m_stereoCalib->calibrate(-1., showCorners, writeCorners);
        this->impl->m_stereoCalib->writeYAMLFile();
        this->impl->m_goodImagesList = this->impl->m_stereoCalib->getGoodChessboardImages();
        auto stereoParams = this->impl->m_stereoCalib->getStereoParams();
        this->impl->m_imageSize = stereoParams.imgsz;
        cv::Mat K = (cv::Mat_<double>(3, 3) <<
            stereoParams.rectified_f,                        0, stereoParams.rectified_cx,
                                   0, stereoParams.rectified_f, stereoParams.rectified_cy,
                                   0,                        0,                         1);
        cv::Mat D = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);
        this->impl->m_stereoIntrinsic = CameraIntrinsic(K, D);
        this->impl->m_rectifyMap = EpipolarRectifyMap(stereoParams.map);
    }
    catch (const std::exception& e)
    {
        LOG_ERROR(e.what());
        throw std::invalid_argument("Error in computing stereo camera calibration");
    }
    
    for (auto it = this->impl->m_goodImagesList.begin(); it != this->impl->m_goodImagesList.end(); ++it)
    {
        cv::Mat left = it->left.image.clone();
        cv::Mat right = it->right.image.clone();    
        cv::Mat stereo;
        cv::hconcat(left, right, stereo);
        this->impl->rectifyImage(stereo, left, right);
        this->impl->m_imageList.push_back(left.clone());
    }
    //-- 找组图棋盘格角点图像坐标和世界坐标
    std::vector<std::vector<cv::Point2f>> imagesCorners;
    std::vector<cv::Point3f> worldCorners;
    this->impl->findCorners(this->impl->m_imageList, imagesCorners, worldCorners);
    //-- 计算End->Base
    std::vector<cv::Mat> Rs_end2base;
    std::vector<cv::Mat> ts_end2base;
    this->impl->computeEnd2Base(this->impl->m_xyzrpy, Rs_end2base, ts_end2base);
    //-- 计算Object->Camera
    std::vector<cv::Mat> Rs_obj2cam;
    std::vector<cv::Mat> ts_obj2cam;
    this->impl->computeObject2Camera(worldCorners, imagesCorners, this->impl->m_stereoIntrinsic.intrinsic_matrix, this->impl->m_stereoIntrinsic.distortion_coefficients, Rs_obj2cam, ts_obj2cam);
    LOG_INFO("Eye-in-Hand result:");
    for (int method = 0; method < 5; method++)
    {
        cv::HandEyeCalibrationMethod flag;
        switch (method)
        {
        case 0: flag = cv::CALIB_HAND_EYE_TSAI; break;
        case 1: flag = cv::CALIB_HAND_EYE_PARK; break;
        case 2: flag = cv::CALIB_HAND_EYE_HORAUD; break;
        case 3: flag = cv::CALIB_HAND_EYE_ANDREFF; break;
        case 4: flag = cv::CALIB_HAND_EYE_DANIILIDIS; break;
        default: break;
        }
        //-- Eye-in-hand手眼标定, 计算Camera->End
        cv::Mat R_cam2end;
        cv::Mat t_cam2end;
        cv::calibrateHandEye(Rs_end2base, ts_end2base, Rs_obj2cam,
            ts_obj2cam, R_cam2end, t_cam2end, flag);
        //-- R+t -> T
        //R
        cv::Mat T;
        this->impl->tr_Rt2T(R_cam2end, t_cam2end, T);
        this->impl->m_TMatrix.push_back(T.clone());
        switch (method)
        {
        case 0: LOG_INFO_MSG_MAT("TSAI:", T); break;
        case 1: LOG_INFO_MSG_MAT("PARK:", T); break;
        case 2: LOG_INFO_MSG_MAT("HORAUD:", T); break;
        case 3: LOG_INFO_MSG_MAT("ANDREFF:", T); break;
        case 4: LOG_INFO_MSG_MAT("DANIILIDIS:", T); break;
        default: break;
        }
    }
    LOG_INFO("Eye-in-Hand calibration done.");

    for (int method = 0; method < 5; method++)
    {
        switch (method)
        {
        case 0: LOG_INFO_MSG("----- TSAI -----"); break;
        case 1: LOG_INFO_MSG("----- PARK -----"); break;
        case 2: LOG_INFO_MSG("----- HORAUD -----"); break;
        case 3: LOG_INFO_MSG("----- ANDREFF -----"); break;
        case 4: LOG_INFO_MSG("----- DANIILIDIS -----"); break;
        default: break;
        }
        this->impl->computeEIHSSD(Rs_end2base, ts_end2base, Rs_obj2cam, ts_obj2cam, this->impl->m_TMatrix[method]);
    }
}

void calib::StereoHandEyeAXXB::calibrateEyeToHand(const bool& showCorners, const bool& writeCorners)
{
    LOG_INFO("Eye-to-Hand calibrating...");
    try
    {
        this->impl->m_stereoCalib->calibrate(-1., showCorners, writeCorners);
        this->impl->m_stereoCalib->writeYAMLFile();
        this->impl->m_goodImagesList = this->impl->m_stereoCalib->getGoodChessboardImages();
        auto stereoParams = this->impl->m_stereoCalib->getStereoParams();
        this->impl->m_imageSize = stereoParams.imgsz;
        cv::Mat K = (cv::Mat_<double>(3, 3) <<
            stereoParams.rectified_f, 0, stereoParams.rectified_cx,
            0, stereoParams.rectified_f, stereoParams.rectified_cy,
            0, 0, 1);
        cv::Mat D = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);
        this->impl->m_stereoIntrinsic = CameraIntrinsic(K, D);
        this->impl->m_rectifyMap = EpipolarRectifyMap(stereoParams.map);
    }
    catch (const std::exception& e)
    {
        LOG_ERROR(e.what());
        throw std::invalid_argument("Error in computing stereo camera calibration");
    }
    for (auto it = this->impl->m_goodImagesList.begin(); it != this->impl->m_goodImagesList.end(); ++it)
    {
        cv::Mat left = it->left.image.clone();
        cv::Mat right = it->right.image.clone();
        cv::Mat stereo;
        cv::hconcat(left, right, stereo);
        this->impl->rectifyImage(stereo, left, right);
        this->impl->m_imageList.push_back(left.clone());
    }
    //-- 找组图棋盘格角点图像坐标和世界坐标
    std::vector<std::vector<cv::Point2f>> imagesCorners;
    std::vector<cv::Point3f> worldCorners;
    this->impl->findCorners(this->impl->m_imageList, imagesCorners, worldCorners);
    //-- 计算Base->End
    std::vector<cv::Mat> Rs_base2end;
    std::vector<cv::Mat> ts_base2end;
    this->impl->computeBase2End(this->impl->m_xyzrpy, Rs_base2end, ts_base2end);
    //-- 计算Object->Camera
    std::vector<cv::Mat> Rs_obj2cam;
    std::vector<cv::Mat> ts_obj2cam;
    this->impl->computeObject2Camera(worldCorners, imagesCorners, this->impl->m_stereoIntrinsic.intrinsic_matrix, this->impl->m_stereoIntrinsic.distortion_coefficients, Rs_obj2cam, ts_obj2cam);
    LOG_INFO("Eye-to-Hand result:");
    for (int method = 0; method < 5; method++)
    {
        cv::HandEyeCalibrationMethod flag;
        switch (method)
        {
        case 0: flag = cv::CALIB_HAND_EYE_TSAI; break;
        case 1: flag = cv::CALIB_HAND_EYE_PARK; break;
        case 2: flag = cv::CALIB_HAND_EYE_HORAUD; break;
        case 3: flag = cv::CALIB_HAND_EYE_ANDREFF; break;
        case 4: flag = cv::CALIB_HAND_EYE_DANIILIDIS; break;
        default: break;
        }
        //-- Eye-to-hand手眼标定, 计算Camera->Base
        cv::Mat R_cam2base;
        cv::Mat t_cam2base;
        cv::calibrateHandEye(Rs_base2end, ts_base2end, Rs_obj2cam,
            ts_obj2cam, R_cam2base, t_cam2base, flag);
        //-- R+t -> T
        cv::Mat T;
        this->impl->tr_Rt2T(R_cam2base, t_cam2base, T);
        this->impl->m_TMatrix.push_back(T.clone());
        switch (method)
        {
        case 0: LOG_INFO_MSG_MAT("TSAI:", T); break;
        case 1: LOG_INFO_MSG_MAT("PARK:", T); break;
        case 2: LOG_INFO_MSG_MAT("HORAUD:", T); break;
        case 3: LOG_INFO_MSG_MAT("ANDREFF:", T); break;
        case 4: LOG_INFO_MSG_MAT("DANIILIDIS:", T); break;
        default: break;
        }
    }
    LOG_INFO("Eye-to-Hand calibration done.");
    for (int method = 0; method < 5; method++)
    {
        switch (method)
        {
        case 0: LOG_INFO_MSG("----- TSAI -----"); break;
        case 1: LOG_INFO_MSG("----- PARK -----"); break;
        case 2: LOG_INFO_MSG("----- HORAUD -----"); break;
        case 3: LOG_INFO_MSG("----- ANDREFF -----"); break;
        case 4: LOG_INFO_MSG("----- DANIILIDIS -----"); break;
        default: break;
        }
        this->impl->computeETHSSD(Rs_base2end, ts_base2end, Rs_obj2cam, ts_obj2cam, this->impl->m_TMatrix[method]);
    }
}

void calib::StereoHandEyeAXXB::writeYAMLFile(const std::string& path) const
{
    std::string tPath = std::string();
    if (path.empty())
    {
        size_t lastSlashPos = this->impl->m_pattern.find_last_of("/\\");
        std::string directory = (lastSlashPos != std::string::npos) ? this->impl->m_pattern.substr(0, lastSlashPos) : ".";
        std::string path = directory;
        tPath = path + "/yml/TMatrix.yml";
    }
    else
        tPath = path;
    utils::generateNewFolder(tPath);
    cv::FileStorage fs(tPath, cv::FileStorage::WRITE);
    if (fs.isOpened())
    {
        fs << "TSAI" << this->impl->m_TMatrix[0] << "PARK" << this->impl->m_TMatrix[1]
            << "HORAUD" << this->impl->m_TMatrix[2] << "ANDREFF" << this->impl->m_TMatrix[3]
            << "DANIILIDIS" << this->impl->m_TMatrix[4];
        fs.release();
        LOG_INFO("TMatrix YML file save in floder: \"" + tPath + "\".");
    }
    else
    {
        fs.release();
        std::string msg = "Failed to save the TMatrix YML.";
        LOG_ERROR(msg);
        throw std::runtime_error(msg);
    }
    fs.release();
}

calib::StereoHandEyeAXXB::SHEAXXBImpl::SHEAXXBImpl()
{
    this->m_stereoCalib = new calib::StereoCalibrate();
}

calib::StereoHandEyeAXXB::SHEAXXBImpl::~SHEAXXBImpl()
{
    if (this->m_stereoCalib != nullptr)
    {
        delete this->m_stereoCalib;
        this->m_stereoCalib = nullptr;
    }
}

void calib::StereoHandEyeAXXB::SHEAXXBImpl::tr_rpy2R(const cv::Vec3d& rpy, cv::Mat& R) const
{
    cv::Vec3d rpyc = rpy;
    for (int i = 0; i < 3; i++)
    {
        rpyc[i] *= deg2rad;
    }
    double rx = rpyc[0];
    double ry = rpyc[1];
    double rz = rpyc[2];
    double rxs = sin(rx), rxc = cos(rx);
    double rys = sin(ry), ryc = cos(ry);
    double rzs = sin(rz), rzc = cos(rz);
    cv::Mat rotx = (cv::Mat_<double>(3, 3) <<
        1, 0, 0,
        0, rxc, -rxs,
        0, rxs, rxc);
    cv::Mat roty = (cv::Mat_<double>(3, 3) <<
        ryc, 0, rys,
        0, 1, 0,
        -rys, 0, ryc);
    cv::Mat rotz = (cv::Mat_<double>(3, 3) <<
        rzc, -rzs, 0,
        rzs, rzc, 0,
        0, 0, 1);
    cv::Mat ttmp = rotz * roty * rotx;
    R = ttmp.clone();
}

void calib::StereoHandEyeAXXB::SHEAXXBImpl::tr_R2rpy(const cv::Mat& R, cv::Vec3d& rpy) const
{
    double roll = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    double pitch = atan2(-R.at<double>(2, 0), sqrt(R.at<double>(2, 1) * R.at<double>(
        2, 1) + R.at<double>(2, 2) * R.at<double>(2, 2)));
    double yaw = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    rpy = cv::Vec3d(roll, pitch, yaw);
}

void calib::StereoHandEyeAXXB::SHEAXXBImpl::tr_xyz2t(const cv::Vec3d& xyz, cv::Mat& t) const
{
    double x = xyz[0];
    double y = xyz[1];
    double z = xyz[2];
    cv::Mat ttmp = (cv::Mat_<double>(3, 1) << x, y, z);
    t = ttmp.clone();
}

void calib::StereoHandEyeAXXB::SHEAXXBImpl::tr_Rt2T(const cv::Mat& R, const cv::Mat& t, cv::Mat& T) const
{
    cv::Mat Rtmp = (cv::Mat_<double>(4, 3) <<
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
        0, 0, 0);
    cv::Mat ttmp = (cv::Mat_<double>(4, 1) <<
        t.at<double>(0, 0),
        t.at<double>(1, 0),
        t.at<double>(2, 0),
        1);
    cv::hconcat(Rtmp, ttmp, T);
}

void calib::StereoHandEyeAXXB::SHEAXXBImpl::tr_T2Rt(const cv::Mat& T, cv::Mat& R, cv::Mat& t) const
{
    cv::Rect R_Rect(0, 0, 3, 3);
    cv::Rect t_Rect(3, 0, 1, 3);
    R = T(R_Rect).clone();
    t = T(t_Rect).clone();
}

void calib::StereoHandEyeAXXB::SHEAXXBImpl::computeBase2End(const std::vector<cv::Vec6d>& xyzrpy, std::vector<cv::Mat>& R_base2end, std::vector<cv::Mat>& t_base2end) const
{
    //-- xyzrpy->T_end2base
    std::vector<cv::Mat> Ts_end2base;
    for (const auto& it : xyzrpy)
    {
        cv::Vec3d xyz, rpy;
        for (int i = 0; i < 3; i++)
        {
            xyz[i] = it[i];
            rpy[i] = it[i + 3];
        }
        //-- xyzrpy->R+t
        cv::Mat R, t;
        //rpy->R
        tr_rpy2R(rpy, R);
        //xyz->t
        tr_xyz2t(xyz, t);
        //R+t->T_end2base
        cv::Mat T_end2base;
        tr_Rt2T(R, t, T_end2base);
        Ts_end2base.push_back(T_end2base.clone());
    }
    //-- T_end2base -> T_base2end
    std::vector<cv::Mat> Ts_base2end;
    for (const auto& it : Ts_end2base)
    {
        Ts_base2end.push_back(it.inv());
    }
    //-- T_base2end -> R+t
    for (const auto& it : Ts_base2end)
    {
        cv::Mat R, t;
        tr_T2Rt(it, R, t);
        R_base2end.push_back(R.clone());
        t_base2end.push_back(t.clone());
    }
}

void calib::StereoHandEyeAXXB::SHEAXXBImpl::computeEnd2Base(const std::vector<cv::Vec6d>& xyzrpy, std::vector<cv::Mat>& R_end2base, std::vector<cv::Mat>& t_end2base) const
{
    //-- xyzrpy->T_end2base
    for (const auto& it : xyzrpy)
    {
        cv::Vec3d xyz, rpy;
        for (int i = 0; i < 3; i++)
        {
            xyz[i] = it[i];
            rpy[i] = it[i + 3];
        }
        //-- xyzrpy->R+t
        cv::Mat R, t;
        //rpy->R
        tr_rpy2R(rpy, R);
        //xyz->t
        tr_xyz2t(xyz, t);
        R_end2base.push_back(R.clone());
        t_end2base.push_back(t.clone());
    }
}

void calib::StereoHandEyeAXXB::SHEAXXBImpl::computeObject2Camera(const std::vector<cv::Point3f>& objectCorners, const std::vector<std::vector<cv::Point2f>>& imagesCorners, const cv::Mat& K, const cv::Mat& D, std::vector<cv::Mat>& R_obj2cam, std::vector<cv::Mat>& t_obj2cam) const
{
    for (const auto& it : imagesCorners)
    {
        cv::Mat R, t;
        cv::solvePnP(objectCorners, it, K, D, R, t);
        cv::Rodrigues(R, R);//3x1->3x3
        R_obj2cam.push_back(R.clone());
        t_obj2cam.push_back(t.clone());
    }
}

void calib::StereoHandEyeAXXB::SHEAXXBImpl::findCorners(const std::vector<cv::Mat>& images, std::vector<std::vector<cv::Point2f>>& imagesCorners, std::vector<cv::Point3f>& objectCorners)
{
    objectCorners = calib::generateWorldPoints(this->m_chessboardParams.board_height, this->m_chessboardParams.board_width, this->m_chessboardParams.square_size);
    for (const auto& it : images)
    {
        cv::Mat src = it.clone();
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(src, cv::Size(this->m_chessboardParams.board_width, this->m_chessboardParams.board_height), corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
        cv::Mat gray;
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
        imagesCorners.push_back(corners);
        //std::cout << found << std::endl;
        //cv::Mat draw = src.clone();
        //cv::drawChessboardCorners(draw, this->m_boardSize, corners, found);
        //cv::imshow("findChessboardCorners", draw);
        //cv::waitKey(0);
    }
}

void calib::StereoHandEyeAXXB::SHEAXXBImpl::rectifyImage(const cv::Mat& stereo, cv::Mat& left, cv::Mat& right) const
{
    cv::Mat srcc = stereo.clone();
    cv::Mat leftRaw = srcc(cv::Rect(0, 0, this->m_imageSize.width, this->m_imageSize.height)).clone();
    cv::Mat rightRaw = srcc(cv::Rect(this->m_imageSize.width, 0, this->m_imageSize.width, this->m_imageSize.height)).clone();
    cv::remap(leftRaw, left, this->m_rectifyMap.map00, this->m_rectifyMap.map01, cv::INTER_LINEAR);
    cv::remap(rightRaw, right, this->m_rectifyMap.map10, this->m_rectifyMap.map11, cv::INTER_LINEAR);
}

void calib::StereoHandEyeAXXB::SHEAXXBImpl::computeEIHSSD(const std::vector<cv::Mat>& Rs_end2base, const std::vector<cv::Mat>& ts_end2base, const std::vector<cv::Mat>& Rs_obj2cam, const std::vector<cv::Mat>& ts_obj2cam, cv::Mat& T)
{
    cv::Mat T_cam2end = T;
    std::vector<cv::Mat> Ts_obj2base;
    for (int i = 0; i < Rs_end2base.size(); ++i)
    {
        cv::Mat R_end2base = Rs_end2base[i].clone();
        cv::Mat t_end2base = ts_end2base[i].clone();
        cv::Mat R_obj2cam = Rs_obj2cam[i].clone();
        cv::Mat t_obj2cam = ts_obj2cam[i].clone();
        cv::Mat T_end2base;
        tr_Rt2T(R_end2base, t_end2base, T_end2base);
        cv::Mat T_obj2cam;
        tr_Rt2T(R_obj2cam, t_obj2cam, T_obj2cam);
        cv::Mat T_obj2base = T_end2base * T_cam2end * T_obj2cam;
        Ts_obj2base.push_back(T_obj2base.clone());
    }

    std::vector<double> std_err_samples = { 0.,0.,0.,0.,0.,0. }; // x, y, z, roll, pitch, yaw
    std::vector<double> mean_samples = { 0.,0.,0.,0.,0.,0. }; // x, y, z, roll, pitch, yaw
    for (int i = 0; i < Ts_obj2base.size(); ++i)
    {
        mean_samples[0] += Ts_obj2base[i].at<double>(0, 3); // x
        mean_samples[1] += Ts_obj2base[i].at<double>(1, 3); // y
        mean_samples[2] += Ts_obj2base[i].at<double>(2, 3); // z
        cv::Vec3d rpy;
        tr_R2rpy(Ts_obj2base[i].colRange(0, 3).rowRange(0, 3), rpy);
        mean_samples[3] += rpy[0]; // roll
        mean_samples[4] += rpy[1]; // pitch
        mean_samples[5] += rpy[2]; // yaw
    }
    for (auto& it : mean_samples)
        it /= Ts_obj2base.size();

    for (int i = 0; i < Ts_obj2base.size(); ++i)
    {
        std_err_samples[0] += pow(Ts_obj2base[i].at<double>(0, 3) - mean_samples[0], 2); // x
        std_err_samples[1] += pow(Ts_obj2base[i].at<double>(1, 3) - mean_samples[1], 2); // y
        std_err_samples[2] += pow(Ts_obj2base[i].at<double>(2, 3) - mean_samples[2], 2); // z
        cv::Vec3d rpy;
        tr_R2rpy(Ts_obj2base[i].colRange(0, 3).rowRange(0, 3), rpy);
        std_err_samples[3] += pow(rpy[0] - mean_samples[3], 2); // roll
        std_err_samples[4] += pow(rpy[1] - mean_samples[4], 2); // pitch
        std_err_samples[5] += pow(rpy[2] - mean_samples[5], 2); // yaw
    }
    for (auto& it : std_err_samples)
        it = sqrt(it / (Ts_obj2base.size() - 1));
    LOG_INFO_MSG("Sample standard deviation:");
    LOG_INFO_MSG("x: " + std::to_string(std_err_samples[0]));
    LOG_INFO_MSG("y: " + std::to_string(std_err_samples[1]));
    LOG_INFO_MSG("z: " + std::to_string(std_err_samples[2]));
    LOG_INFO_MSG("Roll: " + std::to_string(std_err_samples[3] * rad2deg));
    LOG_INFO_MSG("Pitch: " + std::to_string(std_err_samples[4] * rad2deg));
    LOG_INFO_MSG("Yaw: " + std::to_string(std_err_samples[5] * rad2deg));
    LOG_INFO_MSG("translation: " + std::to_string(sqrt(std_err_samples[0] * std_err_samples[0] + std_err_samples[1] * std_err_samples[1] + std_err_samples[2] * std_err_samples[2])));
    LOG_INFO_MSG("rotation: " + std::to_string(sqrt(std_err_samples[3] * std_err_samples[3] + std_err_samples[4] * std_err_samples[4] + std_err_samples[5] * std_err_samples[5]) * rad2deg));
}

void calib::StereoHandEyeAXXB::SHEAXXBImpl::computeETHSSD(const std::vector<cv::Mat>& Rs_base2end, const std::vector<cv::Mat>& ts_base2end, const std::vector<cv::Mat>& Rs_obj2cam, const std::vector<cv::Mat>& ts_obj2cam, const cv::Mat T)
{
    cv::Mat T_cam2base = T;
    std::vector<cv::Mat> Ts_obj2end;
    for (int i = 0; i < Rs_base2end.size(); ++i)
    {
        cv::Mat R_base2end = Rs_base2end[i].clone();
        cv::Mat t_base2end = ts_base2end[i].clone();
        cv::Mat R_obj2cam = Rs_obj2cam[i].clone();
        cv::Mat t_obj2cam = ts_obj2cam[i].clone();
        cv::Mat T_base2end;
        tr_Rt2T(R_base2end, t_base2end, T_base2end);
        cv::Mat T_obj2cam;
        tr_Rt2T(R_obj2cam, t_obj2cam, T_obj2cam);
        cv::Mat T_obj2end = T_base2end * T_cam2base * T_obj2cam;
        Ts_obj2end.push_back(T_obj2end.clone());
    }
    std::vector<double> std_err_samples = { 0.,0.,0.,0.,0.,0. }; // x, y, z, roll, pitch, yaw
    std::vector<double> mean_samples = { 0.,0.,0.,0.,0.,0. }; // x, y, z, roll, pitch, yaw
    for (int i = 0; i < Ts_obj2end.size(); ++i)
    {
        mean_samples[0] += Ts_obj2end[i].at<double>(0, 3); // x
        mean_samples[1] += Ts_obj2end[i].at<double>(1, 3); // y
        mean_samples[2] += Ts_obj2end[i].at<double>(2, 3); // z
        cv::Vec3d rpy;
        tr_R2rpy(Ts_obj2end[i].colRange(0, 3).rowRange(0, 3), rpy);
        mean_samples[3] += rpy[0]; // roll
        mean_samples[4] += rpy[1]; // pitch
        mean_samples[5] += rpy[2]; // yaw
    }
    for (auto& it : mean_samples)
        it /= Ts_obj2end.size();

    for (int i = 0; i < Ts_obj2end.size(); ++i)
    {
        std_err_samples[0] += pow(Ts_obj2end[i].at<double>(0, 3) - mean_samples[0], 2); // x
        std_err_samples[1] += pow(Ts_obj2end[i].at<double>(1, 3) - mean_samples[1], 2); // y
        std_err_samples[2] += pow(Ts_obj2end[i].at<double>(2, 3) - mean_samples[2], 2); // z
        cv::Vec3d rpy;
        tr_R2rpy(Ts_obj2end[i].colRange(0, 3).rowRange(0, 3), rpy);
        std_err_samples[3] += pow(rpy[0] - mean_samples[3], 2); // roll
        std_err_samples[4] += pow(rpy[1] - mean_samples[4], 2); // pitch
        std_err_samples[5] += pow(rpy[2] - mean_samples[5], 2); // yaw
    }
    for (auto& it : std_err_samples)
        it = sqrt(it / (Ts_obj2end.size() - 1));
    LOG_INFO_MSG("Sample standard deviation:");
    LOG_INFO_MSG("x: "+ std::to_string(std_err_samples[0]));
    LOG_INFO_MSG("y: "+ std::to_string(std_err_samples[1]));
    LOG_INFO_MSG("z: "+ std::to_string(std_err_samples[2]));
    LOG_INFO_MSG("Roll: "+ std::to_string(std_err_samples[3] * rad2deg));
    LOG_INFO_MSG("Pitch: "+ std::to_string(std_err_samples[4] * rad2deg));
    LOG_INFO_MSG("Yaw: "+ std::to_string(std_err_samples[5] * rad2deg));
    LOG_INFO_MSG("translation: "+ std::to_string(sqrt(std_err_samples[0] * std_err_samples[0] + std_err_samples[1] * std_err_samples[1] + std_err_samples[2] * std_err_samples[2])));
    LOG_INFO_MSG("rotation: "+ std::to_string(sqrt(std_err_samples[3] * std_err_samples[3] + std_err_samples[4] * std_err_samples[4] + std_err_samples[5] * std_err_samples[5]) * rad2deg));
}
