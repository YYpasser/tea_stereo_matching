/*********************************************************************
 * @file   calib.h
 * @brief  茶叶嫩芽采摘立体视觉相机标定/手眼标定模块
 * @author Qianyao Zhuang
 * @date   May 2025
 * 
 * @details
 * 本模块实现了双目相机标定和双目手眼标定, 主要包含三个功能:
 * 1. 双目相机标定, 棋盘格标定
 * 2. 双目手眼标定, 棋盘格标定, 求解 AX = XB 方程
 * 3. 双目手眼标定, 球拟合 + SVD分解
 * 
 * @section 注意事项
 * 1. 使用球拟合 + SVD分解时, 开启增强指令集
 *    -- 属性配置 -> C/C++ -> 代码生成 -> 启用增强指令集 -> 高级矢量扩展 (X86/X64) (/arch:AVX)
 * 
 * @code demo 1 - 相机标定
	calib::CameraCalibrate cc;
	cc.setChessboardParams(ChessboardParams(11, 8, 6.f));
	cc.loadChessboardImages("../demo-imgs/calib/left/*", false);
	cc.calibrate();
	cc.writeYAMLFile();
 * @code demo 1 - 相机标定
 * 
 * @code demo 2 - 双目相机标定
	calib::StereoCalibrate sc;
	sc.setChessboardParams(ChessboardParams(11, 8, 6.f));
	sc.loadChessboardImages("../demo-imgs/calib/*", false);
	sc.calibrate();
	sc.writeYAMLFile();
 * @endcode demo 2 - 双目相机标定
 * 
 * @code demo 3 - 双目手眼标定 Eye-to-Hand, AXXB
	calib::StereoHandEyeAXXB he;
	he.setChessboardParams(ChessboardParams(11, 8, 6.f));
	he.loadChessboardImages("../demo-imgs/calib/*.bmp");
	he.loadXYZRPYFile("../demo-imgs/calib/HandEye.csv");
	he.calibrateEyeToHand();
	he.writeYAMLFile();
 * @endcode demo 3 - 双目手眼标定 Eye-to-Hand, AXXB
 * 
 * @code demo 4 - 双目手眼标定 Hand-to-Eye, SVD
	calib::StereoHandEyeSVD calib;
	calib.loadImages("../demo-imgs/20240316HandEyeSVD/left/*", "../demo-imgs/20240316HandEyeSVD/xyz/*");
	calib.loadXYZFile("../demo-imgs/20240316HandEyeSVD/HandEye.csv");
	calib.calibrate();
	calib.writeYAMLFile("../demo-imgs/20240316HandEyeSVD");
 * @endcode demo 4 - 双目手眼标定 Hand-to-Eye, SVD
 *********************************************************************/
#pragma once
#include "calib_utils.h"
#include "stereo_utils.h"
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <memory>

namespace calib
{

/**
 * @brief 生成棋盘格角点世界坐标.
 */
std::vector<cv::Point3f> generateWorldPoints(const int& rows, const int& cols, const float& squareSize);
/**
 * @brief 计算重投影误差.
 * @return 重投影误差
 */
double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f>>& worldPoints, const std::vector<std::vector<cv::Point2f>>& pixelPoints,
		const std::vector<cv::Mat>& RVecs, const std::vector<cv::Mat>& tVecs, const cv::Mat& intrinsicMatrix, const cv::Mat& distortionCoefficients, std::vector<double>& perViewErrors);

class CameraCalibrate
{
public:
    CameraCalibrate();
	~CameraCalibrate();
public:
	/**
	 * @brief 设置棋盘格标定板参数.
	 * @param [in] chessboardParams 棋盘格标定板参数.
	 */
	void setChessboardParams(const ChessboardParams& chessboardParams);
	/**
	 * @brief 加载棋盘格标定板图像.
	 * @param [in] pattern   棋盘格标定板图像路径.
	 * @param [in] recursive 是否递归搜索.
	 */
	void loadChessboardImages(const std::string& pattern, const bool& recursive = false);
	/**
	 * @brief 标定.
	 * @param [in] error        允许重投影误差, 默认为0.1px.
	 * @param [in] showCorners  是否开启展示界面, 默认为true
	 * @param [in] writeCorners 是否保存角点检测结果, 默认为true
	 * @return 重投影误差
	 */
	double calibrate(const float& error = 0.1f, const bool& showCorners = true, const bool& writeCorners = true);
	/**
	 * @brief 保存双目参数YAML文件.
	 * @param [in] path 双目参数YAML文件路径.
	 */
	void writeYAMLFile(const std::string& path = std::string()) const;

private:
    class CCImpl;
    std::unique_ptr<CCImpl> impl;
};

/**
 * @brief 双目标定.
 */
class StereoCalibrate
{
public:
	StereoCalibrate();
	~StereoCalibrate();
public:
    /**
     * @brief 设置棋盘格标定板参数.
     * @param [in] chessboardParams 棋盘格标定板参数.
     */
    void setChessboardParams(const ChessboardParams& chessboardParams);
    /**
     * @brief 加载棋盘格标定板图像.
     * @param [in] pattern   棋盘格标定板图像路径.
     * @param [in] recursive 是否递归搜索.
     */
    void loadChessboardImages(const std::string& pattern, const bool& recursive = false);
    /**
     * @brief 标定.
     * @param [in] error        允许重投影误差, 默认为0.1px.
     * @param [in] showCorners  是否开启展示界面, 默认为true
     * @param [in] writeCorners 是否保存角点检测结果, 默认为true
     * @return 重投影误差
     */
    double calibrate(const float& error = 0.1f, const bool& showCorners = true, const bool& writeCorners = true);
    /**
     * @brief 保存双目参数YAML文件.
     * @param [in] path 双目参数YAML文件路径.
     */
    void writeYAMLFile(const std::string& path = std::string()) const;
public:
	/**
	 * @brief 获取双目参数.
	 * @return 双目参数.
	 */
	stereo::StereoParams getStereoParams() const;
	/**
	 * @brief 获取相机相对于棋盘格的旋转变换.
	 * @return 相机相对于棋盘格的旋转变换
	 */
	stereo::StereoPair<std::vector<cv::Mat>> getCameraRotations() const;
	/**
	 * @brief 获取相机相对于棋盘格的平移变换.
	 * @return 相机相对于棋盘格的平移变换
	 */
	stereo::StereoPair<std::vector<cv::Mat>> getCameraTranslations() const;
	/**
	 * @brief 获取棋盘格角点坐标.
	 * @return 棋盘格角点坐标
	 */
	stereo::StereoPair<ChessboardCorners> getChessboardCorners() const;
	/**
	 * @brief 获取优质棋盘格图像.
	 * @return 优质棋盘格图像
	 */
	std::vector<stereo::StereoPair<GoodChessboardImage>> getGoodChessboardImages() const;
	/**
	 * @brief 获取每个视角的重投影误差.
	 * @return 每个视角的重投影误差
	 */
	stereo::StereoPair<std::vector<double>> getPerViewErrors() const;
private:
	class SCImpl;
	std::unique_ptr<SCImpl> impl;
};

/**
 * @brief 双目手眼标定, 求解AX=XB法.
 */
class StereoHandEyeAXXB
{
public:
	StereoHandEyeAXXB();
	~StereoHandEyeAXXB();
public:
	/**
	 * @brief 设置棋盘格标定板参数.
	 * @param [in] chessboardParams 棋盘格标定板参数.
	 */
	void setChessboardParams(const ChessboardParams& chessboardParams);
	/**
	 * @brief 加载棋盘格标定板图像.
	 * @param [in] pattern   棋盘格标定板图像路径.
	 * @param [in] recursive 是否递归搜索.
	 */
	void loadChessboardImages(const std::string& pattern, const bool& recursive = false);
	/**
	 * @brief 加载末端坐标[XYZRPY]csv文件.
	 * @param [in] csvFilePath 末端坐标[XYZRPY]csv文件
	 */
	void loadXYZRPYFile(const std::string& csvFilePath);
	/**
	 * @brief Eye in hand 手眼标定.
	 * @param [in] showCorners  展示角点检测结果
	 * @param [in] writeCorners 保存角点检测结果
	 */
	void calibrateEyeInHand(const bool& showCorners = true, const bool& writeCorners = true);
	/**
	 * @brief Eye to hand 手眼标定.
	 * @param [in] showCorners  展示角点检测结果
	 * @param [in] writeCorners 保存角点检测结果
	 */
	void calibrateEyeToHand(const bool& showCorners = true, const bool& writeCorners = true);
	/**
	 * @brief 保存双目参数YAML文件.
	 * @param [in] path 双目参数YAML文件路径.
	 */
	void writeYAMLFile(const std::string& path = std::string()) const;

private:
    class SHEAXXBImpl;
    std::unique_ptr<SHEAXXBImpl> impl;
};

/**
 * @brief 双目手眼标定, 求解两组点坐标变换关系, 球拟合 + SVD分解法.
 */
class StereoHandEyeSVD
{
public:
    StereoHandEyeSVD();
	~StereoHandEyeSVD();
public:
	/**
	 * @brief 设置末端坐标偏移.
	 * @param [in] x
	 * @param [in] y
	 * @param [in] z
	 */
	void setGripperPosition(const float& x, const float& y, const float& z);
	/**
	 * @brief 加载标定图像.
	 * @param [in] leftRectifiedPattern 极线校正后左目图像路径
	 * @param [in] xyzMatrixPattern     立体匹配+重投影 -> xyz坐标矩阵
	 */
	void loadImages(const std::string& leftRectifiedPattern, const std::string& xyzMatrixPattern);
	/**
	 * @brief 加载末端XYZ坐标文件.
	 * @param [in] csvFilePath 末端XYZ坐标文件
	 */
	void loadXYZFile(const std::string& csvFilePath);
	/**
	 * @brief 标定.
	 * @return 重投影误差
	 */
	double calibrate();
	/**
	 * @brief 保存标定结果YAML文件.
	 * @param [in] path 标定结果YAML文件保存路径
	 */
	void writeYAMLFile(const std::string& path) const;

private:
	class SHESVDImpl;
    std::unique_ptr<SHESVDImpl> impl;
};

}
