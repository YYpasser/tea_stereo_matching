/*********************************************************************
 * @file   calib.h
 * @brief  ��Ҷ��ѿ��ժ�����Ӿ�����궨/���۱궨ģ��
 * @author Qianyao Zhuang
 * @date   May 2025
 * 
 * @details
 * ��ģ��ʵ����˫Ŀ����궨��˫Ŀ���۱궨, ��Ҫ������������:
 * 1. ˫Ŀ����궨, ���̸�궨
 * 2. ˫Ŀ���۱궨, ���̸�궨, ��� AX = XB ����
 * 3. ˫Ŀ���۱궨, ����� + SVD�ֽ�
 * 
 * @section ע������
 * 1. ʹ������� + SVD�ֽ�ʱ, ������ǿָ�
 *    -- �������� -> C/C++ -> �������� -> ������ǿָ� -> �߼�ʸ����չ (X86/X64) (/arch:AVX)
 * 
 * @code demo 1 - ����궨
	calib::CameraCalibrate cc;
	cc.setChessboardParams(ChessboardParams(11, 8, 6.f));
	cc.loadChessboardImages("../demo-imgs/calib/left/*", false);
	cc.calibrate();
	cc.writeYAMLFile();
 * @code demo 1 - ����궨
 * 
 * @code demo 2 - ˫Ŀ����궨
	calib::StereoCalibrate sc;
	sc.setChessboardParams(ChessboardParams(11, 8, 6.f));
	sc.loadChessboardImages("../demo-imgs/calib/*", false);
	sc.calibrate();
	sc.writeYAMLFile();
 * @endcode demo 2 - ˫Ŀ����궨
 * 
 * @code demo 3 - ˫Ŀ���۱궨 Eye-to-Hand, AXXB
	calib::StereoHandEyeAXXB he;
	he.setChessboardParams(ChessboardParams(11, 8, 6.f));
	he.loadChessboardImages("../demo-imgs/calib/*.bmp");
	he.loadXYZRPYFile("../demo-imgs/calib/HandEye.csv");
	he.calibrateEyeToHand();
	he.writeYAMLFile();
 * @endcode demo 3 - ˫Ŀ���۱궨 Eye-to-Hand, AXXB
 * 
 * @code demo 4 - ˫Ŀ���۱궨 Hand-to-Eye, SVD
	calib::StereoHandEyeSVD calib;
	calib.loadImages("../demo-imgs/20240316HandEyeSVD/left/*", "../demo-imgs/20240316HandEyeSVD/xyz/*");
	calib.loadXYZFile("../demo-imgs/20240316HandEyeSVD/HandEye.csv");
	calib.calibrate();
	calib.writeYAMLFile("../demo-imgs/20240316HandEyeSVD");
 * @endcode demo 4 - ˫Ŀ���۱궨 Hand-to-Eye, SVD
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
 * @brief �������̸�ǵ���������.
 */
std::vector<cv::Point3f> generateWorldPoints(const int& rows, const int& cols, const float& squareSize);
/**
 * @brief ������ͶӰ���.
 * @return ��ͶӰ���
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
	 * @brief �������̸�궨�����.
	 * @param [in] chessboardParams ���̸�궨�����.
	 */
	void setChessboardParams(const ChessboardParams& chessboardParams);
	/**
	 * @brief �������̸�궨��ͼ��.
	 * @param [in] pattern   ���̸�궨��ͼ��·��.
	 * @param [in] recursive �Ƿ�ݹ�����.
	 */
	void loadChessboardImages(const std::string& pattern, const bool& recursive = false);
	/**
	 * @brief �궨.
	 * @param [in] error        ������ͶӰ���, Ĭ��Ϊ0.1px.
	 * @param [in] showCorners  �Ƿ���չʾ����, Ĭ��Ϊtrue
	 * @param [in] writeCorners �Ƿ񱣴�ǵ�����, Ĭ��Ϊtrue
	 * @return ��ͶӰ���
	 */
	double calibrate(const float& error = 0.1f, const bool& showCorners = true, const bool& writeCorners = true);
	/**
	 * @brief ����˫Ŀ����YAML�ļ�.
	 * @param [in] path ˫Ŀ����YAML�ļ�·��.
	 */
	void writeYAMLFile(const std::string& path = std::string()) const;

private:
    class CCImpl;
    std::unique_ptr<CCImpl> impl;
};

/**
 * @brief ˫Ŀ�궨.
 */
class StereoCalibrate
{
public:
	StereoCalibrate();
	~StereoCalibrate();
public:
    /**
     * @brief �������̸�궨�����.
     * @param [in] chessboardParams ���̸�궨�����.
     */
    void setChessboardParams(const ChessboardParams& chessboardParams);
    /**
     * @brief �������̸�궨��ͼ��.
     * @param [in] pattern   ���̸�궨��ͼ��·��.
     * @param [in] recursive �Ƿ�ݹ�����.
     */
    void loadChessboardImages(const std::string& pattern, const bool& recursive = false);
    /**
     * @brief �궨.
     * @param [in] error        ������ͶӰ���, Ĭ��Ϊ0.1px.
     * @param [in] showCorners  �Ƿ���չʾ����, Ĭ��Ϊtrue
     * @param [in] writeCorners �Ƿ񱣴�ǵ�����, Ĭ��Ϊtrue
     * @return ��ͶӰ���
     */
    double calibrate(const float& error = 0.1f, const bool& showCorners = true, const bool& writeCorners = true);
    /**
     * @brief ����˫Ŀ����YAML�ļ�.
     * @param [in] path ˫Ŀ����YAML�ļ�·��.
     */
    void writeYAMLFile(const std::string& path = std::string()) const;
public:
	/**
	 * @brief ��ȡ˫Ŀ����.
	 * @return ˫Ŀ����.
	 */
	StereoParams getStereoParams() const;
	/**
	 * @brief ��ȡ�����������̸����ת�任.
	 * @return �����������̸����ת�任
	 */
	StereoPair<std::vector<cv::Mat>> getCameraRotations() const;
	/**
	 * @brief ��ȡ�����������̸��ƽ�Ʊ任.
	 * @return �����������̸��ƽ�Ʊ任
	 */
	StereoPair<std::vector<cv::Mat>> getCameraTranslations() const;
	/**
	 * @brief ��ȡ���̸�ǵ�����.
	 * @return ���̸�ǵ�����
	 */
	StereoPair<ChessboardCorners> getChessboardCorners() const;
	/**
	 * @brief ��ȡ�������̸�ͼ��.
	 * @return �������̸�ͼ��
	 */
	std::vector<StereoPair<GoodChessboardImage>> getGoodChessboardImages() const;
	/**
	 * @brief ��ȡÿ���ӽǵ���ͶӰ���.
	 * @return ÿ���ӽǵ���ͶӰ���
	 */
	StereoPair<std::vector<double>> getPerViewErrors() const;
private:
	class SCImpl;
	std::unique_ptr<SCImpl> impl;
};

/**
 * @brief ˫Ŀ���۱궨, ���AX=XB��.
 */
class StereoHandEyeAXXB
{
public:
	StereoHandEyeAXXB();
	~StereoHandEyeAXXB();
public:
	/**
	 * @brief �������̸�궨�����.
	 * @param [in] chessboardParams ���̸�궨�����.
	 */
	void setChessboardParams(const ChessboardParams& chessboardParams);
	/**
	 * @brief �������̸�궨��ͼ��.
	 * @param [in] pattern   ���̸�궨��ͼ��·��.
	 * @param [in] recursive �Ƿ�ݹ�����.
	 */
	void loadChessboardImages(const std::string& pattern, const bool& recursive = false);
	/**
	 * @brief ����ĩ������[XYZRPY]csv�ļ�.
	 * @param [in] csvFilePath ĩ������[XYZRPY]csv�ļ�
	 */
	void loadXYZRPYFile(const std::string& csvFilePath);
	/**
	 * @brief Eye in hand ���۱궨.
	 * @param [in] showCorners  չʾ�ǵ�����
	 * @param [in] writeCorners ����ǵ�����
	 */
	void calibrateEyeInHand(const bool& showCorners = true, const bool& writeCorners = true);
	/**
	 * @brief Eye to hand ���۱궨.
	 * @param [in] showCorners  չʾ�ǵ�����
	 * @param [in] writeCorners ����ǵ�����
	 */
	void calibrateEyeToHand(const bool& showCorners = true, const bool& writeCorners = true);
	/**
	 * @brief ����˫Ŀ����YAML�ļ�.
	 * @param [in] path ˫Ŀ����YAML�ļ�·��.
	 */
	void writeYAMLFile(const std::string& path = std::string()) const;

private:
    class SHEAXXBImpl;
    std::unique_ptr<SHEAXXBImpl> impl;
};

/**
 * @brief ˫Ŀ���۱궨, ������������任��ϵ, ����� + SVD�ֽⷨ.
 */
class StereoHandEyeSVD
{
public:
    StereoHandEyeSVD();
	~StereoHandEyeSVD();
public:
	/**
	 * @brief ����ĩ������ƫ��.
	 * @param [in] x
	 * @param [in] y
	 * @param [in] z
	 */
	void setGripperPosition(const float& x, const float& y, const float& z);
	/**
	 * @brief ���ر궨ͼ��.
	 * @param [in] leftRectifiedPattern ����У������Ŀͼ��·��
	 * @param [in] xyzMatrixPattern     ����ƥ��+��ͶӰ -> xyz�������
	 */
	void loadImages(const std::string& leftRectifiedPattern, const std::string& xyzMatrixPattern);
	/**
	 * @brief ����ĩ��XYZ�����ļ�.
	 * @param [in] csvFilePath ĩ��XYZ�����ļ�
	 */
	void loadXYZFile(const std::string& csvFilePath);
	/**
	 * @brief �궨.
	 * @return ��ͶӰ���
	 */
	double calibrate();
	/**
	 * @brief ����궨���YAML�ļ�.
	 * @param [in] path �궨���YAML�ļ�����·��
	 */
	void writeYAMLFile(const std::string& path) const;

private:
	class SHESVDImpl;
    std::unique_ptr<SHESVDImpl> impl;
};

}
