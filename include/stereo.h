/*********************************************************************
 * @file   stereo.h
 * @brief  ��Ҷ��ѿ��ժ�����Ӿ�����ģ��.
 * @author Qianyao Zhuang
 * @date   May 2025
 * 
 * @details
 * ��ģ��ʵ���˻���˫Ŀ�����Ӿ��Ĳ�Ҷ��ѿ��ά���ƻ�ȡ, ��Ҫ�����������Ĺ���: 
 * 1. ͼ����У��: ����ƥ��Ԥ����, ʹ������ͼ��Ե��������ж���
 * 2. ����ƥ���㷨: ��ͳ�����Լ����ѧϰģ��(ONNXRuntime/TensorRT)
 * 3. ��ά�����ؽ�: �����Ӳ�ͼ���������, ���ɲ�Ҷ��������
 * 
 * @section �㷨���
 * [ģ��ܹ�]
 * �������ͼ��� (����) -> [����У��] -> [����ƥ��] -> [��������] -> �������� (���)
 * 
 * @section ����ϸ��
 * - ����ƥ���㷨: 
 *   - ��ͳ����: AD-Census
 *   - ���ѧϰ: IGEV-Stereo, FFLO-Net�ȶ˵���ģ��
 * - �����ʽ: 
 *   - �м���: У����ͼ���Ӳ�ͼ
 *   - ���ս��: ����(XYZRGB)
 * 
 * @section ʹ��˵��
 * һ������:
 * 1. ��������������㷨����(ģ��)
 * 2. ��������ͼ���
 * 3. ����У��
 * 4. ѡ��ִ������ƥ���㷨
 * 5. ������ά����
 * 
 * @section ע������
 * 1. ONNXRuntime-gpu �汾Ϊ1.18.1
 * 2. TensorRT �汾Ϊ10.10
 * 3. Ŀǰģ�ͽ������ڹ̶��ߴ�ͼ��, 2560*720
 * 4. ʹ��AD-Census�㷨ʱ, ����OpenMP����
 *    -- �������� -> C/C++ -> ���� -> OpenMP֧�� -> �� (/openmp)
 * 
 * @references
 * [1] AD-Census: 
 *     -- [On Building an Accurate Stereo Matching System on Graphics Hardware]
 *        (https://doi.org/10.1109/ICCVW.2011.6130280)
 *        Xing Mei, Xun Sun, Mingcai Zhou, Shaohui Jiao, Haitao Wang, Xiaopeng Zhang.
 *     -- [StereoVision-ADCensus]
 *        (https://github.com/DLuensch/StereoVision-ADCensus)
 * [2] IGEV-Stereo:
 *     -- [Iterative Geometry Encoding Volume for Stereo Matching]
 *		  (https://arxiv.org/pdf/2303.06615.pdf)
 *		  Gangwei Xu, Xianqi Wang, Xiaohuan Ding, Xin Yang.
 * [3] FFLO-Net:
 *     -- [Stereo Matching Network with Attention-Guided Feature Fusion and ConvLSTM Optimization for Tea Shoot Disparity Prediction]
 *        (...)
 *        Leiying He, Qianyao Zhuang, Yatao Li, ...
 *     -- KITTI Benchmark (stereo): FFLO-Net
 *        |-------------|--------------------|---------------------|---------------------|------------------|
 *        | Method      | KITTI 2012 (3-noc) | KITTI 2015 (D1-all) | Scene Flow (D1-all) | Scene Flow (EPE) |
 *        |-------------|--------------------|---------------------|---------------------|------------------|
 *        | IGEV-Stereo |       1.12 %       |       1.59 %        |       2.47 %        |     0.47 px      |
 *        |  FFLO-Net   |    ** 1.04 % **    |    ** 1.49 % **     |    ** 2.31 % **     |  ** 0.44 px **   |
 *        |-------------|--------------------|---------------------|---------------------|------------------|
 * 
 * [4] Selective AD-Census-HSI
 *     -- [Localization of Tea Shoots for Robotic Plucking using Binocular Stereo Vision]
 *        (https://doi.org/10.1002/rob.22559)
 *        Leiying He, Qianyao Zhuang, Yatao Li, Zhenghao Zhong, Jianneng Chen, Chuanyu Wu.
 *
 * @code demo 1 - Stereo Epipolar Rectification
    cv::Mat stereoImage = cv::imread("../demo-imgs/0071-Stereo.bmp");
    StereoParams params("../yml/stereo.yml");
    stereo::EpipolarRectify rectify(params.map, params.imgsz);
    auto rectifiedPair = rectify.rectify(stereoImage);
    auto imgWithLine = stereo::drawHorizontalLines(rectifiedPair);
    cv::namedWindow("rectified", cv::WINDOW_NORMAL);
    cv::imshow("rectified", imgWithLine);
    cv::waitKey(0);
 * @endcode demo 1 - Stereo Epipolar Rectification
 *  
 * @code demo 2 - ADCensus
    cv::Mat leftImage = cv::imread("../demo-imgs/0600-Left.bmp");
    cv::Mat rightImage = cv::imread("../demo-imgs/0600-Right.bmp");
    stereo::ADCensus adcensus;
    adcensus.setMatchingStrategy(ColorModel::RGB, false, false);
    adcensus.setMinMaxDisparity(0, 192);
    auto disparity = adcensus.compute(leftImage, rightImage);
    auto disparity_color = stereo::applyMapping(disparity);
    cv::imshow("disparity", disparity_color);
    cv::waitKey(0);
 * @endcode demo 2 - ADCensus
 *
 * @code demo 3 - TensorRT
    cv::Mat leftImage = cv::imread("../demo-imgs/0600-Left.bmp");
    cv::Mat rightImage = cv::imread("../demo-imgs/0600-Right.bmp");
    stereo::StereoMatchingTensorRT smtrt;
    smtrt.loadEngine("../models/FFLO_it32.trt");
    auto disparity = smtrt.compute(leftImage, rightImage);
    auto disparity_color = stereo::applyMapping(disparity);
    cv::imshow("disparity", disparity_color);
    cv::waitKey(0);
 * @endcode demo 3 - TensorRT
 * 
 * @code demo 4 - ONNXRuntime
    cv::Mat leftImage = cv::imread("../demo-imgs/0600-Left.bmp");
    cv::Mat rightImage = cv::imread("../demo-imgs/0600-Right.bmp");
    stereo::StereoMatchingONNXRuntime smonnx;
    smonnx.loadModel("../models/FFLO_it32.onnx");
    auto disparity = smonnx.compute(leftImage, rightImage);
    auto disparity_color = stereo::applyMapping(disparity);
    cv::imshow("disparity", disparity_color);
    cv::waitKey(0);
 * @endcode demo 4 - ONNXRuntime
 * 
 * @code demo 5 - rectify -> stereo matching -> reproject
    cv::Mat stereoImage = cv::imread("../demo-imgs/0071-Stereo.bmp");
    StereoParams params("../yml/stereo.yml");
    stereo::EpipolarRectify rectify(params.map, params.imgsz);
    auto rectifiedPair = rectify.rectify(stereoImage);
    stereo::StereoMatchingTensorRT smtrt;
    smtrt.loadEngine("../models/FFLO_it32.trt");
    auto disparity = smtrt.compute(rectifiedPair.left, rectifiedPair.right);
    auto disparity_color = stereo::applyMapping(disparity);
    cv::imshow("disparity", disparity_color);
    cv::waitKey(0);
    auto xyzMatrix = stereo::reprojectTo3D(disparity, params.Q);
    stereo::writePointCloudToPCD("../demo-output/pointcloud.pcd",rectifiedPair.left, xyzMatrix);
 * @endcode demo 5 - rectify -> stereo matching -> reproject
 * 
 *********************************************************************/
#pragma once
#include "stereo_utils.h"
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <memory>

namespace stereo
{

/**
 * @brief ˫Ŀ���������ͶӰ.
 * @param [in] disparity �����Ӳ�ͼ, CV_32FC1
 * @param [in] QMatrix   ��ͶӰ����
 * @return xyz ���ƾ���, CV_32FC3
 */
cv::Mat reprojectTo3D(const cv::Mat& disparity, const cv::Mat& QMatrix);

enum class MapType
{ 
	JET,
};
inline std::string mapTypeToString(MapType mapType);
/**
 * @brief JETӳ���.
 * @return JETӳ���, CV_8UC3.
 */
cv::Mat JETMap();
/**
 * @brief MINMAX Linear + ӳ��� ӳ��ͼ��(�Ӳ�ͼ, disparity >= 0).
 * @param [in] image ����ͼ��, CV_32FC1
 * @param [in] map   ӳ���, CV_8UC3
 * @return ӳ����ͼ��, CV_8UC3
 */
cv::Mat applyMapping(const cv::Mat& image, const MapType& mapType = MapType::JET);
/**
 * @brief MINMAX Linear + ӳ��� ӳ��ͼ��.
 * @param [in] image    ����ͼ��, CV_32FC1
 * @param [in] map      ӳ���, CV_8UC3
 * @param [in] minValue ӳ����Сֵ
 * @param [in] maxValue ӳ�����ֵ
 * @return ӳ����ͼ��, CV_8UC3
 */
cv::Mat applyMapping(const cv::Mat& image, const float& minValue, const float& maxValue, const MapType& mapType = MapType::JET);
/**
 * @brief ����ˮƽ��.
 * @param [in] stereoImage ˫Ŀͼ��
 * @return ����ˮƽ�ߵ�˫Ŀͼ��
 */
cv::Mat drawHorizontalLines(const cv::Mat& stereoImage);
/**
 * @brief ����ˮƽ��.
 * @param [in] leftImage  ��Ŀͼ��
 * @param [in] rightImage ��Ŀͼ��
 * @return ����ˮƽ�ߵ�˫Ŀͼ��
 */
cv::Mat drawHorizontalLines(const cv::Mat& leftImage, const cv::Mat& rightImage);
/**
 * @brief ����ˮƽ��.
 * @param [in] stereoImagePair ����ͼ���
 * @return ����ˮƽ�ߵ�˫Ŀͼ��
 */
cv::Mat drawHorizontalLines(const StereoPair<cv::Mat>& stereoImagePair);
/**
 * @brief ���ƴ�ֱ��.
 * @param [in] stereoImage ˫Ŀͼ��
 * @return ���ƴ�ֱ�ߵ�˫Ŀͼ��
 */
cv::Mat drawVerticalLines(const cv::Mat& stereoImage);
/**
 * @brief ���ƴ�ֱ��.
 * @param [in] leftImage  ��Ŀͼ��
 * @param [in] rightImage ��Ŀͼ��
 * @return ���ƴ�ֱ�ߵ�˫Ŀͼ��
 */
cv::Mat drawVerticalLines(const cv::Mat& leftImage, const cv::Mat& rightImage);
/**
 * @brief ���ƴ�ֱ��.
 * @param [in] stereoImagePair ����ͼ���
 * @return ���ƴ�ֱ�ߵ�˫Ŀͼ��
 */
cv::Mat drawVerticalLines(const StereoPair<cv::Mat>& stereoImagePair);
/**
 * @brief ƴ������ͼ��.
 * @param [in] leftImage  ��Ŀͼ��
 * @param [in] rightImage ��Ŀͼ��
 * @return ƴ�Ӻ��˫Ŀͼ��
 */
cv::Mat concatStereoImage(const cv::Mat& leftImage, const cv::Mat& rightImage);
/**
 * @brief ���˫Ŀͼ��.
 * @param [in] stereoImage ˫Ŀͼ��
 * @return ����ͼ���, StereoPair<cv::Mat>
 */
StereoPair<cv::Mat> splitStereoImage(const cv::Mat& stereoImage);
/**
 * @brief �������PCD�ļ�, ASCII����.
 * @param [in] pcdPath    ����PCD�ļ�·��
 * @param [in] rgbImage   RGBͼ��
 * @param [in] pointCloud ���ƾ���, CV_32FC3
 */
void writePointCloudToPCD(const std::string& pcdPath, const cv::Mat& rgbImage, cv::Mat& pointCloud);

/**
 * @brief ���弫��У��.
 */
class EpipolarRectify
{
public:
    EpipolarRectify();
    EpipolarRectify(const EpipolarRectifyMap& rectifyMap, const cv::Size& imgsz);
    ~EpipolarRectify();
public:
    /**
     * @brief ��������У������.
     * @param [in] rectifyMap ����У������
     * @param [in] imgsz      ͼ��ߴ�
     */
    void loadEpipolarRectifyMap(const EpipolarRectifyMap& rectifyMap, const cv::Size& imgsz);
    /**
     * @brief ����У��.
     * @param [in] stereoImage ˫Ŀͼ��
     * @return ����У���������ͼ���
     */
    StereoPair<cv::Mat> rectify(const cv::Mat& stereoImage);
    /**
     * @brief ����У��.
     * @param [in] leftImage  ��ͼ
     * @param [in] rightImage ��ͼ
     * @return ����У���������ͼ���
     */
    StereoPair<cv::Mat> rectify(const cv::Mat& leftImage, const cv::Mat& rightImage);
    /**
     * @brief ����У��.
     * @param [in] stereoImagePair ����ͼ���
     * @return ����У���������ͼ���
     */
    StereoPair<cv::Mat> rectify(const StereoPair<cv::Mat>& stereoImagePair);
    /**
     * @brief ����У��������.
     * @param [in] stereoPattern ˫Ŀͼ���ļ�·��
     * @param [in] recursive     �ݹ��ȡ�ļ���
     */
    void rectify(const std::string& stereoPattern, const bool& recursive = false);

private:
    class ERImpl;
    std::unique_ptr<ERImpl> impl;
};

/**
 * @brief ����ƥ��ͼ��߽������.
 */
class InputPadder
{
public: 
	InputPadder();
	InputPadder(const cv::Size& imgsz, const int& divis_by);
    ~InputPadder();
public:
    /**
     * @brief ����ǰͼ��߽����.
     * @param [in] imgs ����Ŀ����ͼ��, CV_8UC3
     * @return ���߽�������Ŀͼ��, CV_8UC3
     */
	std::vector<cv::Mat> pad(const std::vector<cv::Mat>& imgs);
    /**
     * @brief �����ͼ��߽�����.
     * @param [in] disparity ���������Ӳ�, CV_32FC1
     * @return �����߽��ĸ��������Ӳ�, CV_32FC1
     */
	cv::Mat unpad(const cv::Mat& disparity);
private:
	class InputPadderImpl;
	std::unique_ptr<InputPadderImpl> impl;
};

/**
 * @brief ����ƥ���㷨ONNXģ���Ӳ�Ԥ��, �˵�������ƥ��ģ��ͨ��, onnxruntime = 1.18.1
 */
class StereoMatchingONNXRuntime
{
public:
    StereoMatchingONNXRuntime();
    ~StereoMatchingONNXRuntime();
public:
    /**
     * @brief ��������ƥ��ONNXģ��, ��֧��GPU.
     * @param [in] modelPath ģ��·��
     * @throw std::runtime_error ģ�ͼ���ʧ��
     */
    void loadModel(const std::string& modelPath);
    /**
     * @brief ����ƥ��Ԥ���Ӳ�.
     * @param [in] leftImage  ����У�������ͼ, CV_8UC3
     * @param [in] rightImage ����У�������ͼ, CV_8UC3
     * @return disparity  ��ͼ���������Ӳ�, CV_32FC1
     */
    cv::Mat compute(const cv::Mat& leftImage, const cv::Mat& rightImage);
    /**
     * @brief ����ƥ��Ԥ���Ӳ�������.
     * @param [in] leftImage  ����У�������ͼ, CV_8UC3
     * @param [in] rightImage ����У�������ͼ, CV_8UC3
     * @return disparities  ��ͼ���������Ӳ�, CV_32FC1
     */
    std::vector<cv::Mat> compute(const std::vector<cv::Mat>& leftImages, const std::vector<cv::Mat>& rightImages);
private:
    class SMONNXImpl;
    std::unique_ptr<SMONNXImpl> impl;
};

/**
 * @brief ����ƥ���㷨TensorRT��������, TensorRT 10.10 WINDOWS CUDA 11.X .
 */
class StereoMatchingTensorRT
{
public:
	StereoMatchingTensorRT();
    StereoMatchingTensorRT(const std::string& enginePath);
	~StereoMatchingTensorRT();
public:
    /**
     * @brief ��������ƥ�� TensorRT��������.
     * @param [in] trtPath TensorRT��������·��
     * @return ����״̬
     */
    bool loadEngine(const std::string& enginePath);
    /**
     * @brief ����ƥ��Ԥ���Ӳ�.
     * @param [in] leftImage  ����У�������ͼ, CV_8UC3
     * @param [in] rightImage ����У�������ͼ, CV_8UC3
     * @return disparity ��ͼ���������Ӳ�, CV_32FC1
     */
    cv::Mat compute(const cv::Mat& leftImage, const cv::Mat& rightImage);
private:
    class SMTRTImpl;
    std::unique_ptr<SMTRTImpl> impl;
};

/**
 * @brief (Selective) AD-Census(-HSI)����ƥ���㷨�Ӳ�Ԥ��.
 */
class ADCensus
{
public:
    ADCensus();
    ~ADCensus();
public:
    /**
     * @brief ������С/����Ӳ�.
     * @param [in] minDisparity ��С�Ӳ�
     * @param [in] maxDisparity ����Ӳ�
     */
    void setMinMaxDisparity(const int& minDisparity, const int& maxDisparity);
    /**
     * @brief ��������ƥ�����.
     * @param [in] colorModel   ��ɫģ��
     * @param [in] roiMatching  ROIƥ��ģʽ
     * @param [in] maskMatching MASKƥ��ģʽ
     */
    void setMatchingStrategy(const ColorModel& colorModel = ColorModel::RGB, const bool& roiMatching = false, const bool& maskMatching = false);
    /**
     * @brief ���ò���.
     * @param [in] offset ROI/Maskƥ��ģʽ�Ӳ��
     */
    void setOffset(const int& offset);
    /**
     * @brief �����Ӳ�ͼ.
     * @param [in] leftImage  ����У�������Ŀͼ��
     * @param [in] rightImage ����У�������Ŀͼ��
     * @return disparity  AD-Census����ƥ���㷨������Ӳ�ͼ
     */
    cv::Mat compute(const cv::Mat& leftImage, const cv::Mat& rightImage);
private:
    class ADCensusImpl;
    std::unique_ptr<ADCensusImpl> impl;
};

///SGBM - OpenCV
/**
 * @code
    cv::Mat leftImage = cv::imread("../demo-imgs/0600-Left.bmp");
    cv::Mat rightImage = cv::imread("../demo-imgs/0600-Right.bmp");
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create();
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(192);
    sgbm->setBlockSize(3);
    sgbm->setP1(8 * 3 * 3 * 3);
    sgbm->setP2(32 * 3 * 3 * 3);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setPreFilterCap(0);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
    cv::Mat disparity;
    sgbm->compute(leftImage, rightImage, disparity);
    disparity.convertTo(disparity, CV_32F, 1.0 / 16);
    auto disparity_color = stereo::applyMapping(disparity, stereo::MapType::JET);
    cv::imshow("disparity", disparity_color);
    cv::waitKey(0);
 * @endcode
 */

///TODO...
class SGM
{
public:
    SGM();
    ~SGM();
};

class PatchMatch
{
public:
    PatchMatch();
    ~PatchMatch();
};

class ELAS
{
    ELAS();
    ~ELAS();
};

}
