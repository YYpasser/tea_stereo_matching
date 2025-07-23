/*********************************************************************
 * @file   stereo.h
 * @brief  茶叶嫩芽采摘立体视觉处理模块.
 * @author Qianyao Zhuang
 * @date   May 2025
 * 
 * @details
 * 本模块实现了基于双目立体视觉的茶叶嫩芽三维点云获取, 主要包含三个核心功能: 
 * 1. 图像极线校正: 立体匹配预处理, 使得立体图像对的特征点行对齐
 * 2. 立体匹配算法: 传统方法以及深度学习模型(ONNXRuntime/TensorRT)
 * 3. 三维点云重建: 基于视差图和相机参数, 生成茶叶点云数据
 * 
 * @section 算法框架
 * [模块架构]
 * 左右相机图像对 (输入) -> [极线校正] -> [立体匹配] -> [点云生成] -> 点云数据 (输出)
 * 
 * @section 技术细节
 * - 立体匹配算法: 
 *   - 传统方法: AD-Census
 *   - 深度学习: IGEV-Stereo, FFLO-Net等端到端模型
 * - 输出格式: 
 *   - 中间结果: 校正后图像、视差图
 *   - 最终结果: 点云(XYZRGB)
 * 
 * @section 使用说明
 * 一般流程:
 * 1. 加载相机参数和算法参数(模型)
 * 2. 加载立体图像对
 * 3. 极线校正
 * 4. 选择并执行立体匹配算法
 * 5. 生成三维点云
 * 
 * @section 注意事项
 * 1. ONNXRuntime-gpu 版本为1.18.1
 * 2. TensorRT 版本为10.10
 * 3. 目前模型仅可用于固定尺寸图像, 2560*720
 * 4. 使用AD-Census算法时, 开启OpenMP加速
 *    -- 属性配置 -> C/C++ -> 语言 -> OpenMP支持 -> 是 (/openmp)
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
 * @brief 双目相机坐标重投影.
 * @param [in] disparity 浮点视差图, CV_32FC1
 * @param [in] QMatrix   重投影矩阵
 * @return xyz 点云矩阵, CV_32FC3
 */
cv::Mat reprojectTo3D(const cv::Mat& disparity, const cv::Mat& QMatrix);

enum class MapType
{ 
	JET,
};
inline std::string mapTypeToString(MapType mapType);
/**
 * @brief JET映射表.
 * @return JET映射表, CV_8UC3.
 */
cv::Mat JETMap();
/**
 * @brief MINMAX Linear + 映射表 映射图像(视差图, disparity >= 0).
 * @param [in] image 输入图像, CV_32FC1
 * @param [in] map   映射表, CV_8UC3
 * @return 映射后的图像, CV_8UC3
 */
cv::Mat applyMapping(const cv::Mat& image, const MapType& mapType = MapType::JET);
/**
 * @brief MINMAX Linear + 映射表 映射图像.
 * @param [in] image    输入图像, CV_32FC1
 * @param [in] map      映射表, CV_8UC3
 * @param [in] minValue 映射最小值
 * @param [in] maxValue 映射最大值
 * @return 映射后的图像, CV_8UC3
 */
cv::Mat applyMapping(const cv::Mat& image, const float& minValue, const float& maxValue, const MapType& mapType = MapType::JET);
/**
 * @brief 绘制水平线.
 * @param [in] stereoImage 双目图像
 * @return 绘制水平线的双目图像
 */
cv::Mat drawHorizontalLines(const cv::Mat& stereoImage);
/**
 * @brief 绘制水平线.
 * @param [in] leftImage  左目图像
 * @param [in] rightImage 右目图像
 * @return 绘制水平线的双目图像
 */
cv::Mat drawHorizontalLines(const cv::Mat& leftImage, const cv::Mat& rightImage);
/**
 * @brief 绘制水平线.
 * @param [in] stereoImagePair 左右图像对
 * @return 绘制水平线的双目图像
 */
cv::Mat drawHorizontalLines(const StereoPair<cv::Mat>& stereoImagePair);
/**
 * @brief 绘制垂直线.
 * @param [in] stereoImage 双目图像
 * @return 绘制垂直线的双目图像
 */
cv::Mat drawVerticalLines(const cv::Mat& stereoImage);
/**
 * @brief 绘制垂直线.
 * @param [in] leftImage  左目图像
 * @param [in] rightImage 右目图像
 * @return 绘制垂直线的双目图像
 */
cv::Mat drawVerticalLines(const cv::Mat& leftImage, const cv::Mat& rightImage);
/**
 * @brief 绘制垂直线.
 * @param [in] stereoImagePair 左右图像对
 * @return 绘制垂直线的双目图像
 */
cv::Mat drawVerticalLines(const StereoPair<cv::Mat>& stereoImagePair);
/**
 * @brief 拼接左右图像.
 * @param [in] leftImage  左目图像
 * @param [in] rightImage 右目图像
 * @return 拼接后的双目图像
 */
cv::Mat concatStereoImage(const cv::Mat& leftImage, const cv::Mat& rightImage);
/**
 * @brief 拆分双目图像.
 * @param [in] stereoImage 双目图像
 * @return 左右图像对, StereoPair<cv::Mat>
 */
StereoPair<cv::Mat> splitStereoImage(const cv::Mat& stereoImage);
/**
 * @brief 保存点云PCD文件, ASCII编码.
 * @param [in] pcdPath    点云PCD文件路径
 * @param [in] rgbImage   RGB图像
 * @param [in] pointCloud 点云矩阵, CV_32FC3
 */
void writePointCloudToPCD(const std::string& pcdPath, const cv::Mat& rgbImage, cv::Mat& pointCloud);

/**
 * @brief 立体极线校正.
 */
class EpipolarRectify
{
public:
    EpipolarRectify();
    EpipolarRectify(const EpipolarRectifyMap& rectifyMap, const cv::Size& imgsz);
    ~EpipolarRectify();
public:
    /**
     * @brief 加载立体校正参数.
     * @param [in] rectifyMap 立体校正参数
     * @param [in] imgsz      图像尺寸
     */
    void loadEpipolarRectifyMap(const EpipolarRectifyMap& rectifyMap, const cv::Size& imgsz);
    /**
     * @brief 极线校正.
     * @param [in] stereoImage 双目图像
     * @return 极线校正后的立体图像对
     */
    StereoPair<cv::Mat> rectify(const cv::Mat& stereoImage);
    /**
     * @brief 极线校正.
     * @param [in] leftImage  左图
     * @param [in] rightImage 右图
     * @return 极线校正后的立体图像对
     */
    StereoPair<cv::Mat> rectify(const cv::Mat& leftImage, const cv::Mat& rightImage);
    /**
     * @brief 极线校正.
     * @param [in] stereoImagePair 立体图像对
     * @return 极线校正后的立体图像对
     */
    StereoPair<cv::Mat> rectify(const StereoPair<cv::Mat>& stereoImagePair);
    /**
     * @brief 极线校正批处理.
     * @param [in] stereoPattern 双目图像文件路径
     * @param [in] recursive     递归读取文件夹
     */
    void rectify(const std::string& stereoPattern, const bool& recursive = false);

private:
    class ERImpl;
    std::unique_ptr<ERImpl> impl;
};

/**
 * @brief 立体匹配图像边界填充器.
 */
class InputPadder
{
public: 
	InputPadder();
	InputPadder(const cv::Size& imgsz, const int& divis_by);
    ~InputPadder();
public:
    /**
     * @brief 推理前图像边界填充.
     * @param [in] imgs 左右目输入图像, CV_8UC3
     * @return 填充边界后的左右目图像, CV_8UC3
     */
	std::vector<cv::Mat> pad(const std::vector<cv::Mat>& imgs);
    /**
     * @brief 推理后图像边界消除.
     * @param [in] disparity 浮点类型视差, CV_32FC1
     * @return 消除边界后的浮点类型视差, CV_32FC1
     */
	cv::Mat unpad(const cv::Mat& disparity);
private:
	class InputPadderImpl;
	std::unique_ptr<InputPadderImpl> impl;
};

/**
 * @brief 立体匹配算法ONNX模型视差预测, 端到端立体匹配模型通用, onnxruntime = 1.18.1
 */
class StereoMatchingONNXRuntime
{
public:
    StereoMatchingONNXRuntime();
    ~StereoMatchingONNXRuntime();
public:
    /**
     * @brief 加载立体匹配ONNX模型, 仅支持GPU.
     * @param [in] modelPath 模型路径
     * @throw std::runtime_error 模型加载失败
     */
    void loadModel(const std::string& modelPath);
    /**
     * @brief 立体匹配预测视差.
     * @param [in] leftImage  极线校正后的左图, CV_8UC3
     * @param [in] rightImage 极线校正后的右图, CV_8UC3
     * @return disparity  左图浮点类型视差, CV_32FC1
     */
    cv::Mat compute(const cv::Mat& leftImage, const cv::Mat& rightImage);
    /**
     * @brief 立体匹配预测视差批处理.
     * @param [in] leftImage  极线校正后的左图, CV_8UC3
     * @param [in] rightImage 极线校正后的右图, CV_8UC3
     * @return disparities  左图浮点类型视差, CV_32FC1
     */
    std::vector<cv::Mat> compute(const std::vector<cv::Mat>& leftImages, const std::vector<cv::Mat>& rightImages);
private:
    class SMONNXImpl;
    std::unique_ptr<SMONNXImpl> impl;
};

/**
 * @brief 立体匹配算法TensorRT引擎推理, TensorRT 10.10 WINDOWS CUDA 11.X .
 */
class StereoMatchingTensorRT
{
public:
	StereoMatchingTensorRT();
    StereoMatchingTensorRT(const std::string& enginePath);
	~StereoMatchingTensorRT();
public:
    /**
     * @brief 加载立体匹配 TensorRT推理引擎.
     * @param [in] trtPath TensorRT推理引擎路径
     * @return 加载状态
     */
    bool loadEngine(const std::string& enginePath);
    /**
     * @brief 立体匹配预测视差.
     * @param [in] leftImage  极线校正后的左图, CV_8UC3
     * @param [in] rightImage 极线校正后的右图, CV_8UC3
     * @return disparity 左图浮点类型视差, CV_32FC1
     */
    cv::Mat compute(const cv::Mat& leftImage, const cv::Mat& rightImage);
private:
    class SMTRTImpl;
    std::unique_ptr<SMTRTImpl> impl;
};

/**
 * @brief (Selective) AD-Census(-HSI)立体匹配算法视差预测.
 */
class ADCensus
{
public:
    ADCensus();
    ~ADCensus();
public:
    /**
     * @brief 设置最小/最大视差.
     * @param [in] minDisparity 最小视差
     * @param [in] maxDisparity 最大视差
     */
    void setMinMaxDisparity(const int& minDisparity, const int& maxDisparity);
    /**
     * @brief 设置立体匹配策略.
     * @param [in] colorModel   颜色模型
     * @param [in] roiMatching  ROI匹配模式
     * @param [in] maskMatching MASK匹配模式
     */
    void setMatchingStrategy(const ColorModel& colorModel = ColorModel::RGB, const bool& roiMatching = false, const bool& maskMatching = false);
    /**
     * @brief 设置补偿.
     * @param [in] offset ROI/Mask匹配模式视差补偿
     */
    void setOffset(const int& offset);
    /**
     * @brief 计算视差图.
     * @param [in] leftImage  极线校正后的左目图像
     * @param [in] rightImage 极线校正后的右目图像
     * @return disparity  AD-Census立体匹配算法计算的视差图
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
