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
	stereo::StereoParams params("../yml/stereo.yml");
	stereo::EpipolarRectify rectify(params.map, params.imgsz);
	cv::Mat rectifiedStereo;
	rectify.rectify(stereoImage, rectifiedStereo);
	auto imgWithLine = stereo::drawHorizontalLine(rectifiedStereo);
	cv::namedWindow("rectified", cv::WINDOW_NORMAL);
	cv::imshow("rectified", imgWithLine);
	cv::waitKey(0);
 * @endcode demo 1 - Stereo Epipolar Rectification
 *
 * @code demo 2 - ADCensus
	cv::Mat leftImage = cv::imread("../demo-imgs/0600-Left.bmp");
	cv::Mat rightImage = cv::imread("../demo-imgs/0600-Right.bmp");
	stereo::ADCensus adcensus;
	adcensus.setMatchingStrategy(stereo::ColorModel::RGB, false, false);
	adcensus.setMinMaxDisparity(0, 192);
	cv::Mat disparity;
	adcensus.compute(leftImage, rightImage, disparity);
	cv::Mat disparityColor;
	stereo::applyColorMap(disparity, disparityColor, stereo::JETColorMap());
	cv::imshow("disparity", disparityColor);
	cv::waitKey(0);
 * @endcode demo 2 - ADCensus
 *
 * @code demo 3 - TensorRT
	cv::Mat leftImage = cv::imread("../demo-imgs/0600-Left.bmp");
	cv::Mat rightImage = cv::imread("../demo-imgs/0600-Right.bmp");
	stereo::TensorRTInference trt;
	trt.loadModel("../models/FFLONet_DepthAny_it32_cuda12.trt");
	cv::Mat disparity;
	trt.compute(leftImage, rightImage, disparity);
	cv::Mat disparityColor;
	stereo::applyColorMap(disparity, disparityColor, stereo::JETColorMap());
	cv::imshow("disparity", disparityColor);
	cv::waitKey(0);
 * @endcode demo 3 - TensorRT
 *
 * @code demo 4 - ONNXRuntime
	cv::Mat leftImage = cv::imread("../demo-imgs/0600-Left.bmp");
	cv::Mat rightImage = cv::imread("../demo-imgs/0600-Right.bmp");
	stereo::ONNXRuntimeInference onnx;
	onnx.loadModel("../models/FFLO_it32.onnx");
	cv::Mat disparity;
	onnx.compute(leftImage, rightImage, disparity);
	cv::Mat disparityColor;
	stereo::applyColorMap(disparity, disparityColor, stereo::JETColorMap());
	cv::imshow("disparity", disparityColor);
	cv::waitKey(0);
 * @endcode demo 4 - ONNXRuntime
 *
 * @code demo 5 - rectify -> stereo matching -> reproject
	cv::Mat stereoImage = cv::imread("../demo-imgs/0071-Stereo.bmp");
	stereo::StereoParams params("../yml/stereo.yml");
	stereo::EpipolarRectify rectify(params.map, params.imgsz);
	cv::Mat leftImage, rightImage;
	rectify.rectify(stereoImage, leftImage, rightImage);
	stereo::TensorRTInference trt;
	trt.loadModel("../models/FFLONet_DepthAny_it32_cuda12.trt");
	cv::Mat disparity;
	trt.compute(leftImage, rightImage, disparity);
	cv::Mat disparityColor;
	stereo::applyColorMap(disparity, disparityColor, stereo::JETColorMap());
	cv::imshow("disparity", disparityColor);
	cv::waitKey(0);
	cv::waitKey(0);
	cv::Mat xyzMatrix;
	stereo::reprojectTo3D(disparity, params.Q, xyzMatrix);
	stereo::writePointCloudToPCD(leftImage, xyzMatrix, "../demo-output/pointcloud.pcd");
 * @endcode demo 5 - rectify -> stereo matching -> reproject
 *
 *********************************************************************/
#pragma once
#include <vector>
#include <string>
#include <memory>
#include <opencv2/core/mat.hpp>
#include "./stereo_utils.h"

namespace stereo
{

/**
 * @brief 水平拼接图像.
 * @param [in]  leftImage   左图.
 * @param [in]  rightImage  右图.
 * @param [out] stereoImage 双目图像.
 */
void hconcat(const cv::Mat& leftImage, const cv::Mat& rightImage, cv::Mat& stereoImage);
/**
 * @brief 垂直拼接图像.
 * @param [in]  topImage    左图.
 * @param [in]  bottomImage 右图.
 * @param [out] stereoImage 双目图像.
 */
void vconcat(const cv::Mat& topImage, const cv::Mat& bottomImage, cv::Mat& stereoImage);
/**
 * @brief 水平拆分图像.
 * @param [in]  stereoImage 双目图像.
 * @param [out] leftImage   左图.
 * @param [out] rightImage  右图.
 */
void hsplit(const cv::Mat& stereoImage, cv::Mat& leftImage, cv::Mat& rightImage);
/**
 * @brief 垂直拆分图像.
 * @param [in]  stereoImage 双目图像.
 * @param [out] topImage    左图.
 * @param [out] bottomImage 右图.
 */
void vsplit(const cv::Mat& stereoImage, cv::Mat& topImage, cv::Mat& bottomImage);
/**
 * @brief 绘制水平线.
 * @param [in]  stereoImage 双目图像.
 * @return 绘制水平线的双目图像.
 */
cv::Mat drawHorizontalLine(const cv::Mat& stereoImage);
/**
 * @brief 绘制垂直线.
 * @param [in]  stereoImage 双目图像.
 * @return 绘制垂直线的双目图像.
 */
cv::Mat drawVerticalLine(const cv::Mat& stereoImage);
/**
 * @brief 生成JET颜色映射表.
 * @return JET颜色映射表.
 */
cv::Mat JETColorMap();
/**
 * @brief MINMAX Linear + Map 映射伪彩色图.
 * @param [in]  src      输入图像. DType: CV_32FC1.
 * @param [out] dst      输出图像. DType: CV_8UC3.
 * @param [in]  colorMap 颜色映射. DType: CV_8UC3.
 */
void applyColorMap(const cv::Mat& src, cv::Mat& dst, const cv::Mat& colorMap);
/**
 * @brief MINMAX Linear + Map 映射伪彩色图.
 * @param [in]  src      输入图像. DType: CV_32FC1.
 * @param [out] dst      输出图像. DType: CV_8UC3.
 * @param [in]  minVal   最小值.
 * @param [in]  maxVal   最大值.
 * @param [in]  colorMap 颜色映射. DType: CV_8UC3.
 */
void applyColorMap(const cv::Mat& src, cv::Mat& dst, float minVal, float maxVal, const cv::Mat& colorMap);
/**
 * @brief 双目重投影获取深度图.
 * @param [in]  disparity   视差图. DType: CV_32FC1.
 * @param [in]  focalLength 焦距.
 * @param [in]  baseline    基线.
 * @param [out] depth       深度图. DType: CV_32FC1.
 */
void reprojectToDepth(const cv::Mat& disparity, float focalLength, float baseline, cv::Mat& depth);
/**
 * @brief 双目重投影获取点云.
 * @param [in]  disparity   视差图. DType: CV_32FC1.
 * @param [in]  focalLength 焦距.
 * @param [in]  baseline    基线.
 * @param [in]  cx          图像中心点x坐标.
 * @param [in]  cy          图像中心点y坐标.
 * @param [out] XYZPoints   点云.   DType: CV_32FC3.
 */
void reprojectTo3D(const cv::Mat& disparity, float focalLength, float baseline, float cx, float cy, cv::Mat& XYZPoints);
/**
 * @brief 双目重投影获取点云.
 * @param [in]  disparity   视差图.    DType: CV_32FC1.
 * @param [in]  Q           重投影矩阵. DType: CV_32FC1/CV_64FC1.
 * @param [out] XYZPoints   点云.      DType: CV_32FC3.
 */
void reprojectTo3D(const cv::Mat& disparity, const cv::Mat& Q, cv::Mat& XYZPoints);
/**
 * @brief 保存点云PCD文件, ASCII编码.
 * @param [in]  RGBImage  RGB图像. DType: CV_8UC3.
 * @param [in]  XYZPoints 点云.    DType: CV_32FC3.
 * @param [in]  pcdPath   PCD文件路径.
 */
void writePointCloudToPCD(const cv::Mat& RGBImage, const cv::Mat& XYZPoints, const std::string& pcdPath);

void writePointCloudToPLY(const cv::Mat& RGBImage, const cv::Mat& XYZPoints, const std::string& plyPath);

/**
 * @brief 极线校正.
 */
class EpipolarRectify
{
public:
	EpipolarRectify();
	EpipolarRectify(const EpipolarRectifyMap& rectifyMap, const cv::Size& imgsz);
	~EpipolarRectify();
	/**
	 * @brief 加载立体校正参数.
	 * @param [in] rectifyMap 立体校正参数
	 * @param [in] imgsz      图像尺寸
	 */
	void loadEpipolarRectifyMap(const EpipolarRectifyMap& rectifyMap, const cv::Size& imgsz);
	/**
	 * @brief 极线校正.
	 * @param [in]  stereoImage          双目图像
	 * @param [out] rectifiedStereoImage 极线校正后的双目图像
	 */
	void rectify(const cv::Mat& stereoImage, cv::Mat& rectifiedStereoImage);
	/**
	 * @brief 极线校正.
	 * @param [in]  stereoImage         双目图像
	 * @param [out] rectifyLeftImage    极线校正后的左目图像
	 * @param [out] rectifiedRightImage 极线校正后的右目图像
	 */
	void rectify(const cv::Mat& stereoImage, cv::Mat& rectifyLeftImage, cv::Mat& rectifiedRightImage);
	/**
	 * @brief 极线校正.
	 * @param [in]  leftImage           左目图像
	 * @param [in]  rightImage          右目图像
	 * @param [out] rectifyLeftImage    极线校正后的左目图像
	 * @param [out] rectifiedRightImage 极线校正后的右目图像
	 */
	void rectify(const cv::Mat& leftImage, const cv::Mat& rightImage, cv::Mat& rectifyLeftImage, cv::Mat& rectifiedRightImage);
	/**
	 * @brief 极线校正批处理.
	 * @param [in] stereoPattern 双目图像文件路径
	 * @param [in] recursive     递归读取文件夹
	 */
	void rectify(const std::string& stereoPattern, const bool& recursive = false);
private:
	class EpipolarRectifyImpl;
    std::unique_ptr<EpipolarRectifyImpl> impl;
};

/**
 * @brief 端到端立体匹配网络图像填充器, 确保图像尺寸能被32整除(1/32下采样).
 */
class InputPadder
{
public:
	InputPadder();
	~InputPadder();
	/**
	 * @brief 立体匹配推理前图像边界填充.
	 * @param [in] images 输入图像.
	 * @return 填充边界后的图像
	 */
	std::vector<cv::Mat> pad(const std::vector<cv::Mat>& images);
	/**
	 * @brief 立体匹配推理后图像边界擦除.
	 * @param [in] disparity 视差图
	 * @return 擦除边界后的视差图
	 */
	cv::Mat unpad(const cv::Mat& disparity);

private:
	class InputPadderImpl;
    std::unique_ptr<InputPadderImpl> impl;
};


class StereoMatching
{
public:
	virtual ~StereoMatching() = 0;

	virtual void compute(const cv::Mat& leftImage, const cv::Mat& rightImage, cv::Mat& disparity) = 0;
};


class TensorRTInference : public StereoMatching
{
public:
	TensorRTInference();
	~TensorRTInference();
	/**
	 * @brief 加载TensorRT引擎.
	 * @param [in] enginePath TensorRT引擎文件路径.
	 */
	void loadModel(const std::string& enginePath);
	/**
	 * @brief 推理视差结果.
	 * @param [in]  leftImage  左图. Shape: [H, W, 3] ([H, W, C]). DType: CV_8UC3.
	 * @param [in]  rightImage 右图. Shape: [H, W, 3] ([H, W, C]). DType: CV_8UC3.
	 * @param [out] disparity  视差. Shape: [H, W, 1] ([H, W, C]). DType: CV_32FC1.
	 */
	void compute(const cv::Mat& leftImage, const cv::Mat& rightImage, cv::Mat& disparity) override;

private:
	class TRTInferenceImpl;
	std::unique_ptr<TRTInferenceImpl> impl;
};


class ONNXRuntimeInference : public StereoMatching
{
public:
	ONNXRuntimeInference();
	~ONNXRuntimeInference();
	/**
	 * @brief 加载ONNX模型.
	 * @param [in] modelPath ONNX模型文件路径.
	 */
	void loadModel(const std::string& modelPath);
	/**
	 * @brief 推理视差结果.
	 * @param [in]  leftImage  左图. Shape: [H, W, 3] ([H, W, C]). DType: CV_8UC3.
	 * @param [in]  rightImage 右图. Shape: [H, W, 3] ([H, W, C]). DType: CV_8UC3.
	 * @param [out] disparity  视差. Shape: [H, W, 1] ([H, W, C]). DType: CV_32FC1.
	 */
	void compute(const cv::Mat& leftImage, const cv::Mat& rightImage, cv::Mat& disparity) override;
	/**
	 * @brief 推理视差结果批处理.
	 * @param [in]  leftImages  左图. Shape: [N, H, W, 3] ([N, H, W, C]). DType: CV_8UC3
	 * @param [in]  rightImages 右图. Shape: [N, H, W, 3] ([N, H, W, C]). DType: CV_8UC3
	 * @param [out] disparities 视差. Shape: [N, H, W, 1] ([N, H, W, C]). DType: CV_32FC1
	 */
	void compute(const std::vector<cv::Mat>& leftImages, const std::vector<cv::Mat>& rightImages, std::vector<cv::Mat>& disparities);
private:
	class ONNXRuntimeImpl;
	std::unique_ptr<ONNXRuntimeImpl> impl;
};


class ADCensus : public StereoMatching
{
public:
	ADCensus();
	~ADCensus();
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
	 * @brief 视差预测结果.
	 * @param [in]  leftImage  左图. Shape: [H, W, 3] ([H, W, C]). DType: CV_8UC3.
	 * @param [in]  rightImage 右图. Shape: [H, W, 3] ([H, W, C]). DType: CV_8UC3.
	 * @param [out] disparity  视差. Shape: [H, W, 1] ([H, W, C]). DType: CV_32FC1.
	 */
	void compute(const cv::Mat& leftImage, const cv::Mat& rightImage, cv::Mat& disparity) override;

private:
	class ADCensusImpl;
	std::unique_ptr<ADCensusImpl> impl;
};


//class SGM : public StereoMatching
//{
//public:
//	SGM();
//	~SGM();
//
//    void compute(const cv::Mat& leftImage, const cv::Mat& rightImage, cv::Mat& disparity) override;
//
//private:
//	class SGMImpl;
//	std::unique_ptr<SGMImpl> impl;
//};
//
//
//class ELAS : public StereoMatching
//{
//public:
//    ELAS();
//	~ELAS();
//
//    void compute(const cv::Mat& leftImage, const cv::Mat& rightImage, cv::Mat& disparity) override;
//
//private:
//    class ELASImpl;
//    std::unique_ptr<ELASImpl> impl;
//};


}
