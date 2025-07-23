#include "../include/stereo.h"
#include "../include/utils.h"
#include "../include/logger.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <limits>
#include <omp.h>

class stereo::ADCensus::ADCensusImpl
{
public:
	ADCensusImpl();
	~ADCensusImpl();
public:
	/**
	 * @brief 计算像素点p和pd在RGB颜色模型中的AD代价.
	 * @param [in] height1 像素点p坐标
	 * @param [in] width1  像素点p坐标
	 * @param [in] height2 像素点pd坐标
	 * @param [in] width2  像素点pd坐标
	 * @return RGB颜色模型的AD代价, float
	 */
	float computeRGBADCost(const int& height1, const int& width1, const int& height2, const int& width2);
	/**
	 * @brief 计算像素点p和pd在HSI颜色模型中的AD代价.
	 * @param [in] height1 像素点p坐标
	 * @param [in] width1  像素点p坐标
	 * @param [in] height2 像素点pd坐标
	 * @param [in] width2  像素点pd坐标
	 * @return HSI颜色模型的AD代价, float
	 */
	float computeHSIADCost(const int& height1, const int& width1, const int& height2, const int& width2);
	/**
	 * @brief 计算像素点p和pd在RGB颜色模型中的Census变换代价[Hamming distance].
	 * @param [in] height1 像素点p坐标
	 * @param [in] width1  像素点p坐标
	 * @param [in] height2 像素点pd坐标
	 * @param [in] width2  像素点pd坐标
	 * @param [in] censusWinHeight Census变换窗尺寸
	 * @param [in] censusWinWidth  Census变换窗尺寸
	 * @return RGB颜色模型的Census变换代价, float
	 */
	float computeRGBCensusCost(const int& height1, const int& width1, const int& height2, const int& width2, const int& censusWinHeight, const int& censusWinWidth);
	/**
	 * @brief 计算像素点p和pd在HSI颜色模型中的Census变换代价[Hamming distance].
	 * @param [in] height1 像素点p坐标
	 * @param [in] width1  像素点p坐标
	 * @param [in] height2 像素点pd坐标
	 * @param [in] width2  像素点pd坐标
	 * @param [in] censusWinHeight Census变换窗尺寸
	 * @param [in] censusWinWidth  Census变换窗尺寸
	 * @return HSI颜色模型的Census变换代价, float
	 */
	float computeHSICensusCost(const int& height1, const int& width1, const int& height2, const int& width2, const int& censusWinHeight, const int& censusWinWidth);
	/**
	 * @brief 计算像素点p和pd的AD-Census代价.
	 * @param [in] height1 像素点p坐标
	 * @param [in] width1  像素点p坐标
	 * @param [in] height2 像素点pd坐标
	 * @param [in] width2  像素点pd坐标
	 * @param [in] censusWinHeight Census变换窗尺寸
	 * @param [in] censusWinWidth  Census变换窗尺寸
	 * @return AD-Census代价
	 */
	float computeADCensusCost(const int& height1, const int& width1, const int& height2, const int& width2, const int& censusWinHeight, const int& censusWinWidth);
	/**
	 * @brief [AD-Census算法] Step.1 初始代价计算, 构建初始代价体C[H,W,D].
	 * @param [in] writeProcess 保存结果
	 * @param [in] writeDir     保存路径
	 */
	void costInitialize(const bool& writeProcess = false, const std::string& writeDir = std::string());
	/**
	 * @brief 计算像素点p和pd的颜色(RGB or Hue)差异.
	 * @param [in] p1 像素点p
	 * @param [in] p2 像素点pd
	 * @return 颜色差异值, int
	 */
	int colorDiff(const cv::Vec3b& p1, const cv::Vec3b& p2);
	/**
	 * @brief 计算像素点p的十字交叉臂臂长.
	 * @param [in] height     像素点p坐标
	 * @param [in] width      像素点p坐标
	 * @param [in] directionH y方向延伸方向
	 * @param [in] directionW x方向延伸方向
	 * @param [in] imageNo    左右视图标志位
	 * @return 像素点p的十字交叉臂臂长, px
	 */
	int computeLimit(const int& height, const int& width, const int& directionH, const int& directionW, const int& imageNo);
	/**
	 * @brief 计算十字交叉臂臂长矩阵.
	 * @param [in] directionH y方向延伸方向
	 * @param [in] directionW x方向延伸方向
	 * @param [in] imageNo    左右视图标志位
	 * @return 十字交叉臂臂长矩阵
	 */
	cv::Mat computeLimits(const int& directionH, const int& directionW, const int& imageNo);
	/**
	 * @brief x or y方向代价聚合.
	 * @param [in]  costMap     代价矩阵
	 * @param [in]  directionH  y方向延伸方向
	 * @param [in]  directionW  x方向延伸方向
	 * @param [out] windowSizes 十字交叉臂窗口尺寸
	 * @param [in]  imageNo     左右视图标志位
	 * @return 聚合代价矩阵
	 */
	cv::Mat aggregation1D(const cv::Mat& costMap, const int& directionH, const int& directionW,
		cv::Mat& windowSizes, const int& imageNo);
	/**
	 * @brief x, y方向代价聚合.
	 * @param [in/out] costMap         输入初始代价矩阵, 输出聚合代价矩阵
	 * @param [in]     horizontalFirst x or y方向延伸
	 * @param [in]     imageNo         左右视图标志位
	 * @return =0: 成功; < 0: 错误代码
	 */
	int aggregation2D(cv::Mat& costMap, bool horizontalFirst, const int& imageNo);
	/**
	 * @brief [AD-Census算法] Step.2 代价聚合, 根据初始代价体计算聚合代价体C[H,W,D].
	 * @param [in] writeProcess 保存结果
	 * @param [in] writeDir     保存路径
	 */
	void costAggregate(const bool& writeProcess = false, const std::string& writeDir = std::string());
	/**
	 * @brief 垂直方向扫描线优化, 逐行优化.
	 * @param [in] height 像素点行坐标
	 * @param [in]     direction  优化方向
	 * @param [in/out] costMaps   代价体
	 * @param [in]     rightFirst 左右视图标志位
	 * @return =0: 成功; < 0: 错误代码
	 */
	int verticalComputation(const int& height, const int& direction, std::vector<cv::Mat>* costMaps, bool rightFirst);
	/**
	 * @brief 垂直方向扫描线优化, 选择两个像素点代价.
	 * @param [in]     height1    像素点1行坐标
	 * @param [in]     height2    像素点2行坐标
	 * @param [in/out] costMaps   代价体
	 * @param [in]     rightFirst 左右视图标志位
	 * @return =0: 成功; < 0: 错误代码
	 */
	int verticalOptimization(const int& height1, const int& height2, std::vector<cv::Mat>* costMaps, bool rightFirst);
	/**
	 * @brief 水平方向扫描线优化, 逐列优化.
	 * @param [in]     width      像素点列坐标
	 * @param [in]     direction  优化方向
	 * @param [in/out] costMaps   代价体
	 * @param [in]     rightFirst 左右视图标志位
	 * @return =0: 成功; < 0: 错误代码
	 */
	int horizontalComputation(const int& width, const int& direction, std::vector<cv::Mat>* costMaps, bool rightFirst);
	/**
	 * @brief 水平方向扫描线优化, 选择两个像素点代价.
	 * @param [in]     width1     像素点1列坐标
	 * @param [in]     width2     像素点2列坐标
	 * @param [in/out] costMaps   代价体
	 * @param [in]     rightFirst 左右视图标志位
	 * @return =0: 成功; < 0: 错误代码
	 */
	int horizontalOptimization(const int& width1, const int& width2, std::vector<cv::Mat>* costMaps, bool rightFirst);
	/**
	 * @brief 逐像素更新扫描线优化代价体.
	 * @param [in]     height1    像素1坐标
	 * @param [in]     height2    像素2坐标
	 * @param [in]     width1     像素1坐标
	 * @param [in]     width2     像素2坐标
	 * @param [in/out] costMaps   代价体
	 * @param [in]     rightFirst 左右视图标志位
	 * @return =0: 成功; < 0: 错误代码
	 */
	int partialOptimization(const int& height1, const int& height2, const int& width1, const int& width2, std::vector<cv::Mat>* costMaps, bool rightFirst);
	/**
	 * @brief 计算惩罚参数P1, P2.
	 * @param [in]  height1    像素点p坐标
	 * @param [in]  height2    像素点pd坐标
	 * @param [in]  width1     像素点p坐标
	 * @param [in]  width2     像素点pd坐标
	 * @param [in]  disparity  视差
	 * @param [out] p1         惩罚参数P1
	 * @param [out] p2         惩罚参数P2
	 * @param [in]  rightFirst 左右视图标志位
	 * @return =0: 成功; < 0: 错误代码
	 */
	int computeP1P2(const int& height1, const int& height2, const int& width1, const int& width2, int disparity, float& p1, float& p2, bool rightFirst);
	/**
	 * @brief 扫描线优化.
	 * @param [in/out] costMaps   代价体
	 * @param [in]     rightFirst 左右视图标志位
	 * @return =0: 成功; < 0: 错误代码
	 */
	int scanline(std::vector<cv::Mat>* costMaps, bool rightFirst);
	/**
	 * @brief [AD-Census算法] Step.3 扫描线优化.
	 * @param [in] writeProcess 保存结果
	 * @param [in] writeDir     保存路径
	 */
	void scanlineOptimize(const bool& writeProcess = false, const std::string& writeDir = std::string());
	/**
	 * @brief 离群点检测.
	 * @param [in] leftDisp  左视角视差图
	 * @param [in] rightDisp 右视角视差图
	 * @return 离群点检测后的左视角视差图
	 */
	cv::Mat outlierElimination(const cv::Mat& leftDisp, const cv::Mat& rightDisp);
	/**
	 * @brief 区域投票.
	 * @param [in/out] disparity       视差图
	 * @param [in/out] upLimits        臂延伸向上限制
	 * @param [in/out] downLimits      臂延伸向下限制
	 * @param [in/out] leftLimits      臂延伸向左限制
	 * @param [in/out] rightLimits     臂延伸向右限制
	 * @param [in]     horizontalFirst 左右视图标志位
	 * @return =0: 成功; < 0: 错误代码
	 */
	int regionVoting(cv::Mat& disparity, std::vector<cv::Mat>& upLimits, std::vector<cv::Mat>& downLimits,
		std::vector<cv::Mat>& leftLimits, std::vector<cv::Mat>& rightLimits, bool horizontalFirst);
	/**
	 * @brief 合理插值.
	 * @param [in/out] disparity 视差图
	 * @param [in]    leftImage 左图
	 * @return =0: 成功; < 0: 错误代码
	 */
	int properInterpolation(cv::Mat& disparity, const cv::Mat& leftImage);
	/**
	 * @brief 创建视差灰度图.
	 * @param [in] disparity 视差图
	 * @return 灰度视差图
	 */
	cv::Mat convertDisp2Gray(const cv::Mat& disparity);
	/**
	 * @brief 不连续调整.
	 * @param [in/out] disparity 视差图
	 * @param [in]     costs     代价体
	 * @return =0: 成功; < 0: 错误代码
	 */
	int discontinuityAdjustment(cv::Mat& disparity, const std::vector<std::vector<cv::Mat>>& costs);
	/**
	 * @brief 亚像素增强.
	 * @param [in] disparity 视差图
	 * @param [in] costs     代价矩阵
	 * @return 浮点型视差
	 */
	cv::Mat subpixelEnhancement(const cv::Mat& disparity, const std::vector<std::vector<cv::Mat>>& costs);
	/**
	 * @brief [AD-Census算法] Step.4 多步骤视差优化.
	 * @return =0: 成功; < 0: 错误代码
	 */
	int multiOptimize();
	/**
	 * @brief "赢者通吃"策略.
	 * @param [in]  imageNo 左右视图标志位
	 * @param [out] dst     视差图
	 * @return =0: 成功; < 0: 错误代码
	 */
	void cost2disparity(const int& imageNo, cv::Mat& dst);
	/**
	 * @brief ROI立体匹配视差补偿.
	 * @param [in] offset 补偿像素, px
	 * @return =0: 成功; < 0: 错误代码
	 */
	int disparityOffset(const int& offset);
	/**
	 * @brief RGB颜色模型转为HSI颜色模型.
	 * @param [in]  src    RGB图像
	 * @param [out] dst    HSI图像
	 * @param [in]  filter 颜色滤波
	 * @return =0: 成功; < 0: 错误代码
	 */
	int bgr2hsi(const cv::Mat src, cv::Mat& dst, const bool& filter = false);
	/**
	 * @brief 计算高斯权重中心, 滤波.
	 * @param [in]  src   原始图像
	 * @param [out] dst   高斯权重中心
	 * @param [in]  kSize 核尺寸
	 * @param [in]  sigma 高斯分布σ
	 */
	void computeGaussMedian(const cv::Mat& src, cv::Mat& dst, const int& kSize = 3, const double& sigma = -1);
public:
	int m_minDisparity;  /*!< 最小搜索范围 */
	int m_maxDisparity;  /*!< 最大搜索范围 */
	ADCensusParams m_paMatching; /*!< 立体匹配参数结构体 */
	ColorModel m_colorModel;     /*!< 立体匹配颜色模型 */
	bool m_roiMatching;  /*!< 立体匹配ROI模式 */
	bool m_maskMatching; /*!< 立体匹配MASK模式 */
	int m_offset;        /*!< ROI/MASK模式视差补偿 */
	cv::Mat m_images[2]; /*!< 极线校正后的左右图 */
	cv::Size m_imageSize;/*!< 左右视图尺寸 */
	std::vector<cv::Mat> m_upLimits;   /*!< 臂延伸 左、右图像像素点垂直臂上极限位置[k](i,j) k值: 0=左，1=右 */
	std::vector<cv::Mat> m_downLimits; /*!< 臂延伸 左、右图像像素点垂直臂下极限位置[k](i,j) k值: 0=左，1=右 */
	std::vector<cv::Mat> m_leftLimits; /*!< 臂延伸 左、右图像像素点水平臂左极限位置[k](i,j) k值: 0=左，1=右 */
	std::vector<cv::Mat> m_rightLimits;/*!< 臂延伸 左、右图像像素点水平臂右极限位置[k](i,j) k值: 0=左，1=右 */
	std::vector<std::vector<cv::Mat>> m_costMaps; /*!< 代价体[k][d](i,j) k值: 0=左，1=右; d-视差; (i,j)-图像坐标 *///Cost volume
	cv::Mat m_disparityMap;      /*!< 视差, 整型视差*/
	cv::Mat m_floatDisparityMap; /*!< 视差, 浮点型真实视差 */
	int m_occlusionValue;        /*!< 遮挡点代价(标志位) */
	int m_mismatchValue;         /*!< 误匹配代价(标志位) */
	static const int DISP_OCCLUSION = 1; /*!< 遮挡点视差值(minDisp - occlusion) */
	static const int DISP_MISMATCH = 2; /*!< 误匹配视差值(minDisp - mismatch) */
};

stereo::ADCensus::ADCensus()
{
	this->impl = std::make_unique<ADCensusImpl>();
}

stereo::ADCensus::~ADCensus()
{
}

void stereo::ADCensus::setMinMaxDisparity(const int& minDisparity, const int& maxDisparity)
{
	if (minDisparity * maxDisparity < 0 or minDisparity >= maxDisparity)
		throw(std::string("[ADCensus] Set MinMaxDisparity error."));
	this->impl->m_minDisparity = minDisparity;
	this->impl->m_maxDisparity = maxDisparity;
}

void stereo::ADCensus::setMatchingStrategy(const ColorModel& colorModel, const bool& roiMatching, const bool& maskMatching)
{
	this->impl->m_colorModel = colorModel;
	this->impl->m_paMatching = ADCensusParams(colorModel);
	this->impl->m_roiMatching = roiMatching;
	this->impl->m_maskMatching = maskMatching;
}

void stereo::ADCensus::setOffset(const int& offset)
{
	if (offset < 0)
		throw(std::string("[ADCensus] Offset must be positive."));
	this->impl->m_offset = offset;
}

cv::Mat stereo::ADCensus::compute(const cv::Mat& leftImage, const cv::Mat& rightImage)
{
	if (leftImage.empty() or rightImage.empty() or leftImage.size() != rightImage.size())
		throw(std::string("[ADCensus] Image error."));
	LOG_INFO("Computing disparity...");
	auto start = std::chrono::steady_clock::now();
	this->impl->m_images[0] = leftImage.clone();
	this->impl->m_images[1] = rightImage.clone();
	this->impl->m_imageSize = leftImage.size();
	if (this->impl->m_roiMatching or this->impl->m_maskMatching)
		this->impl->m_maxDisparity = this->impl->m_imageSize.width / 2;
	//初始化代价体Cost Volume
	this->impl->m_costMaps.resize(2);
	for (int i = 0; i < 2; ++i)
	{
		this->impl->m_costMaps[i].resize(this->impl->m_maxDisparity - this->impl->m_minDisparity + 1);
		for (int j = 0; j < this->impl->m_costMaps[i].size(); ++j)
		{
			this->impl->m_costMaps[i][j].create(this->impl->m_imageSize, CV_32F);
		}
	}
	if (this->impl->m_colorModel == ColorModel::HSI)
	{
		cv::Mat leftHSI, rightHSI;
		if (this->impl->m_maskMatching or this->impl->m_roiMatching)
		{
			this->impl->bgr2hsi(leftImage, leftHSI, true);
			this->impl->bgr2hsi(rightImage, rightHSI, true);
			this->impl->m_images[0] = leftHSI.clone();
			this->impl->m_images[1] = rightHSI.clone();
		}
		else
		{
			this->impl->bgr2hsi(leftImage, leftHSI, false);
			this->impl->bgr2hsi(rightImage, rightHSI, false);
			cv::Mat filt1, filt2;
			this->impl->computeGaussMedian(leftHSI, filt1, 3);
			this->impl->computeGaussMedian(rightHSI, filt2, 3);
			this->impl->m_images[0] = filt1.clone();
			this->impl->m_images[1] = filt2.clone();
		}
	}
	try
	{
		LOG_INFO("Computing cost...");
		this->impl->costInitialize();
		LOG_INFO("Aggregating cost...");
		this->impl->costAggregate();
		LOG_INFO("Scanline optimizing...");
		this->impl->scanlineOptimize();
		LOG_INFO("Multi-optimizing...");
		this->impl->multiOptimize();
	}
	catch (const std::string& e)
	{
		std::cout << e << std::endl;
		throw std::runtime_error(e);
	}
	if (this->impl->m_roiMatching or this->impl->m_maskMatching)
		this->impl->disparityOffset(this->impl->m_offset);
	//-- 计算结果, 左视图-视差图
	cv::Mat disparity = this->impl->m_floatDisparityMap.clone();
	if (this->impl->m_maskMatching or this->impl->m_roiMatching)
	{
		for (int i = 0; i < disparity.rows; ++i)
		{
			for (int j = 0; j < disparity.cols; ++j)
			{
				if ((leftImage.ptr<cv::Vec3b>(i)[j] == cv::Vec3b(0, 0, 0) and disparity.ptr<float>(i)[j] > 0)
					or disparity.ptr<float>(i)[j] == 0)
					disparity.ptr<float>(i)[j] = -1.f;
			}
		}
	}
	auto end = std::chrono::steady_clock::now();
	auto tt = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	LOG_INFO("Disparity map computed. Timing: " + utils::formatMilliseconds(tt.count() / 1000.0) + " ms.");
	return disparity;
}

stereo::ADCensus::ADCensusImpl::ADCensusImpl()
{
	this->m_minDisparity = 0;
	this->m_maxDisparity = 64;
	this->m_colorModel = ColorModel::HSI;
	this->m_paMatching = ADCensusParams(this->m_colorModel);
	this->m_occlusionValue = 0 - DISP_OCCLUSION;
	this->m_mismatchValue = 0 - DISP_MISMATCH;
	this->m_maskMatching = false;
	this->m_roiMatching = false;
	this->m_offset = NULL;
}

stereo::ADCensus::ADCensusImpl::~ADCensusImpl()
{
}

float stereo::ADCensus::ADCensusImpl::computeRGBADCost(const int& height1, const int& width1, const int& height2, const int& width2)
{
	float adCost = 0.f;
	const cv::Vec3b& rgbLP = this->m_images[0].at<cv::Vec3b>(height1, width1);
	const cv::Vec3b& rgbRP = this->m_images[1].at<cv::Vec3b>(height2, width2);
	//p和pd在R,G,B三通道的AD代价
	for (int i = 0; i < 3; ++i)
		adCost += std::abs(rgbLP[i] - rgbRP[i]);
	//p和pd在R,G,B三通道的平均AD代价
	adCost = adCost / 3.f;
	return adCost;
}

float stereo::ADCensus::ADCensusImpl::computeHSIADCost(const int& height1, const int& width1, const int& height2, const int& width2)
{
	float adCost = 0.f;
	const cv::Vec3b& HSILP = this->m_images[0].at<cv::Vec3b>(height1, width1);
	const cv::Vec3b& HSIRP = this->m_images[1].at<cv::Vec3b>(height2, width2);
	//Hue通道代价
	int hueDiff = std::abs(HSILP[0] - HSIRP[0]);
	adCost += cv::min(hueDiff, 255 - hueDiff) * this->m_paMatching.lambdaHue;
	//Saturation通道代价
	adCost += std::abs(HSILP[1] - HSIRP[1]) * this->m_paMatching.lambdaSaturation;
	//Intensity通道代价
	adCost += std::abs(HSILP[2] - HSIRP[2]) * this->m_paMatching.lambdaIntensity;
	return adCost;
}

float stereo::ADCensus::ADCensusImpl::computeRGBCensusCost(const int& height1, const int& width1, const int& height2, const int& width2, const int& censusWinHeight, const int& censusWinWidth)
{
	float censusCost = 0.f;
	const cv::Vec3b& rgbLP = this->m_images[0].at<cv::Vec3b>(height1, width1);
	const cv::Vec3b& rgbRP = this->m_images[1].at<cv::Vec3b>(height2, width2);
	if (this->m_maskMatching and (rgbLP == cv::Vec3b(0, 0, 0) or rgbRP == cv::Vec3b(0, 0, 0)))
		return std::numeric_limits<float>::infinity();
	for (int i = -censusWinHeight / 2; i <= censusWinHeight / 2; ++i)
	{
		for (int j = -censusWinWidth / 2; j <= censusWinWidth / 2; ++j)
		{
			const cv::Vec3b& rgbLPA = this->m_images[0].at<cv::Vec3b>(height1 + i, width1 + j);
			const cv::Vec3b& rgbRPA = this->m_images[1].at<cv::Vec3b>(height2 + i, width2 + j);
			for (int k = 0; k < 3; ++k)
			{
				censusCost += ((rgbLPA[k] - rgbLP[k]) * (rgbRPA[k] - rgbRP[k]) < 0) ? 1 : 0;
			}
		}
	}
	return censusCost;
}

float stereo::ADCensus::ADCensusImpl::computeHSICensusCost(const int& height1, const int& width1, const int& height2, const int& width2, const int& censusWinHeight, const int& censusWinWidth)
{
	float censusCost = 0.f;
	const cv::Vec3b& hsiLP = this->m_images[0].at<cv::Vec3b>(height1, width1);
	const cv::Vec3b& hsiRP = this->m_images[1].at<cv::Vec3b>(height2, width2);
	if (this->m_maskMatching and (hsiLP == cv::Vec3b(0, 0, 0) or hsiRP == cv::Vec3b(0, 0, 0)))
		return std::numeric_limits<float>::infinity();
	for (int i = -censusWinHeight / 2; i <= censusWinHeight / 2; ++i)
	{
		for (int j = -censusWinWidth / 2; j <= censusWinWidth / 2; ++j)
		{
			const cv::Vec3b& hsiLPA = this->m_images[0].at<cv::Vec3b>(height1 + i, width1 + j);
			const cv::Vec3b& hsiRPA = this->m_images[1].at<cv::Vec3b>(height2 + i, width2 + j);
			int dHPL = hsiLPA[0] - hsiLP[0];
			int dHPR = hsiRPA[0] - hsiRP[0];
			censusCost += (dHPL <= -127 or (dHPL >= 0 and dHPL <= 127))
				and (dHPR <= -127 or (dHPR >= 0 and dHPR <= 127)) ? 0 : 1;
			censusCost += ((hsiLPA[1] - hsiLP[1]) * (hsiRPA[1] - hsiRP[1]) < 0) ? 1 : 0;
			censusCost += ((hsiLPA[2] - hsiLP[2]) * (hsiRPA[2] - hsiRP[2]) < 0) ? 1 : 0;
		}
	}
	return censusCost;
}

float stereo::ADCensus::ADCensusImpl::computeADCensusCost(const int& height1, const int& width1, const int& height2, const int& width2, const int& censusWinHeight, const int& censusWinWidth)
{
	float adCost = 0.f;
	float censusCost = 0.f;
	float adCensusCost = 0.f;
	switch (this->m_colorModel)
	{
	case ColorModel::RGB:
		adCost = computeRGBADCost(height1, width1, height2, width2);
		censusCost = computeRGBCensusCost(height1, width1, height2, width2, censusWinHeight, censusWinWidth);
		break;
	case ColorModel::HSI:
		adCost = computeHSIADCost(height1, width1, height2, width2);
		censusCost = computeHSICensusCost(height1, width1, height2, width2, censusWinHeight, censusWinWidth);
		break;
	default:
		return std::numeric_limits<float>::infinity();
	}
	adCensusCost = 2.f - std::exp(-adCost / this->m_paMatching.lambdaAD) - std::exp(-censusCost / this->m_paMatching.lambdaCensus);
	return adCensusCost;
}

void stereo::ADCensus::ADCensusImpl::costInitialize(const bool& writeProcess, const std::string& writeDir)
{
	int censusWinHeight, censusWinWidth;
	switch (this->m_paMatching.censusWin)
	{
	case CensusWin::CENSUSWIN_9x7:
		censusWinWidth = 9;
		censusWinHeight = 7;
		break;
	case CensusWin::CENSUSWIN_7x5:
		censusWinWidth = 7;
		censusWinHeight = 5;
		break;
	default:
		break;
	}
	cv::Size halfCensusWin(censusWinWidth / 2, censusWinHeight / 2);
	for (int imageNo = 0; imageNo < 2; ++imageNo)
	{
		int d, i, j, colL, colR; bool out;
#pragma omp parallel default (shared) private(d, i, j, colL, colR, out) num_threads(omp_get_max_threads())
#pragma omp for schedule(static)
		for (d = 0; d <= this->m_maxDisparity - this->m_minDisparity; ++d)
		{
			for (i = 0; i < this->m_imageSize.height; ++i)
			{
				for (j = 0; j < this->m_imageSize.width; ++j)
				{
					//-- 掩膜图像非有效数据记为越界代价
					if (this->m_maskMatching and this->m_images[imageNo].at<cv::Vec3b>(i, j) == cv::Vec3b(0, 0, 0))
					{
						this->m_costMaps[imageNo][d].at<float>(i, j) = 2.f;
						continue;
					}
					colL = j - this->m_minDisparity;
					colR = j + this->m_minDisparity;
					if (imageNo == 0)//对于左图，搜索右图中对应同名点(向左搜索)
						colR = j - d;
					else             //对于右图，搜索左图中对应同名点(向右搜索)
						colL = j + d;
					out = colL - halfCensusWin.width < 0 or colL + halfCensusWin.width >= this->m_imageSize.width
						or colR - halfCensusWin.width < 0 or colR + halfCensusWin.width >= this->m_imageSize.width
						or i - halfCensusWin.height < 0 or i + halfCensusWin.height >= this->m_imageSize.height;
					if (out)
						this->m_costMaps[imageNo][d].at<float>(i, j) = 2.f;//越界代价
					else
						this->m_costMaps[imageNo][d].at<float>(i, j) = computeADCensusCost(i, colL, i, colR, censusWinHeight, censusWinWidth);
				}
			}
		}
	}
	if (writeProcess and !writeDir.empty())
	{
		cv::Mat disparity0, disparity1;
		cost2disparity(0, disparity0);
		cv::imwrite(writeDir + "adcensusL.bmp", disparity0);
		cost2disparity(1, disparity1);
		cv::imwrite(writeDir + "adcensusR.bmp", disparity1);
	}
}

int stereo::ADCensus::ADCensusImpl::colorDiff(const cv::Vec3b& p1, const cv::Vec3b& p2)
{
	int diff = 0;
	switch (this->m_colorModel)
	{
	case ColorModel::RGB:
		for (int i = 0; i < 3; ++i)
		{
			int colorDiff = std::abs(p1[i] - p2[i]);
			diff = (diff > colorDiff) ? diff : colorDiff;
		}
		break;
	case ColorModel::HSI:
		diff = std::min(std::abs(p1[0] - p2[0]), 255 - std::abs(p1[0] - p2[0]));
		break;
	default:
		return std::numeric_limits<int>::infinity();
	}
	return diff;
}

int stereo::ADCensus::ADCensusImpl::computeLimit(const int& height, const int& width, const int& directionH, const int& directionW, const int& imageNo)
{
	//当前像素点p[row,col]的颜色[B,G,R]
	cv::Vec3b p = this->m_images[imageNo].at<cv::Vec3b>(height, width);
	//臂延伸长度初值=1像素
	int d = 1;
	//边界候选像素点p1坐标[row1,col1]
	int height1 = height + directionH;
	int width1 = width + directionW;
	//p1点的前一点p2的颜色[B,G,R]
	cv::Vec3b p2 = p;
	//判断p1点是否在图像内部区域
	bool inside = (0 <= height1) and (height1 < this->m_imageSize.height) and (0 <= width1) and (width1 < this->m_imageSize.width);
	if (inside)
	{
		bool colorCond = true, wLimitCond = true, fColorCond = true;
		//颜色限制、延伸长度、相似度均满足的条件下逐步推进p2点
		while (colorCond and wLimitCond and fColorCond and inside)
		{
			cv::Vec3b p1 = this->m_images[imageNo].at<cv::Vec3b>(height1, width1);
			//Mask匹配模式下遇到背景像素停止延伸
			if (this->m_maskMatching and p1 == cv::Vec3b(0, 0, 0))
			{
				d++;
				break;
			}
			//判断p1,p2,p点是否有相似的颜色强度？
			colorCond = colorDiff(p, p1) < this->m_paMatching.colorThresh1 and colorDiff(p1, p2) < this->m_paMatching.colorThresh1;
			if (this->m_colorModel == ColorModel::HSI)
			{
				colorCond = std::abs(p[1] - p1[1]) < this->m_paMatching.saturationThresh1 and std::abs(p1[1] - p2[1]) < this->m_paMatching.saturationThresh1;
				colorCond = std::abs(p[2] - p1[2]) < this->m_paMatching.intensityThresh1 and std::abs(p1[2] - p2[2]) < this->m_paMatching.intensityThresh1;
			}
			//判断臂延伸长度是否达到最大阈值1?
			wLimitCond = d < this->m_paMatching.maxLength1;
			//判断较远点是否有更好的颜色相似度？
			fColorCond = (d <= this->m_paMatching.maxLength2) or (d > this->m_paMatching.maxLength2 and colorDiff(p, p1) < this->m_paMatching.colorThresh2);
			if (this->m_colorModel == ColorModel::HSI)
			{
				fColorCond = (d <= this->m_paMatching.maxLength2) or (d > this->m_paMatching.maxLength2 and std::abs(p[1] - p1[1]) < this->m_paMatching.saturationThresh2);
				fColorCond = (d <= this->m_paMatching.maxLength2) or (d > this->m_paMatching.maxLength2 and std::abs(p[2] - p1[2]) < this->m_paMatching.intensityThresh2);
			}
			//p1点向搜寻方向移动1像素，p2点同时移动1像素
			p2 = p1;
			height1 += directionH;
			width1 += directionW;
			//判断p1点是否在图像内部区域
			inside = (0 <= height1) and (height1 < this->m_imageSize.height) and (0 <= width1) and (width1 < this->m_imageSize.width);
			//臂延伸长度增加1像素
			d++;
		}
		//到达臂延伸边界后，为退出循环会多1像素，故需减1像素
		d--;
	}
	return d - 1;//臂延伸真实像素
}

cv::Mat stereo::ADCensus::ADCensusImpl::computeLimits(const int& directionH, const int& directionW, const int& imageNo)
{
	//臂延伸长度矩阵
	cv::Mat limits = cv::Mat::zeros(this->m_imageSize, CV_32S);
	int height, width;
#pragma omp parallel default (shared) private(height, width) num_threads(omp_get_max_threads())
#pragma omp for schedule(static)
	for (height = 0; height < this->m_imageSize.height; ++height)
	{
		for (width = 0; width < this->m_imageSize.width; ++width)
		{
			//Mask匹配背景像素点臂长为0
			if (this->m_maskMatching and this->m_images[imageNo].at<cv::Vec3b>(height, width) == cv::Vec3b(0, 0, 0))
			{
				limits.at<int>(height, width) = 0;
				continue;
			}
			//逐像素计算臂延伸长度
			limits.at<int>(height, width) = computeLimit(height, width, directionH, directionW, imageNo);
		}
	}
	return limits;
}

cv::Mat stereo::ADCensus::ADCensusImpl::aggregation1D(const cv::Mat& costMap, const int& directionH, const int& directionW, cv::Mat& windowSizes, const int& imageNo)
{
	//代价聚合窗口大小
	cv::Mat tmpWindowSizes = cv::Mat::zeros(this->m_imageSize, CV_32S);
	//聚合代价
	cv::Mat aggregatedCosts(this->m_imageSize, CV_32F);//32位浮点数,float
	int height = 0, width = 0;
	//臂延伸区间[dmin,dmax], 迭代当前位置d
	int dmin, dmax, d;
#pragma omp parallel default (shared) private(height, width, d) num_threads(omp_get_max_threads())
#pragma omp for schedule(static)
	for (height = 0; height < this->m_imageSize.height; ++height)
	{
		for (width = 0; width < this->m_imageSize.width; ++width)
		{
			if (directionH == 0)//水平臂
			{
				dmin = -this->m_leftLimits[imageNo].at<int>(height, width);
				dmax = this->m_rightLimits[imageNo].at<int>(height, width);
			}
			else//垂直臂
			{
				dmin = -this->m_upLimits[imageNo].at<int>(height, width);
				dmax = this->m_downLimits[imageNo].at<int>(height, width);
			}
			//代价求和
			float cost = 0;
			//常规方法求代价聚合
			for (d = dmin; d <= dmax; ++d)
			{
				cost += costMap.at<float>(height + d * directionH, width + d * directionW);
				tmpWindowSizes.at<int>(height, width) += windowSizes.at<int>(height + d * directionH, width + d * directionW);
			}
			aggregatedCosts.at<float>(height, width) = cost;
		}
	}
	windowSizes = tmpWindowSizes.clone();
	return aggregatedCosts;
}

int stereo::ADCensus::ADCensusImpl::aggregation2D(cv::Mat& costMap, bool horizontalFirst, const int& imageNo)
{
	//臂搜索方向H为垂直方向，W为水平方向
	int directionH = 1, directionW = 0;
	//先水平臂or先垂直臂
	if (horizontalFirst)
		std::swap(directionH, directionW);
	//十字交叉臂窗口尺寸
	cv::Mat windowsSizes = cv::Mat::ones(this->m_imageSize, CV_32S);
	for (int direction = 0; direction < 2; ++direction)
	{
		horizontalFirst = !horizontalFirst;
		//一维代价聚合，覆盖原匹配代价
		(aggregation1D(costMap, directionH, directionW, windowsSizes, imageNo)).copyTo(costMap);
		//交换搜索方向
		std::swap(directionH, directionW);
	}
	//最终聚合代价=聚合代价/窗口大小
	for (int height = 0; height < this->m_imageSize.height; ++height)
	{
		for (int width = 0; width < this->m_imageSize.width; ++width)
		{
			costMap.at<float>(height, width) /= windowsSizes.at<int>(height, width);
		}
	}
	return 0;
}

void stereo::ADCensus::ADCensusImpl::costAggregate(const bool& writeProcess, const std::string& writeDir)
{
	//-- 计算十字交叉臂臂长
	this->m_upLimits.resize(2);
	this->m_downLimits.resize(2);
	this->m_leftLimits.resize(2);
	this->m_rightLimits.resize(2);
	for (int imageNo = 0; imageNo < 2; ++imageNo)
	{
		this->m_upLimits[imageNo] = computeLimits(-1, 0, imageNo);
		this->m_downLimits[imageNo] = computeLimits(1, 0, imageNo);
		this->m_leftLimits[imageNo] = computeLimits(0, -1, imageNo);
		this->m_rightLimits[imageNo] = computeLimits(0, 1, imageNo);
	}
	//左、右图分别代价聚合
	for (int imageNo = 0; imageNo < 2; ++imageNo)
	{
		int d, i;
#pragma omp parallel default (shared) private(d, i) num_threads(omp_get_max_threads())
#pragma omp for schedule(static)
		//不同视差下的代价聚合
		for (d = 0; d <= this->m_maxDisparity - this->m_minDisparity; ++d)
		{
			bool horizontalFirst = true;
			//n次代价聚合迭代
			for (i = 0; i < this->m_paMatching.iterations; ++i)
			{
				aggregation2D(this->m_costMaps[imageNo][d], horizontalFirst, imageNo);
				horizontalFirst = !horizontalFirst;
			}
		}
	}
	if (writeProcess and !writeDir.empty())
	{
		cv::Mat disparity0, disparity1;
		cost2disparity(0, disparity0);
		cv::imwrite(writeDir + "aggregationL.bmp", disparity0);
		cost2disparity(1, disparity1);
		cv::imwrite(writeDir + "aggregationR.bmp", disparity1);
	}
}

int stereo::ADCensus::ADCensusImpl::verticalComputation(const int& height, const int& direction, std::vector<cv::Mat>* costMaps, bool rightFirst)
{
	//计算垂直方向优化代价
	int height1;
	if (direction == 1)
	{
#pragma omp parallel default(shared) private(height1) num_threads(omp_get_max_threads())
#pragma omp for schedule(static)
		for (height1 = height + direction; height1 < this->m_imageSize.height; height1 += direction)
		{
			verticalOptimization(height1, height1 - direction, costMaps, rightFirst);
		}
	}
	else
	{
#pragma omp parallel default(shared) private(height1) num_threads(omp_get_max_threads())
#pragma omp for schedule(static)
		for (height1 = height + direction; height1 >= 0; height1 += direction)
		{
			verticalOptimization(height1, height1 - direction, costMaps, rightFirst);
		}
	}
	return 0;
}

int stereo::ADCensus::ADCensusImpl::verticalOptimization(const int& height1, const int& height2, std::vector<cv::Mat>* costMaps, bool rightFirst)
{
	for (int width = 0; width < this->m_imageSize.width; width++)
	{
		if (this->m_maskMatching and this->m_images[rightFirst].at<cv::Vec3b>(height2, width) == cv::Vec3b(0, 0, 0))
			continue;
		partialOptimization(height1, height2, width, width, costMaps, rightFirst);
	}
	return 0;
}

int stereo::ADCensus::ADCensusImpl::horizontalComputation(const int& width, const int& direction, std::vector<cv::Mat>* costMaps, bool rightFirst)
{
	//计算水平方向优化代价
	int width1;
	if (direction == 1)
	{
#pragma omp parallel default(shared) private(width1) num_threads(omp_get_max_threads())
#pragma omp for schedule(static)
		for (width1 = width + direction; width1 < this->m_imageSize.width; width1 += direction)
		{
			//printf("[ScanlineOptimization] started rightward vertical height = % d optimization.\n", width1);
			horizontalOptimization(width1, width1 - direction, costMaps, rightFirst);
		}
	}
	else
	{
#pragma omp parallel default(shared) private(width1) num_threads(omp_get_max_threads())
#pragma omp for schedule(static)
		for (width1 = width + direction; width1 >= 0; width1 += direction)
		{
			//printf("[ScanlineOptimization] started leftward vertical height = % d optimization.\n", width1);
			horizontalOptimization(width1, width1 - direction, costMaps, rightFirst);
		}
	}
	return 0;
}

int stereo::ADCensus::ADCensusImpl::horizontalOptimization(const int& width1, const int& width2, std::vector<cv::Mat>* costMaps, bool rightFirst)
{
	for (int height = 0; height < this->m_imageSize.height; height++)
	{
		if (this->m_maskMatching and this->m_images[rightFirst].at<cv::Vec3b>(height, width2) == cv::Vec3b(0, 0, 0))
			continue;
		partialOptimization(height, height, width1, width2, costMaps, rightFirst);
	}
	return 0;
}

int stereo::ADCensus::ADCensusImpl::partialOptimization(const int& height1, const int& height2, const int& width1, const int& width2, std::vector<cv::Mat>* costMaps, bool rightFirst)
{
	float minOptCost = costMaps->at(0).at<float>(height2, width2);
	//计算min_k{ Cr(p-r, k) },找[width2,height2]即前一像素点在不同视差下最小的路径代价
	for (int disparity = 1; disparity <= this->m_maxDisparity - this->m_minDisparity; ++disparity)
	{
		float tmpCost = costMaps->at(disparity).at<float>(height2, width2);
		if (minOptCost > tmpCost)
			minOptCost = tmpCost;
	}
	//如果前一像素点在不同视差下的最小路径代价为0,不执行扫描线优化
	if (minOptCost == 0)
		return 0;
	float minkCr = minOptCost;//min_k{ Cr(p-r, k) }
	for (int disparity = 0; disparity <= this->m_maxDisparity - this->m_minDisparity; ++disparity)
	{
		//计算C1(p,d) - min_k(Cr(p-r,k))
		float cost = costMaps->at(disparity).at<float>(height1, width1) - minkCr;
		//计算惩罚参数P1,P2
		float p1 = 0.f, p2 = 0.f;
		computeP1P2(height1, height2, width1, width2, disparity + this->m_minDisparity, p1, p2, rightFirst);
		//计算min{ Cr(p-r,d), Cr(p-r, d+-1) + P1, min_k(Cr(p-r,k)+P2) }
		minOptCost = minkCr + p2;
		float tmpCost = costMaps->at(disparity).at<float>(height2, width2);
		if (minOptCost > tmpCost)
			minOptCost = tmpCost;
		//沿视差d方向向下1像素搜索, Cr(p-r, d-1) + P1
		if (disparity != 0)
		{
			tmpCost = costMaps->at(disparity - 1).at<float>(height2, width2) + p1;
			if (minOptCost > tmpCost)
				minOptCost = tmpCost;
		}
		//沿视差d方向向上1像素搜索, Cr(p-r, d+1) + P1
		if (disparity != this->m_maxDisparity - this->m_minDisparity)
		{
			tmpCost = costMaps->at(disparity + 1).at<float>(height2, width2) + p1;
			if (minOptCost > tmpCost)
				minOptCost = tmpCost;
		}
		//计算 Cr(p, d) / 2, 1个方向的路径代价
		costMaps->at(disparity).at<float>(height1, width1) = (float)(((cost + minOptCost)) / 2);
	}
	return 0;
}

int stereo::ADCensus::ADCensusImpl::computeP1P2(const int& height1, const int& height2, const int& width1, const int& width2, int disparity, float& p1, float& p2, bool rightFirst)
{
	int imageNo = 0;//左图
	int otherImgNo = 1;//右图
	if (rightFirst)
	{
		imageNo = 1;
		otherImgNo = 0;
		disparity = -disparity;
	}
	//计算左、右图像素点与沿同一方向前一像素点的颜色差值
	int d1 = 0;
	d1 = colorDiff(this->m_images[imageNo].at<cv::Vec3b>(height1, width1), this->m_images[imageNo].at<cv::Vec3b>(height2, width2));
	int d2 = this->m_paMatching.colorDiff + 1;
	if (0 <= width1 + disparity and width1 + disparity < m_imageSize.width
		and 0 <= width2 + disparity and width2 + disparity < m_imageSize.width)
	{
		d2 = colorDiff(this->m_images[otherImgNo].at<cv::Vec3b>(height1, width1 + disparity),
			this->m_images[otherImgNo].at<cv::Vec3b>(height2, width2 + disparity));
	}
	//int ds1 = std::abs(this->m_images[imageNo].at<cv::Vec3b>(height1, width1)[1] - this->m_images[imageNo].at<cv::Vec3b>(height2, width2)[1]);
	//int ds2 = c_colorDiff + 1;
	//if (0 <= width1 + disparity and width1 + disparity < m_imageSize.width
	//	and 0 <= width2 + disparity and width2 + disparity < m_imageSize.width)
	//{
	//	ds2 = std::abs(this->m_images[otherImgNo].at<cv::Vec3b>(height1, width1 + disparity)[1] -
	//		this->m_images[otherImgNo].at<cv::Vec3b>(height2, width2 + disparity)[1]);
	//}
	//int di1 = std::abs(this->m_images[imageNo].at<cv::Vec3b>(height1, width1)[2] - this->m_images[imageNo].at<cv::Vec3b>(height2, width2)[2]);
	//int di2 = c_colorDiff + 1;
	//if (0 <= width1 + disparity and width1 + disparity < m_imageSize.width
	//	and 0 <= width2 + disparity and width2 + disparity < m_imageSize.width)
	//{
	//	di2 = std::abs(this->m_images[otherImgNo].at<cv::Vec3b>(height1, width1 + disparity)[2] -
	//		this->m_images[otherImgNo].at<cv::Vec3b>(height2, width2 + disparity)[2]);
	//}
	//d1 = std::max(std::max(d1, ds1), di1);
	//d2 = std::max(std::max(d2, ds2), di2);
	//色差与阈值的关系来设定惩罚参数
	if (d1 < this->m_paMatching.colorDiff)
	{
		if (d2 < this->m_paMatching.colorDiff)
		{
			p1 = this->m_paMatching.pi1;
			p2 = this->m_paMatching.pi2;
		}
		else
		{
			p1 = this->m_paMatching.pi1 / 4.f;
			p2 = this->m_paMatching.pi2 / 4.f;
		}
	}
	else
	{
		if (d2 < this->m_paMatching.colorDiff)
		{
			p1 = this->m_paMatching.pi1 / 4.f;
			p2 = this->m_paMatching.pi2 / 4.f;
		}
		else
		{
			p1 = this->m_paMatching.pi1 / 10.f;
			p2 = this->m_paMatching.pi2 / 10.f;
		}
	}
	return 0;
}

int stereo::ADCensus::ADCensusImpl::scanline(std::vector<cv::Mat>* costMaps, bool rightFirst)
{
	//计算4个方向优化代价
	//1-垂直向下
	verticalComputation(0, 1, costMaps, rightFirst);
	//2-垂直向上
	verticalComputation(this->m_imageSize.height - 1, -1, costMaps, rightFirst);
	//3-水平向左
	horizontalComputation(0, 1, costMaps, rightFirst);
	//4-水平向右
	horizontalComputation(this->m_imageSize.width - 1, -1, costMaps, rightFirst);
	return 0;
}

void stereo::ADCensus::ADCensusImpl::scanlineOptimize(const bool& writeProcess, const std::string& writeDir)
{
	for (int imageNo = 0; imageNo < 2; imageNo++)
	{
		scanline(&this->m_costMaps[imageNo], (imageNo == 1));
	}
	if (writeProcess and !writeDir.empty())
	{
		cv::Mat disparity0, disparity1;
		cost2disparity(0, disparity0);
		cv::imwrite(writeDir + "scanlineL.bmp", disparity0);
		cost2disparity(1, disparity1);
		cv::imwrite(writeDir + "scanlineR.bmp", disparity1);
	}
}

cv::Mat stereo::ADCensus::ADCensusImpl::outlierElimination(const cv::Mat& leftDisp, const cv::Mat& rightDisp)
{
	cv::Size dispSize = leftDisp.size();
	cv::Mat disparityMap(dispSize, CV_32S);
#pragma omp parallel for
	for (int height = 0; height < dispSize.height; ++height)
	{
		for (int width = 0; width < dispSize.width; ++width)
		{
			int disparity = leftDisp.at<int>(height, width);
			//若该像素点为离群点,判断是遮挡点还是误匹配点
			//[1]左图像素点在右图中点同名点超过图像范围
			//[2]左图像素点与右图中的同名点视差差异超过允许差异阈值
			if (width - disparity < 0 or abs(disparity - rightDisp.at<int>(height, width - disparity)) > this->m_paMatching.dispTolerance)
			{
				bool occlusion = true;//遮挡
				for (int d = this->m_minDisparity; d <= this->m_maxDisparity; ++d)
				{
					//修正视差值可以在右图中找到对应点,记为误匹配点
					if (width - d >= 0 and d == rightDisp.at<int>(height, width - d))
					{
						occlusion = false;
						break;
					}
				}
				disparity = (occlusion) ? this->m_occlusionValue : this->m_mismatchValue;
			}
			disparityMap.at<int>(height, width) = disparity;
		}
	}
	return disparityMap;
}

int stereo::ADCensus::ADCensusImpl::regionVoting(cv::Mat& disparity, std::vector<cv::Mat>& upLimits, std::vector<cv::Mat>& downLimits, std::vector<cv::Mat>& leftLimits, std::vector<cv::Mat>& rightLimits, bool horizontalFirst)
{
	cv::Size dispSize = disparity.size();
	cv::Mat dispTemp(dispSize, CV_32S);
	//投票直方图
	std::vector<int> hist(this->m_maxDisparity - this->m_minDisparity + 1, 0);
	//十字交叉臂臂长
	const cv::Mat* outerLimitsA;
	const cv::Mat* outerLimitsB;
	const cv::Mat* innerLimitsA;
	const cv::Mat* innerLimitsB;
	//均取左视角臂长
	if (horizontalFirst)
	{
		outerLimitsA = &upLimits[0];    //竖直向上臂长矩阵,对于水平方向迭代,竖直方向为离群
		outerLimitsB = &downLimits[0];  //竖直向下臂长矩阵
		innerLimitsA = &leftLimits[0];  //水平向左臂长矩阵,对于水平方向迭代,水平方向为内部
		innerLimitsB = &rightLimits[0]; //水平向右臂长矩阵
	}
	else
	{
		outerLimitsA = &leftLimits[0];  //水平向左臂长矩阵,对于竖直方向迭代,水平方向为离群
		outerLimitsB = &rightLimits[0]; //水平向右臂长矩阵
		innerLimitsA = &upLimits[0];    //竖直向上臂长矩阵,对于竖直方向迭代,竖直方向为内部
		innerLimitsB = &downLimits[0];  //竖直向下臂长矩阵
	}
	for (int h = 0; h < dispSize.height; ++h)
	{
		for (int w = 0; w < dispSize.width; ++w)
		{
			//非离群点,不更新像素视差
			if (disparity.at<int>(h, w) >= this->m_minDisparity)
			{
				dispTemp.at<int>(h, w) = disparity.at<int>(h, w);
			}
			//离群点
			else
			{
				//离群臂长
				int outerLimitA = -outerLimitsA->at<int>(h, w);
				int outerLimitB = outerLimitsB->at<int>(h, w);
				//内部臂长
				int innerLimitA;
				int innerLimitB;
				//票数
				int vote = 0;
				//十字交叉臂内左->右or上->下
				for (int outer = outerLimitA; outer <= outerLimitB; ++outer)
				{
					//内部左->右
					if (horizontalFirst)
					{
						innerLimitA = -innerLimitsA->at<int>(h + outer, w);
						innerLimitB = innerLimitsB->at<int>(h + outer, w);
					}
					//内部上->下
					else
					{
						innerLimitA = -innerLimitsA->at<int>(h, w + outer);
						innerLimitB = innerLimitsB->at<int>(h, w + outer);
					}
					//内部搜索
					for (int inner = innerLimitA; inner <= innerLimitB; ++inner)
					{
						int height, width;
						if (horizontalFirst)
						{
							height = h + outer;
							width = w + inner;
						}
						else
						{
							height = h + inner;
							width = w + outer;
						}
						//有效像素票数
						if (disparity.at<int>(height, width) >= this->m_minDisparity)
						{
							//票数
							vote++;
							//更新直方图
							hist[disparity.at<int>(height, width) - this->m_minDisparity] += 1;
						}
					}
				}
				//可靠像素数量不够多,不更新像素视差
				if (vote <= this->m_paMatching.votingThresh)
				{
					dispTemp.at<int>(h, w) = disparity.at<int>(h, w);
				}
				//可靠像素数量足够多
				else
				{
					int disp = disparity.at<int>(h, w);
					float voteRatio;
					float voteRatioMax = 0;
					for (int d = this->m_minDisparity; d <= this->m_maxDisparity; ++d)
					{
						voteRatio = hist[d - this->m_minDisparity] / (float)vote;
						if (voteRatio > voteRatioMax)
						{
							voteRatioMax = voteRatio;
							disp = (voteRatioMax > this->m_paMatching.votingRatioThresh) ? d : disp;
						}
						hist[d - this->m_minDisparity] = 0;
					}
					dispTemp.at<int>(h, w) = disp;
				}
			}
		}
	}
	dispTemp.copyTo(disparity);
	return 0;
}

int stereo::ADCensus::ADCensusImpl::properInterpolation(cv::Mat& disparity, const cv::Mat& leftImage)
{
	cv::Size dispSize = disparity.size();
	cv::Mat dispTemp(dispSize, CV_32S);
	//16方向搜索,相对中心像素坐标
	int directionsW[] = { 0, 2, 2,  2,  0, -2, -2, -2, 1, 2,  2,  1, -1, -2, -2, -1 };
	int directionsH[] = { 2, 2, 0, -2, -2, -2,  0,  2, 2, 1, -1, -2, -2, -1,  1,  2 };
	for (int h = 0; h < dispSize.height; ++h)
	{
		for (int w = 0; w < dispSize.width; ++w)
		{
			//非离群点
			if (disparity.at<int>(h, w) >= this->m_minDisparity)
			{
				dispTemp.at<int>(h, w) = disparity.at<int>(h, w);
			}
			//离群点
			else
			{
				std::vector<int> neighborDisps(16, disparity.at<int>(h, w));
				std::vector<int> neighborDiffs(16, -1);
				for (int direction = 0; direction < 16; ++direction)
				{
					int hD = h, wD = w;
					bool inside = true, gotDisp = false;
					for (int sD = 0; sD < this->m_paMatching.maxSearchDepth and inside and !gotDisp; ++sD)
					{
						//16方向步进
						if (sD % 2 == 0)
						{
							hD += directionsH[direction] / 2;
							wD += directionsW[direction] / 2;
						}
						else
						{
							hD += directionsH[direction] - directionsH[direction] / 2;
							wD += directionsW[direction] - directionsW[direction] / 2;
						}
						inside = hD >= 0 and hD < dispSize.height and wD >= 0 and wD < dispSize.width;
						if (inside and disparity.at<int>(hD, wD) >= this->m_minDisparity)
						{
							neighborDisps[direction] = disparity.at<int>(hD, wD);
							neighborDiffs[direction] = colorDiff(leftImage.at<cv::Vec3b>(h, w), leftImage.at<cv::Vec3b>(hD, wD));
							gotDisp = true;
						}
					}
				}
				//对于遮挡点,以16方向的最小视差来代替遮挡点视差
				if (disparity.at<int>(h, w) == this->m_minDisparity - this->DISP_OCCLUSION)
				{
					int minDisp = neighborDisps[0];
					for (int direction = 1; direction < 16; ++direction)
					{
						if (minDisp > neighborDisps[direction])
							minDisp = neighborDisps[direction];
					}
					dispTemp.at<int>(h, w) = minDisp;
				}
				//对于误匹配点,以16方向颜色差异最小视差来代替误匹配点视差
				else
				{
					int minDisp = neighborDisps[0];
					int minDiff = neighborDiffs[0];
					for (int dir = 1; dir < 16; ++dir)
					{
						if (minDiff < 0 or (minDiff > neighborDiffs[dir] and neighborDiffs[dir] > 0))
						{
							minDisp = neighborDisps[dir];
							minDiff = neighborDiffs[dir];
						}
					}
					dispTemp.at<int>(h, w) = minDisp;
				}
			}
		}
	}
	dispTemp.copyTo(disparity);
	return 0;
}

cv::Mat stereo::ADCensus::ADCensusImpl::convertDisp2Gray(const cv::Mat& disparity)
{
	cv::Size dispSize = disparity.size();
	cv::Mat dispU(dispSize, CV_8U);
	for (int h = 0; h < dispSize.height; ++h)
	{
		for (int w = 0; w < dispSize.width; ++w)
		{
			dispU.at<uchar>(h, w) = (disparity.at<int>(h, w) < 0) ? 0 : (uchar)disparity.at<int>(h, w);
		}
	}
	cv::equalizeHist(dispU, dispU);
	return dispU;
}

int stereo::ADCensus::ADCensusImpl::discontinuityAdjustment(cv::Mat& disparity, const std::vector<std::vector<cv::Mat>>& costs)
{
	cv::Size dispSize = disparity.size();
	cv::Mat dispTemp, detectedEdges, dispGray;
	disparity.copyTo(dispTemp);
	//视差图边缘检测
	dispGray = convertDisp2Gray(disparity);
	blur(dispGray, detectedEdges, cv::Size(this->m_paMatching.blurKernelSize, this->m_paMatching.blurKernelSize));
	Canny(detectedEdges, detectedEdges, this->m_paMatching.cannyThresh1, this->m_paMatching.cannyThresh2, this->m_paMatching.cannyKernelSize);
	//3x3邻域 左上,右下,上,下,右上,左下,左,右
	int directionsH[] = { -1, 1, -1, 1, -1,  1,  0, 0 };
	int directionsW[] = { -1, 1,  0, 0,  1, -1, -1, 1 };
	for (int h = 1; h < dispSize.height - 1; h++)
	{
		for (int w = 1; w < dispSize.width - 1; w++)
		{
			//边缘像素点
			if (detectedEdges.at<uchar>(h, w) != 0)
			{
				int direction = -1;
				//左上 and 右下
				if (detectedEdges.at<uchar>(h - 1, w - 1) != 0 and detectedEdges.at<uchar>(h + 1, w + 1) != 0)
				{
					direction = 0;
				}
				//右上 and 左下
				else if (detectedEdges.at<uchar>(h - 1, w + 1) != 0 and detectedEdges.at<uchar>(h + 1, w - 1) != 0)
				{
					direction = 4;
				}
				//上 or 下
				else if (detectedEdges.at<uchar>(h - 1, w) != 0 or detectedEdges.at<uchar>(h + 1, w) != 0)
				{
					//左上 or 上 or 右上
					if (detectedEdges.at<uchar>(h - 1, w - 1) != 0 or detectedEdges.at<uchar>(h - 1, w) != 0 or detectedEdges.at<uchar>(h - 1, w + 1) != 0)
						//左下 or 下 or 右下
						if (detectedEdges.at<uchar>(h + 1, w - 1) != 0 or detectedEdges.at<uchar>(h + 1, w) != 0 or detectedEdges.at<uchar>(h + 1, w + 1) != 0)
							direction = 2;
				}
				else
				{
					if (detectedEdges.at<uchar>(h - 1, w - 1) != 0 or detectedEdges.at<uchar>(h, w - 1) != 0 or detectedEdges.at<uchar>(h + 1, w - 1) != 0)
						if (detectedEdges.at<uchar>(h - 1, w + 1) != 0 or detectedEdges.at<uchar>(h, w + 1) != 0 or detectedEdges.at<uchar>(h + 1, w + 1) != 0)
							direction = 6;
				}
				if (direction != -1)
				{
					dispTemp.at<int>(h, w) = this->m_minDisparity - this->DISP_MISMATCH;
					int disp = disparity.at<int>(h, w);
					//收集边缘像素p两侧像素p1,p2
					direction = (direction + 4) % 8;
					if (disp >= this->m_minDisparity)
					{
						//p像素点代价
						float cost = costs[0][disp - m_minDisparity].at<float>(h, w);
						//p1像素点视差
						int d1 = disparity.at<int>(h + directionsH[direction], w + directionsW[direction]);
						//p2像素点视差
						int d2 = disparity.at<int>(h + directionsH[direction + 1], w + directionsW[direction + 1]);
						//p1像素点代价
						float cost1 = (d1 >= this->m_minDisparity)
							? costs[0][d1 - this->m_minDisparity].at<float>(h + directionsH[direction], w + directionsW[direction])
							: -1;
						//p2像素点代价
						float cost2 = (d2 >= this->m_minDisparity)
							? costs[0][d2 - this->m_minDisparity].at<float>(h + directionsH[direction + 1], w + directionsW[direction + 1])
							: -1;
						//p1像素点代价小于p像素点代价
						if (cost1 != -1 and cost1 < cost)
						{
							disp = d1;
							cost = cost1;
						}
						//p2像素点代价小于p像素点代价
						if (cost2 != -1 and cost2 < cost)
						{
							disp = d2;
						}
					}
					dispTemp.at<int>(h, w) = disp;
				}
			}
		}
	}
	dispTemp.copyTo(disparity);
	return 0;
}

cv::Mat stereo::ADCensus::ADCensusImpl::subpixelEnhancement(const cv::Mat& disparity, const std::vector<std::vector<cv::Mat>>& costs)
{
	cv::Size dispSize = disparity.size();
	cv::Mat dispTemp(dispSize, CV_32F);
	for (int h = 0; h < dispSize.height; ++h)
	{
		for (int w = 0; w < dispSize.width; ++w)
		{
			int disp = disparity.at<int>(h, w);
			float interDisp = (float)disp;
			//对非区间端点插值
			if (disp > this->m_minDisparity and disp < this->m_maxDisparity)
			{
				//当前视差代价
				float cost = costs[0][disp - this->m_minDisparity].at<float>(h, w);
				//当前视差+1代价
				float costPlus = costs[0][disp + 1 - this->m_minDisparity].at<float>(h, w);
				//当前视差-1代价
				float costMinus = costs[0][disp - 1 - this->m_minDisparity].at<float>(h, w);
				//一元二次曲线拟合,d_sub=d+(c1-c2)/[2*(c1+c2-2*c0)]
				float diff = (costPlus - costMinus) / (2 * (costPlus + costMinus - 2 * cost));
				if (diff > -1 and diff < 1)
					interDisp -= diff;
			}
			dispTemp.at<float>(h, w) = interDisp;
		}
	}
	//抑制噪声,中值滤波
	cv::medianBlur(dispTemp, dispTemp, 3);
	return dispTemp;
}

int stereo::ADCensus::ADCensusImpl::multiOptimize()
{
	cv::Mat disp0, disp1;
	cost2disparity(0, disp0);//扫描线优化后的左图视差
	cost2disparity(1, disp1);//扫描线优化后的右图视差
	this->m_disparityMap = outlierElimination(disp0, disp1);
	bool horizontalFirst = false;
	for (int i = 0; i < 5; i++)
	{
		regionVoting(this->m_disparityMap, this->m_upLimits, this->m_downLimits, this->m_leftLimits, this->m_rightLimits, horizontalFirst);
		horizontalFirst = !horizontalFirst;
	}
	properInterpolation(this->m_disparityMap, this->m_images[0]);
	discontinuityAdjustment(this->m_disparityMap, this->m_costMaps);
	this->m_floatDisparityMap = subpixelEnhancement(this->m_disparityMap, this->m_costMaps);
	return 0;
}

void stereo::ADCensus::ADCensusImpl::cost2disparity(const int& imageNo, cv::Mat& dst)
{
	cv::Mat disp(this->m_imageSize, CV_32S);
	cv::Mat lowCost(this->m_imageSize, CV_32F, cv::Scalar(std::numeric_limits<float>::max()));
	for (int d = this->m_minDisparity; d <= this->m_maxDisparity - this->m_minDisparity; ++d)
	{
		for (int h = 0; h < this->m_imageSize.height; ++h)
		{
			for (int w = 0; w < this->m_imageSize.width; ++w)
			{
				if (lowCost.at<float>(h, w) > this->m_costMaps[imageNo][d].at<float>(h, w))
				{
					lowCost.at<float>(h, w) = this->m_costMaps[imageNo][d].at<float>(h, w);
					disp.at<int>(h, w) = d;
				}
			}
		}
	}
	dst = disp.clone();
}

int stereo::ADCensus::ADCensusImpl::disparityOffset(const int& offset)
{
	for (int height = 0; height < this->m_imageSize.height; ++height)
	{
		for (int width = 0; width < this->m_imageSize.width; ++width)
		{
			float d = this->m_floatDisparityMap.ptr<float>(height)[width];
			if (d > 0)
				this->m_floatDisparityMap.ptr<float>(height)[width] = d + offset;
		}
	}
	return 0;
}

int stereo::ADCensus::ADCensusImpl::bgr2hsi(const cv::Mat src, cv::Mat& dst, const bool& filter)
{
	cv::Mat srcc = src.clone();
	cv::Mat dstc(src.size(), CV_8UC3);
	float hValue, sValue, iValue = 0;
	for (int i = 0; i < srcc.rows; i++)
	{
		for (int j = 0; j < srcc.cols; j++)
		{
			float blueValue = srcc.at<cv::Vec3b>(i, j)[0] / 255.f;
			float greenValue = srcc.at<cv::Vec3b>(i, j)[1] / 255.f;
			float redValue = srcc.at<cv::Vec3b>(i, j)[2] / 255.f;
			float sum = blueValue + greenValue + redValue;
			float iValue = sum / 3.0f;
			dstc.ptr<cv::Vec3b>(i)[j][2] = iValue * 255;
			if (sum == 0)
				sValue = 0;
			else
				sValue = 1 - 3 * cv::min(cv::min(blueValue, greenValue), redValue) / sum;
			dstc.ptr<cv::Vec3b>(i)[j][1] = sValue * 255;
			float den = sqrtf((redValue - greenValue) * (redValue - greenValue)
				+ (redValue - blueValue) * (greenValue - blueValue));
			float num = (2 * redValue - greenValue - blueValue) / 2.f;
			if (den == 0.f or den <= num or sValue < 0.05f)
				hValue = 0;
			else
			{
				float theta = acosf(num / den);
				hValue = blueValue <= greenValue ?
					theta / (2 * CV_PI) : 1 - theta / (2 * CV_PI);
			}
			dstc.ptr<cv::Vec3b>(i)[j][0] = hValue * 255;
		}
	}
	if (filter)
	{
		for (auto it = dstc.begin<cv::Vec3b>(); it != dstc.end<cv::Vec3b>(); ++it)
		{
			if ((*it)[0] >= 60 or (*it)[0] <= 10)
				*it = cv::Vec3b(0, 0, 0);
		}
	}
	dst = dstc;
	return 0;
}

void stereo::ADCensus::ADCensusImpl::computeGaussMedian(const cv::Mat& src, cv::Mat& dst, const int& kSize, const double& sigma)
{
	cv::Mat gaussKernel1 = cv::getGaussianKernel(kSize, sigma, CV_32F);
	cv::Mat gaussKernel2 = gaussKernel1 * gaussKernel1.t();
	cv::Mat median = cv::Mat::zeros(src.size(), CV_8UC3);
	cv::filter2D(src, median, -1, gaussKernel2, cv::Point(-1, -1), 0.0, cv::BORDER_CONSTANT);
	dst = src.clone();
	auto srcIt = src.begin<cv::Vec3b>();
	auto medianIt = median.begin<cv::Vec3b>();
	auto dstIt = dst.begin<cv::Vec3b>();
	for (; srcIt != src.end<cv::Vec3b>(); ++srcIt, ++medianIt, ++dstIt)
	{
		int hDiff = std::abs((*srcIt)[0] - (*medianIt)[0]);
		hDiff = std::min(hDiff, 255 - hDiff);
		if (hDiff >= 2)
			(*dstIt)[0] = (*medianIt)[0];
		for (int i = 1; i < 3; ++i)
		{
			if (std::abs((*srcIt)[i] - (*medianIt)[i]) < 3)
				continue;
			else
				(*dstIt)[i] = (*medianIt)[i];
		}
	}
}
