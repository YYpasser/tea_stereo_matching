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
	 * @brief �������ص�p��pd��RGB��ɫģ���е�AD����.
	 * @param [in] height1 ���ص�p����
	 * @param [in] width1  ���ص�p����
	 * @param [in] height2 ���ص�pd����
	 * @param [in] width2  ���ص�pd����
	 * @return RGB��ɫģ�͵�AD����, float
	 */
	float computeRGBADCost(const int& height1, const int& width1, const int& height2, const int& width2);
	/**
	 * @brief �������ص�p��pd��HSI��ɫģ���е�AD����.
	 * @param [in] height1 ���ص�p����
	 * @param [in] width1  ���ص�p����
	 * @param [in] height2 ���ص�pd����
	 * @param [in] width2  ���ص�pd����
	 * @return HSI��ɫģ�͵�AD����, float
	 */
	float computeHSIADCost(const int& height1, const int& width1, const int& height2, const int& width2);
	/**
	 * @brief �������ص�p��pd��RGB��ɫģ���е�Census�任����[Hamming distance].
	 * @param [in] height1 ���ص�p����
	 * @param [in] width1  ���ص�p����
	 * @param [in] height2 ���ص�pd����
	 * @param [in] width2  ���ص�pd����
	 * @param [in] censusWinHeight Census�任���ߴ�
	 * @param [in] censusWinWidth  Census�任���ߴ�
	 * @return RGB��ɫģ�͵�Census�任����, float
	 */
	float computeRGBCensusCost(const int& height1, const int& width1, const int& height2, const int& width2, const int& censusWinHeight, const int& censusWinWidth);
	/**
	 * @brief �������ص�p��pd��HSI��ɫģ���е�Census�任����[Hamming distance].
	 * @param [in] height1 ���ص�p����
	 * @param [in] width1  ���ص�p����
	 * @param [in] height2 ���ص�pd����
	 * @param [in] width2  ���ص�pd����
	 * @param [in] censusWinHeight Census�任���ߴ�
	 * @param [in] censusWinWidth  Census�任���ߴ�
	 * @return HSI��ɫģ�͵�Census�任����, float
	 */
	float computeHSICensusCost(const int& height1, const int& width1, const int& height2, const int& width2, const int& censusWinHeight, const int& censusWinWidth);
	/**
	 * @brief �������ص�p��pd��AD-Census����.
	 * @param [in] height1 ���ص�p����
	 * @param [in] width1  ���ص�p����
	 * @param [in] height2 ���ص�pd����
	 * @param [in] width2  ���ص�pd����
	 * @param [in] censusWinHeight Census�任���ߴ�
	 * @param [in] censusWinWidth  Census�任���ߴ�
	 * @return AD-Census����
	 */
	float computeADCensusCost(const int& height1, const int& width1, const int& height2, const int& width2, const int& censusWinHeight, const int& censusWinWidth);
	/**
	 * @brief [AD-Census�㷨] Step.1 ��ʼ���ۼ���, ������ʼ������C[H,W,D].
	 * @param [in] writeProcess ������
	 * @param [in] writeDir     ����·��
	 */
	void costInitialize(const bool& writeProcess = false, const std::string& writeDir = std::string());
	/**
	 * @brief �������ص�p��pd����ɫ(RGB or Hue)����.
	 * @param [in] p1 ���ص�p
	 * @param [in] p2 ���ص�pd
	 * @return ��ɫ����ֵ, int
	 */
	int colorDiff(const cv::Vec3b& p1, const cv::Vec3b& p2);
	/**
	 * @brief �������ص�p��ʮ�ֽ���۱۳�.
	 * @param [in] height     ���ص�p����
	 * @param [in] width      ���ص�p����
	 * @param [in] directionH y�������췽��
	 * @param [in] directionW x�������췽��
	 * @param [in] imageNo    ������ͼ��־λ
	 * @return ���ص�p��ʮ�ֽ���۱۳�, px
	 */
	int computeLimit(const int& height, const int& width, const int& directionH, const int& directionW, const int& imageNo);
	/**
	 * @brief ����ʮ�ֽ���۱۳�����.
	 * @param [in] directionH y�������췽��
	 * @param [in] directionW x�������췽��
	 * @param [in] imageNo    ������ͼ��־λ
	 * @return ʮ�ֽ���۱۳�����
	 */
	cv::Mat computeLimits(const int& directionH, const int& directionW, const int& imageNo);
	/**
	 * @brief x or y������۾ۺ�.
	 * @param [in]  costMap     ���۾���
	 * @param [in]  directionH  y�������췽��
	 * @param [in]  directionW  x�������췽��
	 * @param [out] windowSizes ʮ�ֽ���۴��ڳߴ�
	 * @param [in]  imageNo     ������ͼ��־λ
	 * @return �ۺϴ��۾���
	 */
	cv::Mat aggregation1D(const cv::Mat& costMap, const int& directionH, const int& directionW,
		cv::Mat& windowSizes, const int& imageNo);
	/**
	 * @brief x, y������۾ۺ�.
	 * @param [in/out] costMap         �����ʼ���۾���, ����ۺϴ��۾���
	 * @param [in]     horizontalFirst x or y��������
	 * @param [in]     imageNo         ������ͼ��־λ
	 * @return =0: �ɹ�; < 0: �������
	 */
	int aggregation2D(cv::Mat& costMap, bool horizontalFirst, const int& imageNo);
	/**
	 * @brief [AD-Census�㷨] Step.2 ���۾ۺ�, ���ݳ�ʼ���������ۺϴ�����C[H,W,D].
	 * @param [in] writeProcess ������
	 * @param [in] writeDir     ����·��
	 */
	void costAggregate(const bool& writeProcess = false, const std::string& writeDir = std::string());
	/**
	 * @brief ��ֱ����ɨ�����Ż�, �����Ż�.
	 * @param [in] height ���ص�������
	 * @param [in]     direction  �Ż�����
	 * @param [in/out] costMaps   ������
	 * @param [in]     rightFirst ������ͼ��־λ
	 * @return =0: �ɹ�; < 0: �������
	 */
	int verticalComputation(const int& height, const int& direction, std::vector<cv::Mat>* costMaps, bool rightFirst);
	/**
	 * @brief ��ֱ����ɨ�����Ż�, ѡ���������ص����.
	 * @param [in]     height1    ���ص�1������
	 * @param [in]     height2    ���ص�2������
	 * @param [in/out] costMaps   ������
	 * @param [in]     rightFirst ������ͼ��־λ
	 * @return =0: �ɹ�; < 0: �������
	 */
	int verticalOptimization(const int& height1, const int& height2, std::vector<cv::Mat>* costMaps, bool rightFirst);
	/**
	 * @brief ˮƽ����ɨ�����Ż�, �����Ż�.
	 * @param [in]     width      ���ص�������
	 * @param [in]     direction  �Ż�����
	 * @param [in/out] costMaps   ������
	 * @param [in]     rightFirst ������ͼ��־λ
	 * @return =0: �ɹ�; < 0: �������
	 */
	int horizontalComputation(const int& width, const int& direction, std::vector<cv::Mat>* costMaps, bool rightFirst);
	/**
	 * @brief ˮƽ����ɨ�����Ż�, ѡ���������ص����.
	 * @param [in]     width1     ���ص�1������
	 * @param [in]     width2     ���ص�2������
	 * @param [in/out] costMaps   ������
	 * @param [in]     rightFirst ������ͼ��־λ
	 * @return =0: �ɹ�; < 0: �������
	 */
	int horizontalOptimization(const int& width1, const int& width2, std::vector<cv::Mat>* costMaps, bool rightFirst);
	/**
	 * @brief �����ظ���ɨ�����Ż�������.
	 * @param [in]     height1    ����1����
	 * @param [in]     height2    ����2����
	 * @param [in]     width1     ����1����
	 * @param [in]     width2     ����2����
	 * @param [in/out] costMaps   ������
	 * @param [in]     rightFirst ������ͼ��־λ
	 * @return =0: �ɹ�; < 0: �������
	 */
	int partialOptimization(const int& height1, const int& height2, const int& width1, const int& width2, std::vector<cv::Mat>* costMaps, bool rightFirst);
	/**
	 * @brief ����ͷ�����P1, P2.
	 * @param [in]  height1    ���ص�p����
	 * @param [in]  height2    ���ص�pd����
	 * @param [in]  width1     ���ص�p����
	 * @param [in]  width2     ���ص�pd����
	 * @param [in]  disparity  �Ӳ�
	 * @param [out] p1         �ͷ�����P1
	 * @param [out] p2         �ͷ�����P2
	 * @param [in]  rightFirst ������ͼ��־λ
	 * @return =0: �ɹ�; < 0: �������
	 */
	int computeP1P2(const int& height1, const int& height2, const int& width1, const int& width2, int disparity, float& p1, float& p2, bool rightFirst);
	/**
	 * @brief ɨ�����Ż�.
	 * @param [in/out] costMaps   ������
	 * @param [in]     rightFirst ������ͼ��־λ
	 * @return =0: �ɹ�; < 0: �������
	 */
	int scanline(std::vector<cv::Mat>* costMaps, bool rightFirst);
	/**
	 * @brief [AD-Census�㷨] Step.3 ɨ�����Ż�.
	 * @param [in] writeProcess ������
	 * @param [in] writeDir     ����·��
	 */
	void scanlineOptimize(const bool& writeProcess = false, const std::string& writeDir = std::string());
	/**
	 * @brief ��Ⱥ����.
	 * @param [in] leftDisp  ���ӽ��Ӳ�ͼ
	 * @param [in] rightDisp ���ӽ��Ӳ�ͼ
	 * @return ��Ⱥ���������ӽ��Ӳ�ͼ
	 */
	cv::Mat outlierElimination(const cv::Mat& leftDisp, const cv::Mat& rightDisp);
	/**
	 * @brief ����ͶƱ.
	 * @param [in/out] disparity       �Ӳ�ͼ
	 * @param [in/out] upLimits        ��������������
	 * @param [in/out] downLimits      ��������������
	 * @param [in/out] leftLimits      ��������������
	 * @param [in/out] rightLimits     ��������������
	 * @param [in]     horizontalFirst ������ͼ��־λ
	 * @return =0: �ɹ�; < 0: �������
	 */
	int regionVoting(cv::Mat& disparity, std::vector<cv::Mat>& upLimits, std::vector<cv::Mat>& downLimits,
		std::vector<cv::Mat>& leftLimits, std::vector<cv::Mat>& rightLimits, bool horizontalFirst);
	/**
	 * @brief �����ֵ.
	 * @param [in/out] disparity �Ӳ�ͼ
	 * @param [in]    leftImage ��ͼ
	 * @return =0: �ɹ�; < 0: �������
	 */
	int properInterpolation(cv::Mat& disparity, const cv::Mat& leftImage);
	/**
	 * @brief �����Ӳ�Ҷ�ͼ.
	 * @param [in] disparity �Ӳ�ͼ
	 * @return �Ҷ��Ӳ�ͼ
	 */
	cv::Mat convertDisp2Gray(const cv::Mat& disparity);
	/**
	 * @brief ����������.
	 * @param [in/out] disparity �Ӳ�ͼ
	 * @param [in]     costs     ������
	 * @return =0: �ɹ�; < 0: �������
	 */
	int discontinuityAdjustment(cv::Mat& disparity, const std::vector<std::vector<cv::Mat>>& costs);
	/**
	 * @brief ��������ǿ.
	 * @param [in] disparity �Ӳ�ͼ
	 * @param [in] costs     ���۾���
	 * @return �������Ӳ�
	 */
	cv::Mat subpixelEnhancement(const cv::Mat& disparity, const std::vector<std::vector<cv::Mat>>& costs);
	/**
	 * @brief [AD-Census�㷨] Step.4 �ಽ���Ӳ��Ż�.
	 * @return =0: �ɹ�; < 0: �������
	 */
	int multiOptimize();
	/**
	 * @brief "Ӯ��ͨ��"����.
	 * @param [in]  imageNo ������ͼ��־λ
	 * @param [out] dst     �Ӳ�ͼ
	 * @return =0: �ɹ�; < 0: �������
	 */
	void cost2disparity(const int& imageNo, cv::Mat& dst);
	/**
	 * @brief ROI����ƥ���Ӳ��.
	 * @param [in] offset ��������, px
	 * @return =0: �ɹ�; < 0: �������
	 */
	int disparityOffset(const int& offset);
	/**
	 * @brief RGB��ɫģ��תΪHSI��ɫģ��.
	 * @param [in]  src    RGBͼ��
	 * @param [out] dst    HSIͼ��
	 * @param [in]  filter ��ɫ�˲�
	 * @return =0: �ɹ�; < 0: �������
	 */
	int bgr2hsi(const cv::Mat src, cv::Mat& dst, const bool& filter = false);
	/**
	 * @brief �����˹Ȩ������, �˲�.
	 * @param [in]  src   ԭʼͼ��
	 * @param [out] dst   ��˹Ȩ������
	 * @param [in]  kSize �˳ߴ�
	 * @param [in]  sigma ��˹�ֲ���
	 */
	void computeGaussMedian(const cv::Mat& src, cv::Mat& dst, const int& kSize = 3, const double& sigma = -1);
public:
	int m_minDisparity;  /*!< ��С������Χ */
	int m_maxDisparity;  /*!< ���������Χ */
	ADCensusParams m_paMatching; /*!< ����ƥ������ṹ�� */
	ColorModel m_colorModel;     /*!< ����ƥ����ɫģ�� */
	bool m_roiMatching;  /*!< ����ƥ��ROIģʽ */
	bool m_maskMatching; /*!< ����ƥ��MASKģʽ */
	int m_offset;        /*!< ROI/MASKģʽ�Ӳ�� */
	cv::Mat m_images[2]; /*!< ����У���������ͼ */
	cv::Size m_imageSize;/*!< ������ͼ�ߴ� */
	std::vector<cv::Mat> m_upLimits;   /*!< ������ ����ͼ�����ص㴹ֱ���ϼ���λ��[k](i,j) kֵ: 0=��1=�� */
	std::vector<cv::Mat> m_downLimits; /*!< ������ ����ͼ�����ص㴹ֱ���¼���λ��[k](i,j) kֵ: 0=��1=�� */
	std::vector<cv::Mat> m_leftLimits; /*!< ������ ����ͼ�����ص�ˮƽ������λ��[k](i,j) kֵ: 0=��1=�� */
	std::vector<cv::Mat> m_rightLimits;/*!< ������ ����ͼ�����ص�ˮƽ���Ҽ���λ��[k](i,j) kֵ: 0=��1=�� */
	std::vector<std::vector<cv::Mat>> m_costMaps; /*!< ������[k][d](i,j) kֵ: 0=��1=��; d-�Ӳ�; (i,j)-ͼ������ *///Cost volume
	cv::Mat m_disparityMap;      /*!< �Ӳ�, �����Ӳ�*/
	cv::Mat m_floatDisparityMap; /*!< �Ӳ�, ��������ʵ�Ӳ� */
	int m_occlusionValue;        /*!< �ڵ������(��־λ) */
	int m_mismatchValue;         /*!< ��ƥ�����(��־λ) */
	static const int DISP_OCCLUSION = 1; /*!< �ڵ����Ӳ�ֵ(minDisp - occlusion) */
	static const int DISP_MISMATCH = 2; /*!< ��ƥ���Ӳ�ֵ(minDisp - mismatch) */
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
	//��ʼ��������Cost Volume
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
	//-- ������, ����ͼ-�Ӳ�ͼ
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
	//p��pd��R,G,B��ͨ����AD����
	for (int i = 0; i < 3; ++i)
		adCost += std::abs(rgbLP[i] - rgbRP[i]);
	//p��pd��R,G,B��ͨ����ƽ��AD����
	adCost = adCost / 3.f;
	return adCost;
}

float stereo::ADCensus::ADCensusImpl::computeHSIADCost(const int& height1, const int& width1, const int& height2, const int& width2)
{
	float adCost = 0.f;
	const cv::Vec3b& HSILP = this->m_images[0].at<cv::Vec3b>(height1, width1);
	const cv::Vec3b& HSIRP = this->m_images[1].at<cv::Vec3b>(height2, width2);
	//Hueͨ������
	int hueDiff = std::abs(HSILP[0] - HSIRP[0]);
	adCost += cv::min(hueDiff, 255 - hueDiff) * this->m_paMatching.lambdaHue;
	//Saturationͨ������
	adCost += std::abs(HSILP[1] - HSIRP[1]) * this->m_paMatching.lambdaSaturation;
	//Intensityͨ������
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
					//-- ��Ĥͼ�����Ч���ݼ�ΪԽ�����
					if (this->m_maskMatching and this->m_images[imageNo].at<cv::Vec3b>(i, j) == cv::Vec3b(0, 0, 0))
					{
						this->m_costMaps[imageNo][d].at<float>(i, j) = 2.f;
						continue;
					}
					colL = j - this->m_minDisparity;
					colR = j + this->m_minDisparity;
					if (imageNo == 0)//������ͼ��������ͼ�ж�Ӧͬ����(��������)
						colR = j - d;
					else             //������ͼ��������ͼ�ж�Ӧͬ����(��������)
						colL = j + d;
					out = colL - halfCensusWin.width < 0 or colL + halfCensusWin.width >= this->m_imageSize.width
						or colR - halfCensusWin.width < 0 or colR + halfCensusWin.width >= this->m_imageSize.width
						or i - halfCensusWin.height < 0 or i + halfCensusWin.height >= this->m_imageSize.height;
					if (out)
						this->m_costMaps[imageNo][d].at<float>(i, j) = 2.f;//Խ�����
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
	//��ǰ���ص�p[row,col]����ɫ[B,G,R]
	cv::Vec3b p = this->m_images[imageNo].at<cv::Vec3b>(height, width);
	//�����쳤�ȳ�ֵ=1����
	int d = 1;
	//�߽��ѡ���ص�p1����[row1,col1]
	int height1 = height + directionH;
	int width1 = width + directionW;
	//p1���ǰһ��p2����ɫ[B,G,R]
	cv::Vec3b p2 = p;
	//�ж�p1���Ƿ���ͼ���ڲ�����
	bool inside = (0 <= height1) and (height1 < this->m_imageSize.height) and (0 <= width1) and (width1 < this->m_imageSize.width);
	if (inside)
	{
		bool colorCond = true, wLimitCond = true, fColorCond = true;
		//��ɫ���ơ����쳤�ȡ����ƶȾ���������������ƽ�p2��
		while (colorCond and wLimitCond and fColorCond and inside)
		{
			cv::Vec3b p1 = this->m_images[imageNo].at<cv::Vec3b>(height1, width1);
			//Maskƥ��ģʽ��������������ֹͣ����
			if (this->m_maskMatching and p1 == cv::Vec3b(0, 0, 0))
			{
				d++;
				break;
			}
			//�ж�p1,p2,p���Ƿ������Ƶ���ɫǿ�ȣ�
			colorCond = colorDiff(p, p1) < this->m_paMatching.colorThresh1 and colorDiff(p1, p2) < this->m_paMatching.colorThresh1;
			if (this->m_colorModel == ColorModel::HSI)
			{
				colorCond = std::abs(p[1] - p1[1]) < this->m_paMatching.saturationThresh1 and std::abs(p1[1] - p2[1]) < this->m_paMatching.saturationThresh1;
				colorCond = std::abs(p[2] - p1[2]) < this->m_paMatching.intensityThresh1 and std::abs(p1[2] - p2[2]) < this->m_paMatching.intensityThresh1;
			}
			//�жϱ����쳤���Ƿ�ﵽ�����ֵ1?
			wLimitCond = d < this->m_paMatching.maxLength1;
			//�жϽ�Զ���Ƿ��и��õ���ɫ���ƶȣ�
			fColorCond = (d <= this->m_paMatching.maxLength2) or (d > this->m_paMatching.maxLength2 and colorDiff(p, p1) < this->m_paMatching.colorThresh2);
			if (this->m_colorModel == ColorModel::HSI)
			{
				fColorCond = (d <= this->m_paMatching.maxLength2) or (d > this->m_paMatching.maxLength2 and std::abs(p[1] - p1[1]) < this->m_paMatching.saturationThresh2);
				fColorCond = (d <= this->m_paMatching.maxLength2) or (d > this->m_paMatching.maxLength2 and std::abs(p[2] - p1[2]) < this->m_paMatching.intensityThresh2);
			}
			//p1������Ѱ�����ƶ�1���أ�p2��ͬʱ�ƶ�1����
			p2 = p1;
			height1 += directionH;
			width1 += directionW;
			//�ж�p1���Ƿ���ͼ���ڲ�����
			inside = (0 <= height1) and (height1 < this->m_imageSize.height) and (0 <= width1) and (width1 < this->m_imageSize.width);
			//�����쳤������1����
			d++;
		}
		//���������߽��Ϊ�˳�ѭ�����1���أ������1����
		d--;
	}
	return d - 1;//��������ʵ����
}

cv::Mat stereo::ADCensus::ADCensusImpl::computeLimits(const int& directionH, const int& directionW, const int& imageNo)
{
	//�����쳤�Ⱦ���
	cv::Mat limits = cv::Mat::zeros(this->m_imageSize, CV_32S);
	int height, width;
#pragma omp parallel default (shared) private(height, width) num_threads(omp_get_max_threads())
#pragma omp for schedule(static)
	for (height = 0; height < this->m_imageSize.height; ++height)
	{
		for (width = 0; width < this->m_imageSize.width; ++width)
		{
			//Maskƥ�䱳�����ص�۳�Ϊ0
			if (this->m_maskMatching and this->m_images[imageNo].at<cv::Vec3b>(height, width) == cv::Vec3b(0, 0, 0))
			{
				limits.at<int>(height, width) = 0;
				continue;
			}
			//�����ؼ�������쳤��
			limits.at<int>(height, width) = computeLimit(height, width, directionH, directionW, imageNo);
		}
	}
	return limits;
}

cv::Mat stereo::ADCensus::ADCensusImpl::aggregation1D(const cv::Mat& costMap, const int& directionH, const int& directionW, cv::Mat& windowSizes, const int& imageNo)
{
	//���۾ۺϴ��ڴ�С
	cv::Mat tmpWindowSizes = cv::Mat::zeros(this->m_imageSize, CV_32S);
	//�ۺϴ���
	cv::Mat aggregatedCosts(this->m_imageSize, CV_32F);//32λ������,float
	int height = 0, width = 0;
	//����������[dmin,dmax], ������ǰλ��d
	int dmin, dmax, d;
#pragma omp parallel default (shared) private(height, width, d) num_threads(omp_get_max_threads())
#pragma omp for schedule(static)
	for (height = 0; height < this->m_imageSize.height; ++height)
	{
		for (width = 0; width < this->m_imageSize.width; ++width)
		{
			if (directionH == 0)//ˮƽ��
			{
				dmin = -this->m_leftLimits[imageNo].at<int>(height, width);
				dmax = this->m_rightLimits[imageNo].at<int>(height, width);
			}
			else//��ֱ��
			{
				dmin = -this->m_upLimits[imageNo].at<int>(height, width);
				dmax = this->m_downLimits[imageNo].at<int>(height, width);
			}
			//�������
			float cost = 0;
			//���淽������۾ۺ�
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
	//����������HΪ��ֱ����WΪˮƽ����
	int directionH = 1, directionW = 0;
	//��ˮƽ��or�ȴ�ֱ��
	if (horizontalFirst)
		std::swap(directionH, directionW);
	//ʮ�ֽ���۴��ڳߴ�
	cv::Mat windowsSizes = cv::Mat::ones(this->m_imageSize, CV_32S);
	for (int direction = 0; direction < 2; ++direction)
	{
		horizontalFirst = !horizontalFirst;
		//һά���۾ۺϣ�����ԭƥ�����
		(aggregation1D(costMap, directionH, directionW, windowsSizes, imageNo)).copyTo(costMap);
		//������������
		std::swap(directionH, directionW);
	}
	//���վۺϴ���=�ۺϴ���/���ڴ�С
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
	//-- ����ʮ�ֽ���۱۳�
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
	//����ͼ�ֱ���۾ۺ�
	for (int imageNo = 0; imageNo < 2; ++imageNo)
	{
		int d, i;
#pragma omp parallel default (shared) private(d, i) num_threads(omp_get_max_threads())
#pragma omp for schedule(static)
		//��ͬ�Ӳ��µĴ��۾ۺ�
		for (d = 0; d <= this->m_maxDisparity - this->m_minDisparity; ++d)
		{
			bool horizontalFirst = true;
			//n�δ��۾ۺϵ���
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
	//���㴹ֱ�����Ż�����
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
	//����ˮƽ�����Ż�����
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
	//����min_k{ Cr(p-r, k) },��[width2,height2]��ǰһ���ص��ڲ�ͬ�Ӳ�����С��·������
	for (int disparity = 1; disparity <= this->m_maxDisparity - this->m_minDisparity; ++disparity)
	{
		float tmpCost = costMaps->at(disparity).at<float>(height2, width2);
		if (minOptCost > tmpCost)
			minOptCost = tmpCost;
	}
	//���ǰһ���ص��ڲ�ͬ�Ӳ��µ���С·������Ϊ0,��ִ��ɨ�����Ż�
	if (minOptCost == 0)
		return 0;
	float minkCr = minOptCost;//min_k{ Cr(p-r, k) }
	for (int disparity = 0; disparity <= this->m_maxDisparity - this->m_minDisparity; ++disparity)
	{
		//����C1(p,d) - min_k(Cr(p-r,k))
		float cost = costMaps->at(disparity).at<float>(height1, width1) - minkCr;
		//����ͷ�����P1,P2
		float p1 = 0.f, p2 = 0.f;
		computeP1P2(height1, height2, width1, width2, disparity + this->m_minDisparity, p1, p2, rightFirst);
		//����min{ Cr(p-r,d), Cr(p-r, d+-1) + P1, min_k(Cr(p-r,k)+P2) }
		minOptCost = minkCr + p2;
		float tmpCost = costMaps->at(disparity).at<float>(height2, width2);
		if (minOptCost > tmpCost)
			minOptCost = tmpCost;
		//���Ӳ�d��������1��������, Cr(p-r, d-1) + P1
		if (disparity != 0)
		{
			tmpCost = costMaps->at(disparity - 1).at<float>(height2, width2) + p1;
			if (minOptCost > tmpCost)
				minOptCost = tmpCost;
		}
		//���Ӳ�d��������1��������, Cr(p-r, d+1) + P1
		if (disparity != this->m_maxDisparity - this->m_minDisparity)
		{
			tmpCost = costMaps->at(disparity + 1).at<float>(height2, width2) + p1;
			if (minOptCost > tmpCost)
				minOptCost = tmpCost;
		}
		//���� Cr(p, d) / 2, 1�������·������
		costMaps->at(disparity).at<float>(height1, width1) = (float)(((cost + minOptCost)) / 2);
	}
	return 0;
}

int stereo::ADCensus::ADCensusImpl::computeP1P2(const int& height1, const int& height2, const int& width1, const int& width2, int disparity, float& p1, float& p2, bool rightFirst)
{
	int imageNo = 0;//��ͼ
	int otherImgNo = 1;//��ͼ
	if (rightFirst)
	{
		imageNo = 1;
		otherImgNo = 0;
		disparity = -disparity;
	}
	//��������ͼ���ص�����ͬһ����ǰһ���ص����ɫ��ֵ
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
	//ɫ������ֵ�Ĺ�ϵ���趨�ͷ�����
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
	//����4�������Ż�����
	//1-��ֱ����
	verticalComputation(0, 1, costMaps, rightFirst);
	//2-��ֱ����
	verticalComputation(this->m_imageSize.height - 1, -1, costMaps, rightFirst);
	//3-ˮƽ����
	horizontalComputation(0, 1, costMaps, rightFirst);
	//4-ˮƽ����
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
			//�������ص�Ϊ��Ⱥ��,�ж����ڵ��㻹����ƥ���
			//[1]��ͼ���ص�����ͼ�е�ͬ���㳬��ͼ��Χ
			//[2]��ͼ���ص�����ͼ�е�ͬ�����Ӳ���쳬�����������ֵ
			if (width - disparity < 0 or abs(disparity - rightDisp.at<int>(height, width - disparity)) > this->m_paMatching.dispTolerance)
			{
				bool occlusion = true;//�ڵ�
				for (int d = this->m_minDisparity; d <= this->m_maxDisparity; ++d)
				{
					//�����Ӳ�ֵ��������ͼ���ҵ���Ӧ��,��Ϊ��ƥ���
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
	//ͶƱֱ��ͼ
	std::vector<int> hist(this->m_maxDisparity - this->m_minDisparity + 1, 0);
	//ʮ�ֽ���۱۳�
	const cv::Mat* outerLimitsA;
	const cv::Mat* outerLimitsB;
	const cv::Mat* innerLimitsA;
	const cv::Mat* innerLimitsB;
	//��ȡ���ӽǱ۳�
	if (horizontalFirst)
	{
		outerLimitsA = &upLimits[0];    //��ֱ���ϱ۳�����,����ˮƽ�������,��ֱ����Ϊ��Ⱥ
		outerLimitsB = &downLimits[0];  //��ֱ���±۳�����
		innerLimitsA = &leftLimits[0];  //ˮƽ����۳�����,����ˮƽ�������,ˮƽ����Ϊ�ڲ�
		innerLimitsB = &rightLimits[0]; //ˮƽ���ұ۳�����
	}
	else
	{
		outerLimitsA = &leftLimits[0];  //ˮƽ����۳�����,������ֱ�������,ˮƽ����Ϊ��Ⱥ
		outerLimitsB = &rightLimits[0]; //ˮƽ���ұ۳�����
		innerLimitsA = &upLimits[0];    //��ֱ���ϱ۳�����,������ֱ�������,��ֱ����Ϊ�ڲ�
		innerLimitsB = &downLimits[0];  //��ֱ���±۳�����
	}
	for (int h = 0; h < dispSize.height; ++h)
	{
		for (int w = 0; w < dispSize.width; ++w)
		{
			//����Ⱥ��,�����������Ӳ�
			if (disparity.at<int>(h, w) >= this->m_minDisparity)
			{
				dispTemp.at<int>(h, w) = disparity.at<int>(h, w);
			}
			//��Ⱥ��
			else
			{
				//��Ⱥ�۳�
				int outerLimitA = -outerLimitsA->at<int>(h, w);
				int outerLimitB = outerLimitsB->at<int>(h, w);
				//�ڲ��۳�
				int innerLimitA;
				int innerLimitB;
				//Ʊ��
				int vote = 0;
				//ʮ�ֽ��������->��or��->��
				for (int outer = outerLimitA; outer <= outerLimitB; ++outer)
				{
					//�ڲ���->��
					if (horizontalFirst)
					{
						innerLimitA = -innerLimitsA->at<int>(h + outer, w);
						innerLimitB = innerLimitsB->at<int>(h + outer, w);
					}
					//�ڲ���->��
					else
					{
						innerLimitA = -innerLimitsA->at<int>(h, w + outer);
						innerLimitB = innerLimitsB->at<int>(h, w + outer);
					}
					//�ڲ�����
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
						//��Ч����Ʊ��
						if (disparity.at<int>(height, width) >= this->m_minDisparity)
						{
							//Ʊ��
							vote++;
							//����ֱ��ͼ
							hist[disparity.at<int>(height, width) - this->m_minDisparity] += 1;
						}
					}
				}
				//�ɿ���������������,�����������Ӳ�
				if (vote <= this->m_paMatching.votingThresh)
				{
					dispTemp.at<int>(h, w) = disparity.at<int>(h, w);
				}
				//�ɿ����������㹻��
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
	//16��������,���������������
	int directionsW[] = { 0, 2, 2,  2,  0, -2, -2, -2, 1, 2,  2,  1, -1, -2, -2, -1 };
	int directionsH[] = { 2, 2, 0, -2, -2, -2,  0,  2, 2, 1, -1, -2, -2, -1,  1,  2 };
	for (int h = 0; h < dispSize.height; ++h)
	{
		for (int w = 0; w < dispSize.width; ++w)
		{
			//����Ⱥ��
			if (disparity.at<int>(h, w) >= this->m_minDisparity)
			{
				dispTemp.at<int>(h, w) = disparity.at<int>(h, w);
			}
			//��Ⱥ��
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
						//16���򲽽�
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
				//�����ڵ���,��16�������С�Ӳ��������ڵ����Ӳ�
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
				//������ƥ���,��16������ɫ������С�Ӳ���������ƥ����Ӳ�
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
	//�Ӳ�ͼ��Ե���
	dispGray = convertDisp2Gray(disparity);
	blur(dispGray, detectedEdges, cv::Size(this->m_paMatching.blurKernelSize, this->m_paMatching.blurKernelSize));
	Canny(detectedEdges, detectedEdges, this->m_paMatching.cannyThresh1, this->m_paMatching.cannyThresh2, this->m_paMatching.cannyKernelSize);
	//3x3���� ����,����,��,��,����,����,��,��
	int directionsH[] = { -1, 1, -1, 1, -1,  1,  0, 0 };
	int directionsW[] = { -1, 1,  0, 0,  1, -1, -1, 1 };
	for (int h = 1; h < dispSize.height - 1; h++)
	{
		for (int w = 1; w < dispSize.width - 1; w++)
		{
			//��Ե���ص�
			if (detectedEdges.at<uchar>(h, w) != 0)
			{
				int direction = -1;
				//���� and ����
				if (detectedEdges.at<uchar>(h - 1, w - 1) != 0 and detectedEdges.at<uchar>(h + 1, w + 1) != 0)
				{
					direction = 0;
				}
				//���� and ����
				else if (detectedEdges.at<uchar>(h - 1, w + 1) != 0 and detectedEdges.at<uchar>(h + 1, w - 1) != 0)
				{
					direction = 4;
				}
				//�� or ��
				else if (detectedEdges.at<uchar>(h - 1, w) != 0 or detectedEdges.at<uchar>(h + 1, w) != 0)
				{
					//���� or �� or ����
					if (detectedEdges.at<uchar>(h - 1, w - 1) != 0 or detectedEdges.at<uchar>(h - 1, w) != 0 or detectedEdges.at<uchar>(h - 1, w + 1) != 0)
						//���� or �� or ����
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
					//�ռ���Ե����p��������p1,p2
					direction = (direction + 4) % 8;
					if (disp >= this->m_minDisparity)
					{
						//p���ص����
						float cost = costs[0][disp - m_minDisparity].at<float>(h, w);
						//p1���ص��Ӳ�
						int d1 = disparity.at<int>(h + directionsH[direction], w + directionsW[direction]);
						//p2���ص��Ӳ�
						int d2 = disparity.at<int>(h + directionsH[direction + 1], w + directionsW[direction + 1]);
						//p1���ص����
						float cost1 = (d1 >= this->m_minDisparity)
							? costs[0][d1 - this->m_minDisparity].at<float>(h + directionsH[direction], w + directionsW[direction])
							: -1;
						//p2���ص����
						float cost2 = (d2 >= this->m_minDisparity)
							? costs[0][d2 - this->m_minDisparity].at<float>(h + directionsH[direction + 1], w + directionsW[direction + 1])
							: -1;
						//p1���ص����С��p���ص����
						if (cost1 != -1 and cost1 < cost)
						{
							disp = d1;
							cost = cost1;
						}
						//p2���ص����С��p���ص����
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
			//�Է�����˵��ֵ
			if (disp > this->m_minDisparity and disp < this->m_maxDisparity)
			{
				//��ǰ�Ӳ����
				float cost = costs[0][disp - this->m_minDisparity].at<float>(h, w);
				//��ǰ�Ӳ�+1����
				float costPlus = costs[0][disp + 1 - this->m_minDisparity].at<float>(h, w);
				//��ǰ�Ӳ�-1����
				float costMinus = costs[0][disp - 1 - this->m_minDisparity].at<float>(h, w);
				//һԪ�����������,d_sub=d+(c1-c2)/[2*(c1+c2-2*c0)]
				float diff = (costPlus - costMinus) / (2 * (costPlus + costMinus - 2 * cost));
				if (diff > -1 and diff < 1)
					interDisp -= diff;
			}
			dispTemp.at<float>(h, w) = interDisp;
		}
	}
	//��������,��ֵ�˲�
	cv::medianBlur(dispTemp, dispTemp, 3);
	return dispTemp;
}

int stereo::ADCensus::ADCensusImpl::multiOptimize()
{
	cv::Mat disp0, disp1;
	cost2disparity(0, disp0);//ɨ�����Ż������ͼ�Ӳ�
	cost2disparity(1, disp1);//ɨ�����Ż������ͼ�Ӳ�
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
