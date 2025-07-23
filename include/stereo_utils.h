/*********************************************************************
 * @file   stereo_utils.h
 * @brief  ˫Ŀ������ݽṹ
 * @author Qianyao Zhuang
 * @date   May 2025
 *********************************************************************/
#pragma once
#include <string>
#include <vector>
#include <opencv2/core.hpp>

template <typename T>
struct StereoPair
{
	T left;  /*!< left */
	T right; /*!< right */
	StereoPair()
		: left(T()), right(T()) {
	};
	StereoPair(const T& left, const T& right)
		: left(left), right(right) {
	};
	StereoPair(const StereoPair& other)
		: left(other.left), right(other.right) {
	};
	StereoPair(StereoPair&& other) noexcept
		: left(std::move(other.left)), right(std::move(other.right)) {
	};
	~StereoPair() {};

	StereoPair& operator=(const StereoPair& other) {
		if (this != &other) {
			left = other.left; right = other.right; return *this;
		}
	}
	StereoPair& operator=(StereoPair&& other) noexcept {
		if (this != &other) {
			left = std::move(other.left); right = std::move(other.right); return *this;
		}
	}

	void swap() noexcept(noexcept(std::swap(std::declval<T&>(), std::declval<T&>()))) {
		using std::swap;
		swap(left, right);
	}
};

class CameraIntrinsic
{
public:
	cv::Mat intrinsic_matrix;        /*!< �ڲξ��� */
	cv::Mat distortion_coefficients; /*!< ����ϵ�� */

	CameraIntrinsic();
	CameraIntrinsic(const cv::Mat& intrinsic_matrix, const cv::Mat& distortion_coefficients);
	CameraIntrinsic(const CameraIntrinsic& other);
	CameraIntrinsic(CameraIntrinsic&& other) noexcept;
	~CameraIntrinsic() = default;
	CameraIntrinsic& operator=(const CameraIntrinsic& other);
    CameraIntrinsic& operator=(CameraIntrinsic&& other) noexcept;
	/**
	 * @brief �ж�����ڲ��Ƿ�Ϊ��.
	 * @return true: Ϊ��, false: ��Ϊ��
	 */
	bool empty() const;
};

class StereoExtrinsic
{
public:
	cv::Mat R;      /*!< �������֮�����ת���� */
	cv::Mat T;      /*!< �������֮���ƽ������ */
	cv::Mat E;      /*!< ���ʾ���, �������ϵͶӰ��ϵ */
	cv::Mat F;      /*!< ��������, ��������ϵͶӰ��ϵ */
	StereoExtrinsic();
	StereoExtrinsic(const cv::Mat& R, const cv::Mat& T, const cv::Mat& E, const cv::Mat& F);
	StereoExtrinsic(const StereoExtrinsic& other);
	StereoExtrinsic(StereoExtrinsic&& other) noexcept;
	~StereoExtrinsic() = default;
    StereoExtrinsic& operator=(const StereoExtrinsic& other);
    StereoExtrinsic& operator=(StereoExtrinsic&& other) noexcept;
	/**
	 * @brief �ж�˫Ŀ�������Ƿ�Ϊ��.
	 * @return true: Ϊ��, false: ��Ϊ��
	 */
	bool empty() const;
};

class EpipolarRectifyMap
{
public:
	cv::Mat R1; /*!< �������ת����, ���������ϵ -> ����У�������������ϵ */
	cv::Mat R2; /*!< �������ת����, ���������ϵ -> ����У�������������ϵ */
	cv::Mat P1; /*!< �����ͶӰ����, ���������P, 3D�� -> 2Dͼ�� */
	cv::Mat P2; /*!< �����ͶӰ����, ���������P, 3D�� -> 2Dͼ��) */
	cv::Mat map00, map01, map10, map11; /*!< ����У��ӳ����� */
	
	EpipolarRectifyMap();
	EpipolarRectifyMap(const cv::Mat& R1, const cv::Mat& R2, const cv::Mat& P1, const cv::Mat& P2, const cv::Mat& map00, const cv::Mat& map01, const cv::Mat& map10, const cv::Mat& map11);
	EpipolarRectifyMap(const EpipolarRectifyMap& other);
	EpipolarRectifyMap(EpipolarRectifyMap&& other) noexcept;
	EpipolarRectifyMap& operator=(const EpipolarRectifyMap& other);
	EpipolarRectifyMap& operator=(EpipolarRectifyMap&& other) noexcept;
	~EpipolarRectifyMap() = default;
	/**
	 * @brief ���ؼ���У��ӳ��YML�ļ�.
	 * @param [in] ymlFilePath ����У��ӳ��YML�ļ�·��
	 */
	void loadRectifyMapsYMLFile(const std::string& ymlFilePath);
	/**
	 * @brief ���㼫��У��ӳ��.
	 * @param [in] intrinsic ����ڲ�
	 * @param [in] imgsz     ͼ��ߴ�
	 */
	void compute(const StereoPair<CameraIntrinsic>& intrinsic, const cv::Size& imgsz);
	/**
	 * @brief �жϼ���У��ӳ������Ƿ�Ϊ��.
	 * @return true: ӳ�����Ϊ��; false: ӳ�����Ϊ��.
	 */
	bool empty() const;
};

class StereoParams
{
public:
	StereoPair<CameraIntrinsic> intrinsic; /*!< ��������ڲ� */
	StereoExtrinsic extrinsic;   /*!< ˫Ŀ������ */
	EpipolarRectifyMap map; /*!< ���弫��У��ӳ�� */

	cv::Mat Q; /*!< ��ͶӰ���� */

	float rectified_f;  /*!< У���󽹾�f, px */
	float rectified_cx; /*!< У�������x, px */
	float rectified_cy; /*!< У�������y, px */
	float baseline;     /*!< ���߾��� */
	cv::Size imgsz;     /*!< ͼ��ߴ� */

	StereoParams();
	StereoParams(const std::string& ymlFilePath);
	~StereoParams() = default;
	/**
	 * @brief ����˫Ŀ�������.
	 * @param [in] ymlFilePath ˫Ŀ�������YAML�ļ�·��
	 */
	void loadYAMLFile(const std::string& ymlFilePath);
	/**
	 * @brief �ж�˫Ŀ��������Ƿ�Ϊ��.
	 * @return true: Ϊ��, false: ��Ϊ��
	 */
	bool empty() const;
	/**
	 * @brief ��ӡ����.
	 */
	void print() const;
};

/**
 * @brief ��ɫģ������.
 */
enum class ColorModel
{
	RGB = 0,/*!< RGB��ɫģ������ƥ�� */
	HSI = 1,/*!< HSI��ɫģ������ƥ�� */
};

/**
 * @brief Census֧�ִ��ڳߴ�����.
 */
enum class CensusWin
{
	CENSUSWIN_9x7 = 0, /*!< Census Support Window, 9 x 7 */
	CENSUSWIN_7x5 = 1, /*!< Census Support Window, 7 x 5 */
};

/**
 * @brief (Selective) AD-Census(-HSI)����ƥ���㷨�ؼ�����.
 */
class ADCensusParams
{
public:
	ADCensusParams() { setADCensusParams(ColorModel::RGB); };
	ADCensusParams(const ColorModel& colorModel) { setADCensusParams(colorModel); };
	~ADCensusParams() {};
public:
	void setADCensusParams(const ColorModel& colorModel);
public:
	float lambdaAD;          /*!< [Step.1 ��ʼ���ۼ���] AD����Ȩ�� */
	CensusWin censusWin;     /*!< [Step.1 ��ʼ���ۼ���] Census�任���� */
	float lambdaCensus;      /*!< [Step.1 ��ʼ���ۼ���] Census����Ȩ�� */
	float lambdaHue;         /*!< [Step.1 ��ʼ���ۼ���] ɫ������Ȩ��, HSI */
	float lambdaSaturation;  /*!< [Step.1 ��ʼ���ۼ���] ���Ͷȷ���Ȩ��, HSI */
	float lambdaIntensity;   /*!< [Step.1 ��ʼ���ۼ���] ǿ�ȷ���Ȩ��, HSI */
	int   colorThresh1;      /*!< [Step.2 ���۾ۺ�] ��Ϊ���ɵ���ɫ/ɫ����ֵ */
	int   colorThresh2;      /*!< [Step.2 ���۾ۺ�] ��Ϊ�ϸ����ɫ/ɫ����ֵ */
	int   saturationThresh1; /*!< [Step.2 ���۾ۺ�] ��Ϊ���ɵı��Ͷ���ֵ, HSI */
	int   saturationThresh2; /*!< [Step.2 ���۾ۺ�] ��Ϊ�ϸ�ı��Ͷ���ֵ, HSI */
	int   intensityThresh1;  /*!< [Step.2 ���۾ۺ�] ��Ϊ���ɵ�ǿ����ֵ, HSI */
	int   intensityThresh2;  /*!< [Step.2 ���۾ۺ�] ��Ϊ�ϸ��ǿ����ֵ, HSI */
	int   maxLength1;        /*!< [Step.2 ���۾ۺ�] ʮ�ֽ����������󳤶� */
	int   maxLength2;        /*!< [Step.2 ���۾ۺ�] ʮ�ֽ��������Ҫ���Ϊ�ϸ񳤶� */
	int   iterations;        /*!< [Step.2 ���۾ۺ�] �������� */
	int   colorDiff;         /*!< [Step.3 ɨ�����Ż�] ������ɫ���� */
	float pi1;               /*!< [Step.3 ɨ�����Ż�] �ͷ�����P1 */
	float pi2;               /*!< [Step.3 ɨ�����Ż�] �ͷ�����P2 */
	int   dispTolerance;     /*!< [Step.4 ��Ⱥ����] �����Ӳ����,px */
	int   votingThresh;      /*!< [Step.4 ����ͶƱ] ͶƱ��С��ֵ */
	float votingRatioThresh; /*!< [Step.4 ����ͶƱ] ͶƱ����С��ֵ */
	int   maxSearchDepth;    /*!< [Step.4 �����ֵ] ��Ե����������� */
	int   blurKernelSize;    /*!< [Step.4 ��Ȳ���������] �˲��˴�С,����,��3x3 */
	int   cannyThresh1;      /*!< [Step.4 ��Ȳ���������] Canny��Ե�����ֵ1 */
	int   cannyThresh2;      /*!< [Step.4 ��Ȳ���������] Canny��Ե�����ֵ2 */
	int   cannyKernelSize;   /*!< [Step.4 ��Ȳ���������] Canny��Ե���˴�С,����,��3x3 */
};
