/*********************************************************************
 * @file   stereo_utils.h
 * @brief  双目相关数据结构
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
	cv::Mat intrinsic_matrix;        /*!< 内参矩阵 */
	cv::Mat distortion_coefficients; /*!< 畸变系数 */

	CameraIntrinsic();
	CameraIntrinsic(const cv::Mat& intrinsic_matrix, const cv::Mat& distortion_coefficients);
	CameraIntrinsic(const CameraIntrinsic& other);
	CameraIntrinsic(CameraIntrinsic&& other) noexcept;
	~CameraIntrinsic() = default;
	CameraIntrinsic& operator=(const CameraIntrinsic& other);
    CameraIntrinsic& operator=(CameraIntrinsic&& other) noexcept;
	/**
	 * @brief 判断相机内参是否为空.
	 * @return true: 为空, false: 不为空
	 */
	bool empty() const;
};

class StereoExtrinsic
{
public:
	cv::Mat R;      /*!< 左右相机之间的旋转矩阵 */
	cv::Mat T;      /*!< 左右相机之间的平移向量 */
	cv::Mat E;      /*!< 本质矩阵, 相机坐标系投影关系 */
	cv::Mat F;      /*!< 基础矩阵, 像素坐标系投影关系 */
	StereoExtrinsic();
	StereoExtrinsic(const cv::Mat& R, const cv::Mat& T, const cv::Mat& E, const cv::Mat& F);
	StereoExtrinsic(const StereoExtrinsic& other);
	StereoExtrinsic(StereoExtrinsic&& other) noexcept;
	~StereoExtrinsic() = default;
    StereoExtrinsic& operator=(const StereoExtrinsic& other);
    StereoExtrinsic& operator=(StereoExtrinsic&& other) noexcept;
	/**
	 * @brief 判断双目相机外参是否为空.
	 * @return true: 为空, false: 不为空
	 */
	bool empty() const;
};

class EpipolarRectifyMap
{
public:
	cv::Mat R1; /*!< 左相机旋转矩阵, 左相机坐标系 -> 极线校正后左相机坐标系 */
	cv::Mat R2; /*!< 右相机旋转矩阵, 右相机坐标系 -> 极线校正后右相机坐标系 */
	cv::Mat P1; /*!< 左相机投影矩阵, 摄像机矩阵P, 3D点 -> 2D图像 */
	cv::Mat P2; /*!< 右相机投影矩阵, 摄像机矩阵P, 3D点 -> 2D图像) */
	cv::Mat map00, map01, map10, map11; /*!< 极线校正映射矩阵 */
	
	EpipolarRectifyMap();
	EpipolarRectifyMap(const cv::Mat& R1, const cv::Mat& R2, const cv::Mat& P1, const cv::Mat& P2, const cv::Mat& map00, const cv::Mat& map01, const cv::Mat& map10, const cv::Mat& map11);
	EpipolarRectifyMap(const EpipolarRectifyMap& other);
	EpipolarRectifyMap(EpipolarRectifyMap&& other) noexcept;
	EpipolarRectifyMap& operator=(const EpipolarRectifyMap& other);
	EpipolarRectifyMap& operator=(EpipolarRectifyMap&& other) noexcept;
	~EpipolarRectifyMap() = default;
	/**
	 * @brief 加载极线校正映射YML文件.
	 * @param [in] ymlFilePath 极线校正映射YML文件路径
	 */
	void loadRectifyMapsYMLFile(const std::string& ymlFilePath);
	/**
	 * @brief 计算极线校正映射.
	 * @param [in] intrinsic 相机内参
	 * @param [in] imgsz     图像尺寸
	 */
	void compute(const StereoPair<CameraIntrinsic>& intrinsic, const cv::Size& imgsz);
	/**
	 * @brief 判断极线校正映射矩阵是否为空.
	 * @return true: 映射矩阵为空; false: 映射矩阵不为空.
	 */
	bool empty() const;
};

class StereoParams
{
public:
	StereoPair<CameraIntrinsic> intrinsic; /*!< 左右相机内参 */
	StereoExtrinsic extrinsic;   /*!< 双目相机外参 */
	EpipolarRectifyMap map; /*!< 立体极线校正映射 */

	cv::Mat Q; /*!< 重投影矩阵 */

	float rectified_f;  /*!< 校正后焦距f, px */
	float rectified_cx; /*!< 校正后光心x, px */
	float rectified_cy; /*!< 校正后光心y, px */
	float baseline;     /*!< 基线距离 */
	cv::Size imgsz;     /*!< 图像尺寸 */

	StereoParams();
	StereoParams(const std::string& ymlFilePath);
	~StereoParams() = default;
	/**
	 * @brief 加载双目相机参数.
	 * @param [in] ymlFilePath 双目相机参数YAML文件路径
	 */
	void loadYAMLFile(const std::string& ymlFilePath);
	/**
	 * @brief 判断双目相机参数是否为空.
	 * @return true: 为空, false: 不为空
	 */
	bool empty() const;
	/**
	 * @brief 打印参数.
	 */
	void print() const;
};

/**
 * @brief 颜色模型序列.
 */
enum class ColorModel
{
	RGB = 0,/*!< RGB颜色模型立体匹配 */
	HSI = 1,/*!< HSI颜色模型立体匹配 */
};

/**
 * @brief Census支持窗口尺寸序列.
 */
enum class CensusWin
{
	CENSUSWIN_9x7 = 0, /*!< Census Support Window, 9 x 7 */
	CENSUSWIN_7x5 = 1, /*!< Census Support Window, 7 x 5 */
};

/**
 * @brief (Selective) AD-Census(-HSI)立体匹配算法关键参数.
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
	float lambdaAD;          /*!< [Step.1 初始代价计算] AD代价权重 */
	CensusWin censusWin;     /*!< [Step.1 初始代价计算] Census变换窗口 */
	float lambdaCensus;      /*!< [Step.1 初始代价计算] Census代价权重 */
	float lambdaHue;         /*!< [Step.1 初始代价计算] 色调分量权重, HSI */
	float lambdaSaturation;  /*!< [Step.1 初始代价计算] 饱和度分量权重, HSI */
	float lambdaIntensity;   /*!< [Step.1 初始代价计算] 强度分量权重, HSI */
	int   colorThresh1;      /*!< [Step.2 代价聚合] 较为宽松的颜色/色调阈值 */
	int   colorThresh2;      /*!< [Step.2 代价聚合] 较为严格的颜色/色调阈值 */
	int   saturationThresh1; /*!< [Step.2 代价聚合] 较为宽松的饱和度阈值, HSI */
	int   saturationThresh2; /*!< [Step.2 代价聚合] 较为严格的饱和度阈值, HSI */
	int   intensityThresh1;  /*!< [Step.2 代价聚合] 较为宽松的强度阈值, HSI */
	int   intensityThresh2;  /*!< [Step.2 代价聚合] 较为严格的强度阈值, HSI */
	int   maxLength1;        /*!< [Step.2 代价聚合] 十字交叉臂延伸最大长度 */
	int   maxLength2;        /*!< [Step.2 代价聚合] 十字交叉臂延伸要求较为严格长度 */
	int   iterations;        /*!< [Step.2 代价聚合] 迭代次数 */
	int   colorDiff;         /*!< [Step.3 扫描线优化] 允许颜色差异 */
	float pi1;               /*!< [Step.3 扫描线优化] 惩罚参数P1 */
	float pi2;               /*!< [Step.3 扫描线优化] 惩罚参数P2 */
	int   dispTolerance;     /*!< [Step.4 离群点检测] 允许视差误差,px */
	int   votingThresh;      /*!< [Step.4 区域投票] 投票最小阈值 */
	float votingRatioThresh; /*!< [Step.4 区域投票] 投票率最小阈值 */
	int   maxSearchDepth;    /*!< [Step.4 合理插值] 边缘最大搜索距离 */
	int   blurKernelSize;    /*!< [Step.4 深度不连续调整] 滤波核大小,奇数,如3x3 */
	int   cannyThresh1;      /*!< [Step.4 深度不连续调整] Canny边缘检测阈值1 */
	int   cannyThresh2;      /*!< [Step.4 深度不连续调整] Canny边缘检测阈值2 */
	int   cannyKernelSize;   /*!< [Step.4 深度不连续调整] Canny边缘检测核大小,奇数,如3x3 */
};
