#include "../include/calib.h"
#include "../include/utils.h"
#include "../include/logger.h"
#include <vector>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

struct CalibrateParams
{
	float minDisZ;
	float maxDisZ;
	float inlierDis;
	CalibrateParams()
	{
		this->minDisZ = 100;
		this->maxDisZ = 1000;
		this->inlierDis = 0.1;
	}
	CalibrateParams(float minDisZ, float maxDisZ, float inlierDis)
	{
		this->minDisZ = minDisZ;
		this->maxDisZ = maxDisZ;
		this->inlierDis = inlierDis;
	}
};

class calib::StereoHandEyeSVD::SHESVDImpl
{
public:
    SHESVDImpl();
    ~SHESVDImpl();
public:
	/**
	 * @brief 选择图像ROI区域, 按"r"键重置, 按"Enter"键记录ROI.
	 * @param [in]  image 待框选的图像
	 * @param [out] roi   框选的ROI区域
	 */
	void selectImageRect(const cv::Mat& image, cv::Rect& roi);
	/**
	 * @brief 鼠标事件函数.
	 * @param [in] event 鼠标事件类型
	 * @param [in] x     x坐标
	 * @param [in] y     y坐标
	 * @param [in] flags 鼠标事件标志
	 * @param [in] param 用户数据
	 */
	static void imageHandle(int event, int x, int y, int flags, void* param);
	/**
	 * @brief 鼠标事件响应.
	 * @param [in] event   鼠标事件类型
	 * @param [in] x       x坐标
	 * @param [in] y       y坐标
	 * @param [in] flags   鼠标事件标志
	 * @param [in] image   图像
	 * @param [in] winName 图像窗口
	 */
	void mouseClick(int event, int x, int y, int flags, cv::Mat image, std::string winName);
	/**
	 * @brief 显示ROI图像.
	 * @param [in] image   图像
	 * @param [in] winName 图像窗口名称
	 */
	void showImage(const cv::Mat& image, const std::string& winName);
	/**
	 * @brief 重置矩形ROI区域.
	 */
	void reset();
	/**
	 * @brief xyz坐标矩阵转为点向量.
	 * @param [in]  xyz    坐标矩阵
	 * @param [out] points 点向量
	 */
	void xyzMat2Vector(const cv::Mat& xyz, std::vector<cv::Point3f>& points);
	/**
	 * @brief 生成相机坐标系下球心坐标矩阵.
	 * @param [in]  points      相机坐标系下球点云
	 * @param [out] sphereParam 相机坐标系下拟合球参数
	 */
	void getSphereInCamera(const std::vector<cv::Point3f>& points, cv::Vec4f& sphereParam);
	/**
	 * @brief 标定.
	 * @param [in]  baseSphere 基座坐标系球心坐标
	 * @param [in]  camSphere  相机坐标系球心坐标
	 * @param [out] R          旋转矩阵
	 * @param [out] t          平移矩阵
	 * @return 误差
	 */
	double calibrate(const std::vector<cv::Vec3f>& baseSphere, const std::vector<cv::Vec4f>& camSphere, cv::Mat& R, cv::Mat& t);
public:
	/**
	 * @brief ROI区域状态.
	 */
	enum rectState
	{
		NOT_SET = 0,    /*!< 未绘制 */
		IN_PROCESS = 1, /*!< 绘制中 */
		SET = 2         /*!< 已绘制 */
	};
	cv::Mat m_image;       /*!< 待框选的图像 */
	cv::Mat m_imageClone;  /*!< 待框选的图像备份 */
	cv::Size m_imageSize;  /*!< 待框选的图像尺寸 */
	cv::Rect m_rect;       /*!< 框选的ROI区域 */
	std::string m_winName; /*!< 待框选的图像窗口名称 */
	rectState m_rectState; /*!< ROI区域绘制状态 */

	cv::Point3f m_gripperPosition;   /*!< 标定球相对于末端位姿 */
	std::vector<cv::Mat> m_leftImage;/*!< 左图 */
	std::vector<cv::Mat> m_xyz;      /*!< 深度图 */
	CalibrateParams m_CalibrateParams;/*!< 标定参数 */
	std::vector<cv::Vec4f> m_sphereCamParam; /*!< 球心在相机坐标系下的坐标/半径 */
	std::vector<cv::Vec3f> m_sphereBaseParam;/*!< 球心在基座坐标系下的坐标 */
	cv::Mat R, t, TMatrix;
};

calib::StereoHandEyeSVD::StereoHandEyeSVD()
{
    this->impl = std::make_unique<SHESVDImpl>();
}

calib::StereoHandEyeSVD::~StereoHandEyeSVD()
{
}

void calib::StereoHandEyeSVD::setGripperPosition(const float& x, const float& y, const float& z)
{
	this->impl->m_gripperPosition = cv::Point3f(x, y, z);
}

void calib::StereoHandEyeSVD::loadImages(const std::string& leftRectifiedPattern, const std::string& xyzMatrixPattern)
{
	if (leftRectifiedPattern.empty() or xyzMatrixPattern.empty())
	{
		std::string msg = "Empty image or xyz matrix path!";
		LOG_ERROR(msg);
		throw std::runtime_error(msg);
	}

	std::vector<std::string> refImageList = utils::glob(leftRectifiedPattern, false);
	std::vector<std::string> xyzMatrixList = utils::glob(xyzMatrixPattern, false);
	if (refImageList.size() != xyzMatrixList.size())
	{
		std::string msg = "Inconsistent number of images and xyz matrix!";
		LOG_ERROR(msg);
		throw std::runtime_error(msg);
	}

	{
		cv::Mat src = cv::imread(refImageList[0], cv::IMREAD_UNCHANGED);
		this->impl->m_imageSize = src.size();
	}

	for (size_t i = 0; i < refImageList.size(); ++i)
	{
		this->impl->m_leftImage.push_back(cv::imread(refImageList[i], cv::IMREAD_UNCHANGED));
		this->impl->m_xyz.push_back(cv::imread(xyzMatrixList[i], cv::IMREAD_UNCHANGED));
	}
}

void calib::StereoHandEyeSVD::loadXYZFile(const std::string& csvFilePath)
{
	LOG_INFO("Loading end to base xyzrpy data...");
	std::ifstream xyzrpyData(csvFilePath, std::ios::in);
	if (!xyzrpyData.is_open())
	{
		xyzrpyData.close();
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
		cv::Vec3f data;
		for (int j = 1; j <= 3; ++j)//从第2列开始
			data[j - 1] = (*it)[j];
		this->impl->m_sphereBaseParam.push_back(data);
	}
	LOG_INFO("Load end to base xyzrpy data done.");
}

double calib::StereoHandEyeSVD::calibrate()
{
	for (size_t i = 0; i < this->impl->m_leftImage.size(); ++i)
	{
		cv::Rect rect;
		this->impl->selectImageRect(this->impl->m_leftImage[i], rect);
		cv::Mat xyzROI = this->impl->m_xyz[i](rect);
		std::vector<cv::Point3f> spherePoints;
		this->impl->xyzMat2Vector(xyzROI, spherePoints);
		cv::Vec4f sphereCamParam;
		this->impl->getSphereInCamera(spherePoints, sphereCamParam);
		this->impl->m_sphereCamParam.push_back(sphereCamParam);
	}
	if (this->impl->m_sphereBaseParam.size() != this->impl->m_sphereCamParam.size())
		return -1;
	cv::Mat a;
	double err = this->impl->calibrate(this->impl->m_sphereBaseParam, this->impl->m_sphereCamParam, this->impl->R, this->impl->t);
	cv::hconcat(this->impl->R, this->impl->t, a);
	cv::Mat tmp = (cv::Mat_<float>(1, 4) << 0, 0, 0, 1);
	cv::vconcat(a, tmp, this->impl->TMatrix);
	return err;
}

void calib::StereoHandEyeSVD::writeYAMLFile(const std::string& path) const
{
	std::string intrinsicYMLDir = path + "/TMatrix.yml";
	cv::FileStorage fs(intrinsicYMLDir, cv::FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << this->impl->R << "t" << this->impl->t << "TMatrix" << this->impl->TMatrix;
		fs.release();
		LOG_INFO("TMatrix YML file save in floder: \"" + intrinsicYMLDir + "\".");
	}
	else
		LOG_ERROR("Cannot save the TMatrix.");
}

calib::StereoHandEyeSVD::SHESVDImpl::SHESVDImpl()
{
	this->m_rectState = NOT_SET;
}

calib::StereoHandEyeSVD::SHESVDImpl::~SHESVDImpl()
{
}

void calib::StereoHandEyeSVD::SHESVDImpl::selectImageRect(const cv::Mat& image, cv::Rect& roi)
{
	this->m_winName = "[Rect] image";
	this->m_image = image;
	this->m_imageClone = image;
	this->m_imageSize = image.size();
	//-- 设置鼠标回调
	cv::namedWindow(this->m_winName, cv::WINDOW_NORMAL);
	cv::setMouseCallback(this->m_winName, StereoHandEyeSVD::SHESVDImpl::imageHandle, this);
	showImage(this->m_image, this->m_winName);
	//-- 按键检测(在图像窗口中)
	char ch = 0;
	while (ch != 13)
	{
		ch = cv::waitKey(1);
		switch (ch)
		{
		case 13: //"Enter"键记录矩形ROI区域坐标数据
			LOG_INFO("Record roi data...");
			break;
		case 82: case 114: //"r"键重置图像
			reset();
			showImage(this->m_image, this->m_winName);
			break;
		default: break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	//-- 销毁图像窗口
	cv::destroyWindow(this->m_winName);
	//-- 传出矩形区域
	roi = this->m_rect;
	reset();
}

void calib::StereoHandEyeSVD::SHESVDImpl::imageHandle(int event, int x, int y, int flags, void* param)
{
	StereoHandEyeSVD::SHESVDImpl* p = reinterpret_cast<StereoHandEyeSVD::SHESVDImpl*>(param);
	p->mouseClick(event, x, y, flags, p->m_image, p->m_winName);
}

void calib::StereoHandEyeSVD::SHESVDImpl::mouseClick(int event, int x, int y, int flags, cv::Mat image, std::string winName)
{
	//-- 防止越界
	if (x < 0)
		x = 0;
	if (y < 0)
		y = 0;
	if (x >= this->m_imageSize.width)
		x = this->m_imageSize.width - 1;
	if (y >= this->m_imageSize.height)
		y = this->m_imageSize.height - 1;
	//-- 鼠标事件
	switch (event)
	{
		//-- 鼠标按下事件
	case cv::EVENT_LBUTTONDOWN:
		this->m_rectState = IN_PROCESS;
		this->m_rect = cv::Rect(x, y, 1, 1);
		break;
		//-- 鼠标松开事件
	case cv::EVENT_LBUTTONUP:
		if (this->m_rectState == IN_PROCESS)
		{
			//-- 不允许矩形区域为点
			if (this->m_rect.x == x || this->m_rect.y == y)
				this->m_rectState = NOT_SET;
			else
			{
				this->m_rect = cv::Rect(cv::Point(this->m_rect.x, this->m_rect.y), cv::Point(x, y));
				this->m_rectState = SET;
			}
			showImage(image, winName);
		}
		break;
		//-- 鼠标移动事件
	case cv::EVENT_MOUSEMOVE:
		if (this->m_rectState == IN_PROCESS)
		{
			this->m_rect = cv::Rect(cv::Point(this->m_rect.x, this->m_rect.y), cv::Point(x, y));
			showImage(image, winName);
		}
		break;
	default: break;
	}
}

void calib::StereoHandEyeSVD::SHESVDImpl::showImage(const cv::Mat& image, const std::string& winName)
{
	cv::Mat imageClone = image.clone();
	//-- 绘制矩形框
	if (m_rectState == IN_PROCESS or m_rectState == SET)
	{
		cv::drawMarker(imageClone, cv::Point(this->m_rect.x, this->m_rect.y), cv::Scalar(255, 0, 0),
			cv::MARKER_TILTED_CROSS, 20, 3, cv::LINE_AA);
		cv::rectangle(imageClone, cv::Point(this->m_rect.x, this->m_rect.y), cv::Point(this->m_rect.x + this->m_rect.width,
			this->m_rect.y + this->m_rect.height), cv::Scalar(0, 0, 255), 2);
	}
	cv::imshow(winName, imageClone);
	cv::waitKey(1);
}

void calib::StereoHandEyeSVD::SHESVDImpl::reset()
{
	this->m_rectState = NOT_SET;
	this->m_imageClone = this->m_image;
	this->m_rect = cv::Rect(0, 0, 0, 0);
}

void calib::StereoHandEyeSVD::SHESVDImpl::xyzMat2Vector(const cv::Mat& xyz, std::vector<cv::Point3f>& points)
{
	for (int i = 0; i < xyz.rows; ++i)
	{
		for (int j = 0; j < xyz.cols; ++j)
		{
			cv::Point3f point;
			point.x = xyz.ptr<cv::Vec3f>(i)[j][0];
			point.y = xyz.ptr<cv::Vec3f>(i)[j][1];
			point.z = xyz.ptr<cv::Vec3f>(i)[j][2];
			points.push_back(point);
		}
	}
}

void calib::StereoHandEyeSVD::SHESVDImpl::getSphereInCamera(const std::vector<cv::Point3f>& points, cv::Vec4f& sphereParam)
{
	//-- Step.1 创建PCL点云
	LOG_INFO("Creating point cloud...");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (std::vector<cv::Point3f>::const_iterator iter = points.begin(); iter != points.end(); ++iter)
	{
		pcl::PointXYZ p;
		p.x = iter->x;
		p.y = iter->y;
		p.z = iter->z;
		cloud->push_back(p);
	}
	//-- Step.2 直通滤波
	LOG_INFO("Filtering point cloud...");
	LOG_INFO("Original points = " + std::to_string(cloud->points.size()));
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(this->m_CalibrateParams.minDisZ, this->m_CalibrateParams.maxDisZ);
	pass.filter(*cloud);
    LOG_INFO("Filtered points = " + std::to_string(cloud->points.size()));
	//--Step.3 RANSAC球拟合
    LOG_INFO("RANSAC--fitting sphere...");
	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_sphere(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud, true));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_sphere);		//创建随机采样一致性对象
	ransac.setDistanceThreshold(this->m_CalibrateParams.inlierDis);//设置内点阈值
	ransac.computeModel();
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_sphere(new pcl::PointCloud <pcl::PointXYZ>);
	//std::vector<int> inliers;	
	//ransac.getInliers(inliers); //内点索引
	//pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_sphere);
	Eigen::VectorXf coefficient;
	ransac.getModelCoefficients(coefficient);
	LOG_INFO("Spherical equation:");
	std::stringstream ss;
	ss << "(x - " << coefficient[0] << ") ^ 2 + (y - " << coefficient[1]
		<< ") ^ 2 + (z - " << coefficient[2] << ") ^ 2 = " << coefficient[3] << " ^ 2";
	LOG_INFO_MSG(ss.str());
	sphereParam[0] = coefficient[0];
	sphereParam[1] = coefficient[1];
	sphereParam[2] = coefficient[2];
	sphereParam[3] = coefficient[3];
}

double calib::StereoHandEyeSVD::SHESVDImpl::calibrate(const std::vector<cv::Vec3f>& baseSphere, const std::vector<cv::Vec4f>& camSphere, cv::Mat& R, cv::Mat& t)
{
	//-- 球拟合误差?
	int num = baseSphere.size();
	double radiusVariance = 0.;
	double radiusAverage = 0.;
	double radiusSquare = 0.;
	for (int i = 0; i < num; ++i)
		radiusVariance += camSphere[i][3];
	radiusAverage = radiusVariance / num;
	radiusVariance = 0;
	for (int i = 0; i < num; ++i)
		radiusVariance += camSphere[i][3] * camSphere[i][3];
	radiusSquare = radiusVariance / num;
	radiusVariance = radiusSquare - radiusAverage * radiusAverage;
	LOG_INFO("Each sphere radius: ");
	for (int i = 0; i < num; ++i)
		LOG_INFO_MSG(std::to_string(camSphere[i][3]));
	LOG_INFO("Variance of fitting sphere radius: " + std::to_string(radiusVariance));
	Eigen::MatrixXf camMatrix = Eigen::MatrixXf::Zero(3, num);
	Eigen::MatrixXf baseMatrix = Eigen::MatrixXf::Zero(3, num);
	//-- 创建Camera 和 Base坐标系下的坐标矩阵
	for (int i = 0; i < num; ++i)
	{
		camMatrix(0, i) = camSphere[i][0];
		camMatrix(1, i) = camSphere[i][1];
		camMatrix(2, i) = camSphere[i][2];
		baseMatrix(0, i) = baseSphere[i][0];
		baseMatrix(1, i) = baseSphere[i][1];
		baseMatrix(2, i) = baseSphere[i][2];
	}
	LOG_INFO("Points Matrix: ");
	cv::Mat baseMat;
	cv::eigen2cv(baseMatrix, baseMat);
	LOG_INFO_MSG_MAT("baseMatrix: ", baseMat);
    cv::Mat camMat;
    cv::eigen2cv(camMatrix, camMat);
    LOG_INFO_MSG_MAT("camMatrix: ", camMat);
	//-- 去中心化

	Eigen::MatrixXf oneCols(1, num);
	oneCols.setOnes();
	// 计算相机坐标系下球心坐标均值
	Eigen::MatrixXf camSphereMean = camMatrix.rowwise().mean();
	// 计算去中心化相机坐标系下球心坐标
	Eigen::MatrixXf camDeSphereMatrix(3, num);
	camDeSphereMatrix = camMatrix - camSphereMean * oneCols;
	// 计算基座坐标系下球心坐标均值
	Eigen::MatrixXf baseSphereMean = baseMatrix.rowwise().mean();
	// 计算去中心化基座坐标系下球心坐标
	Eigen::MatrixXf baseDeSphereMatrix(3, num);
	baseDeSphereMatrix = baseMatrix - baseSphereMean * oneCols;
	// 协方差矩阵
	Eigen::MatrixXf Matrix(3, 3);
	Matrix = camDeSphereMatrix * baseDeSphereMatrix.transpose();
	// SVD分解
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(Matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix3f V = svd.matrixV(), U = svd.matrixU();
	// R
	Eigen::Matrix3f rotMatrix = V * U.transpose();
	// t
	Eigen::MatrixXf tranVector = baseSphereMean - rotMatrix * camSphereMean;
	LOG_INFO("Transform matrix:");
	cv::Mat rotMat;
	cv::eigen2cv(rotMatrix, rotMat);
	LOG_INFO_MSG_MAT("baseMatrix: ", rotMat);
    cv::Mat tranMat;
    cv::eigen2cv(tranVector, tranMat);
    LOG_INFO_MSG_MAT("tranVector: ", tranMat);
	Eigen::MatrixXf Yf(3, num), dY(3, num);
	Yf = rotMatrix * camMatrix + tranVector * oneCols;
	dY = baseMatrix - Yf;
	//求矩阵f范数
	double error = sqrt((dY * dY.transpose()).trace()) / num;
	LOG_INFO("Error = " + std::to_string(error));
	cv::Mat RTmp = cv::Mat_<float>::eye(3, 3);
	cv::Mat tTmp = cv::Mat_<float>::zeros(3, 1);
	cv::eigen2cv(rotMatrix, RTmp);
	cv::eigen2cv(tranVector, tTmp);
	R = RTmp;
	t = tTmp;
	return error;
}


