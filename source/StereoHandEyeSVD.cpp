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
	 * @brief ѡ��ͼ��ROI����, ��"r"������, ��"Enter"����¼ROI.
	 * @param [in]  image ����ѡ��ͼ��
	 * @param [out] roi   ��ѡ��ROI����
	 */
	void selectImageRect(const cv::Mat& image, cv::Rect& roi);
	/**
	 * @brief ����¼�����.
	 * @param [in] event ����¼�����
	 * @param [in] x     x����
	 * @param [in] y     y����
	 * @param [in] flags ����¼���־
	 * @param [in] param �û�����
	 */
	static void imageHandle(int event, int x, int y, int flags, void* param);
	/**
	 * @brief ����¼���Ӧ.
	 * @param [in] event   ����¼�����
	 * @param [in] x       x����
	 * @param [in] y       y����
	 * @param [in] flags   ����¼���־
	 * @param [in] image   ͼ��
	 * @param [in] winName ͼ�񴰿�
	 */
	void mouseClick(int event, int x, int y, int flags, cv::Mat image, std::string winName);
	/**
	 * @brief ��ʾROIͼ��.
	 * @param [in] image   ͼ��
	 * @param [in] winName ͼ�񴰿�����
	 */
	void showImage(const cv::Mat& image, const std::string& winName);
	/**
	 * @brief ���þ���ROI����.
	 */
	void reset();
	/**
	 * @brief xyz�������תΪ������.
	 * @param [in]  xyz    �������
	 * @param [out] points ������
	 */
	void xyzMat2Vector(const cv::Mat& xyz, std::vector<cv::Point3f>& points);
	/**
	 * @brief �����������ϵ�������������.
	 * @param [in]  points      �������ϵ�������
	 * @param [out] sphereParam �������ϵ����������
	 */
	void getSphereInCamera(const std::vector<cv::Point3f>& points, cv::Vec4f& sphereParam);
	/**
	 * @brief �궨.
	 * @param [in]  baseSphere ��������ϵ��������
	 * @param [in]  camSphere  �������ϵ��������
	 * @param [out] R          ��ת����
	 * @param [out] t          ƽ�ƾ���
	 * @return ���
	 */
	double calibrate(const std::vector<cv::Vec3f>& baseSphere, const std::vector<cv::Vec4f>& camSphere, cv::Mat& R, cv::Mat& t);
public:
	/**
	 * @brief ROI����״̬.
	 */
	enum rectState
	{
		NOT_SET = 0,    /*!< δ���� */
		IN_PROCESS = 1, /*!< ������ */
		SET = 2         /*!< �ѻ��� */
	};
	cv::Mat m_image;       /*!< ����ѡ��ͼ�� */
	cv::Mat m_imageClone;  /*!< ����ѡ��ͼ�񱸷� */
	cv::Size m_imageSize;  /*!< ����ѡ��ͼ��ߴ� */
	cv::Rect m_rect;       /*!< ��ѡ��ROI���� */
	std::string m_winName; /*!< ����ѡ��ͼ�񴰿����� */
	rectState m_rectState; /*!< ROI�������״̬ */

	cv::Point3f m_gripperPosition;   /*!< �궨�������ĩ��λ�� */
	std::vector<cv::Mat> m_leftImage;/*!< ��ͼ */
	std::vector<cv::Mat> m_xyz;      /*!< ���ͼ */
	CalibrateParams m_CalibrateParams;/*!< �궨���� */
	std::vector<cv::Vec4f> m_sphereCamParam; /*!< �������������ϵ�µ�����/�뾶 */
	std::vector<cv::Vec3f> m_sphereBaseParam;/*!< �����ڻ�������ϵ�µ����� */
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
	for (; it != strArray.end(); ++it)//�ӵ�2�п�ʼ
	{
		cv::Vec3f data;
		for (int j = 1; j <= 3; ++j)//�ӵ�2�п�ʼ
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
	//-- �������ص�
	cv::namedWindow(this->m_winName, cv::WINDOW_NORMAL);
	cv::setMouseCallback(this->m_winName, StereoHandEyeSVD::SHESVDImpl::imageHandle, this);
	showImage(this->m_image, this->m_winName);
	//-- �������(��ͼ�񴰿���)
	char ch = 0;
	while (ch != 13)
	{
		ch = cv::waitKey(1);
		switch (ch)
		{
		case 13: //"Enter"����¼����ROI������������
			LOG_INFO("Record roi data...");
			break;
		case 82: case 114: //"r"������ͼ��
			reset();
			showImage(this->m_image, this->m_winName);
			break;
		default: break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	//-- ����ͼ�񴰿�
	cv::destroyWindow(this->m_winName);
	//-- ������������
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
	//-- ��ֹԽ��
	if (x < 0)
		x = 0;
	if (y < 0)
		y = 0;
	if (x >= this->m_imageSize.width)
		x = this->m_imageSize.width - 1;
	if (y >= this->m_imageSize.height)
		y = this->m_imageSize.height - 1;
	//-- ����¼�
	switch (event)
	{
		//-- ��갴���¼�
	case cv::EVENT_LBUTTONDOWN:
		this->m_rectState = IN_PROCESS;
		this->m_rect = cv::Rect(x, y, 1, 1);
		break;
		//-- ����ɿ��¼�
	case cv::EVENT_LBUTTONUP:
		if (this->m_rectState == IN_PROCESS)
		{
			//-- �������������Ϊ��
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
		//-- ����ƶ��¼�
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
	//-- ���ƾ��ο�
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
	//-- Step.1 ����PCL����
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
	//-- Step.2 ֱͨ�˲�
	LOG_INFO("Filtering point cloud...");
	LOG_INFO("Original points = " + std::to_string(cloud->points.size()));
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(this->m_CalibrateParams.minDisZ, this->m_CalibrateParams.maxDisZ);
	pass.filter(*cloud);
    LOG_INFO("Filtered points = " + std::to_string(cloud->points.size()));
	//--Step.3 RANSAC�����
    LOG_INFO("RANSAC--fitting sphere...");
	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_sphere(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud, true));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_sphere);		//�����������һ���Զ���
	ransac.setDistanceThreshold(this->m_CalibrateParams.inlierDis);//�����ڵ���ֵ
	ransac.computeModel();
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_sphere(new pcl::PointCloud <pcl::PointXYZ>);
	//std::vector<int> inliers;	
	//ransac.getInliers(inliers); //�ڵ�����
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
	//-- ��������?
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
	//-- ����Camera �� Base����ϵ�µ��������
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
	//-- ȥ���Ļ�

	Eigen::MatrixXf oneCols(1, num);
	oneCols.setOnes();
	// �����������ϵ�����������ֵ
	Eigen::MatrixXf camSphereMean = camMatrix.rowwise().mean();
	// ����ȥ���Ļ��������ϵ����������
	Eigen::MatrixXf camDeSphereMatrix(3, num);
	camDeSphereMatrix = camMatrix - camSphereMean * oneCols;
	// �����������ϵ�����������ֵ
	Eigen::MatrixXf baseSphereMean = baseMatrix.rowwise().mean();
	// ����ȥ���Ļ���������ϵ����������
	Eigen::MatrixXf baseDeSphereMatrix(3, num);
	baseDeSphereMatrix = baseMatrix - baseSphereMean * oneCols;
	// Э�������
	Eigen::MatrixXf Matrix(3, 3);
	Matrix = camDeSphereMatrix * baseDeSphereMatrix.transpose();
	// SVD�ֽ�
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
	//�����f����
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


