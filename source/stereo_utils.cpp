#include "../include/stereo_utils.h"
#include "../include/logger.h"
#include <opencv2/opencv.hpp>

EpipolarRectifyMap::EpipolarRectifyMap() : R1(cv::Mat()), R2(cv::Mat()), P1(cv::Mat()), P2(cv::Mat()), map00(cv::Mat()), map01(cv::Mat()), map10(cv::Mat()), map11(cv::Mat())
{
}

EpipolarRectifyMap::EpipolarRectifyMap(const cv::Mat& R1, const cv::Mat& R2, const cv::Mat& P1, const cv::Mat& P2, const cv::Mat& map00, const cv::Mat& map01, const cv::Mat& map10, const cv::Mat& map11) : R1(R1), R2(R2), P1(P1), P2(P2), map00(map00), map01(map01), map10(map10), map11(map11)
{
}

EpipolarRectifyMap::EpipolarRectifyMap(const EpipolarRectifyMap& other) : R1(other.R1), R2(other.R2), P1(other.P1), P2(other.P2), map00(other.map00), map01(other.map01), map10(other.map10), map11(other.map11)
{
}

EpipolarRectifyMap::EpipolarRectifyMap(EpipolarRectifyMap&& other) noexcept : R1(std::move(other.R1)), R2(std::move(other.R2)), P1(std::move(other.P1)), P2(std::move(other.P2)), map00(std::move(other.map00)), map01(std::move(other.map01)), map10(std::move(other.map10)), map11(std::move(other.map11))
{
}

EpipolarRectifyMap& EpipolarRectifyMap::operator=(const EpipolarRectifyMap& other)
{
	if (this != &other) {
		R1 = other.R1;
		R2 = other.R2;
		P1 = other.P1;
		P2 = other.P2;
		map00 = other.map00;
		map01 = other.map01;
		map10 = other.map10;
		map11 = other.map11;
	}
	return *this;
}

EpipolarRectifyMap& EpipolarRectifyMap::operator=(EpipolarRectifyMap&& other) noexcept
{
	if (this != &other) {
		R1 = std::move(other.R1);
		R2 = std::move(other.R2);
		P1 = std::move(other.P1);
		P2 = std::move(other.P2);
		map00 = std::move(other.map00);
		map01 = std::move(other.map01);
		map10 = std::move(other.map10);
		map11 = std::move(other.map11);
	}
	return *this;
}

void EpipolarRectifyMap::loadRectifyMapsYMLFile(const std::string& ymlFilePath)
{
	cv::FileStorage fs;
	fs.open(ymlFilePath, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		fs.release();
		std::string errstr = "Cannot open rectify maps yml file.";
		LOG_ERROR(errstr);
		throw std::runtime_error(errstr);
	}
	fs["map00"] >> this->map00;
	fs["map01"] >> this->map01;
	fs["map10"] >> this->map10;
	fs["map11"] >> this->map11;
	fs.release();
	if (this->empty()) {
		LOG_ERROR("Cannot load rectify maps from yml file.");
	}
	else {
        LOG_INFO("Loaded rectify maps from yml file.");
	}
}

void EpipolarRectifyMap::compute(const StereoPair<CameraIntrinsic>& intrinsic, const cv::Size& imgsz)
{
	if (intrinsic.left.empty() || intrinsic.right.empty())
		return;

	LOG_INFO("Computing stereo rectify reproject maps...");
	cv::initUndistortRectifyMap(intrinsic.left.intrinsic_matrix, intrinsic.left.distortion_coefficients,
		this->R1, this->P1, imgsz, CV_16SC2, this->map00, this->map01);
	cv::initUndistortRectifyMap(intrinsic.right.intrinsic_matrix, intrinsic.right.distortion_coefficients,
		this->R2, this->P2, imgsz, CV_16SC2, this->map10, this->map11);
	LOG_INFO("Computed stereo rectify reproject maps!");
}

bool EpipolarRectifyMap::empty() const
{
	return map00.empty() || map01.empty() || map10.empty() || map11.empty();
}

void ADCensusParams::setADCensusParams(const ColorModel& colorModel)
{
	this->lambdaAD = 10.f;
	this->censusWin = CensusWin::CENSUSWIN_9x7;
	this->lambdaCensus = 30.f;
	this->lambdaHue = 1.f;
	this->lambdaSaturation = 2.5f;
	this->lambdaIntensity = 2.5f;
	this->iterations = 4;
	this->pi1 = 1.f;
	this->pi2 = 3.f;
	this->dispTolerance = 0;
	this->votingThresh = 20;
	this->votingRatioThresh = 0.4f;
	this->maxSearchDepth = 20;
	this->blurKernelSize = 3;
	this->cannyThresh1 = 30;
	this->cannyThresh2 = 90;
	this->cannyKernelSize = 3;
	switch (colorModel)
	{
	case ColorModel::HSI:
		this->colorThresh1 = 5;
		this->colorThresh2 = 1;
		this->maxLength1 = 17;
		this->maxLength2 = 8;
		this->colorDiff = 3;
		this->saturationThresh1 = 10;
		this->saturationThresh2 = 2;
		this->intensityThresh1 = 12;
		this->intensityThresh2 = 3;
		break;
	case ColorModel::RGB:
		this->colorThresh1 = 20;
		this->colorThresh2 = 6;
		this->maxLength1 = 34;
		this->maxLength2 = 17;
		this->colorDiff = 15;
		this->saturationThresh1 = NULL;
		this->saturationThresh2 = NULL;
		this->intensityThresh1 = NULL;
		this->intensityThresh2 = NULL;
		break;
	default:
		this->colorThresh1 = 5;
		this->colorThresh2 = 1;
		this->maxLength1 = 17;
		this->maxLength2 = 8;
		this->colorDiff = 3;
		this->saturationThresh1 = 10;
		this->saturationThresh2 = 2;
		this->intensityThresh1 = 12;
		this->intensityThresh2 = 3;
		break;
	}
}

CameraIntrinsic::CameraIntrinsic() : intrinsic_matrix(cv::Mat()), distortion_coefficients(cv::Mat())
{
}

CameraIntrinsic::CameraIntrinsic(const cv::Mat& intrinsic_matrix, const cv::Mat& distortion_coefficients) : intrinsic_matrix(intrinsic_matrix), distortion_coefficients(distortion_coefficients)
{
}

CameraIntrinsic::CameraIntrinsic(const CameraIntrinsic& other) : intrinsic_matrix(other.intrinsic_matrix), distortion_coefficients(other.distortion_coefficients)
{
}

CameraIntrinsic::CameraIntrinsic(CameraIntrinsic&& other) noexcept : intrinsic_matrix(std::move(other.intrinsic_matrix)), distortion_coefficients(std::move(other.distortion_coefficients))
{
}

CameraIntrinsic& CameraIntrinsic::operator=(const CameraIntrinsic& other)
{
	if (this != &other) {
        this->intrinsic_matrix = other.intrinsic_matrix;
        this->distortion_coefficients = other.distortion_coefficients;
	}
    return *this;
}

CameraIntrinsic& CameraIntrinsic::operator=(CameraIntrinsic&& other) noexcept
{
	if (this != &other) {
		this->intrinsic_matrix = std::move(other.intrinsic_matrix);
		this->distortion_coefficients = std::move(other.distortion_coefficients);
	}
    return *this;
}

bool CameraIntrinsic::empty() const
{   	
	return this->intrinsic_matrix.empty() || this->distortion_coefficients.empty();
}

StereoExtrinsic::StereoExtrinsic() : R(cv::Mat()), T(cv::Mat()), E(cv::Mat()), F(cv::Mat())
{
}

StereoExtrinsic::StereoExtrinsic(const cv::Mat& R, const cv::Mat& T, const cv::Mat& E, const cv::Mat& F) : R(R), T(T), E(E), F(F)
{
}

StereoExtrinsic::StereoExtrinsic(const StereoExtrinsic& other) : R(other.R), T(other.T), E(other.E), F(other.F)
{
}

StereoExtrinsic::StereoExtrinsic(StereoExtrinsic&& other) noexcept : R(std::move(other.R)), T(std::move(other.T)), E(std::move(other.E)), F(std::move(other.F))
{
}

StereoExtrinsic& StereoExtrinsic::operator=(const StereoExtrinsic& other)
{
	if (this != &other) {
		this->R = other.R;
		this->T = other.T;
		this->E = other.E;
		this->F = other.F;
	}
    return *this;
}

StereoExtrinsic& StereoExtrinsic::operator=(StereoExtrinsic&& other) noexcept
{
	if (this != &other) {
		this->R = std::move(other.R);
		this->T = std::move(other.T);
		this->E = std::move(other.E);
		this->F = std::move(other.F);
	}
	return *this;
}

bool StereoExtrinsic::empty() const
{ 	
	return this->R.empty() || this->T.empty() || this->E.empty() || this->F.empty();
}

StereoParams::StereoParams() : intrinsic(StereoPair<CameraIntrinsic>()), extrinsic(StereoExtrinsic()), map(EpipolarRectifyMap()), Q(cv::Mat()), rectified_f(0.f), rectified_cx(0.f), rectified_cy(0.f), baseline(0.f), imgsz(cv::Size())
{
}

StereoParams::StereoParams(const std::string& ymlFilePath)
{
	loadYAMLFile(ymlFilePath);
}

void StereoParams::loadYAMLFile(const std::string& ymlFilePath)
{
	if (ymlFilePath.empty())
	{
		std::string errstr = "Stereo YAML file path is empty.";
		LOG_ERROR(errstr);
		throw std::invalid_argument(errstr);
	}

	LOG_INFO("Loading Stereo YAML file...");
	cv::FileStorage fs;
	fs.open(ymlFilePath, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		fs.release();
		std::string errstr = "Cannot open stereo yml file.";
		LOG_ERROR(errstr);
		throw std::runtime_error(errstr);
	}
	fs["leftK"] >> this->intrinsic.left.intrinsic_matrix;
	fs["leftD"] >> this->intrinsic.left.distortion_coefficients;
	fs["rightK"] >> this->intrinsic.right.intrinsic_matrix;
	fs["rightD"] >> this->intrinsic.right.distortion_coefficients;
	fs["E"] >> this->extrinsic.E;
	fs["F"] >> this->extrinsic.F;
	fs["R"] >> this->extrinsic.R;
	fs["T"] >> this->extrinsic.T;
	fs["R1"] >> this->map.R1;
	fs["R2"] >> this->map.R2;
	fs["P1"] >> this->map.P1;
	fs["P2"] >> this->map.P2;
	fs["Q"] >> this->Q;
	fs["imgsz"] >> this->imgsz;
	fs.release();
	LOG_INFO("Loaded Stereo YAML file.");

	if (Q.empty())
		return;

	this->rectified_f = static_cast<float>(Q.at<double>(2, 3));
	this->rectified_cx = static_cast<float>(-Q.at<double>(0, 3));
	this->rectified_cy = static_cast<float>(-Q.at<double>(1, 3));
	this->baseline = 1.f / static_cast<float>(Q.at<double>(3, 2));

	this->map.compute(intrinsic, imgsz);

	print();
}

bool StereoParams::empty() const
{
	return intrinsic.left.empty() || intrinsic.right.empty()
		|| extrinsic.empty() || map.empty() || Q.empty();
}

void StereoParams::print() const
{
	if (empty())
	{
		LOG_ERROR("Stereo params is empty!");
		return;
	}
	LOG_INFO_MSG("------------------------- Intrinsic -------------------------");
	LOG_INFO_MSG_MAT("Left:", intrinsic.left.intrinsic_matrix);
	LOG_INFO_MSG_MAT("Right:", intrinsic.right.intrinsic_matrix);
	LOG_INFO_MSG("------------------------- Extrinsic -------------------------");
	LOG_INFO_MSG_MAT("R:", extrinsic.R);
	LOG_INFO_MSG_MAT("T:", extrinsic.T);
	LOG_INFO_MSG_MAT("E:", extrinsic.E);
	LOG_INFO_MSG_MAT("F:", extrinsic.F);
	LOG_INFO_MSG("-------------------------  Rectify  -------------------------");
	LOG_INFO_MSG_MAT("R1:", map.R1);
	LOG_INFO_MSG_MAT("R2:", map.R2);
	LOG_INFO_MSG_MAT("P1:", map.P1);
	LOG_INFO_MSG_MAT("P2:", map.P2);
	LOG_INFO_MSG("------------------------- Reproject -------------------------");
	LOG_INFO_MSG_MAT("Q:", Q);
	LOG_INFO_MSG("--------------------------- Other ---------------------------");
	LOG_INFO_MSG("imgsz: " + std::to_string(imgsz.width) + "x" + std::to_string(imgsz.height));
	LOG_INFO_MSG("rectified_f: " + std::to_string(rectified_f));
	LOG_INFO_MSG("rectified_cx: " + std::to_string(rectified_cx));
	LOG_INFO_MSG("rectified_cy: " + std::to_string(rectified_cy));
	LOG_INFO_MSG("baseline: " + std::to_string(baseline));
	LOG_INFO_MSG("-------------------------------------------------------------");
}
