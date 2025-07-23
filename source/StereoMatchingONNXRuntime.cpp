#include "../include/stereo.h"
#include "../include/logger.h"
#include "../include/stereo_utils.h"
#include "../include/utils.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

class stereo::StereoMatchingONNXRuntime::SMONNXImpl
{
public:
	SMONNXImpl();
	~SMONNXImpl();
public:
	Ort::Env m_ortEnv = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_ERROR, "StereoMatchingNet");
	Ort::Session* m_ortSession = nullptr;
	InputPadder padder;

	std::vector<std::string> inputNames;
	std::vector<std::string> outputNames;
	std::vector<std::vector<int64_t>> inputShapes;
	std::vector<std::vector<int64_t>> outputShapes;
};

stereo::StereoMatchingONNXRuntime::StereoMatchingONNXRuntime()
{
	this->impl = std::make_unique<SMONNXImpl>();
}

stereo::StereoMatchingONNXRuntime::~StereoMatchingONNXRuntime()
{
}

void stereo::StereoMatchingONNXRuntime::loadModel(const std::string& modelPath)
{
	LOG_INFO("Loading onnx model from \"" + modelPath + "\"...");
	auto start = std::chrono::steady_clock::now();
	OrtCUDAProviderOptions cudaOption;
	cudaOption.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchDefault;/*!< 这个不能改 */
	cudaOption.device_id = 0;
	cudaOption.arena_extend_strategy = 1;/*!< 这个不能改 */
	cudaOption.gpu_mem_limit = SIZE_MAX;
	cudaOption.do_copy_in_default_stream = 1;
	Ort::SessionOptions ortSessionOptions;
	try
	{
		ortSessionOptions.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
		ortSessionOptions.AppendExecutionProvider_CUDA(cudaOption);
		std::wstring widestr = std::wstring(modelPath.begin(), modelPath.end());
		this->impl->m_ortSession = new Ort::Session(this->impl->m_ortEnv, widestr.c_str(), ortSessionOptions);
	}
	catch (const Ort::Exception& e)
	{
		std::string errstr = std::string(e.what());
		LOG_ERROR(errstr);
		throw std::runtime_error(errstr);
	}
	auto end = std::chrono::steady_clock::now();
	auto tt = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	LOG_INFO("Model Loaded. Timing: " + utils::formatMilliseconds(tt.count() / 1000.0) + " ms.");
	
	try {
		size_t numInputs = this->impl->m_ortSession->GetInputCount();
		this->impl->inputNames.resize(numInputs);
		this->impl->inputShapes.resize(numInputs);

		for (size_t i = 0; i < numInputs; i++) {
			Ort::AllocatorWithDefaultOptions allocator;
			this->impl->inputNames[i] = this->impl->m_ortSession->GetInputNameAllocated(i, allocator).get();
			Ort::TypeInfo typeInfo = this->impl->m_ortSession->GetInputTypeInfo(i);
			auto tensorInfo = typeInfo.GetTensorTypeAndShapeInfo();
			this->impl->inputShapes[i] = tensorInfo.GetShape();
		}

		size_t numOutputs = this->impl->m_ortSession->GetOutputCount();
		this->impl->outputNames.resize(numOutputs);
		this->impl->outputShapes.resize(numOutputs);

		for (size_t i = 0; i < numOutputs; i++) {
			Ort::AllocatorWithDefaultOptions allocator;
			this->impl->outputNames[i] = this->impl->m_ortSession->GetOutputNameAllocated(i, allocator).get();
			Ort::TypeInfo typeInfo = this->impl->m_ortSession->GetOutputTypeInfo(i);
			auto tensorInfo = typeInfo.GetTensorTypeAndShapeInfo();
			this->impl->outputShapes[i] = tensorInfo.GetShape();
		}
	}
	catch (const std::exception& e) {
		LOG_ERROR("Failed to retrieve model metadata: " + std::string(e.what()));
		throw std::runtime_error("ONNX model metadata extraction failed");
	}

}

cv::Mat stereo::StereoMatchingONNXRuntime::compute(const cv::Mat& leftImage, const cv::Mat& rightImage)
{
	LOG_INFO("Computing disparity map using ONNXRuntime...");
	auto start = std::chrono::steady_clock::now();
	if (leftImage.empty() or rightImage.empty() or leftImage.size() != rightImage.size()
		or leftImage.type() != CV_8UC3 or rightImage.type() != CV_8UC3)
	{
		std::string errstr = "Invalid input images.";
		LOG_ERROR(errstr);
		throw std::runtime_error(errstr);
	}

	cv::Mat left = leftImage.clone();
	cv::Mat right = rightImage.clone();
	cv::Mat disp = cv::Mat::zeros(left.size(), CV_32FC1);
	auto inputdata = this->impl->padder.pad({ left,right });
	//Mat[H, W, C] -> Tensor[B, C, H, W]
	cv::Mat leftBlob = cv::dnn::blobFromImage(inputdata[0], 1.0, inputdata[0].size(), cv::Scalar(0, 0, 0), true, false);
	cv::Mat rightBlob = cv::dnn::blobFromImage(inputdata[1], 1.0, inputdata[1].size(), cv::Scalar(0, 0, 0), true, false);
	std::vector<int64_t> inputTensorShape = { 1, 3, inputdata[0].rows, inputdata[0].cols };
	size_t inputTensorSize = 3 * inputdata[0].rows * inputdata[0].cols;
	Ort::MemoryInfo ortMemoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtDeviceAllocator, OrtMemType::OrtMemTypeDefault);
	Ort::Value leftTensor = Ort::Value::CreateTensor<float>(ortMemoryInfo, (float*)leftBlob.data, inputTensorSize, inputTensorShape.data(), inputTensorShape.size());
	Ort::Value rightTensor = Ort::Value::CreateTensor<float>(ortMemoryInfo, (float*)rightBlob.data, inputTensorSize, inputTensorShape.data(), inputTensorShape.size());
	std::vector<Ort::Value> ortInputs;
	ortInputs.push_back(std::move(leftTensor));
	ortInputs.push_back(std::move(rightTensor));
	std::vector<const char*> input_node_names = { this->impl->inputNames[0].c_str(), this->impl->inputNames[1].c_str() };
	std::vector<const char*> output_node_names = { this->impl->outputNames[0].c_str() };
	std::vector<Ort::Value> dispTensors =
		this->impl->m_ortSession->Run(
			Ort::RunOptions{ nullptr },
			input_node_names.data(),
			ortInputs.data(),
			ortInputs.size(),
			output_node_names.data(),
			output_node_names.size());
	float* floatarr = dispTensors[0].GetTensorMutableData<float>();
	disp = cv::Mat(left.size(), CV_32FC1, floatarr);
	auto end = std::chrono::steady_clock::now();
	auto tt = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	LOG_INFO("Disparity map computed. Timing: " + utils::formatMilliseconds(tt.count() / 1000.0) + " ms.");
	return this->impl->padder.unpad(disp);
}

std::vector<cv::Mat> stereo::StereoMatchingONNXRuntime::compute(const std::vector<cv::Mat>& leftImages, const std::vector<cv::Mat>& rightImages)
{
	LOG_INFO("Computing disparity maps...");
	if (leftImages.size() != rightImages.size())
	{
		std::string errstr = "Input images must be the same size.";
		LOG_ERROR(errstr);
		throw std::runtime_error(errstr);
	}

	// Padding
	std::vector<cv::Mat> leftPad, rightPad;
	for (int i = 0; i < leftImages.size(); ++i)
	{
		cv::Mat left = leftImages[i].clone();
		cv::Mat right = rightImages[i].clone();
		auto inputdata = this->impl->padder.pad({ left,right });
		leftPad.push_back(inputdata[0].clone());
		rightPad.push_back(inputdata[1].clone());
	}

	// Mat[B, H, W, C] -> Tensor[B, C, H, W]
	cv::Mat leftBlob = cv::dnn::blobFromImages(leftPad, 1.0, leftPad[0].size(), cv::Scalar(0, 0, 0), true, false);
	cv::Mat rightBlob = cv::dnn::blobFromImages(rightPad, 1.0, rightPad[0].size(), cv::Scalar(0, 0, 0), true, false);
	int numImages = leftPad.size();
	int channels = 3;
	int imageRows = leftPad[0].rows;
	int imageCols = rightPad[0].cols;
	std::vector<int64_t> inputTensorShape = { numImages, channels, imageRows, imageCols };
	size_t inputTensorSize = numImages * channels * imageRows * imageCols;
	Ort::MemoryInfo ortMemoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtDeviceAllocator, OrtMemType::OrtMemTypeDefault);
	Ort::Value leftTensor = Ort::Value::CreateTensor<float>(ortMemoryInfo, (float*)leftBlob.data, inputTensorSize, inputTensorShape.data(), inputTensorShape.size());
	Ort::Value rightTensor = Ort::Value::CreateTensor<float>(ortMemoryInfo, (float*)rightBlob.data, inputTensorSize, inputTensorShape.data(), inputTensorShape.size());

	std::vector<Ort::Value> ortInputs;
	ortInputs.push_back(std::move(leftTensor));
	ortInputs.push_back(std::move(rightTensor));
	std::vector<const char*> input_node_names = { this->impl->inputNames[0].c_str(),this->impl->inputNames[1].c_str() };
	std::vector<const char*> output_node_names = { this->impl->outputNames[0].c_str()};

	std::vector<Ort::Value> dispTensors =
		this->impl->m_ortSession->Run(
			Ort::RunOptions{ nullptr },
			input_node_names.data(),
			ortInputs.data(),
			ortInputs.size(),
			output_node_names.data(),
			output_node_names.size());

	// Tensor[B, C, H, W] -> Mat[B, H, W, C] & unpadding
	float* floatarr = dispTensors[0].GetTensorMutableData<float>();
	int mat_size = imageRows * imageCols;
	std::vector<cv::Mat> disparities;
	for (int i = 0; i < numImages; ++i)
	{
		float* mat_data = floatarr + i * mat_size;
		cv::Mat disp(leftPad[i].size(), CV_32FC1, mat_data);
		disparities.push_back(this->impl->padder.unpad(disp).clone());
	}
	return disparities;
}

stereo::StereoMatchingONNXRuntime::SMONNXImpl::SMONNXImpl()
{
}

stereo::StereoMatchingONNXRuntime::SMONNXImpl::~SMONNXImpl()
{
	if (this->m_ortSession != nullptr)
		delete this->m_ortSession;
}
