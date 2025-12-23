#include "../include/stereo.h"
#include "../include/logger.h"
#include "../include/utils.h"
#include <fstream>
#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>

// 简单的日志记录器类
class Logger : public nvinfer1::ILogger
{
	void log(Severity severity, const char* msg) noexcept override {
		LOG_INFO(msg);
		//if (severity != Severity::kINFO) {
		//	
		//	std::cerr << "TensorRT Logger: " << msg << std::endl;
		//}
	}
};

class stereo::StereoMatchingTensorRT::SMTRTImpl
{
public:
	SMTRTImpl();
	~SMTRTImpl();
public:
	std::vector<char> readTrtFile(const std::string& file_path);
	size_t tensorVolume(const nvinfer1::Dims& dims);
	void allocateBuffers();
	void doInference();
public:
	Logger logger;
	InputPadder padder;
	nvinfer1::IRuntime* runtime;
	nvinfer1::ICudaEngine* engine;
	nvinfer1::IExecutionContext* context;
	void* d_input1;
	void* d_input2;
	void* d_output;
	std::vector<float> h_input1, h_input2, h_output;
};

stereo::StereoMatchingTensorRT::StereoMatchingTensorRT()
{
	this->impl = std::make_unique<SMTRTImpl>();
}

stereo::StereoMatchingTensorRT::StereoMatchingTensorRT(const std::string& enginePath)
{
	this->impl = std::make_unique<SMTRTImpl>();
	this->loadEngine(enginePath);
}

stereo::StereoMatchingTensorRT::~StereoMatchingTensorRT()
{
}

bool stereo::StereoMatchingTensorRT::loadEngine(const std::string& enginePath)
{
	LOG_INFO("Loading TensorRT engine from file: \"" + enginePath + "\"...");
	auto start = std::chrono::steady_clock::now();
	if (!this->impl->runtime) {
		return false;
	}

	std::vector<char> trt_file_content = this->impl->readTrtFile(enginePath);
	if (trt_file_content.empty()) {
		return false;
	}

	this->impl->engine = this->impl->runtime->deserializeCudaEngine(trt_file_content.data(), trt_file_content.size());
	if (!this->impl->engine) {
		LOG_ERROR("Failed to deserialize TensorRT engine.");
		return false;
	}
	this->impl->context = this->impl->engine->createExecutionContext();
	if (this->impl->context) {
		this->impl->allocateBuffers();
	}
	auto end = std::chrono::steady_clock::now();
	auto tt = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	LOG_INFO("Engine Loaded. Timing: " + utils::formatMilliseconds(tt.count() / 1000.0) + " ms.");
	return true;
}

cv::Mat stereo::StereoMatchingTensorRT::compute(const cv::Mat& leftImage, const cv::Mat& rightImage)
{
	LOG_INFO("Computing disparity map using TensorRT...");
	auto start = std::chrono::steady_clock::now();
	auto inputdata = this->impl->padder.pad({ leftImage,rightImage });
	cv::Mat leftBlob = cv::dnn::blobFromImage(inputdata[0], 1.0, inputdata[0].size(), cv::Scalar(0, 0, 0), true, false);
	cv::Mat rightBlob = cv::dnn::blobFromImage(inputdata[1], 1.0, inputdata[1].size(), cv::Scalar(0, 0, 0), true, false);
	this->impl->h_input1.assign((float*)leftBlob.data, (float*)leftBlob.data + leftBlob.total());
	this->impl->h_input2.assign((float*)rightBlob.data, (float*)rightBlob.data + rightBlob.total());

	this->impl->doInference();

	cv::Mat dispBlob(inputdata[0].rows, inputdata[0].cols, CV_32F, this->impl->h_output.data());
	auto end = std::chrono::steady_clock::now();
	auto tt = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	LOG_INFO("Disparity map computed. Timing: " + utils::formatMilliseconds(tt.count() / 1000.0) + " ms.");
	return this->impl->padder.unpad(dispBlob);
}

stereo::StereoMatchingTensorRT::SMTRTImpl::SMTRTImpl()
{
	runtime = nvinfer1::createInferRuntime(logger);
	if (!runtime) {
		LOG_ERROR("Failed to create TensorRT runtime.");
	}
	this->engine = nullptr;
	this->context = nullptr;
	this->d_input1 = nullptr;
	this->d_input2 = nullptr;
	this->d_output = nullptr;
}

stereo::StereoMatchingTensorRT::SMTRTImpl::~SMTRTImpl()
{
	if (d_input1) {
		cudaFree(d_input1);
	}
	if (d_input2) {
		cudaFree(d_input2);
	}
	if (d_output) {
		cudaFree(d_output);
	}
	if (context) {
		delete context;
	}
	if (engine) {
		delete engine;
	}
	if (runtime) {
		delete runtime;
	}
}

std::vector<char> stereo::StereoMatchingTensorRT::SMTRTImpl::readTrtFile(const std::string& file_path)
{
	std::ifstream file(file_path, std::ios::binary);
	if (!file.good()) {
		LOG_ERROR("Failed to open file: " + file_path);
		return {};
	}
	file.seekg(0, std::ios::end);
	size_t size = file.tellg();
	file.seekg(0, std::ios::beg);
	std::vector<char> buffer(size);
	file.read(buffer.data(), size);
	file.close();
	return buffer;
}

size_t stereo::StereoMatchingTensorRT::SMTRTImpl::tensorVolume(const nvinfer1::Dims& dims)
{
	size_t volume = 1;
	for (int i = 0; i < dims.nbDims; ++i)
	{
		volume *= dims.d[i];
	}
	return volume;
}

void stereo::StereoMatchingTensorRT::SMTRTImpl::allocateBuffers()
{
	const nvinfer1::ICudaEngine& engine = context->getEngine();
	size_t leftImgsz = tensorVolume(engine.getTensorShape(engine.getIOTensorName(0)));
	size_t rightImgsz = tensorVolume(engine.getTensorShape(engine.getIOTensorName(1)));
	size_t dispImgsz = tensorVolume(engine.getTensorShape(engine.getIOTensorName(2)));

	h_input1.resize(leftImgsz);
	h_input2.resize(rightImgsz);
	h_output.resize(dispImgsz);

	cudaMalloc(&d_input1, leftImgsz * sizeof(float));
	cudaMalloc(&d_input2, rightImgsz * sizeof(float));
	cudaMalloc(&d_output, dispImgsz * sizeof(float));
}

void stereo::StereoMatchingTensorRT::SMTRTImpl::doInference()
{
	cudaMemcpy(d_input1, h_input1.data(), h_input1.size() * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(d_input2, h_input2.data(), h_input2.size() * sizeof(float), cudaMemcpyHostToDevice);

	void* bindings[] = { d_input1, d_input2, d_output };
	context->executeV2(bindings);

	cudaMemcpy(h_output.data(), d_output, h_output.size() * sizeof(float), cudaMemcpyDeviceToHost);
}
