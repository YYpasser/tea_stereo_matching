#include "../include/stereo.h"
#include "../include/utils.h"
#include "../include/logger.h"
#include <iostream>
#include <fstream>
#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>


class Logger : public nvinfer1::ILogger
{
	void log(Severity severity, const char* msg) noexcept override {
		LOG_INFO(msg);
	}
};


class stereo::TensorRTInference::TRTInferenceImpl
{
public:
	TRTInferenceImpl();
	~TRTInferenceImpl();
	/**
	 * @brief 读取TensorRT引擎文件.
	 * @param [in] file_path 引擎文件路径.
	 * @return 引擎文件内容.
	 */
	std::vector<char> readTRTFile(const std::string& file_path);
	/**
	 * @brief 计算张量的体积.
	 * @param [in] dims 张量的维度.
	 * @return 张量的体积.
	 */
	size_t tensorVolume(const nvinfer1::Dims& dims);
	/**
	 * @brief 分配内存.
	 */
	void allocateBuffers();
	/**
	 * @brief 推理.
	 */
	void infer();

	Logger logger;                        /*!< TensorRT日志记录器 */
	InputPadder padder;                   /*!< 立体匹配图像填充器 */
	nvinfer1::IRuntime* runtime;          /*!< TensorRT反序列化 */
	nvinfer1::ICudaEngine* engine;        /*!< TensorRT构建引擎 */
	nvinfer1::IExecutionContext* context; /*!< TensorRT执行推理 */

	void* deviceInputLeftTensor;  /*!< 设备端(GPU)输入左图张量, shape: [N, C, H, W] */
	void* deviceInputRightTensor; /*!< 设备端(GPU)输入右图张量, shape: [N, C, H, W] */
	void* deviceOutputDispTensor; /*!< 设备端(GPU)输出视差张量, shape: [N, C, H, W] */

	std::vector<float> hostInputLeftTensor;  /*!< 主机端(CPU)输入左图张量, shape: N*C*H*W */
	std::vector<float> hostInputRightTensor; /*!< 主机端(CPU)输入右图张量, shape: N*C*H*W */
	std::vector<float> hostOutputDispTensor; /*!< 主机端(CPU)输出视差张量, shape: N*C*H*W */
};

stereo::TensorRTInference::TensorRTInference()
{
	this->impl = std::make_unique<TRTInferenceImpl>();
}

stereo::TensorRTInference::~TensorRTInference()
{

}

void stereo::TensorRTInference::loadModel(const std::string& enginePath)
{
	LOG_INFO("Loading TensorRT engine from file: \"" + enginePath + "\"...");
	auto start = std::chrono::steady_clock::now();
	if (!this->impl->runtime) {
		LOG_ERROR("Failed to create TensorRT runtime.");
		return;
	}

	std::vector<char> trtFileContent = this->impl->readTRTFile(enginePath);
	if (trtFileContent.empty()) {
        LOG_ERROR("Failed to read TensorRT engine file.");
		return;
	}

	this->impl->engine = this->impl->runtime->deserializeCudaEngine(trtFileContent.data(), trtFileContent.size());
	if (!this->impl->engine) {
		LOG_ERROR("Failed to deserialize TensorRT engine.");
		return;
	}
	this->impl->context = this->impl->engine->createExecutionContext();
	if (this->impl->context) {
		this->impl->allocateBuffers();
	}
	auto end = std::chrono::steady_clock::now();
	auto tt = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	LOG_INFO("Engine Loaded. Timing: " + utils::formatMilliseconds(tt.count() / 1000.0) + " ms.");
}

void stereo::TensorRTInference::compute(const cv::Mat& leftImage, const cv::Mat& rightImage, cv::Mat& disparity)
{
	LOG_INFO("Computing disparity map using TensorRT...");
	auto start = std::chrono::steady_clock::now();
	
	// padding image size to be multiple of 32
	auto inputdata = this->impl->padder.pad({ leftImage,rightImage });
	
	// Data shape conversion: [H, W, C] -> [N, C, H, W]
	cv::Mat leftBlob = cv::dnn::blobFromImage(inputdata[0], 1.0, inputdata[0].size(), cv::Scalar(0, 0, 0), true, false);
	cv::Mat rightBlob = cv::dnn::blobFromImage(inputdata[1], 1.0, inputdata[1].size(), cv::Scalar(0, 0, 0), true, false);
	
	// Device data
	this->impl->hostInputLeftTensor.assign((float*)leftBlob.data, (float*)leftBlob.data + leftBlob.total());
	this->impl->hostInputRightTensor.assign((float*)rightBlob.data, (float*)rightBlob.data + rightBlob.total());

	// Stereo Matching End-to-End Model Inference
	this->impl->infer();

	// Convert output tensor to disparity map
	cv::Mat dispBlob(inputdata[0].rows, inputdata[0].cols, CV_32F, this->impl->hostOutputDispTensor.data());
	disparity = this->impl->padder.unpad(dispBlob);

	auto end = std::chrono::steady_clock::now();
	auto tt = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	LOG_INFO("Disparity map computed. Timing: " + utils::formatMilliseconds(tt.count() / 1000.0) + " ms.");
}

stereo::TensorRTInference::TRTInferenceImpl::TRTInferenceImpl()
{
	runtime = nvinfer1::createInferRuntime(logger);
	if (!runtime) {
		LOG_ERROR("Failed to create TensorRT runtime.");
	}
	this->engine = nullptr;
	this->context = nullptr;
	this->deviceInputLeftTensor = nullptr;
	this->deviceInputRightTensor = nullptr;
	this->deviceOutputDispTensor = nullptr;
}

stereo::TensorRTInference::TRTInferenceImpl::~TRTInferenceImpl()
{
	if (this->deviceInputLeftTensor) {
		cudaFree(this->deviceInputLeftTensor);
	}
	if (this->deviceInputRightTensor) {
		cudaFree(this->deviceInputRightTensor);
	}
	if (this->deviceOutputDispTensor) {
		cudaFree(this->deviceOutputDispTensor);
	}
	if (this->context) {
		delete this->context;
	}
	if (this->engine) {
		delete this->engine;
	}
	if (this->runtime) {
		delete this->runtime;
	}
}

std::vector<char> stereo::TensorRTInference::TRTInferenceImpl::readTRTFile(const std::string& file_path)
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

size_t stereo::TensorRTInference::TRTInferenceImpl::tensorVolume(const nvinfer1::Dims& dims)
{
	size_t volume = 1;
	for (int i = 0; i < dims.nbDims; ++i) {
		volume *= dims.d[i];
	}
	return volume;
}

void stereo::TensorRTInference::TRTInferenceImpl::allocateBuffers()
{
	const nvinfer1::ICudaEngine& engine = context->getEngine();

	// Get the dimensions of the input and output tensors
	size_t leftImgsz = tensorVolume(engine.getTensorShape(engine.getIOTensorName(0)));
	size_t rightImgsz = tensorVolume(engine.getTensorShape(engine.getIOTensorName(1)));
	size_t dispImgsz = tensorVolume(engine.getTensorShape(engine.getIOTensorName(2)));

	// Reserve host memory for input and output tensors
	this->hostInputLeftTensor.reserve(leftImgsz);
	this->hostInputRightTensor.reserve(rightImgsz);
	this->hostOutputDispTensor.reserve(dispImgsz);
	this->hostInputLeftTensor.resize(leftImgsz);
	this->hostInputRightTensor.resize(rightImgsz);
	this->hostOutputDispTensor.resize(dispImgsz);

	// Allocate device memory for input and output tensors
	cudaMalloc(&this->deviceInputLeftTensor, leftImgsz * sizeof(float));
	cudaMalloc(&this->deviceInputRightTensor, rightImgsz * sizeof(float));
	cudaMalloc(&this->deviceOutputDispTensor, dispImgsz * sizeof(float));
}

void stereo::TensorRTInference::TRTInferenceImpl::infer()
{
	// Copy input tensors to device memory
	cudaMemcpy(this->deviceInputLeftTensor, this->hostInputLeftTensor.data(), this->hostInputLeftTensor.size() * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(this->deviceInputRightTensor, this->hostInputRightTensor.data(), this->hostInputRightTensor.size() * sizeof(float), cudaMemcpyHostToDevice);

	// Execute inference
	void* bindings[] = { this->deviceInputLeftTensor, this->deviceInputRightTensor, this->deviceOutputDispTensor };
	context->executeV2(bindings);

	// Copy output tensor from device memory to host memory
	cudaMemcpy(this->hostOutputDispTensor.data(), this->deviceOutputDispTensor, this->hostOutputDispTensor.size() * sizeof(float), cudaMemcpyDeviceToHost);
}
