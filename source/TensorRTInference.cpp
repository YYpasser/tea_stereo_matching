//#include "../include/stereo.h"
//#include "../include/utils.h"
//#include "../include/logger.h"
//#include <iostream>
//#include <fstream>
//#include <NvInfer.h>
//#include <NvInferRuntime.h>
//#include <cuda_runtime.h>
//#include <opencv2/opencv.hpp>
//
//
//class Logger : public nvinfer1::ILogger
//{
//	void log(Severity severity, const char* msg) noexcept override {
//		LOG_INFO(msg);
//	}
//};
//
//
//class stereo::TensorRTInference::TRTInferenceImpl
//{
//public:
//	TRTInferenceImpl();
//	~TRTInferenceImpl();
//	/**
//	 * @brief 读取TensorRT引擎文件.
//	 * @param [in] file_path 引擎文件路径.
//	 * @return 引擎文件内容.
//	 */
//	std::vector<char> readTRTFile(const std::string& file_path);
//	/**
//	 * @brief 计算张量的体积.
//	 * @param [in] dims 张量的维度.
//	 * @return 张量的体积.
//	 */
//	size_t tensorVolume(const nvinfer1::Dims& dims);
//	/**
//	 * @brief 分配内存.
//	 */
//	void allocateBuffers();
//	/**
//	 * @brief 推理.
//	 */
//	void infer();
//
//	Logger logger;                        /*!< TensorRT日志记录器 */
//	InputPadder padder;                   /*!< 立体匹配图像填充器 */
//	nvinfer1::IRuntime* runtime;          /*!< TensorRT反序列化 */
//	nvinfer1::ICudaEngine* engine;        /*!< TensorRT构建引擎 */
//	nvinfer1::IExecutionContext* context; /*!< TensorRT执行推理 */
//
//	void* deviceInputLeftTensor;  /*!< 设备端(GPU)输入左图张量, shape: [N, C, H, W] */
//	void* deviceInputRightTensor; /*!< 设备端(GPU)输入右图张量, shape: [N, C, H, W] */
//	void* deviceOutputDispTensor; /*!< 设备端(GPU)输出视差张量, shape: [N, C, H, W] */
//
//	std::vector<float> hostInputLeftTensor;  /*!< 主机端(CPU)输入左图张量, shape: N*C*H*W */
//	std::vector<float> hostInputRightTensor; /*!< 主机端(CPU)输入右图张量, shape: N*C*H*W */
//	std::vector<float> hostOutputDispTensor; /*!< 主机端(CPU)输出视差张量, shape: N*C*H*W */
//};
//
//stereo::TensorRTInference::TensorRTInference()
//{
//	this->impl = std::make_unique<TRTInferenceImpl>();
//}
//
//stereo::TensorRTInference::~TensorRTInference()
//{
//
//}
//
//void stereo::TensorRTInference::loadModel(const std::string& enginePath)
//{
//	LOG_INFO("Loading TensorRT engine from file: \"" + enginePath + "\"...");
//	auto start = std::chrono::steady_clock::now();
//	if (!this->impl->runtime) {
//		LOG_ERROR("Failed to create TensorRT runtime.");
//		return;
//	}
//
//	std::vector<char> trtFileContent = this->impl->readTRTFile(enginePath);
//	if (trtFileContent.empty()) {
//        LOG_ERROR("Failed to read TensorRT engine file.");
//		return;
//	}
//
//	this->impl->engine = this->impl->runtime->deserializeCudaEngine(trtFileContent.data(), trtFileContent.size());
//	if (!this->impl->engine) {
//		LOG_ERROR("Failed to deserialize TensorRT engine.");
//		return;
//	}
//	this->impl->context = this->impl->engine->createExecutionContext();
//	if (this->impl->context) {
//		this->impl->allocateBuffers();
//	}
//	auto end = std::chrono::steady_clock::now();
//	auto tt = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
//	LOG_INFO("Engine Loaded. Timing: " + utils::formatMilliseconds(tt.count() / 1000.0) + " ms.");
//}
//
//void stereo::TensorRTInference::compute(const cv::Mat& leftImage, const cv::Mat& rightImage, cv::Mat& disparity)
//{
//	LOG_INFO("Computing disparity map using TensorRT...");
//	auto start = std::chrono::steady_clock::now();
//	
//	// padding image size to be multiple of 32
//	auto inputdata = this->impl->padder.pad({ leftImage,rightImage });
//	
//	// Data shape conversion: [H, W, C] -> [N, C, H, W]
//	cv::Mat leftBlob = cv::dnn::blobFromImage(inputdata[0], 1.0, inputdata[0].size(), cv::Scalar(0, 0, 0), true, false);
//	cv::Mat rightBlob = cv::dnn::blobFromImage(inputdata[1], 1.0, inputdata[1].size(), cv::Scalar(0, 0, 0), true, false);
//	
//	// Device data
//	this->impl->hostInputLeftTensor.assign((float*)leftBlob.data, (float*)leftBlob.data + leftBlob.total());
//	this->impl->hostInputRightTensor.assign((float*)rightBlob.data, (float*)rightBlob.data + rightBlob.total());
//
//	// Stereo Matching End-to-End Model Inference
//	this->impl->infer();
//
//	// Convert output tensor to disparity map
//	cv::Mat dispBlob(inputdata[0].rows, inputdata[0].cols, CV_32F, this->impl->hostOutputDispTensor.data());
//	disparity = this->impl->padder.unpad(dispBlob);
//
//	auto end = std::chrono::steady_clock::now();
//	auto tt = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
//	LOG_INFO("Disparity map computed. Timing: " + utils::formatMilliseconds(tt.count() / 1000.0) + " ms.");
//}
//
//stereo::TensorRTInference::TRTInferenceImpl::TRTInferenceImpl()
//{
//	runtime = nvinfer1::createInferRuntime(logger);
//	if (!runtime) {
//		LOG_ERROR("Failed to create TensorRT runtime.");
//	}
//	this->engine = nullptr;
//	this->context = nullptr;
//	this->deviceInputLeftTensor = nullptr;
//	this->deviceInputRightTensor = nullptr;
//	this->deviceOutputDispTensor = nullptr;
//}
//
//stereo::TensorRTInference::TRTInferenceImpl::~TRTInferenceImpl()
//{
//	if (this->deviceInputLeftTensor) {
//		cudaFree(this->deviceInputLeftTensor);
//	}
//	if (this->deviceInputRightTensor) {
//		cudaFree(this->deviceInputRightTensor);
//	}
//	if (this->deviceOutputDispTensor) {
//		cudaFree(this->deviceOutputDispTensor);
//	}
//	if (this->context) {
//		delete this->context;
//	}
//	if (this->engine) {
//		delete this->engine;
//	}
//	if (this->runtime) {
//		delete this->runtime;
//	}
//}
//
//std::vector<char> stereo::TensorRTInference::TRTInferenceImpl::readTRTFile(const std::string& file_path)
//{
//	std::ifstream file(file_path, std::ios::binary);
//	if (!file.good()) {
//		LOG_ERROR("Failed to open file: " + file_path);
//		return {};
//	}
//	file.seekg(0, std::ios::end);
//	size_t size = file.tellg();
//	file.seekg(0, std::ios::beg);
//	std::vector<char> buffer(size);
//	file.read(buffer.data(), size);
//	file.close();
//	return buffer;
//}
//
//size_t stereo::TensorRTInference::TRTInferenceImpl::tensorVolume(const nvinfer1::Dims& dims)
//{
//	size_t volume = 1;
//	for (int i = 0; i < dims.nbDims; ++i) {
//		volume *= dims.d[i];
//	}
//	return volume;
//}
//
//void stereo::TensorRTInference::TRTInferenceImpl::allocateBuffers()
//{
//	const nvinfer1::ICudaEngine& engine = context->getEngine();
//
//	// Get the dimensions of the input and output tensors
//	size_t leftImgsz = tensorVolume(engine.getTensorShape(engine.getIOTensorName(0)));
//	size_t rightImgsz = tensorVolume(engine.getTensorShape(engine.getIOTensorName(1)));
//	size_t dispImgsz = tensorVolume(engine.getTensorShape(engine.getIOTensorName(2)));
//
//	// Reserve host memory for input and output tensors
//	this->hostInputLeftTensor.reserve(leftImgsz);
//	this->hostInputRightTensor.reserve(rightImgsz);
//	this->hostOutputDispTensor.reserve(dispImgsz);
//	this->hostInputLeftTensor.resize(leftImgsz);
//	this->hostInputRightTensor.resize(rightImgsz);
//	this->hostOutputDispTensor.resize(dispImgsz);
//
//	// Allocate device memory for input and output tensors
//	cudaMalloc(&this->deviceInputLeftTensor, leftImgsz * sizeof(float));
//	cudaMalloc(&this->deviceInputRightTensor, rightImgsz * sizeof(float));
//	cudaMalloc(&this->deviceOutputDispTensor, dispImgsz * sizeof(float));
//}
//
//void stereo::TensorRTInference::TRTInferenceImpl::infer()
//{
//	// Copy input tensors to device memory
//	cudaMemcpy(this->deviceInputLeftTensor, this->hostInputLeftTensor.data(), this->hostInputLeftTensor.size() * sizeof(float), cudaMemcpyHostToDevice);
//	cudaMemcpy(this->deviceInputRightTensor, this->hostInputRightTensor.data(), this->hostInputRightTensor.size() * sizeof(float), cudaMemcpyHostToDevice);
//
//	// Execute inference
//	void* bindings[] = { this->deviceInputLeftTensor, this->deviceInputRightTensor, this->deviceOutputDispTensor };
//	context->executeV2(bindings);
//
//	// Copy output tensor from device memory to host memory
//	cudaMemcpy(this->hostOutputDispTensor.data(), this->deviceOutputDispTensor, this->hostOutputDispTensor.size() * sizeof(float), cudaMemcpyDeviceToHost);
//}

#include "../include/stereo.h"
#include "../include/utils.h"
#include "../include/logger.h"

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include <string>
#include <chrono>

using namespace nvinfer1;


class Logger : public ILogger
{
public:
    void log(Severity severity, const char* msg) noexcept override
    {
        if (severity <= Severity::kINFO)
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
	 * @param [in] filePath 引擎文件路径
	 * @return 引擎文件内容
     */
    std::vector<char> readTRTFile(const std::string& filePath);
    /**
     * @brief 计算张量的体积.
     * @param [in] dims 张量维度
	 * @return 张量体积
     */
    size_t tensorVolume(const Dims& dims);
    /**
     * @brief 分配设备缓冲区.
     */
    void allocateBuffers();
    /**
     * @brief 执行推理.
     */
    void infer();

public:
    Logger m_logger;
    InputPadder m_padder;

    IRuntime* m_runtime{ nullptr };
    ICudaEngine* m_engine{ nullptr };
    IExecutionContext* m_context{ nullptr };

    cudaStream_t m_stream{};

    /* tensor names */
    std::string m_inputLeftName;
    std::string m_inputRightName;
    std::string m_outputDispName;

    /* device buffers */
    void* m_deviceInputLeftTensor{ nullptr };
    void* m_deviceInputRightTensor{ nullptr };
    void* m_deviceOutputDispTensor{ nullptr };

    /* host buffers */
    std::vector<float> m_hostInputLeftTensor;
    std::vector<float> m_hostInputRightTensor;
    std::vector<float> m_hostOutputDispTensor;

	/* current input dimensions */
    Dims m_currentInputLeftTensorDims{ Dims() };
    Dims m_currentInputRightTensorDims{ Dims() };
};


stereo::TensorRTInference::TensorRTInference()
{
    this->impl = std::make_unique<TRTInferenceImpl>();
}

stereo::TensorRTInference::~TensorRTInference() = default;

void stereo::TensorRTInference::loadModel(const std::string& enginePath)
{
    auto start = std::chrono::steady_clock::now();
    LOG_INFO("Loading TensorRT engine: " + enginePath);

    // Create TensorRT runtime
    if (!this->impl->m_runtime)
    {
        LOG_ERROR("TensorRT runtime is null.");
        return;
    }

    // Read engine file
    auto buffer = this->impl->readTRTFile(enginePath);
    if (buffer.empty())
    {
        LOG_ERROR("Failed to read engine file.");
        return;
    }

	// Deserialize engine
    this->impl->m_engine = this->impl->m_runtime->deserializeCudaEngine(buffer.data(), buffer.size());
    if (!this->impl->m_engine)
    {
        LOG_ERROR("Failed to deserialize engine.");
        return;
    }

	// Create execution context
    this->impl->m_context = impl->m_engine->createExecutionContext();
    if (!this->impl->m_context)
    {
        LOG_ERROR("Failed to create execution context.");
        return;
    }

	// Get tensor names
    const ICudaEngine& eng = this->impl->m_context->getEngine();
    this->impl->m_inputLeftName = eng.getIOTensorName(0);  // leftImage
    this->impl->m_inputRightName = eng.getIOTensorName(1); // rightImage
	this->impl->m_outputDispName = eng.getIOTensorName(2); // disparity

    auto end = std::chrono::steady_clock::now();
    LOG_INFO("Engine loaded in " + utils::formatMilliseconds(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()) + " ms");
}

void stereo::TensorRTInference::compute(const cv::Mat& leftImage, const cv::Mat& rightImage, cv::Mat& disparity)
{
    auto start = std::chrono::steady_clock::now();

	// Padding input images
    auto padded = impl->m_padder.pad({ leftImage, rightImage });
	// Data shape conversion: [H, W, C] -> [N, C, H, W] & [BGR] -> [RGB] & No normalization
    cv::Mat leftBlob = cv::dnn::blobFromImage( padded[0], 1.0, padded[0].size(), cv::Scalar(), true, false);
    cv::Mat rightBlob = cv::dnn::blobFromImage( padded[1], 1.0, padded[1].size(), cv::Scalar(), true, false);

	// Input tensor dimensions
    Dims inputLeftTensorDims = Dims4(1, 3, padded[0].rows, padded[0].cols);
    Dims inputRightTensorDims = Dims4(1, 3, padded[1].rows, padded[1].cols);
	// Update buffers if input size changes
    if (memcmp(&inputLeftTensorDims, &this->impl->m_currentInputLeftTensorDims, sizeof(nvinfer1::Dims)) != 0 ||
        memcmp(&inputRightTensorDims, &this->impl->m_currentInputRightTensorDims, sizeof(nvinfer1::Dims)) != 0)
    {
        // Update input tensor dimensions
        this->impl->m_context->setInputShape(this->impl->m_inputLeftName.c_str(), inputLeftTensorDims);
        this->impl->m_context->setInputShape(this->impl->m_inputRightName.c_str(), inputRightTensorDims);
		// Store current input tensor dimensions
        this->impl->m_currentInputLeftTensorDims = inputLeftTensorDims;
        this->impl->m_currentInputRightTensorDims = inputRightTensorDims;
        // Allocate device and host buffers
        this->impl->allocateBuffers();
    }

	// Copy input data to host buffers
    this->impl->m_hostInputLeftTensor.assign((float*)leftBlob.data, (float*)leftBlob.data + leftBlob.total());
    this->impl->m_hostInputRightTensor.assign((float*)rightBlob.data,(float*)rightBlob.data + rightBlob.total());

	// Stereo Matching Inference
    this->impl->infer();

	// Convert output tensor to disparity map
    cv::Mat dispBlob(padded[0].size(), CV_32F, this->impl->m_hostOutputDispTensor.data());

	// Unpadding disparity map
    disparity = this->impl->m_padder.unpad(dispBlob);

    auto end = std::chrono::steady_clock::now();
    LOG_INFO("Inference done in " + utils::formatMilliseconds(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()) + " ms");
}

stereo::TensorRTInference::TRTInferenceImpl::TRTInferenceImpl()
{
    this->m_runtime = createInferRuntime(this->m_logger);
    cudaStreamCreate(&this->m_stream);
}

stereo::TensorRTInference::TRTInferenceImpl::~TRTInferenceImpl()
{
    // Free device buffers
    if (this->m_deviceInputLeftTensor)
        cudaFree(this->m_deviceInputLeftTensor);
    if (this->m_deviceInputRightTensor)
        cudaFree(this->m_deviceInputRightTensor);
    if (this->m_deviceOutputDispTensor)
        cudaFree(this->m_deviceOutputDispTensor);

    // Free TensorRT objects
    if (this->m_context)
        delete this->m_context;
    if (this->m_engine)
        delete this->m_engine;
    if (this->m_runtime)
        delete this->m_runtime;

	// Destroy CUDA stream
    cudaStreamDestroy(this->m_stream);
}

std::vector<char> stereo::TensorRTInference::TRTInferenceImpl::readTRTFile(const std::string& filePath)
{
    std::ifstream file(filePath, std::ios::binary);
    if (!file.good())
        return {};

    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> buffer(size);
    file.read(buffer.data(), size);
    return buffer;
}

size_t stereo::TensorRTInference::TRTInferenceImpl::tensorVolume(const Dims& dims)
{
    size_t v = 1;
    for (int i = 0; i < dims.nbDims; ++i)
        v *= dims.d[i];
    return v;
}

void stereo::TensorRTInference::TRTInferenceImpl::allocateBuffers()
{
	// Free existing device buffers
    if (this->m_deviceInputLeftTensor)
        cudaFree(this->m_deviceInputLeftTensor);
    if (this->m_deviceInputRightTensor)
        cudaFree(this->m_deviceInputRightTensor);
    if (this->m_deviceOutputDispTensor)
        cudaFree(this->m_deviceOutputDispTensor);

	// Calculate input tensor sizes
    size_t leftImgsz = tensorVolume(this->m_currentInputLeftTensorDims);
    size_t rightImgsz = tensorVolume(this->m_currentInputRightTensorDims);
	// Get output tensor dimensions
    Dims outputDims = this->m_context->getTensorShape(this->m_outputDispName.c_str());
    // Calculate output tensor size
    size_t dispImgsz = tensorVolume(outputDims);

	// Allocate device buffers
    cudaMalloc(&this->m_deviceInputLeftTensor, leftImgsz * sizeof(float));
	cudaMalloc(&this->m_deviceInputRightTensor, rightImgsz * sizeof(float));
	cudaMalloc(&this->m_deviceOutputDispTensor, dispImgsz * sizeof(float));

	// Check allocation success
    if (!this->m_deviceInputLeftTensor || !this->m_deviceInputRightTensor || !this->m_deviceOutputDispTensor)
    {
        LOG_ERROR("CUDA buffer allocation failed!");
        throw std::runtime_error("CUDA malloc error");
    }

	// Reserve host buffers
	this->m_hostInputLeftTensor.reserve(leftImgsz); 
	this->m_hostInputRightTensor.reserve(rightImgsz);   
	this->m_hostOutputDispTensor.reserve(dispImgsz);
   
    // Resize host buffers
    this->m_hostInputLeftTensor.resize(leftImgsz);
    this->m_hostInputRightTensor.resize(rightImgsz);
    this->m_hostOutputDispTensor.resize(dispImgsz);
}

void stereo::TensorRTInference::TRTInferenceImpl::infer()
{
	// Copy input data from host to device
    cudaMemcpyAsync(
        this->m_deviceInputLeftTensor,
        this->m_hostInputLeftTensor.data(),
        this->m_hostInputLeftTensor.size() * sizeof(float),
        cudaMemcpyHostToDevice,
        this->m_stream);
    cudaMemcpyAsync(
        this->m_deviceInputRightTensor,
        this->m_hostInputRightTensor.data(),
        this->m_hostInputRightTensor.size() * sizeof(float),
        cudaMemcpyHostToDevice,
        this->m_stream);

    this->m_context->setTensorAddress(this->m_inputLeftName.c_str(), this->m_deviceInputLeftTensor);
    this->m_context->setTensorAddress(this->m_inputRightName.c_str(), this->m_deviceInputRightTensor);
    this->m_context->setTensorAddress(this->m_outputDispName.c_str(), this->m_deviceOutputDispTensor);

	// Execute inference
    this->m_context->enqueueV3(this->m_stream);

    // Copy output data from device to host
    cudaMemcpyAsync(
        this->m_hostOutputDispTensor.data(),
        this->m_deviceOutputDispTensor,
        this->m_hostOutputDispTensor.size() * sizeof(float),
        cudaMemcpyDeviceToHost,
        this->m_stream);

    cudaStreamSynchronize(this->m_stream);
}

