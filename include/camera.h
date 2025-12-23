/*********************************************************************
 * @file   camera.h
 * @brief  茶叶嫩芽采摘双目相机采集图像模块
 * @author Qianyao Zhuang
 * @date   May 2025
 *
 * @details
 *
 * @section 注意事项
 *
 * @code demo 1 - xyz3d相机采集

#include <conio.h>
	...

	using namespace std::literals;

	camera::XYZ3D xyz3d;
	try
	{
		xyz3d.connect(...);
		xyz3d.startCaptureThread();
		xyz3d.startLiveThread();
		xyz3d.startUSBDEVMonitor();
	}
	catch (...)
	{
		return -1;
	}
	int key = -1;
	do
	{
		if (_kbhit())
		{
			char ch = _getch();
			switch (ch)
			{
			case 27://"ESC"
				xyz3d.release();
				key = 27;
				break;
			case 'c': case 'C':
			{
				break;
			}
			default:
				break;
			}
		}
		std::this_thread::sleep_for(20ms);
	} while (key != 27);

	...
 * @endcode demo 1 - xyz3d相机采集
 *
 * @code demo 2 - 相机采集
 * 
	
	std::string pidvid = "vid_2f9d&pid_0024";
	camera::WebCamera cam;
	cam.connect(pidvid, "", camera::ImageSize(640, 480), camera::DSHOW, camera::MJPG, 3);
	cam.startCaptureThread();
	cam.startLiveThread();
	//cam.writeFrame("test.png");
	//cam.startScheduledCapture("../Capture", 100ms);
	//cam.startRecording("../Record/test.avi");
	std::this_thread::sleep_for(10s);
	cam.release();

 * @endcode demo 2 - 相机采集
 *********************************************************************/
#pragma once
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <opencv2/core/mat.hpp>
#include "./camera_utils.h"

namespace camera
{


class Camera
{
public:
	virtual ~Camera() = 0;
	virtual void connect(const std::string& pidvid, const std::string& port, camera::ImageSize imgsz, const camera::MediaAPI& api, const camera::VideoEncoding& en, const int& retry) = 0;
	virtual void startCaptureThread() = 0;
	virtual void stopCaptureThread() = 0;
	virtual void startLiveThread() = 0;
	virtual void stopLiveThread() = 0;
	virtual void release() = 0;
	virtual bool isConnected() = 0;
	virtual cv::Mat getFrame() = 0;
	virtual void writeFrame(const std::string& path) = 0;
	virtual void startScheduledCapture(const std::string& path, std::chrono::milliseconds interval) = 0; 
	virtual void stopScheduledCapture() = 0;
	virtual void startRecording(const std::string& path) = 0;
    virtual void stopRecording() = 0;
};


class XYZ3D : public Camera
{
public:
	XYZ3D();
	~XYZ3D();
	/**
	 * @brief 加载双目参数文件.
	 * @param [in] stereoYAML 双目参数YAML文件, optional
	 */
	void loadStereoYAMLFile(const std::string& stereoYAML);
	/**
	 * @brief 连接相机.
	 * @param [in] pidvid 相机PIDVID
	 * @param [in] port   相机端口
	 * @param [in] imgsz  相机图像尺寸
	 * @param [in] api    相机调用API, DSHOW or MSMF
	 * @param [in] en     相机编码格式, MJPG or YUY2
	 * @param [in] retry  重试次数
	 */
	void connect(const std::string& pidvid, const std::string& port, camera::ImageSize imgsz, const camera::MediaAPI& api, const camera::VideoEncoding& en, const int& retry) override;
	/**
	 * @brief 开启实时采集线程 [Capture].
	 */
	void startCaptureThread() override;
	/**
	 * @brief 停止实时采集线程 [Capture].
	 */
	void stopCaptureThread() override;
	/**
	 * @brief 开启实时展示线程 [Live].
	 */
	void startLiveThread() override;
	/**
	 * @brief 停止实时展示线程 [Live].
	 */
	void stopLiveThread() override;
	/**
	 * @brief 释放相机资源.
	 */
	void release() override;
	/**
	 * @brief 相机是否连接.
	 * @return 相机连接状态
	 */
	bool isConnected() override;
	/**
	 * @brief 获取当前图像帧.
	 * @return 当前图像帧
	 */
	cv::Mat getFrame() override;
	/**
	 * @brief 捕获当前帧, 已开启实时采集线程可正常捕获.
	 * @param [out] stereo 当前帧双目图像, cv::Mat类型, 左右图像拼接, 未极线校正.
	 * @param [out] left   极线校正后的左目图像
	 * @param [out] right  极线校正后的右目图像
	 * @param [in]  lrSwap 左右目图像交换(取决于相机是否反装), 默认为false
	 */
	void getFrame(cv::Mat& stereo, cv::Mat& left, cv::Mat& right, const bool& lrSwap = false);
	/**
	 * @brief 保存当前帧图像.
	 * @param [in] path 保存路径, 含文件名
	 */
	void writeFrame(const std::string& path) override;
	/**
	 * @brief 开启定时采集图像功能.
	 * @param [in] path 保存路径, 不含文件名
	 * @param [in] interval 采集间隔, ms (若使用using namespace std::literals; 则可使用1s, 10ms, ...)
	 */
	void startScheduledCapture(const std::string& path, std::chrono::milliseconds interval) override;
	/**
	 * @brief 停止定时采集图像功能.
	 */
	void stopScheduledCapture() override;
	/**
	 * @brief 开启录像功能, 只能保存avi文件, mp4文件需要编译FFMPEG.
	 * @param [in] path 保存路径, 含文件名
	 */
	void startRecording(const std::string& path) override;
	/**
	 * @brief 停止录像功能.
	 */
	void stopRecording() override;
	/**
	 * @brief 开启USB设备插拔监视.
	 */
	void startUSBDEVMonitor();
	/**
	 * @brief 停止USB设备插拔监视.
	 */
	void stopUSBDEVMonitor();
	/**
	 * @brief 设置AE状态.
	 * @param [in] autoExposure 自动曝光状态, true为开启, false为关闭
	 */
	void setAE(bool autoExposure = true);
	/**
	 * @brief 设置AWB状态.
	 * @param [in] autoWhiteBalance 自动白平衡状态, true为开启, false为关闭
	 */
	void setAWB(bool autoWhiteBalance = true);
	/**
	 * @brief 获取当前相机曝光增益ISO.
	 * @return ISO
	 */
	float getISO();
	/**
	 * @brief 获取当前相机曝光时长EXPTIME.
	 * @return EXPTIME
	 */
	float getExposureTime();
	/**
	 * @brief 设置相机曝光增益ISO.
	 * @param [in] iso 曝光增益
	 */
	void setISO(float iso = 1.f);
	/**
	 * @brief 设置相机曝光时长.
	 * @param [in] exposureTime 曝光时长, ms. (!< Min = 0.037ms.)
	 */
	void setExposureTime(float exposureTime);

private:
	class XYZ3DImpl;
	std::unique_ptr<XYZ3DImpl> impl;
};


class WebCamera : public Camera
{
public:
	WebCamera();
	WebCamera(const WebCamera& other) = delete;
    WebCamera(WebCamera&& other) noexcept = delete;
    WebCamera& operator=(const WebCamera& other) = delete;
    WebCamera& operator=(WebCamera&& other) noexcept = delete;
	~WebCamera();
	/**
	 * @brief 连接相机.
	 * @param [in] pidvid 相机PIDVID
	 * @param [in] port   相机端口
	 * @param [in] imgsz  相机图像尺寸
	 * @param [in] api    相机调用API, DSHOW or MSMF
	 * @param [in] en     相机编码格式, MJPG or YUY2
	 * @param [in] retry  重试次数
	 */
	void connect(const std::string& pidvid, const std::string& port, camera::ImageSize imgsz, const camera::MediaAPI& api, const camera::VideoEncoding& en, const int& retry) override;
	/**
	 * @brief 开启实时采集线程 [Capture].
	 */
	void startCaptureThread() override;
	/**
	 * @brief 停止实时采集线程 [Capture].
	 */
	void stopCaptureThread() override;
	/**
	 * @brief 开启实时展示线程 [Live].
	 */
	void startLiveThread() override;
	/**
	 * @brief 停止实时展示线程 [Live].
	 */
	void stopLiveThread() override;
	/**
	 * @brief 释放相机资源.
	 */
	void release() override;
	/**
	 * @brief 相机是否连接.
	 * @return 相机连接状态
	 */
	bool isConnected() override;
	/**
	 * @brief 获取当前图像帧.
	 * @return 当前图像帧
	 */
	cv::Mat getFrame() override;
	/**
	 * @brief 保存当前帧图像.
	 * @param [in] path 保存路径, 含文件名
	 */
	void writeFrame(const std::string& path) override;
	/**
	 * @brief 开启定时采集图像功能.
	 * @param [in] path 保存路径, 不含文件名
	 * @param [in] interval 采集间隔, ms (若使用using namespace std::literals; 则可使用1s, 10ms, ...)
	 */
	void startScheduledCapture(const std::string& path, std::chrono::milliseconds interval) override;
	/**
	 * @brief 停止定时采集图像功能.
	 */
	void stopScheduledCapture() override;
	/**
	 * @brief 开启录像功能, 只能保存avi文件, mp4文件需要编译FFMPEG.
	 * @param [in] path 保存路径, 含文件名
	 */
	void startRecording(const std::string& path) override;
	/**
	 * @brief 停止录像功能.
	 */
	void stopRecording() override;

private:
	class WebCameraImpl;
    std::unique_ptr<WebCameraImpl> impl;
};

}
