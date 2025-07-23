/*********************************************************************
 * @file   camera.h
 * @brief  ��Ҷ��ѿ��ժ˫Ŀ����ɼ�ͼ��ģ��
 * @author Qianyao Zhuang
 * @date   May 2025
 * 
 * @details
 * 
 * @section ע������
 * 
 * @code demo 1 - xyz3d����ɼ�

#include <conio.h>
...
	camera::XYZ3D xyz3d;
	try
	{
		xyz3d.connect();
		xyz3d.startCaptureThread();
		xyz3d.startLiveThread(true);
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
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	} while (key != 27);
 * @endcode demo 1 - xyz3d����ɼ�
 * 
 *********************************************************************/
#pragma once
#include "camera_utils.hpp"
#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include <memory>

namespace camera
{

class XYZ3D
{
public:
	XYZ3D();
	~XYZ3D();
public:
	/**
	 * @brief ����˫Ŀ�����ļ�.
	 * @param [in] stereoYAML ˫Ŀ����YAML�ļ�, optional
	 */
	void loadStereoYAMLFile(const std::string& stereoYAML);
	/**
	 * @brief �������.
	 * @param [in] pid_vid_str ���PID_VID�ַ���
	 * @param [in] port        �˿ں�
	 * @param [in] imgsz       ͼ��ֱ���
	 * @param [in] api         ý��API
	 * @param [in] en          �����ʽ
	 * @param [in] retry       ����������Դ���
	 */
	void connect(const std::string& pid_vid_str = "vid_0211&pid_5838", const std::string& port = std::string(), camera::ImageSize imgsz = camera::ImageSize(2560, 720),
		const camera::MediaAPI& api = camera::MediaAPI::DSHOW, const camera::VideoEncoding& en = camera::VideoEncoding::YUY2, const int& retry = 3);
	/**
	 * @brief �ͷŵ�ǰ���.
	 */
	void release();
	/**
	 * @brief ����ʵʱ�ɼ��߳�.
	 * @throw ����ʵʱ�ɼ��߳��쳣/��ʱ
	 */
	void startCaptureThread();
	/**
	 * @brief ֹͣʵʱ�ɼ��߳�.
	 */
	void stopCaptureThread();
	/**
	 * @brief ����ʵʱչʾ�߳�.
	 * @param [in] userBar �Ƿ�չʾ�ع������, Ĭ��Ϊfalse
	 * @throw ����ʵʱչʾ�߳��쳣/��ʱ
	 */
	void startLiveThread(const bool& userBar = false);
	/**
	 * @brief ֹͣʵʱչʾ�߳�.
	 */
	void stopLiveThread();
	/**
	 * @brief ����USB�豸��μ���.
	 * @throw ����USB�豸��μ����쳣/��ʱ
	 */
	void startUSBDEVMonitor();
	/**
	 * @brief ֹͣUSB�豸��μ���.
	 */
	void stopUSBDEVMonitor();
	/**
	 * @brief ����AE״̬.
	 * @param [in] autoExposure �Զ��ع�״̬, trueΪ����, falseΪ�ر�
	 */
	void setAE(const bool& autoExposure = true);
	/**
	 * @brief ����AWB״̬.
	 * @param [in] autoWhiteBalance �Զ���ƽ��״̬, trueΪ����, falseΪ�ر�
	 */
	void setAWB(const bool& autoWhiteBalance = true);
	/**
	 * @brief ��ȡ��ǰ����ع�����ISO.
	 * @return ISO
	 */
	float getISO();
	/**
	 * @brief ��ȡ��ǰ����ع�ʱ��EXPTIME.
	 * @return EXPTIME
	 */
	float getExposureTime();
	/**
	 * @brief ��������ع�����ISO.
	 * @param [in] iso �ع�����
	 */
	void setISO(const float& iso = 1.f);
	/**
	 * @brief ��������ع�ʱ��.
	 * @param [in] exposureTime �ع�ʱ��, ms. (!< Min = 0.037ms.)
	 */
	void setExposureTime(const float& exposureTime);
	/**
	 * @brief ����ǰ֡, �ѿ���ʵʱ�ɼ��߳̿���������.
	 * @return ��ǰ֡˫Ŀͼ��, cv::Mat����, ����ͼ��ƴ��, δ����У��.
	 * @throw ����ǰ֡�쳣
	 */
	cv::Mat getRGBFrame();
	/**
	 * @brief ����ǰ֡, �ѿ���ʵʱ�ɼ��߳̿���������.
	 * @param [out] stereo ��ǰ֡˫Ŀͼ��, cv::Mat����, ����ͼ��ƴ��, δ����У��.
	 * @param [out] left   ����У�������Ŀͼ��
	 * @param [out] right  ����У�������Ŀͼ��
	 * @param [in]  lrSwap ����Ŀͼ�񽻻�(ȡ��������Ƿ�װ), Ĭ��Ϊfalse
	 * @throw ����ǰ֡�쳣
	 */
	void getRGBFrame(cv::Mat& stereo, cv::Mat& left, cv::Mat& right, const bool& lrSwap = false);
	/**
	 * @brief ��ȡ�������״̬.
	 * @return ����״̬
	 */
	bool isConnected() const;

private:
	class XYZ3DImpl;
    std::unique_ptr<XYZ3DImpl> impl;
};

}
