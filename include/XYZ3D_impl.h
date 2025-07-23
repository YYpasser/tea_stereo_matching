/*********************************************************************
 * @file   XYZ3D_impl.h
 * @brief  ��Ҷ��ѿ��ժ˫Ŀ����ɼ�ͼ��ģ��ʵ����
 * @author Qianyao Zhuang
 * @date   May 2025
 *********************************************************************/
#pragma once
#include "../include/camera.h"
#include "../include/stereo.h"
#include <thread>
#include <atomic>
#include <shared_mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
//-- XYZ3D������
#include "LenaDDIUtility.h"
#include "stdafx.h"
//-- �Ȳ�����
#include <windows.h>
#include <dbt.h>
#include <initguid.h>
#include <usbiodef.h>
//-- ��������
#include <condition_variable>
#include <mutex>

class camera::XYZ3D::XYZ3DImpl
{
public:
    XYZ3DImpl();
    ~XYZ3DImpl();
public:
    /**
     * @brief �����������.
     * @param [in] pid_vid_str ���PID_VID�ַ���
     * @param [in] port        �˿ں�, empty = no check
     * @param [in] imgsz       ͼ��ֱ���
     * @param [in] api         ý��API
     * @param [in] en          �����ʽ
     * @param [in] retry       ����������Դ���
     */
    void connectTask(const std::string& pid_vid_str, const std::string& port, camera::ImageSize imgsz, const camera::MediaAPI& api, const camera::VideoEncoding& en);
    /**
     * @brief ʵʱ�ɼ�����.
     */
    void captureTask();
    /**
     * @brief ʵʱչʾ����.
     */
    void liveTask(const bool& userBar);
    /**
     * @brief �Ȳ������.
     */
    void hotplugTask();
    /**
     * @brief ������Ϣ����ص�����.
     * @param [in] hwnd    ���ھ��
     * @param [in] message ��Ϣ
     * @param [in] wParam  ������Ϣ
     * @param [in] lParam  ������Ϣ
     * @return ״̬
     */
    static LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);
    /**
     * @brief USB�豸��������.
     */
    void usbDeviceMonitorTask();
    /**
     * @brief ����AE.
     */
    void enableAE();
    /**
     * @brief ����AWB.
     */
    void enableAWB();
    /**
     * @brief �ر�AE.
     */
    void disableAE();
    /**
     * @brief �ر�AWB.
     */
    void disableAWB();
    /**
     * @brief ��ȡ�ع�ʱ��.
     * @return �ع�ʱ��, ms
     */
    float getExposureTime();
    /**
     * @brief ��ȡ�ع�����.
     * @return �ع�����
     */
    float getISO();
    /**
     * @brief �����ع�ʱ��.
     * @param [in] t �ع�ʱ��, ms
     */
    void setExposureTime(const float& t);
    /**
     * @brief �����ع�����.
     * @param [in] iso �ع�����
     */
    void setISO(const float& iso);
    /**
     * @brief ����, ��ȡ����ͼ��֡.
     * @return ����ͼ��֡
     */
    cv::Mat readFrame() const;
    /**
     * @brief �ع�ʱ��������¼��ص�����.
     * @param [in] pos   ������λ��
     * @param [in] param �û�����
     */
    static void onExpTimeChange(int pos, void* param);
    /**
     * @brief �ع�����������¼��ص�����.
     * @param [in] pos   ������λ��
     * @param [in] param �û�����
     */
    static void onISOChange(int pos, void* param);
    /**
     * @brief ��ȡ�������״̬.
     * @return ����״̬
     */
    bool isConnected() const;
public:
    ///// thread /////
    enum ThreadTask /*!< �߳�����״̬ö�� */
    {
        STOP = 0, /*!< ֹͣ */
        RUN = 1, /*!< ���� */
    };

    ThreadTask m_captureTaskSignal;     /*!< ʵʱ�ɼ�ͼ���߳������ź� */
    ThreadTask m_liveTaskSignal;        /*!< ʵʱ����չʾ�߳������ź� */
    ThreadTask m_capHotplugTaskSignal;  /*!< ʵʱ�ɼ�ͼ���Ȳ���߳������ź� */
    ThreadTask m_liveHotplugTaskSignal; /*!< ʵʱ����չʾ�Ȳ���߳������ź� */

    std::atomic<ThreadTask> m_captureTaskState; /*!< ʵʱ�ɼ��߳�����״̬ */
    std::atomic<ThreadTask> m_liveTaskState;    /*!< ʵʱչʾ�߳�����״̬ */
    std::atomic<bool> m_firstFrame;   /*!< ��1֡��־λ */
    std::atomic<int>  m_frameCount;   /*!< ͼ��֡���� */

    std::string m_liveWinName;        /*!< ʵʱ����չʾ�������� */
    std::string m_expBarName;         /*!< ʵʱ����չʾ�����ع�ʱ�们�������� */
    std::string m_isoBarName;         /*!< ʵʱ����չʾ����ISO���������� */
    bool m_userBar;                   /*!< �Ƿ�ʹ���û��Զ����ع�ʱ�������� */

    std::string m_vidstr;           /*!< VID�ַ� */
    std::string m_pidstr;           /*!< PID�ַ� */
    ThreadTask m_monitorTaskSignal; /*!< USB�豸�����߳������ź� */
    std::atomic<ThreadTask> m_monitorTaskState; /*!< USB�豸�����߳�����״̬ */
    std::atomic<bool> m_deviceOffline; /*!< USB�豸���߱�־λ */
    HWND m_hwnd;                       /*!< Windows���ھ�� */


    StereoParams m_stereoParams;
    cv::VideoCapture* m_cap;          /*!< ����豸�ɼ����� */
    MediaAPI m_api;
    VideoEncoding m_encoding;
    std::string m_vid_pid;
    std::string m_port;
    ImageSize m_imgsz;

    ///// XYZ3D /////
    void* m_hLenaDDI;                 /*!< �豸ָ�� */
    DEVSELINFO m_DevSelInfo;          /*!< �豸ѡ����Ϣ */

    cv::Mat m_frameShared;            /*!< �̹߳���ͼ��֡ */
    std::unique_ptr<std::shared_mutex> m_sharedMutex; /*!< �̹߳��������� */

    std::mutex m_mtx;                 /*!< ������ */
    std::condition_variable m_cond;   /*!< �������� -> �¼�֪ͨ */

    stereo::EpipolarRectify* m_rectify; /*!< ����У������ */
};