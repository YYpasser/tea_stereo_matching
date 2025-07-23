#include "../include/camera.h"
#include "../include/XYZ3D_impl.h"
#include "../include/stereo.h"
#include "../include/utils.h"
#include "../include/logger.h"
#include <iostream>
#include <sstream>
#include <limits>
#include <ctime>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <charconv>

//#include <xphoto/white_balance.hpp>

camera::XYZ3D::XYZ3D()
{
	this->impl = std::make_unique<XYZ3DImpl>();
}

camera::XYZ3D::~XYZ3D()
{
}

void camera::XYZ3D::loadStereoYAMLFile(const std::string& stereoYAML)
{
    if (!stereoYAML.empty())
        this->impl->m_stereoParams.loadYAMLFile(stereoYAML);
    if (this->impl->m_stereoParams.empty())
        return;
    if (this->impl->m_rectify == nullptr)
        this->impl->m_rectify = new stereo::EpipolarRectify(
            this->impl->m_stereoParams.map, this->impl->m_stereoParams.imgsz);
}

void camera::XYZ3D::connect(const std::string& pid_vid_str, const std::string& port, camera::ImageSize imgsz, const camera::MediaAPI& api, const camera::VideoEncoding& en, const int& retry)
{
    if (pid_vid_str.empty())
    {
        std::string msg = "pid_vid_str is empty.";
        LOG_ERROR(msg);
        throw std::runtime_error(msg);
    }
    LOG_INFO("Connecting to camera device " + pid_vid_str + "...");
    this->impl->connectTask(pid_vid_str, port, imgsz, api, en);
    if (!this->isConnected())
    {
        LOG_INFO("Failed to connect to camera device " + pid_vid_str +", retrying...");
        for (int i = 0; i < retry; ++i)
        {
            LOG_INFO("Retrying [" + std::to_string(i + 1) + " / " + std::to_string(retry) + "]...");
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            this->impl->connectTask(pid_vid_str, port, imgsz, api, en);
            if (this->isConnected())
            {
                LOG_INFO("Successfully connected to camera device " + pid_vid_str + " after retrying.");
                break;
            }
        }
        if (!this->isConnected())
        {
            std::string msg = "Failed to connect to camera device " + pid_vid_str +".";
            LOG_ERROR(msg);
            throw std::runtime_error(msg);
        }
    }
    LOG_INFO("Connecting to camera device " + pid_vid_str + " done.");
    this->impl->m_api = api;
    this->impl->m_encoding = en;
    this->impl->m_vid_pid = pid_vid_str;
    this->impl->m_imgsz = imgsz;
    this->impl->m_port = port;
}

void camera::XYZ3D::release()
{
    LOG_INFO("Releasing the camera...");
    if (this->impl->m_monitorTaskState.load() == this->impl->RUN)
        stopUSBDEVMonitor();
    if (this->impl->m_liveTaskState.load() == this->impl->RUN)
        stopLiveThread();
    if (this->impl->m_captureTaskState.load() == this->impl->RUN)
        stopCaptureThread();
    if (this->impl->m_cap != nullptr)
        this->impl->m_cap->release();
    LOG_INFO("Released camera.");
}

void camera::XYZ3D::startCaptureThread()
{
    printf("[XYZ3D] Start Capture Task Thread...\n");
    this->impl->m_captureTaskSignal = this->impl->RUN;
    //std::thread task(std::bind(&XYZ3D::XYZ3DImpl::captureTask, std::ref(this->impl)));
    std::thread task([this]() { this->impl->captureTask(); });
    task.detach();

    std::unique_lock<std::mutex> lock(this->impl->m_mtx);
    if (this->impl->m_cond.wait_for(lock, std::chrono::seconds(5), [&] {return this->impl->m_captureTaskState.load() == this->impl->RUN; }))
    {
        LOG_INFO("Start capture thread done.");
    }
    else
    {
        std::string errstr = "Start capture thread timeout.";
        LOG_ERROR(errstr);
        throw std::runtime_error(errstr);
    }
}

void camera::XYZ3D::stopCaptureThread()
{
    LOG_INFO("Stopping capture thread...");
    if (this->impl->m_liveTaskState.load() == this->impl->RUN)
        stopLiveThread();
    this->impl->m_captureTaskSignal = this->impl->STOP;
    std::unique_lock<std::mutex> lock(this->impl->m_mtx);
    this->impl->m_cond.wait(lock, [&] {return this->impl->m_captureTaskState.load() == this->impl->STOP; });
}

void camera::XYZ3D::startLiveThread(const bool& userBar)
{
    LOG_INFO("Starting live thread...");
    this->impl->m_liveTaskSignal = this->impl->RUN;

    this->impl->m_userBar = userBar;
    //std::thread task(std::bind(&XYZ3D::XYZ3DImpl::liveTask, std::ref(this->impl), std::ref(userBar)));
    std::thread task([this, &userBar]() { this->impl->liveTask(userBar); });
    task.detach();

    std::unique_lock<std::mutex> lock(this->impl->m_mtx);
    if (this->impl->m_cond.wait_for(lock, std::chrono::seconds(5), [&] {return this->impl->m_liveTaskState.load() == this->impl->RUN; }))
    {
        LOG_INFO("Start live thread done.");
    }
    else
    {
        std::string errstr = "Start live thread timeout.";
        LOG_ERROR(errstr);
        throw std::runtime_error(errstr);
    }
}

void camera::XYZ3D::stopLiveThread()
{
    LOG_INFO("Stopping live thread...");
    this->impl->m_liveTaskSignal = this->impl->STOP;
    std::unique_lock<std::mutex> lock(this->impl->m_mtx);
    this->impl->m_cond.wait(lock, [&] {return this->impl->m_liveTaskState.load() == this->impl->STOP; });
}

void camera::XYZ3D::startUSBDEVMonitor()
{
    LOG_INFO("Start USB Device Monitor thread...");
    this->impl->m_monitorTaskSignal = this->impl->RUN;
    //std::thread task(std::bind(&XYZ3D::XYZ3DImpl::usbDeviceMonitorTask, std::ref(this->impl)));
    std::thread task([this]() { this->impl->usbDeviceMonitorTask(); });
    task.detach();

    std::unique_lock<std::mutex> lock(this->impl->m_mtx);
    if (this->impl->m_cond.wait_for(lock, std::chrono::seconds(5), [&] {return this->impl->m_monitorTaskState.load() == this->impl->RUN; }))
    {
        LOG_INFO("Start monitor thread done.");
    }
    else
    {
        std::string errstr = "Start monitor thread timeout.";
        LOG_ERROR(errstr);
        throw std::runtime_error(errstr);
    }
}

void camera::XYZ3D::stopUSBDEVMonitor()
{
    LOG_INFO("Stopping USB Device Monitor thread...");
    this->impl->m_monitorTaskSignal = this->impl->STOP;
    std::unique_lock<std::mutex> lock(this->impl->m_mtx);
    this->impl->m_cond.wait(lock, [&] {return this->impl->m_monitorTaskState.load() == this->impl->STOP; });
}

void camera::XYZ3D::setAE(const bool& autoExposure)
{
    if (autoExposure)
        this->impl->enableAE();
    else
        this->impl->disableAE();
}

void camera::XYZ3D::setAWB(const bool& autoWhiteBalance)
{
    if (autoWhiteBalance)
        this->impl->enableAWB();
    else
        this->impl->disableAWB();
}

float camera::XYZ3D::getISO()
{
    return this->impl->getISO();
}

float camera::XYZ3D::getExposureTime()
{
    return this->impl->getExposureTime();
}

void camera::XYZ3D::setISO(const float& iso)
{
    this->impl->setISO(iso);
}

void camera::XYZ3D::setExposureTime(const float& exposureTime)
{
    this->impl->setExposureTime(exposureTime);
}

cv::Mat camera::XYZ3D::getRGBFrame()
{
    if (this->impl->m_captureTaskState.load() == this->impl->STOP)
    {
        std::string errstr = "Unstart capture thread";
        LOG_ERROR(errstr);
        throw std::runtime_error(errstr);
    }
    return this->impl->readFrame();
}

void camera::XYZ3D::getRGBFrame(cv::Mat& stereo, cv::Mat& left, cv::Mat& right, const bool& lrSwap)
{
    if (this->impl->m_rectify == nullptr)
    {
        std::string errstr = "Unload rectify matrix";
        LOG_ERROR(errstr);
        throw std::runtime_error(errstr);
    }
    if (this->impl->m_captureTaskState.load() == this->impl->STOP)
    {
        std::string errstr = "Unstart capture thread";
        LOG_ERROR(errstr);
        throw std::runtime_error(errstr);
    }

    if (!lrSwap)
    {
        stereo = this->impl->readFrame();
        auto rectified = this->impl->m_rectify->rectify(stereo);
        left = rectified.left.clone();
        right = rectified.right.clone();
    }
    else
    {
        cv::Mat ss = this->impl->readFrame();
        cv::Mat left = ss(cv::Rect(0, 0, ss.cols / 2, ss.rows)).clone();
        cv::Mat right = ss(cv::Rect(ss.cols / 2, 0, ss.cols / 2, ss.rows)).clone();
        if (lrSwap)//left-right swap
            std::swap(left, right);
        cv::hconcat(left, right, stereo);
        auto rectified = this->impl->m_rectify->rectify(stereo);
        left = rectified.left.clone();
        right = rectified.right.clone();
    }
}

bool camera::XYZ3D::isConnected() const
{
    return this->impl->isConnected();
}

camera::XYZ3D::XYZ3DImpl::XYZ3DImpl()
{
    this->m_captureTaskState.store(STOP);
    this->m_liveTaskState.store(STOP);
    this->m_captureTaskSignal = STOP;
    this->m_liveTaskSignal = STOP;
    this->m_capHotplugTaskSignal = STOP;
    this->m_liveHotplugTaskSignal = STOP;
    this->m_firstFrame.store(false);
    this->m_frameCount.store(0);

    this->m_cap = nullptr;
    this->m_api = DSHOW;
    this->m_encoding = MJPG;
    this->m_hLenaDDI = nullptr;
    this->m_DevSelInfo.index = NULL;

    this->m_sharedMutex = std::make_unique<std::shared_mutex>();

    this->m_expBarName = "ExpTime";
    this->m_isoBarName = "ISO";
    this->m_userBar = false;
    this->m_liveWinName = std::string();
    this->m_vidstr = std::string();
    this->m_pidstr = std::string();

    this->m_monitorTaskSignal = STOP;
    this->m_monitorTaskState.store(STOP);
    this->m_hwnd = NULL;
    this->m_deviceOffline.store(false);

    this->m_rectify = nullptr;
}

camera::XYZ3D::XYZ3DImpl::~XYZ3DImpl()
{
    if (this->m_cap != nullptr)
    {
        this->m_cap->release();
        this->m_cap = nullptr;
    }
}

void camera::XYZ3D::XYZ3DImpl::connectTask(const std::string& pid_vid_str, const std::string& port, camera::ImageSize imgsz, const camera::MediaAPI& api, const camera::VideoEncoding& en)
{
    LOG_INFO("Searching camera devices...");
    camera::CameraList cam;
    cam.update(api);
    cam.extract(en);
    if (cam.getList().empty())      
    {
        LOG_INFO("No camera devices found.");
        return;
    }

    cam.print();

    int deviceID = -1;
    int resoID = -1;
    auto camList = cam.getList();
    int xyz3dDeviceIDTmp = 0;
    int xyz3dDeviceID = -1;
    for (auto p = camList.begin(); p != camList.end(); ++p)
    {
        auto i = std::distance(camList.begin(), p);
        if (p->symbolicLink.find(pid_vid_str) == std::string::npos)
            continue;
        if (!port.empty() and p->portLocation != port)
        {
            ++xyz3dDeviceIDTmp;
            continue;
        }
        deviceID = static_cast<int>(i);
        xyz3dDeviceID = xyz3dDeviceIDTmp;
        auto resoCnt = 0;
        for (auto it = p->prop.begin(); it != p->prop.end(); ++it)
        {
            auto [res, fps, enc] = *it;
            if (res->width == imgsz.width and res->height == imgsz.height)
                resoID = (int)resoCnt;
            ++resoCnt;
        }
    }
    if (deviceID < 0)
    {
        LOG_INFO("No camera pid_vid_str consistent.");
        return;
    }
    if (resoID < 0)
    {
        LOG_INFO("[XYZ3D] Searching nearest resolution...");
        auto resoCnt = 0;
        double minDistance = std::numeric_limits<double>::infinity();
        for (auto it = camList[deviceID].prop.begin(); it != camList[deviceID].prop.end(); ++it)
        {
            auto [res, fps, enc] = *it;
            double distance = std::sqrt(std::pow(res->width - imgsz.width, 2) + std::pow(res->height - imgsz.height, 2));
            if (distance < minDistance)
            {
                minDistance = distance;
                resoID = resoCnt;
            }
            ++resoCnt;
        }
        LOG_INFO("Done searching nearest resolution.\n");
    }
    LOG_INFO("Selected camera: " + camList[deviceID].friendlyName + ".");
    LOG_INFO("Selected media API reference: " + camera::convertEnumToString(api) + ".");
    LOG_INFO("Selected video encoding: " + camera::convertEnumToString(en) + ".");
    LOG_INFO("Selected resolution with fps: " + std::to_string(camList[deviceID].prop.resolution[resoID].width)
        + " x " + std::to_string(camList[deviceID].prop.resolution[resoID].height) + " with " + std::to_string(camList[deviceID].prop.fps[resoID]) + " fps.");
    LOG_INFO("Connecting camera...");
    if (this->m_cap != nullptr)
    {
        delete this->m_cap;
        this->m_cap = nullptr;
    }
    this->m_cap = new cv::VideoCapture();
    switch (api)
    {
    case camera::DSHOW:
        this->m_cap->open(deviceID, cv::CAP_DSHOW);
        break;
    case camera::MSMF:
        this->m_cap->open(deviceID, cv::CAP_MSMF);
        break;
    default:
        break;
    }
    if (!this->m_cap->isOpened())
        return;

    LOG_INFO("Camera initializing...");
    switch (en)
    {
    case YUY2:
        this->m_cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', '2'));
        break;
    case MJPG:
        this->m_cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        break;
    default:
        break;
    }
    this->m_cap->set(cv::CAP_PROP_FPS, camList[deviceID].prop.fps[resoID]);
    this->m_cap->set(cv::CAP_PROP_FRAME_WIDTH, camList[deviceID].prop.resolution[resoID].width);
    this->m_cap->set(cv::CAP_PROP_FRAME_HEIGHT, camList[deviceID].prop.resolution[resoID].height);
    LOG_INFO("Camera initialized.");

    LOG_INFO("\"XYZ 3d\" Initializing...");
    this->m_DevSelInfo.index = xyz3dDeviceID;
    LenaDDI_Init2(&this->m_hLenaDDI, false, true);
    this->enableAE();
    this->enableAWB();
    LenaDDI_SetSensorTypeName(this->m_hLenaDDI, LenaDDI_SENSOR_TYPE_AR0135);
    LOG_INFO("\"XYZ 3d\" Initialized.");

    std::string portName = camList[deviceID].portLocation;
    size_t pos = 0;
    for (int i = 0; i < 5; ++i)
        pos = portName.find(".", pos) + 1;
    portName = portName.substr(0, pos - 1);
    this->m_liveWinName = "[" + portName + "] Live";
}

void camera::XYZ3D::XYZ3DImpl::captureTask()
{
    std::stringstream ss;
    ss << std::this_thread::get_id();
    LOG_INFO("Thread ID " + ss.str() + ": Started capture thread.");
    {
        std::unique_lock<std::mutex> lock(this->m_mtx);
        this->m_captureTaskState.store(RUN);
        this->m_cond.notify_all();
    }

    while (!this->m_cap->grab() and this->m_captureTaskSignal == RUN)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    this->m_firstFrame.store(true);

    while (this->m_captureTaskSignal == RUN)
    {
        if (!this->m_cap->grab()) //捕获
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        cv::Mat frame;
        if (!this->m_cap->retrieve(frame)) //解码
            continue;
        //cv::Ptr<cv::xphoto::SimpleWB> wb = cv::xphoto::createSimpleWB();
        //wb->balanceWhite(frame, frame);
        {
            std::unique_lock<std::shared_mutex> lock(*this->m_sharedMutex);
            this->m_frameShared = frame.clone();
        }
        if (this->m_liveTaskState == RUN)
            this->m_frameCount.fetch_add(1);
    }

    LOG_INFO("Stopped capture thread.");
    this->m_captureTaskState.store(STOP);
    this->m_cond.notify_all();
}

void camera::XYZ3D::XYZ3DImpl::liveTask(const bool& userBar)
{
    std::stringstream ss;
    ss << std::this_thread::get_id();
    LOG_INFO("Thread ID " + ss.str() + ": Started live thread.");
    {
        std::unique_lock<std::mutex> lock(this->m_mtx);
        this->m_liveTaskState.store(RUN);
        this->m_cond.notify_all();
    }

    while (!this->m_firstFrame and this->m_liveTaskSignal == RUN)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    cv::namedWindow(this->m_liveWinName, cv::WINDOW_NORMAL);
    cv::resizeWindow(this->m_liveWinName, 640, 480);

    if (userBar)
    {
        cv::createTrackbar(this->m_isoBarName, this->m_liveWinName, 0, 100, XYZ3D::XYZ3DImpl::onISOChange, this);
        cv::createTrackbar(this->m_expBarName, this->m_liveWinName, 0, 1000, XYZ3D::XYZ3DImpl::onExpTimeChange, this);
    }

    float FPS = 0;
    auto start = std::chrono::steady_clock::now();
    while (this->m_liveTaskSignal == RUN)
    {
        cv::Mat frame = readFrame();
        if (!frame.empty())
        {
            std::string f = std::to_string(FPS);
            std::string str = "FPS:" + f.substr(0, f.find(".") + 2);
            cv::putText(frame, str.c_str(), cv::Point(10, 30), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0);
            cv::imshow(this->m_liveWinName, frame);
            cv::waitKey(1);
        }
        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        if (duration > 2000)
        {
            int count = this->m_frameCount.load();
            this->m_frameCount.store(0);
            FPS = (float)count * 1000 / duration;
            start = std::chrono::steady_clock::now();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // 点击窗口关闭按钮退出线程
        if (cv::getWindowProperty(this->m_liveWinName, cv::WND_PROP_VISIBLE) < 1)
        {
            printf("[XYZ3D] Stopping live thread...\n");
            this->m_liveTaskSignal = STOP;
        }
    }

    LOG_INFO("Stopped live thread.");
    if (cv::getWindowProperty(this->m_liveWinName, cv::WND_PROP_VISIBLE))
        cv::destroyWindow(this->m_liveWinName);
    this->m_liveTaskState.store(STOP);
    this->m_cond.notify_all();
}

void camera::XYZ3D::XYZ3DImpl::hotplugTask()
{
    std::stringstream ss;
    ss << std::this_thread::get_id();
    LOG_INFO("Thread ID " + ss.str() + ": Started hotplug task.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    LOG_INFO("Connecting to camera device " + this->m_vid_pid + "...");
    connectTask(this->m_vid_pid, this->m_port, this->m_imgsz, this->m_api, this->m_encoding);

    if (!isConnected())
    {
        LOG_INFO("Failed to connect to camera device " + this->m_vid_pid + ", retrying...");
        for (int i = 0; i < 3; ++i)
        {
            LOG_INFO("Retrying [" + std::to_string(i + 1) + " / 3]...");
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            connectTask(this->m_vid_pid, this->m_port, this->m_imgsz, this->m_api, this->m_encoding);
            if (isConnected())
            {
                LOG_INFO("Successfully reconnected to camera device " + this->m_vid_pid + ".");
                break;
            }
        }
        if (!isConnected())
        {
            std::string msg = "Failed to connect to camera device " + this->m_vid_pid;
            LOG_ERROR(msg);
            return;
        }
    }
    LOG_INFO("Connecting to camera device done.");

    if (this->m_capHotplugTaskSignal == RUN)
    {
        LOG_INFO("Start Capture Task Thread...");
        this->m_captureTaskSignal = RUN;
        //std::thread task(std::bind(&XYZ3D::XYZ3DImpl::captureTask, std::ref(this->impl)));
        std::thread task([this]() { this->captureTask(); });
        task.detach();

        std::unique_lock<std::mutex> lock(this->m_mtx);
        if (this->m_cond.wait_for(lock, std::chrono::seconds(5), [&] {return this->m_captureTaskState.load() == RUN; }))
        {
            LOG_INFO("Start capture thread done.");
        }
        else
        {
            std::string errstr = "Start capture thread timeout.";
            LOG_ERROR(errstr);
            throw std::runtime_error(errstr);
        }
    }
    if (this->m_liveHotplugTaskSignal == RUN)
    {
        LOG_INFO("Starting Live Task Thread...");
        this->m_liveTaskSignal = RUN;

        //std::thread task(std::bind(&XYZ3D::XYZ3DImpl::liveTask, std::ref(this->impl), std::ref(userBar)));
        std::thread task([this]() { this->liveTask(this->m_userBar); });
        task.detach();

        std::unique_lock<std::mutex> lock(this->m_mtx);
        if (this->m_cond.wait_for(lock, std::chrono::seconds(5), [&] {return this->m_liveTaskState.load() == RUN; }))
        {
            LOG_INFO("Start live thread done.");
        }
        else
        {
            std::string errstr = "Start live thread timeout.";
            LOG_ERROR(errstr);
            throw std::runtime_error(errstr);
        }
    }
}

LRESULT camera::XYZ3D::XYZ3DImpl::WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    XYZ3D::XYZ3DImpl* p = (XYZ3D::XYZ3DImpl*)GetWindowLongPtr(hwnd, GWLP_USERDATA);
    if (message == WM_NCCREATE)
    {
        // 从 lParam 中提取 `CREATESTRUCT`，获取创建时传入的 `this` 指针
        CREATESTRUCT* pCreate = reinterpret_cast<CREATESTRUCT*>(lParam);
        p = reinterpret_cast<XYZ3D::XYZ3DImpl*>(pCreate->lpCreateParams);
        SetWindowLongPtr(hwnd, GWLP_USERDATA, (LONG_PTR)p);
        p->m_hwnd = hwnd;
    }
    else
    {
        // 获取关联的 `XYZ3D::Impl` 实例
        p = reinterpret_cast<XYZ3D::XYZ3DImpl*>(GetWindowLongPtr(hwnd, GWLP_USERDATA));
    }
    switch (message)
    {
    case WM_DEVICECHANGE:
    {
        if (wParam == DBT_DEVICEARRIVAL || wParam == DBT_DEVICEREMOVECOMPLETE)
        {
            PDEV_BROADCAST_HDR pHdr = (PDEV_BROADCAST_HDR)lParam;
            if (pHdr->dbch_devicetype == DBT_DEVTYP_DEVICEINTERFACE)
            {
                PDEV_BROADCAST_DEVICEINTERFACE pDevInf = (PDEV_BROADCAST_DEVICEINTERFACE)pHdr;
                // 获取设备路径
                std::wstring devicePath = pDevInf->dbcc_name;
                std::string devicePathStr(devicePath.begin(), devicePath.end());
                LOG_INFO("USB Device Status Changed: " + devicePathStr);
                // 检查 VID 和 PID	
                if (devicePathStr.find("VID_" + p->m_vidstr) != std::string::npos &&
                    devicePathStr.find("PID_" + p->m_pidstr) != std::string::npos)
                {
                    if (wParam == DBT_DEVICEARRIVAL)
                    {
                        LOG_INFO("Camera found.");
                        LOG_INFO("Starting hotplug task thread...");
                        std::thread task(std::bind(&XYZ3D::XYZ3DImpl::hotplugTask, std::ref(p)));
                        task.detach();
                    }
                    else if (wParam == DBT_DEVICEREMOVECOMPLETE)
                    {
                        LOG_INFO("Camera offline.");
                        if (p->m_captureTaskState == RUN)
                        {
                            p->m_captureTaskSignal = STOP;
                            p->m_capHotplugTaskSignal = RUN;
                        }
                        if (p->m_liveTaskState == RUN)
                        {
                            p->m_liveTaskSignal = STOP;
                            p->m_liveHotplugTaskSignal = RUN;
                        }
                    }
                }
            }
        }
        break;
    }
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    default:
        return DefWindowProc(hwnd, message, wParam, lParam);
    }
    return 0;
}

void camera::XYZ3D::XYZ3DImpl::usbDeviceMonitorTask()
{
    std::stringstream ss;
    ss << std::this_thread::get_id();
    LOG_INFO("Thread ID " + ss.str() + ": Started USB Device Monitor thread.");
    {
        std::unique_lock<std::mutex> lock(this->m_mtx);
        this->m_monitorTaskState.store(RUN);
        this->m_cond.notify_all();
    }

    auto pos1 = this->m_vid_pid.find("_");
    this->m_vidstr = this->m_vid_pid.substr(pos1 + 1, 4);
    pos1 = this->m_vid_pid.rfind("_");
    this->m_pidstr = this->m_vid_pid.substr(pos1 + 1);
    // 创建窗口类
    WNDCLASS wc = {};
    wc.lpfnWndProc = this->WndProc;
    wc.hInstance = GetModuleHandle(NULL);
    std::stringstream name;
    name << "USBMonitorClass" << std::this_thread::get_id();
    std::string namestr = name.str();
    std::wstring wStrName(namestr.begin(), namestr.end());
    wc.lpszClassName = wStrName.c_str();
    if (!RegisterClass(&wc))
    {
        std::string errstr = "Failed to register window class.";
        LOG_ERROR(errstr);
        throw std::runtime_error(errstr);
    }
    // 创建隐藏窗口
    this->m_hwnd = CreateWindowEx(
        0,                              // 扩展样式
        wc.lpszClassName,               // 类名
        L"USB Monitor",                 // 窗口名
        0,                              // 样式
        0, 0, 0, 0,                     // x, y, width, height
        HWND_MESSAGE,                   // 父窗口
        NULL, wc.hInstance, this        // 菜单、实例、参数
    );
    if (!this->m_hwnd)
    {
        std::string errstr = "Failed to create window.";
        LOG_ERROR(errstr);
        throw std::runtime_error(errstr);
    }
    // 注册设备通知
    DEV_BROADCAST_DEVICEINTERFACE notificationFilter = {};
    notificationFilter.dbcc_size = sizeof(DEV_BROADCAST_DEVICEINTERFACE);
    notificationFilter.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
    notificationFilter.dbcc_classguid = GUID_DEVINTERFACE_USB_DEVICE; // 监控USB设备

    HDEVNOTIFY hDevNotify = RegisterDeviceNotification(
        this->m_hwnd, &notificationFilter, DEVICE_NOTIFY_WINDOW_HANDLE);
    if (!hDevNotify)
    {
        std::stringstream errstrs;
        errstrs << "Failed to register device notification. Error: " << GetLastError() << ".";
        LOG_ERROR(errstrs.str());
        throw std::runtime_error(errstrs.str());
    }
    // 消息循环
    MSG msg;
    do
    {
        if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (this->m_monitorTaskSignal == RUN);
    // 注销设备通知
    UnregisterDeviceNotification(hDevNotify);

    LOG_INFO("Stopped USB Device Monitor thread.");
    this->m_monitorTaskState.store(STOP);
    this->m_cond.notify_all();
}

void camera::XYZ3D::XYZ3DImpl::enableAE()
{
    if (LenaDDI_EnableAE(this->m_hLenaDDI, &this->m_DevSelInfo) != LenaDDI_OK)
        LOG_INFO("Cannot enable AE.");
    else
        LOG_INFO("Enabled AE.");
}

void camera::XYZ3D::XYZ3DImpl::enableAWB()
{
    if (LenaDDI_EnableAWB(this->m_hLenaDDI, &this->m_DevSelInfo) != LenaDDI_OK)
        LOG_INFO("Cannot enable AWB.");
    else
        LOG_INFO("Enabled AWB.");
}

void camera::XYZ3D::XYZ3DImpl::disableAE()
{
    if (LenaDDI_DisableAE(this->m_hLenaDDI, &this->m_DevSelInfo) != LenaDDI_OK)
        LOG_INFO("Cannot disable AE.");
    else
        LOG_INFO("Disabled AE.");
}

void camera::XYZ3D::XYZ3DImpl::disableAWB()
{
    if (LenaDDI_DisableAWB(this->m_hLenaDDI, &this->m_DevSelInfo) != LenaDDI_OK)
        printf("[XYZ3D] Cannot disable AWB.");
    else
        printf("[XYZ3D] Disabled AWB.");
}

float camera::XYZ3D::XYZ3DImpl::getExposureTime()
{
    float expTime = 0.f;
    if (LenaDDI_GetExposureTime(this->m_hLenaDDI, &this->m_DevSelInfo, 2, &expTime) != LenaDDI_OK)
        LOG_INFO("Cannot get exposure time(S).");
    else
        LOG_INFO("Get S = " + std::to_string(expTime) + ".");
    return expTime;
}

float camera::XYZ3D::XYZ3DImpl::getISO()
{
    float globalGain = 0.f;
    if (LenaDDI_GetGlobalGain(this->m_hLenaDDI, &this->m_DevSelInfo, 2, &globalGain) != LenaDDI_OK)
        LOG_INFO("Cannot get global gain(ISO).");
    else
        LOG_INFO("Get ISO = " + std::to_string(globalGain) + ".");
    return globalGain;
}

void camera::XYZ3D::XYZ3DImpl::setExposureTime(const float& t)
{
    if (LenaDDI_SetExposureTime(this->m_hLenaDDI, &this->m_DevSelInfo, 2, t) != LenaDDI_OK)
        LOG_INFO("Cannot set exposure time(S).");
    else
        LOG_INFO("Set S = " + std::to_string(t) + ".");
}

void camera::XYZ3D::XYZ3DImpl::setISO(const float& iso)
{
    if (LenaDDI_SetGlobalGain(this->m_hLenaDDI, &this->m_DevSelInfo, 2, iso) != LenaDDI_OK)
        LOG_INFO("[XYZ3D] Cannot set global gain(ISO).");
    else
        LOG_INFO("Set ISO = " + std::to_string(iso) + ".");
}

cv::Mat camera::XYZ3D::XYZ3DImpl::readFrame() const
{
    std::shared_lock<std::shared_mutex> lock(*this->m_sharedMutex);
    return this->m_frameShared.clone();
}

void camera::XYZ3D::XYZ3DImpl::onExpTimeChange(int pos, void* param)
{
    XYZ3D::XYZ3DImpl* p = reinterpret_cast<XYZ3D::XYZ3DImpl*>(param); // 静态成员函数获取this指针访问非静态成员
    if (pos == 0)
    {
        p->enableAE();
        cv::setTrackbarPos(p->m_expBarName, p->m_liveWinName, static_cast<int>(p->getExposureTime() * 10));
        cv::setTrackbarPos(p->m_isoBarName, p->m_liveWinName, static_cast<int>(p->getISO() * 10));
    }
    else
    {
        p->disableAE();
        p->setExposureTime(static_cast<float>(pos / 10.f));
    }
}

void camera::XYZ3D::XYZ3DImpl::onISOChange(int pos, void* param)
{
    XYZ3D::XYZ3DImpl* p = reinterpret_cast<XYZ3D::XYZ3DImpl*>(param);
    if (pos == 0)
    {
        p->enableAE();
        cv::setTrackbarPos(p->m_expBarName, p->m_liveWinName, static_cast<int>(p->getExposureTime() * 10));
        cv::setTrackbarPos(p->m_isoBarName, p->m_liveWinName, static_cast<int>(p->getISO() * 10));
    }
    else
    {
        p->disableAE();
        p->setISO(static_cast<float>(pos / 10.f));
    }
}

bool camera::XYZ3D::XYZ3DImpl::isConnected() const
{
    if (this->m_cap == nullptr or !this->m_cap->isOpened())
        return false;
    return true;
}
