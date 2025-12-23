#include "../include/camera.h"
#include "../include/XYZ3DImpl.h"
#include "../include/logger.h"
#include "../include/utils.h"
#include <chrono>

using namespace std::literals;

camera::XYZ3D::XYZ3D()
{
	this->impl = std::make_unique<XYZ3DImpl>();
}

camera::XYZ3D::~XYZ3D() = default;

void camera::XYZ3D::loadStereoYAMLFile(const std::string& stereoYAML)
{
    if (!stereoYAML.empty())
        this->impl->m_stereoParams.loadYAMLFile(stereoYAML);
    if (this->impl->m_stereoParams.empty())
        return;
    if (this->impl->m_rectify != nullptr)
        this->impl->m_rectify.reset();
    this->impl->m_rectify = std::make_unique<stereo::EpipolarRectify>(
            this->impl->m_stereoParams.map, this->impl->m_stereoParams.imgsz);
}

void camera::XYZ3D::connect(const std::string& pidvid, const std::string& port, camera::ImageSize imgsz, const camera::MediaAPI& api, const camera::VideoEncoding& en, const int& retry)
{
    if (pidvid.empty())
    {
        std::string msg = "pid_vid_str is empty.";
        LOG_ERROR(msg);
        throw std::runtime_error(msg);
    }
    LOG_INFO("Connecting to camera device " + pidvid + "...");
    this->impl->connectTask(pidvid, port, imgsz, api, en);
    if (!this->isConnected())
    {
        LOG_INFO("Failed to connect to camera device " + pidvid + ", retrying...");
        for (int i = 0; i < retry; ++i)
        {
            LOG_INFO("Retrying [" + std::to_string(i + 1) + " / " + std::to_string(retry) + "]...");
            std::this_thread::sleep_for(2s);
            this->impl->connectTask(pidvid, port, imgsz, api, en);
            if (this->isConnected())
            {
                LOG_INFO("Successfully connected to camera device " + pidvid + " after retrying.");
                break;
            }
        }
        if (!this->isConnected())
        {
            std::string msg = "Failed to connect to camera device " + pidvid + ".";
            LOG_ERROR(msg);
            throw std::runtime_error(msg);
        }
    }
    LOG_INFO("Connecting to camera device " + pidvid + " done.");
    this->impl->m_api = api;
    this->impl->m_encoding = en;
    this->impl->m_vidpid = pidvid;
    this->impl->m_imgsz = imgsz;
    this->impl->m_port = port;
}

void camera::XYZ3D::startCaptureThread()
{
    if (this->impl->m_captureTaskState.load() == this->impl->RUN)
    {
        LOG_WARNING("Capture thread is already running.");
        return;
    }

    LOG_INFO("Start Capture Task Thread...");
    this->impl->m_captureTaskSignal.store(this->impl->RUN);

    this->impl->m_captureThread = std::thread([this]() {
        this->impl->captureTask();
        });

    // 等待线程启动完成
    std::unique_lock<std::mutex> lock(this->impl->m_captureMtx);
    if (this->impl->m_captureCV.wait_for(lock, 5s, [this]() {
        return this->impl->m_captureTaskState.load() == this->impl->RUN;
        }))
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
    if (this->impl->m_captureTaskState.load() == this->impl->STOP)
    {
        return;
    }

    LOG_INFO("Stopping capture thread...");
    if (this->impl->m_liveTaskState.load() == this->impl->RUN)
        stopLiveThread();

    impl->m_frameQueue.stop();
    this->impl->m_captureTaskSignal.store(this->impl->STOP);

    std::unique_lock<std::mutex> lock(this->impl->m_captureMtx);
    if (!this->impl->m_captureCV.wait_for(lock, 5s, [this]() {
        return this->impl->m_captureTaskState.load() == this->impl->STOP;
        }))
    {
        LOG_ERROR("Timeout waiting for capture thread to stop (5s)");
    }
}

void camera::XYZ3D::startLiveThread()
{
    if (this->impl->m_liveTaskState.load() == this->impl->RUN)
    {
        LOG_WARNING("Live thread is already running.");
        return;
    }

    LOG_INFO("Starting live thread...");
    this->impl->m_liveTaskSignal.store(this->impl->RUN);

    this->impl->m_liveThread = std::thread([this]() {
        this->impl->liveTask();
        });

    // 等待线程启动完成
    std::unique_lock<std::mutex> lock(this->impl->m_liveMtx);
    if (this->impl->m_liveCV.wait_for(lock, 5s, [this]() {
        return this->impl->m_liveTaskState.load() == this->impl->RUN;
        }))
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
    if (this->impl->m_liveTaskState.load() == this->impl->STOP)
    {
        return;
    }

    LOG_INFO("Stopping live thread...");
    this->impl->m_liveTaskSignal.store(this->impl->STOP);

    std::unique_lock<std::mutex> lock(this->impl->m_liveMtx);
    if (!this->impl->m_liveCV.wait_for(lock, 5s, [this]() {
        return this->impl->m_liveTaskState.load() == this->impl->STOP;
        }))
    {
        LOG_ERROR("Timeout waiting for live thread to stop (5s)");
    }
}

void camera::XYZ3D::release()
{
    LOG_INFO("Releasing camera resources...");

    if (this->impl->m_scheduledTimer != nullptr)
    {
        this->stopScheduledCapture();
    }
    if (this->impl->m_videoThread.joinable())
    {
        this->stopRecording();
    }
    if (this->impl->m_writeThread.joinable())
    {
        this->impl->stopWriteThread();
    }
    if (this->impl->m_liveThread.joinable())
    {
        this->stopLiveThread();
    }
    if (this->impl->m_captureThread.joinable())
    {
        this->stopCaptureThread();
    }

    this->impl->m_videoQueue.clear();
    this->impl->m_writeQueue.clear();
    this->impl->m_frameQueue.clear();

    LOG_INFO("Camera resources released done.");
}

bool camera::XYZ3D::isConnected()
{
    return this->impl->isConnected();
}

cv::Mat camera::XYZ3D::getFrame()
{
    return this->impl->readFrame();
}

void camera::XYZ3D::getFrame(cv::Mat& stereo, cv::Mat& left, cv::Mat& right, const bool& lrSwap)
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
        this->impl->m_rectify->rectify(stereo, left, right);
    }
    else
    {
        cv::Mat ss = this->impl->readFrame();
        cv::Mat leftTmp, rightTmp;
        stereo::hsplit(ss, leftTmp, rightTmp);
        if (lrSwap)//left-right swap
            std::swap(leftTmp, rightTmp);
        this->impl->m_rectify->rectify(leftTmp, rightTmp, left, right);
    }
}

void camera::XYZ3D::writeFrame(const std::string& path)
{
    if (!this->isConnected() || this->impl->m_writeTaskState.load() == this->impl->STOP)
    {
        LOG_ERROR("Camera is not connected or write task is not running.");
        return;
    }

    if (path.empty())
    {
        LOG_ERROR("Path is empty.");
        return;
    }

    this->impl->m_writeQueue.push(std::move(WriteData(path, std::move(this->impl->readFrame()))));
}

void camera::XYZ3D::startScheduledCapture(const std::string& path, std::chrono::milliseconds interval)
{
    if (!this->isConnected() || this->impl->m_writeTaskState.load() == this->impl->STOP)
    {
        LOG_ERROR("Camera is not connected or write task is not running.");
        return;
    }

    if (path.empty())
    {
        LOG_ERROR("Path is empty.");
        return;
    }

    if (this->impl->m_scheduledTimer != nullptr)
    {
        LOG_WARNING("Scheduled capture is already running.");
        return;
    }

    this->impl->m_scheduledTimer.reset();
    this->impl->m_scheduledTimer = std::make_unique<Timer>(
        "ScheduledCapture",
        [this, path]() {
            std::string filename = utils::getCurrentTimeMS() + ".png";
            std::string filepath = path + "/" + filename;
            this->writeFrame(filepath);
        },
        interval
    );
    this->impl->m_scheduledTimer->start();
}

void camera::XYZ3D::stopScheduledCapture()
{
    this->impl->m_scheduledTimer->stop();
}

void camera::XYZ3D::startRecording(const std::string& path)
{
    if (!this->isConnected())
    {
        LOG_ERROR("Camera is not connected.");
        return;
    }

    if (path.empty())
    {
        LOG_ERROR("Path is empty.");
        return;
    }

    if (this->impl->m_videoTaskState.load() == this->impl->RUN)
    {
        LOG_WARNING("Video thread is already running.");
        return;
    }

    LOG_INFO("Start Recording Task Thread...");
    this->impl->m_video.reset();
    this->impl->m_video = std::make_unique<cv::VideoWriter>(
        path,
        cv::VideoWriter::fourcc('M', 'P', '4', '2'),
        this->impl->m_cap->get(cv::CAP_PROP_FPS),
        cv::Size(this->impl->m_cap->get(cv::CAP_PROP_FRAME_WIDTH), this->impl->m_cap->get(cv::CAP_PROP_FRAME_HEIGHT)),
        true
    );

    this->impl->m_videoTaskSignal.store(this->impl->RUN);

    this->impl->m_videoThread = std::thread([this, path]() {
        this->impl->videoTask(path);
        });

    std::unique_lock<std::mutex> lock(this->impl->m_videoMtx);
    if (this->impl->m_videoCV.wait_for(lock, 5s, [this]() {
        return this->impl->m_videoTaskState.load() == this->impl->RUN;
        }))
    {
        LOG_INFO("Start recording thread done.");
    }
    else
    {
        std::string errstr = "Start recording thread timeout.";
        LOG_ERROR(errstr);
        throw std::runtime_error(errstr);
    }
}

void camera::XYZ3D::stopRecording()
{
    if (this->impl->m_videoTaskState.load() == this->impl->STOP)
    {
        return;
    }

    LOG_INFO("Stopping recording thread...");

    impl->m_videoQueue.stop();

    this->impl->m_videoTaskSignal.store(this->impl->STOP);

    std::unique_lock<std::mutex> lock(this->impl->m_videoMtx);
    if (!this->impl->m_videoCV.wait_for(lock, 5s, [this]() {
        return this->impl->m_videoTaskState.load() == this->impl->STOP;
        }))
    {
        LOG_ERROR("Timeout waiting for recording thread to stop (5s)");
    }
}

void camera::XYZ3D::startUSBDEVMonitor()
{
    if (this->impl->m_monitorTaskState.load() == this->impl->RUN)
    {
        LOG_WARNING("Monitor thread is already running.");
        return;
    }

    LOG_INFO("Starting USB Device Monitor thread...");
    this->impl->m_monitorTaskSignal = this->impl->RUN;
    std::thread task([this]() { this->impl->usbDeviceMonitorTask(); });

    std::unique_lock<std::mutex> lock(this->impl->m_monitorMtx);
    if (this->impl->m_monitorCV.wait_for(lock, 5s, [&] {
        return this->impl->m_monitorTaskState.load() == this->impl->RUN;
        }))
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
    if (this->impl->m_monitorTaskState.load() == this->impl->STOP)
    {
        return;
    }
    LOG_INFO("Stopping USB Device Monitor thread...");
    this->impl->m_monitorTaskSignal = this->impl->STOP;
    std::unique_lock<std::mutex> lock(this->impl->m_monitorMtx);
    if (!this->impl->m_monitorCV.wait_for(lock, 5s, [&] {
        return this->impl->m_monitorTaskState.load() == this->impl->STOP;
        }))
    {
        LOG_ERROR("Timeout waiting for monitor thread to stop (5s)");
    }
}

void camera::XYZ3D::setAE(bool autoExposure)
{
    if (autoExposure)
        this->impl->enableAE();
    else
        this->impl->disableAE();
}

void camera::XYZ3D::setAWB(bool autoWhiteBalance)
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

void camera::XYZ3D::setISO(float iso)
{
    this->impl->setISO(iso);
}

void camera::XYZ3D::setExposureTime(float exposureTime)
{
    this->impl->setExposureTime(exposureTime);
}

camera::XYZ3D::XYZ3DImpl::XYZ3DImpl() :
    m_captureTaskState{ STOP },
    m_liveTaskState{ STOP },
    m_writeTaskState{ STOP },
    m_videoTaskState{ STOP },
    m_monitorTaskSignal{ STOP },
    m_capHotTaskSignal{ STOP },
    m_liveHotTaskSignal{ STOP },
    m_captureTaskSignal{ STOP },
    m_liveTaskSignal{ STOP },
    m_writeTaskSignal{ STOP },
    m_videoTaskSignal{ STOP },
    m_monitorTaskState{ STOP },
    m_firstFrame{ false },
    m_deviceOffline{ false },
    m_api(DSHOW),
    m_encoding(MJPG),
    m_vidpid(std::string()),
    m_port(std::string()),
    m_imgsz(ImageSize()),
    m_vidstr(std::string()),
    m_pidstr(std::string()),
    m_expBarName("ExpTime"),
    m_isoBarName("ISO"),
    m_userBar(false),
    m_hwnd(NULL),
    m_hLenaDDI(nullptr),
    m_curMtx(std::make_unique<std::shared_mutex>()),
    m_frameQueue(10),
    m_writeQueue(),
    m_videoQueue()
{
    m_DevSelInfo.index = 0;
}

camera::XYZ3D::XYZ3DImpl::~XYZ3DImpl()
{
    this->m_videoQueue.stop();
    this->m_writeQueue.stop();
    this->m_frameQueue.stop();

    if (this->m_scheduledTimer != nullptr)
    {
        this->m_scheduledTimer->stop();
    }

    this->m_videoTaskSignal.store(this->STOP);
    this->m_writeTaskSignal.store(this->STOP);
    this->m_liveTaskSignal.store(this->STOP);
    this->m_captureTaskSignal.store(this->STOP);

    if (this->m_videoThread.joinable())
    {
        this->m_videoThread.join();
    }
    if (this->m_writeThread.joinable())
    {
        this->m_writeThread.join();
    }
    if (this->m_liveThread.joinable())
    {
        this->m_liveThread.join();
    }
    if (this->m_captureThread.joinable())
    {
        this->m_captureThread.join();
    }

    this->m_videoQueue.clear();
    this->m_writeQueue.clear();
    this->m_frameQueue.clear();
}

void camera::XYZ3D::XYZ3DImpl::connectTask(const std::string& pidvid, const std::string& port, camera::ImageSize imgsz, const camera::MediaAPI& api, const camera::VideoEncoding& en)
{
    LOG_INFO("Searching camera devices...");
    // 更新USB相机设备列表
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
        if (p->symbolicLink.find(pidvid) == std::string::npos)
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
            if (res->width == imgsz.width && res->height == imgsz.height)
            {
                resoID = (int)resoCnt;
                break;
            }
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
        LOG_INFO("Searching nearest resolution...");
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
    this->m_cap.reset();
    this->m_cap = std::make_unique<cv::VideoCapture>();
    switch (api)
    {
    case camera::DSHOW:
        if (!this->m_cap->open(deviceID, cv::CAP_DSHOW))
        {
            LOG_ERROR("Failed to open camera with DSHOW API.");
        }
        break;
    case camera::MSMF:
        if (!this->m_cap->open(deviceID, cv::CAP_MSMF))
        {
            LOG_ERROR("Failed to open camera with MSMF API.");
        }
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
        if (!this->m_cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', '2')))
        {
            LOG_WARNING("Failed to set camera encoding to YUY2.");
        }
        break;
    case MJPG:
        if (!this->m_cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')))
        {
            LOG_WARNING("Failed to set camera encoding to MJPG.");
        }
        break;
    default:
        break;
    }
    // 设置相机帧率
    if (!this->m_cap->set(cv::CAP_PROP_FPS, camList[deviceID].prop.fps[resoID]))
    {
        LOG_WARNING("Failed to set camera FPS to: " + std::to_string(camList[deviceID].prop.fps[resoID]));
    }
    // 设置相机分辨率宽度
    if (!this->m_cap->set(cv::CAP_PROP_FRAME_WIDTH, camList[deviceID].prop.resolution[resoID].width))
    {
        LOG_WARNING("Failed to set camera resolution width to: " + std::to_string(camList[deviceID].prop.resolution[resoID].width));
    }
    // 设置相机分辨率高度
    if (!this->m_cap->set(cv::CAP_PROP_FRAME_HEIGHT, camList[deviceID].prop.resolution[resoID].height))
    {
        LOG_WARNING("Failed to set camera resolution height to: " + std::to_string(camList[deviceID].prop.resolution[resoID].height));
    }
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

    this->startWriteThread();
}

void camera::XYZ3D::XYZ3DImpl::captureTask()
{
    LOG_INFO("Thread ID " + getThreadID() + ": Started capture thread.");
    {
        std::unique_lock<std::mutex> lock(this->m_captureMtx);
        this->m_captureTaskState.store(RUN);
        this->m_captureCV.notify_one();
    }

    // 等待相机捕获第一帧
    while (!this->m_cap->grab() and this->m_captureTaskSignal.load() == RUN)
        std::this_thread::sleep_for(10ms);
    this->m_firstFrame.store(true);

    while (this->m_captureTaskSignal.load() == RUN)
    {
        // 捕获
        if (!this->m_cap->grab())
        {
            std::this_thread::sleep_for(10ms);
            continue;
        }

        // 解码
        cv::Mat frame;
        if (!this->m_cap->retrieve(frame))
            continue;

        if (frame.empty())
            continue;

        // 更新当前帧
        {
            std::unique_lock<std::shared_mutex> lock(*this->m_curMtx);
            this->m_curFrame = frame.clone();
        }

        // 推送到视频队列, 如果队列已满则丢弃最旧的帧
        if (this->m_videoTaskState.load() == RUN)
        {
            this->m_videoQueue.pushWithDropOld(frame.clone());
        }

        // 推送到缓存队列, 如果队列已满则丢弃最旧的帧
        if (this->m_liveTaskState.load() == RUN)
        {
            this->m_frameQueue.pushWithDropOld(std::move(frame));
        }

    }

    LOG_INFO("Thread ID " + getThreadID() + "Stopped capture thread.");
    {
        std::unique_lock<std::mutex> lock(this->m_captureMtx);
        this->m_captureTaskState.store(STOP);
        this->m_captureCV.notify_one();
    }
}

void camera::XYZ3D::XYZ3DImpl::liveTask()
{
    LOG_INFO("Thread ID " + getThreadID() + ": Started live thread.");
    {
        std::unique_lock<std::mutex> lock(this->m_liveMtx);
        this->m_liveTaskState.store(RUN);
        this->m_liveCV.notify_one();
    }

    // 等待相机捕获第一帧
    while (!this->m_firstFrame.load() && this->m_liveTaskSignal.load() == RUN)
        std::this_thread::sleep_for(10ms);

    cv::namedWindow(this->m_liveWinName, cv::WINDOW_NORMAL);
    cv::resizeWindow(this->m_liveWinName, 640, 480);


    float FPS = 0.f;
    auto start = std::chrono::steady_clock::now();
    int liveFrameCount = 0;
    while (this->m_liveTaskSignal.load() == RUN)
    {
        auto frameOpt = m_frameQueue.tryFrontAndPop();

        if (frameOpt.has_value() && !frameOpt.value().empty())
        {
            liveFrameCount++;

            cv::Mat frame = std::move(frameOpt.value());

            std::string f = std::to_string(FPS);
            size_t dotPos = f.find(".");
            std::string str;
            if (dotPos != std::string::npos && dotPos + 2 < f.size())
            {
                str = "FPS:" + f.substr(0, dotPos + 2);
            }
            else
            {
                str = "FPS:" + f;
            }
            cv::putText(frame, str.c_str(), cv::Point(10, 30),
                cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0);

            cv::imshow(m_liveWinName, frame);
            cv::waitKey(1);
        }
        else
        {
            std::this_thread::sleep_for(10ms);
            cv::waitKey(10);
        }

        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        if (duration > 2000)
        {
            FPS = static_cast<float>(static_cast<float>(liveFrameCount * 1000) / duration);
            liveFrameCount = 0; // 重置局部计数器
            start = std::chrono::steady_clock::now();
        }

        // 窗口关闭检测
        if (cv::getWindowProperty(this->m_liveWinName, cv::WND_PROP_VISIBLE) < 1)
        {
            LOG_INFO("Stopping live thread...");
            this->m_liveTaskSignal.store(STOP);
        }
    }

    LOG_INFO("Thread ID " + getThreadID() + ": Stopped live thread.");
    if (cv::getWindowProperty(this->m_liveWinName, cv::WND_PROP_VISIBLE))
        cv::destroyWindow(this->m_liveWinName);
    {
        std::unique_lock<std::mutex> lock(this->m_liveMtx);
        this->m_liveTaskState.store(STOP);
        this->m_liveCV.notify_one();
    }
}

void camera::XYZ3D::XYZ3DImpl::writeTask()
{
    LOG_INFO("Thread ID " + getThreadID() + ": Started write thread.");
    {
        std::unique_lock<std::mutex> lock(this->m_writeMtx);
        this->m_writeTaskState.store(RUN);
        this->m_writeCV.notify_one();
    }

    while (this->m_writeTaskSignal.load() == RUN)
    {
        auto frameOpt = this->m_writeQueue.tryFrontAndPop();

        if (frameOpt.has_value() && !frameOpt.value().frame.empty() && !frameOpt.value().path.empty())
        {
            utils::generateNewFolder(frameOpt.value().path);
            if (cv::imwrite(frameOpt.value().path, frameOpt.value().frame))
            {
                LOG_INFO("Saved image to " + frameOpt.value().path);
            }
            else
            {
                LOG_ERROR("Failed to save image to " + frameOpt.value().path);
            }
        }
        else
        {
            std::this_thread::sleep_for(10ms);
        }
    }

    while (!this->m_writeQueue.empty())
    {
        auto frameOpt = this->m_writeQueue.tryFrontAndPop();
        if (frameOpt.has_value() && !frameOpt.value().frame.empty() && !frameOpt.value().path.empty())
        {
            utils::generateNewFolder(frameOpt.value().path);
            if (cv::imwrite(frameOpt.value().path, frameOpt.value().frame))
            {
                LOG_INFO("Saved image to " + frameOpt.value().path);
            }
            else
            {
                LOG_ERROR("Failed to save image to " + frameOpt.value().path);
            }
        }
    }

    this->m_writeQueue.stop();
    LOG_INFO("Thread ID " + getThreadID() + ": Stopped write thread.");
    {
        std::unique_lock<std::mutex> lock(this->m_writeMtx);
        this->m_writeTaskState.store(STOP);
        this->m_writeCV.notify_one();
    }
}

void camera::XYZ3D::XYZ3DImpl::videoTask(const std::string& path)
{
    LOG_INFO("Thread ID " + getThreadID() + ": Started recording thread.");
    {
        std::unique_lock<std::mutex> lock(this->m_videoMtx);
        this->m_videoTaskState.store(RUN);
        this->m_videoCV.notify_one();
    }

    while (this->m_videoTaskSignal.load() == RUN)
    {
        auto frameOpt = this->m_videoQueue.tryFrontAndPop();
        if (frameOpt.has_value() && !frameOpt.value().empty())
        {
            this->m_video->write(std::move(frameOpt.value()));
        }
        else
        {
            std::this_thread::sleep_for(10ms);
        }
    }

    while (!this->m_videoQueue.empty())
    {
        auto frameOpt = this->m_videoQueue.tryFrontAndPop();
        if (frameOpt.has_value() && !frameOpt.value().empty())
        {
            this->m_video->write(std::move(frameOpt.value()));
        }
    }

    this->m_video.release();

    LOG_INFO("Thread ID " + getThreadID() + ": Stopped recording thread.");
    {
        std::unique_lock<std::mutex> lock(this->m_videoMtx);
        this->m_videoTaskState.store(STOP);
        this->m_videoCV.notify_one();
    }
}

void camera::XYZ3D::XYZ3DImpl::startWriteThread()
{
    if (this->m_writeTaskState.load() == RUN)
    {
        LOG_WARNING("Write thread is already running.");
        return;
    }

    LOG_INFO("Starting Write Task Thread...");
    this->m_writeTaskSignal.store(RUN);

    this->m_writeThread = std::thread([this]() {
        this->writeTask();
        });

    std::unique_lock<std::mutex> lock(this->m_writeMtx);
    if (this->m_writeCV.wait_for(lock, 5s, [this]() {
        return this->m_writeTaskState.load() == RUN;
        }))
    {
        LOG_INFO("Start write thread done.");
    }
    else
    {
        std::string errstr = "Start write thread timeout.";
        LOG_ERROR(errstr);
        throw std::runtime_error(errstr);
    }
}

void camera::XYZ3D::XYZ3DImpl::stopWriteThread()
{
    LOG_INFO("Stopping write thread...");

    this->m_writeTaskSignal.store(this->STOP);

    std::unique_lock<std::mutex> lock(this->m_writeMtx);
    if (!this->m_writeCV.wait_for(lock, 10s, [this]() {
        return this->m_writeTaskState.load() == STOP;
        }))
    {
        LOG_ERROR("Timeout waiting for write thread to stop (10s)");
    }
}

void camera::XYZ3D::XYZ3DImpl::hotplugTask()
{
    std::stringstream ss;
    ss << std::this_thread::get_id();
    LOG_INFO("Thread ID " + ss.str() + ": Started hotplug task.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    LOG_INFO("Connecting to camera device " + this->m_vidpid + "...");
    connectTask(this->m_vidpid, this->m_port, this->m_imgsz, this->m_api, this->m_encoding);

    if (!isConnected())
    {
        LOG_INFO("Failed to connect to camera device " + this->m_vidpid + ", retrying...");
        for (int i = 0; i < 3; ++i)
        {
            LOG_INFO("Retrying [" + std::to_string(i + 1) + " / 3]...");
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            connectTask(this->m_vidpid, this->m_port, this->m_imgsz, this->m_api, this->m_encoding);
            if (isConnected())
            {
                LOG_INFO("Successfully reconnected to camera device " + this->m_vidpid + ".");
                break;
            }
        }
        if (!isConnected())
        {
            std::string msg = "Failed to connect to camera device " + this->m_vidpid;
            LOG_ERROR(msg);
            return;
        }
    }
    LOG_INFO("Connecting to camera device done.");

    if (this->m_capHotTaskSignal == RUN)
    {
        LOG_INFO("Start Capture Task Thread...");
        this->m_captureTaskSignal = RUN;
        //std::thread task(std::bind(&XYZ3D::XYZ3DImpl::captureTask, std::ref(this->impl)));
        std::thread task([this]() { this->captureTask(); });
        task.detach();

        std::unique_lock<std::mutex> lock(this->m_captureMtx);
        if (this->m_captureCV.wait_for(lock, std::chrono::seconds(5), [&] {return this->m_captureTaskState.load() == RUN; }))
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
    if (this->m_liveHotTaskSignal == RUN)
    {
        LOG_INFO("Starting Live Task Thread...");
        this->m_liveTaskSignal = RUN;

        //std::thread task(std::bind(&XYZ3D::XYZ3DImpl::liveTask, std::ref(this->impl), std::ref(userBar)));
        std::thread task([this]() { this->liveTask(); });
        task.detach();

        std::unique_lock<std::mutex> lock(this->m_liveMtx);
        if (this->m_liveCV.wait_for(lock, std::chrono::seconds(5), [&] {return this->m_liveTaskState.load() == RUN; }))
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
                            p->m_capHotTaskSignal = RUN;
                        }
                        if (p->m_liveTaskState == RUN)
                        {
                            p->m_liveTaskSignal = STOP;
                            p->m_liveHotTaskSignal = RUN;
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
    LOG_INFO("Thread ID " + getThreadID() + ": Started USB Device Monitor thread.");
    {
        std::unique_lock<std::mutex> lock(this->m_monitorMtx);
        this->m_monitorTaskState.store(RUN);
        this->m_monitorCV.notify_one();
    }

    auto pos1 = this->m_vidpid.find("_");
    this->m_vidstr = this->m_vidpid.substr(pos1 + 1, 4);
    pos1 = this->m_vidpid.rfind("_");
    this->m_pidstr = this->m_vidpid.substr(pos1 + 1);
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

    LOG_INFO("Thread ID " + getThreadID() + ": Stopped USB Device Monitor thread.");
    {
        std::unique_lock<std::mutex> lock(this->m_monitorMtx);
        this->m_monitorTaskState.store(STOP);
        this->m_monitorCV.notify_all();
    }

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
        LOG_INFO("Cannot disable AWB.");
    else
        LOG_INFO("Disabled AWB.");
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
    std::shared_lock<std::shared_mutex> lock(*this->m_curMtx);
    return this->m_curFrame;
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

std::string camera::XYZ3D::XYZ3DImpl::getThreadID() const
{
    std::stringstream ss;
    ss << std::this_thread::get_id();
    return ss.str();
}

bool camera::XYZ3D::XYZ3DImpl::isConnected() const
{
    if (this->m_cap == nullptr or !this->m_cap->isOpened())
        return false;
    return true;
}
