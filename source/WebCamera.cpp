#include "../include/camera.h"
#include "../include/WebCameraImpl.h"
#include "../include/logger.h"
#include "../include/utils.h"
#include <chrono>

using namespace std::literals;

camera::WebCamera::WebCamera()
{
    this->impl = std::make_unique<WebCameraImpl>();
}

camera::WebCamera::~WebCamera() = default;

void camera::WebCamera::connect(const std::string& pidvid, const std::string& port, camera::ImageSize imgsz, const camera::MediaAPI& api, const camera::VideoEncoding& en, const int& retry)
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

void camera::WebCamera::startCaptureThread()
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

void camera::WebCamera::stopCaptureThread()
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

void camera::WebCamera::startLiveThread()
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

void camera::WebCamera::stopLiveThread()
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

void camera::WebCamera::release()
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

bool camera::WebCamera::isConnected()
{
    return this->impl->isConnected();
}

cv::Mat camera::WebCamera::getFrame()
{
    return this->impl->readFrame();
}

void camera::WebCamera::writeFrame(const std::string& path)
{
    if (!this->isConnected() || this->impl->m_writeTaskState.load() == this->impl->STOP)
    {
        LOG_ERROR ("Camera is not connected or write task is not running.");
        return;
    }

    if (path.empty())
    {
        LOG_ERROR("Path is empty.");
        return;
    }

    this->impl->m_writeQueue.push(std::move(WriteData(path, std::move(this->impl->readFrame()))));
}

void camera::WebCamera::startScheduledCapture(const std::string& path, std::chrono::milliseconds interval)
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

void camera::WebCamera::stopScheduledCapture()
{
    this->impl->m_scheduledTimer->stop();
}

void camera::WebCamera::startRecording(const std::string& path)
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

void camera::WebCamera::stopRecording()
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

camera::WebCamera::WebCameraImpl::WebCameraImpl() :
    m_captureTaskState{ STOP },
    m_liveTaskState{ STOP },
    m_writeTaskState{ STOP },
    m_videoTaskState{ STOP },
    m_captureTaskSignal{ STOP },
    m_liveTaskSignal{ STOP },
    m_writeTaskSignal{ STOP },
    m_videoTaskSignal{ STOP },
    m_firstFrame{ false },
    m_api(DSHOW),
    m_encoding(MJPG),
    m_vidpid(std::string()),
    m_port(std::string()),
    m_imgsz(ImageSize()),
    m_curMtx(std::make_unique<std::shared_mutex>()),
    m_frameQueue(10),
    m_writeQueue(),
    m_videoQueue()
{
}

camera::WebCamera::WebCameraImpl::~WebCameraImpl()
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

void camera::WebCamera::WebCameraImpl::connectTask(const std::string& pidvid, const std::string& port, camera::ImageSize imgsz, const camera::MediaAPI& api, const camera::VideoEncoding& en)
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
    for (auto p = camList.begin(); p != camList.end(); ++p)
    {
        auto i = std::distance(camList.begin(), p);
        if (p->symbolicLink.find(pidvid) == std::string::npos)
            continue;

        if (!port.empty() and p->portLocation != port)
            continue;

        deviceID = static_cast<int>(i);
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

    std::string portName = camList[deviceID].portLocation;
    size_t pos = 0;
    for (int i = 0; i < 5; ++i)
        pos = portName.find(".", pos) + 1;
    portName = portName.substr(0, pos - 1);
    this->m_liveWinName = "[" + portName + "] Live";

    this->startWriteThread();
}

void camera::WebCamera::WebCameraImpl::captureTask()
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

void camera::WebCamera::WebCameraImpl::liveTask()
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

void camera::WebCamera::WebCameraImpl::writeTask()
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

void camera::WebCamera::WebCameraImpl::videoTask(const std::string& path)
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

void camera::WebCamera::WebCameraImpl::startWriteThread()
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

void camera::WebCamera::WebCameraImpl::stopWriteThread()
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

cv::Mat camera::WebCamera::WebCameraImpl::readFrame()
{
    std::shared_lock<std::shared_mutex> lock(*this->m_curMtx);
    return this->m_curFrame;
}

std::string camera::WebCamera::WebCameraImpl::getThreadID() const
{
    std::stringstream ss;
    ss << std::this_thread::get_id();
    return ss.str();
}

bool camera::WebCamera::WebCameraImpl::isConnected() const
{
    if (this->m_cap == nullptr or !this->m_cap->isOpened())
        return false;
    return true;
}
