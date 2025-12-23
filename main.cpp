#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include "./include/calib.h"
#include "./include/stereo.h"
#include "./include/stereo_utils.h"
#include "./include/utils.h"
#include "./include/timer.h"
#include "./include/camera.h"
#include "./include/camera_utils.h"
#include <chrono>

using namespace std::literals;

int main()
{
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
	return 0;
}
