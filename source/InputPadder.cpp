#include "../include/stereo.h"

class stereo::InputPadder::InputPadderImpl
{
public:
	InputPadderImpl() {
		const cv::Size imgsz = cv::Size(1280, 720);
		const int divis_by = 32;
		int ht = imgsz.height;
		int wd = imgsz.width;
		int pad_ht = (((ht / divis_by) + 1) * divis_by - ht) % divis_by;
		int pad_wd = (((wd / divis_by) + 1) * divis_by - wd) % divis_by;
		this->m_pad = { pad_wd / 2, pad_wd - pad_wd / 2,pad_ht / 2, pad_ht - pad_ht / 2 };	
	};
	InputPadderImpl(const cv::Size& imgsz, const int& divis_by) {
		int ht = imgsz.height;
		int wd = imgsz.width;
		int pad_ht = (((ht / divis_by) + 1) * divis_by - ht) % divis_by;
		int pad_wd = (((wd / divis_by) + 1) * divis_by - wd) % divis_by;
		this->m_pad = { pad_wd / 2, pad_wd - pad_wd / 2,pad_ht / 2, pad_ht - pad_ht / 2 };
	}
    ~InputPadderImpl() {};
public:
	std::vector<int> m_pad;
};

stereo::InputPadder::InputPadder()
{
	this->impl = std::make_unique<InputPadderImpl>();
}

stereo::InputPadder::InputPadder(const cv::Size& imgsz, const int& divis_by)
{
    this->impl = std::make_unique<InputPadderImpl>(imgsz, divis_by);
}

stereo::InputPadder::~InputPadder()
{

}

std::vector<cv::Mat> stereo::InputPadder::pad(const std::vector<cv::Mat>& imgs)
{
	std::vector<cv::Mat> padded_inputs;
	for (auto& img : imgs) {
		cv::Mat padded_img;
		cv::copyMakeBorder(img, padded_img, this->impl->m_pad[2], this->impl->m_pad[3],
			this->impl->m_pad[0], this->impl->m_pad[1], cv::BORDER_REPLICATE);
		padded_inputs.push_back(padded_img);
	}
	return padded_inputs;
}

cv::Mat stereo::InputPadder::unpad(const cv::Mat& disparity)
{
	int ht = disparity.rows;
	int wd = disparity.cols;
	int c[] = { this->impl->m_pad[2], ht - this->impl->m_pad[3],
		this->impl->m_pad[0], wd - this->impl->m_pad[1] };
	return disparity(cv::Rect(c[2], c[0], c[3] - c[2], c[1] - c[0]));
}
