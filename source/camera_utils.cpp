#include "../include/camera_utils.hpp"

// Iterator 构造函数
camera::MediaProperty::Iterator::Iterator(const MediaProperty* mediaProperty, size_t index)
	: mediaProperty_(mediaProperty), index_(index) {
}

// 前置递增
camera::MediaProperty::Iterator& camera::MediaProperty::Iterator::operator++() {
	++index_;
	return *this;
}

// 后置递增
camera::MediaProperty::Iterator camera::MediaProperty::Iterator::operator++(int) {
	Iterator temp = *this;
	++(*this);
	return temp;
}

// 解引用操作
camera::MediaProperty::Iterator::value_type camera::MediaProperty::Iterator::operator*() const {
	auto res = (index_ < mediaProperty_->resolution.size()) ? std::optional(mediaProperty_->resolution[index_]) : std::nullopt;
	auto fps = (index_ < mediaProperty_->fps.size()) ? std::optional(mediaProperty_->fps[index_]) : std::nullopt;
	auto enc = (index_ < mediaProperty_->encoding.size()) ? std::optional(mediaProperty_->encoding[index_]) : std::nullopt;
	return std::make_tuple(res, fps, enc);
}

// 不等于操作符
bool camera::MediaProperty::Iterator::operator!=(const Iterator& other) const {
	return index_ != other.index_;
}

// 等于操作符
bool camera::MediaProperty::Iterator::operator==(const Iterator& other) const {
	return index_ == other.index_;
}

// MediaProperty::begin()
camera::MediaProperty::Iterator camera::MediaProperty::begin() const {
	return Iterator(this, 0);
}

// MediaProperty::end()
camera::MediaProperty::Iterator camera::MediaProperty::end() const {
	size_t max_size = std::max({ resolution.size(), fps.size(), encoding.size() });
	return Iterator(this, max_size);
}

camera::MediaProperty& camera::MediaProperty::operator=(const camera::MediaProperty& other)
{
	if (this != &other)
	{
		this->resolution = other.resolution;
		this->fps = other.fps;
		this->encoding = other.encoding;
	}
	return *this;
}

camera::CameraInfo& camera::CameraInfo::operator=(const camera::CameraInfo& other)
{
	if (this != &other)
	{
		this->friendlyName = other.friendlyName;
		this->symbolicLink = other.symbolicLink;
		this->portLocation = other.portLocation;

		this->prop = other.prop;
	}
	return *this;
}

camera::MediaProperty camera::CameraInfo::extract(const camera::VideoEncoding& encoding)
{
	auto resolutionIt = prop.resolution.begin();
	auto fpsIt = prop.fps.begin();
	auto encodingIt = prop.encoding.begin();
	for (; encodingIt != prop.encoding.end(); )
	{
		if (encodingIt->find(camera::convertEnumToString(encoding)) == std::string::npos)
		{
			resolutionIt = prop.resolution.erase(resolutionIt);
			fpsIt = prop.fps.erase(fpsIt);
			encodingIt = prop.encoding.erase(encodingIt);
		}
		else
		{
			++resolutionIt;
			++fpsIt;
			++encodingIt;
		}
	}
	return prop;
}

camera::ImageSize& camera::ImageSize::operator=(const camera::ImageSize& other)
{
	if (this != &other)
	{
		this->width = other.width;
		this->height = other.height;
	}
	return *this;
}