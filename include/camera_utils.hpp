/*********************************************************************
 * @file   camera_utils.hpp
 * @brief  ��ȡ�豸�е�����б�, ��ӦOpenCV��deviceID
 * @author Qianyao Zhuang
 * @date   May 2025
 * 
 * @code demo 1 - ��ȡ�豸�е�����б�
 	camera::CameraList cam;
	cam.update(camera::MediaAPI::DSHOW);
	cam.extract(camera::VideoEncoding::YUY2);
	cam.print();
	auto a = cam.getList();
	for (const auto& i : a)
	{
		std::cout << i.friendlyName << std::endl;
		std::cout << i.symbolicLink << std::endl;
		std::cout << i.portLocation << std::endl;
		for (auto j = i.prop.begin(); j != i.prop.end(); j++)
		{
			auto [res, fps, enc] = *j;
			std::stringstream ss;
			ss<< "Resolution: " << (res ? std::to_string(res->width) + "x" + std::to_string(res->height) : "N/A") << ", "
								<< "FPS: " << (fps ? std::to_string(*fps) : "N/A") << ","
								<< "Encoder: " << (enc ? *enc : "N/A");
			LOG_INFO_MSG(ss.str());
		}
	}
 * @endcode demo 1 - ��ȡ�豸�е�����б�
 *********************************************************************/
#pragma once
#include <vector>
#include <string>
#include <tuple>
#include <optional>
#include <memory>

namespace camera
{

/**
 * @brief ý��ӿ�.
 */
enum MediaAPI
{
	DSHOW = 0, /*!< DirectShow */
	MSMF = 1, /*!< Microsoft Media Foundation */
};

/**
 * @brief ��Ƶ�����ʽ.
 */
enum VideoEncoding
{
	MJPG = 0, /*!< FOURCC = 'M','J','P','G' */
	YUY2 = 1, /*!< FOURCC = 'Y','U','Y','2' */
};

/**
 * @brief ö�����Ͷ�Ӧ���ַ���.
 */
template<typename T>
struct EnumStr {
	static std::vector<std::string> List;
};
std::vector<std::string> EnumStr<MediaAPI>::List = {
	"DirectShow (DSHOW)",
	"Microsoft Media Foundation (MSMF)" 
};
std::vector<std::string> EnumStr<VideoEncoding>::List = {
	"MJPG",
	"YUY2" 
};
/**
 * @brief ���ַ���ת��Ϊö������.
 * @param [in] pStr �ַ���
 * @return ö������
 */
template<typename T>
T convertStringToEnum(std::string pStr) {
	T fooEnum = static_cast<T>(-1);
	for (int i = 0; i < EnumStr<T>::List.size(); ++i)
	{
		if (!pStr.compare(EnumStr<T>::List[i]))
		{
			fooEnum = static_cast<T>(i);
			break;
		}
	}
	return fooEnum;
}

/**
 * @brief ��ö������ת��Ϊ�ַ���.
 * @param [in] paEnum ö������
 * @return ö�����Ͷ�Ӧ���ַ���
 */
template<typename T>
std::string convertEnumToString(T paEnum) {
	return EnumStr<T>::List[paEnum];
}

/**
 * @brief ����ֱ���.
 */
struct ImageSize
{
	int width;  /*!< ��� */
	int height; /*!< �߶� */
	ImageSize() : width(0), height(0) {};
	ImageSize(int w, int h) : width(w), height(h) {};
	ImageSize& operator=(const ImageSize& other);
};

/**
 * @brief ý�����Ա�.
 */
class MediaProperty
{
public:
	std::vector<camera::ImageSize> resolution; /*!< �ֱ��� */
	std::vector<float> fps;                    /*!< ֡�� */
	std::vector<std::string> encoding;         /*!< �����ʽ,YUY2,MJPG... */

	class Iterator
	{
	public:
		using value_type = std::tuple<std::optional<camera::ImageSize>, std::optional<float>, std::optional<std::string>>;

		Iterator(const MediaProperty* mediaProperty, size_t index);

		Iterator& operator++();
		Iterator operator++(int);

		value_type operator*() const;

		bool operator!=(const Iterator& other) const;
		bool operator==(const Iterator& other) const;

	private:
		const MediaProperty* mediaProperty_;
		size_t index_;
	};

	Iterator begin() const;
	Iterator end() const;

	MediaProperty& operator=(const MediaProperty& other);
};

/**
 * @brief �����Ϣ��.
 */
class CameraInfo
{
public:
	CameraInfo() = default;
	~CameraInfo() = default;
	CameraInfo& operator=(const CameraInfo& other);
public:
	/**
	 * @brief ��ȡָ�������ʽ�����Ա�.
	 * @param [in] encoding ָ�������ʽ
	 * @return ָ�������ʽ�����Ա�
	 */
	camera::MediaProperty extract(const camera::VideoEncoding& encoding);
public:
	std::string friendlyName; /*!< ����Ѻ����� */
	std::string symbolicLink; /*!< �豸ʵ��·�� */
	std::string portLocation; /*!< �豸λ�� */
	camera::MediaProperty prop; /*!< ��������б� */
};

/**
 * @brief ��ȡ����б�.
 */
class CameraList
{
public:
	CameraList() {};
	~CameraList() {};
	CameraList& operator=(const CameraList& other);
public:
	/**
	 * @brief ��������б�.
	 * @param [in] mediaAPI ָ��ý��API
	 */
	void update(const camera::MediaAPI& mediaAPI);
	/**
	 * @brief ��ȡ����б�.
	 * @param [in] encoding ָ�������ʽ
	 */
	void extract(const camera::VideoEncoding& encoding);
	/**
	 * @brief ��ȡ����б�.
	 * @return ����б�
	 */
	std::vector<camera::CameraInfo> getList();
	/**
	 * @brief ��ӡ����б�.
	 */
	void print();
public:
	std::vector<camera::CameraInfo> list; /*!< ����б� */
};
}
