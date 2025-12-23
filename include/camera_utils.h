/*********************************************************************
 * @file   camera_utils.hpp
 * @brief  获取设备中的相机列表, 对应OpenCV的deviceID
 * @author Qianyao Zhuang
 * @date   May 2025
 *
 * @code demo 1 - 获取设备中的相机列表
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
 * @endcode demo 1 - 获取设备中的相机列表
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
	 * @brief 媒体接口.
	 */
	enum MediaAPI
	{
		DSHOW = 0, /*!< DirectShow */
		MSMF = 1, /*!< Microsoft Media Foundation */
	};

	/**
	 * @brief 视频编码格式.
	 */
	enum VideoEncoding
	{
		MJPG = 0, /*!< FOURCC = 'M','J','P','G' */
		YUY2 = 1, /*!< FOURCC = 'Y','U','Y','2' */
	};

	/**
	 * @brief 枚举类型对应的字符串.
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
	 * @brief 将字符串转换为枚举类型.
	 * @param [in] pStr 字符串
	 * @return 枚举类型
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
	 * @brief 将枚举类型转换为字符串.
	 * @param [in] paEnum 枚举类型
	 * @return 枚举类型对应的字符串
	 */
	template<typename T>
	std::string convertEnumToString(T paEnum) {
		return EnumStr<T>::List[paEnum];
	}

	/**
	 * @brief 相机分辨率.
	 */
	struct ImageSize
	{
		int width;  /*!< 宽度 */
		int height; /*!< 高度 */
		ImageSize() : width(0), height(0) {};
		ImageSize(int w, int h) : width(w), height(h) {};
		ImageSize& operator=(const ImageSize& other);
	};

	/**
	 * @brief 媒体属性表.
	 */
	class MediaProperty
	{
	public:
		std::vector<camera::ImageSize> resolution; /*!< 分辨率 */
		std::vector<float> fps;                    /*!< 帧率 */
		std::vector<std::string> encoding;         /*!< 编码格式,YUY2,MJPG... */

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
	 * @brief 相机信息类.
	 */
	class CameraInfo
	{
	public:
		CameraInfo() = default;
		~CameraInfo() = default;
		CameraInfo& operator=(const CameraInfo& other);
	public:
		/**
		 * @brief 提取指定编码格式的属性表.
		 * @param [in] encoding 指定编码格式
		 * @return 指定编码格式的属性表
		 */
		camera::MediaProperty extract(const camera::VideoEncoding& encoding);

	public:
		std::string friendlyName; /*!< 相机友好名称 */
		std::string symbolicLink; /*!< 设备实例路径 */
		std::string portLocation; /*!< 设备位置 */
		camera::MediaProperty prop; /*!< 相机属性列表 */
	};

	/**
	 * @brief 获取相机列表.
	 */
	class CameraList
	{
	public:
		CameraList() {};
		~CameraList() {};
		CameraList& operator=(const CameraList& other);
	public:
		/**
		 * @brief 更新相机列表.
		 * @param [in] mediaAPI 指定媒体API
		 */
		void update(const camera::MediaAPI& mediaAPI);
		/**
		 * @brief 提取相机列表.
		 * @param [in] encoding 指定编码格式
		 */
		void extract(const camera::VideoEncoding& encoding);
		/**
		 * @brief 获取相机列表.
		 * @return 相机列表
		 */
		std::vector<camera::CameraInfo> getList();
		/**
		 * @brief 打印相机列表.
		 */
		void print();
	public:
		std::vector<camera::CameraInfo> list; /*!< 相机列表 */
	};
}
