/*********************************************************************
 * @file   calib_utils.h
 * @brief  标定板参数结构
 * @author Qianyao Zhuang
 * @date   May 2025
 *********************************************************************/
#pragma once
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <memory>

/**
 * @brief 棋盘格标定板参数设定.
 */
struct ChessboardParams
{
	int board_width;   /*!< 棋盘格列数 */
	int board_height;  /*!< 棋盘格行数 */
	float square_size; /*!< 棋盘格格子宽度 */

	ChessboardParams();
	ChessboardParams(const int& board_width, const int& board_height, const float& square_size);
	ChessboardParams(const ChessboardParams& other);
	ChessboardParams(ChessboardParams&& other) noexcept;
	ChessboardParams& operator=(const ChessboardParams& other);
	ChessboardParams& operator=(ChessboardParams&& other) noexcept;
	~ChessboardParams() = default;
};

struct ChessboardCorners
{
	std::vector<std::vector<cv::Point3f>> world_points; /*!< 世界坐标系下棋盘格角点坐标 */
	std::vector<std::vector<cv::Point2f>> pixel_points; /*!< 像素坐标系下棋盘格角点坐标 */

	ChessboardCorners() = default;
	ChessboardCorners(const std::vector<std::vector<cv::Point3f>>& world_points, std::vector<std::vector<cv::Point2f>>& pixel_points);
	ChessboardCorners(const ChessboardCorners& other);
	ChessboardCorners(ChessboardCorners&& other) noexcept;
	~ChessboardCorners() = default;
};

/**
 * @brief 优质棋盘格标定板图像.
 */
struct GoodChessboardImage
{
	std::string file_name; /*!< 图像路径 */
	cv::Mat image;         /*!< 图像 */
	int image_id;          /*!< 图像ID */

	GoodChessboardImage();
	GoodChessboardImage(const std::string& file_name, const cv::Mat& image, const int& image_id);
	GoodChessboardImage(const GoodChessboardImage& other);
	GoodChessboardImage(GoodChessboardImage&& other) noexcept;
	GoodChessboardImage& operator=(const GoodChessboardImage& other);
	GoodChessboardImage& operator=(GoodChessboardImage&& other) noexcept;
	~GoodChessboardImage() = default;
};
