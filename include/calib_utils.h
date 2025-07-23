/*********************************************************************
 * @file   calib_utils.h
 * @brief  �궨������ṹ
 * @author Qianyao Zhuang
 * @date   May 2025
 *********************************************************************/
#pragma once
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <memory>

/**
 * @brief ���̸�궨������趨.
 */
struct ChessboardParams
{
	int board_width;   /*!< ���̸����� */
	int board_height;  /*!< ���̸����� */
	float square_size; /*!< ���̸���ӿ�� */

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
	std::vector<std::vector<cv::Point3f>> world_points; /*!< ��������ϵ�����̸�ǵ����� */
	std::vector<std::vector<cv::Point2f>> pixel_points; /*!< ��������ϵ�����̸�ǵ����� */

	ChessboardCorners() = default;
	ChessboardCorners(const std::vector<std::vector<cv::Point3f>>& world_points, std::vector<std::vector<cv::Point2f>>& pixel_points);
	ChessboardCorners(const ChessboardCorners& other);
	ChessboardCorners(ChessboardCorners&& other) noexcept;
	~ChessboardCorners() = default;
};

/**
 * @brief �������̸�궨��ͼ��.
 */
struct GoodChessboardImage
{
	std::string file_name; /*!< ͼ��·�� */
	cv::Mat image;         /*!< ͼ�� */
	int image_id;          /*!< ͼ��ID */

	GoodChessboardImage();
	GoodChessboardImage(const std::string& file_name, const cv::Mat& image, const int& image_id);
	GoodChessboardImage(const GoodChessboardImage& other);
	GoodChessboardImage(GoodChessboardImage&& other) noexcept;
	GoodChessboardImage& operator=(const GoodChessboardImage& other);
	GoodChessboardImage& operator=(GoodChessboardImage&& other) noexcept;
	~GoodChessboardImage() = default;
};
