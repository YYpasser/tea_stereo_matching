#include "../include/calib_utils.h"

ChessboardParams::ChessboardParams() : board_width(NULL), board_height(NULL), square_size(0.0f)
{
}

ChessboardParams::ChessboardParams(const int& board_width, const int& board_height, const float& square_size) : board_width(board_width), board_height(board_height), square_size(square_size)
{
}

ChessboardParams::ChessboardParams(const ChessboardParams& other) : board_width(other.board_width), board_height(other.board_height), square_size(other.square_size)
{
}

ChessboardParams::ChessboardParams(ChessboardParams&& other) noexcept : board_width(std::move(other.board_width)), board_height(std::move(other.board_height)), square_size(std::move(other.square_size))
{
}

ChessboardParams& ChessboardParams::operator=(const ChessboardParams& other)
{
	if (this != &other) {
		board_width = other.board_width;
		board_height = other.board_height;
		square_size = other.square_size; 
	}
	return *this;
}

ChessboardParams& ChessboardParams::operator=(ChessboardParams&& other) noexcept
{
	if (this != &other) {
		board_width = std::move(other.board_width);
		board_height = std::move(other.board_height);
		square_size = std::move(other.square_size);
	}
	return *this;
}

ChessboardCorners::ChessboardCorners(const std::vector<std::vector<cv::Point3f>>& world_points, std::vector<std::vector<cv::Point2f>>& pixel_points) : world_points(world_points), pixel_points(pixel_points)
{
}

ChessboardCorners::ChessboardCorners(const ChessboardCorners& other) : world_points(other.world_points), pixel_points(other.pixel_points)
{
}

ChessboardCorners::ChessboardCorners(ChessboardCorners&& other) noexcept : world_points(std::move(other.world_points)), pixel_points(std::move(other.pixel_points))
{
}

GoodChessboardImage::GoodChessboardImage() : file_name(std::string()), image(cv::Mat()), image_id(-1)
{
}

GoodChessboardImage::GoodChessboardImage(const std::string& file_name, const cv::Mat& image, const int& image_id) : file_name(file_name), image(image), image_id(image_id)
{
}

GoodChessboardImage::GoodChessboardImage(const GoodChessboardImage& other) : file_name(other.file_name), image(other.image), image_id(other.image_id)
{
}

GoodChessboardImage::GoodChessboardImage(GoodChessboardImage&& other) noexcept : file_name(std::move(other.file_name)), image(std::move(other.image)), image_id(std::move(other.image_id))
{
}

GoodChessboardImage& GoodChessboardImage::operator=(const GoodChessboardImage& other)
{
	if (this != &other) {
		file_name = other.file_name;
		image = other.image;
		image_id = other.image_id;
	}
	return *this;
}

GoodChessboardImage& GoodChessboardImage::operator=(GoodChessboardImage&& other) noexcept
{
	if (this != &other) {
		file_name = std::move(other.file_name);
		image = std::move(other.image);
		image_id = std::move(other.image_id);
	}
	return *this;
}
