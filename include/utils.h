/*********************************************************************
 * @file   utils.h
 * @brief  常用函数
 * @author Qianyao Zhuang
 * @date   May 2025
 *********************************************************************/
#pragma once
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <memory>

namespace utils
{

/**
 * @brief 新建文件夹.
 * @param [in] path 文件夹路径.
 */
void generateNewFolder(const std::string& path);
/**
 * @brief 检索文件夹内指定类型的所有文件.
 * @param [in] path      文件夹路径.
 * @param [in] recursive 是否递归检索.
 * @return 文件列表.
 * @throw std::filesystem_error 检索失败.
 */
std::vector<std::string> glob(const std::string& pattern, const bool& recursive = false);

/**
 * @brief 获取当前时间戳字符串.
 */
std::string getCurrentTime();
std::string getCurrentTime06lld();
/**
 * @brief 格式化时间戳.
 */
std::string formatMilliseconds(double ms);
}
