#pragma once
#include <string>
#include <vector>


namespace utils
{

/**
 * @brief 新建文件夹.
 * @param [in] path 文件夹路径(GB2312).
 * @return true: 创建成功/已存在; false: 创建失败.
 */
bool generateNewFolder(const std::string& path);
/**
 * @brief 检索文件夹内指定类型的所有文件.
 * @param [in] path      文件夹路径.
 * @param [in] recursive 是否递归检索.
 * @return 文件列表.
 */
std::vector<std::string> glob(const std::string& pattern, const bool& recursive = false);
/**
 * @brief 获取当前时间.
 */
std::string getCurrentTime();
std::string getCurrentTimeMS();
std::string formatMilliseconds(double ms);
class cout
{

};


}
