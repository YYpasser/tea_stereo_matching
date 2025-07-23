/*********************************************************************
 * @file   utils.h
 * @brief  ���ú���
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
 * @brief �½��ļ���.
 * @param [in] path �ļ���·��.
 */
void generateNewFolder(const std::string& path);
/**
 * @brief �����ļ�����ָ�����͵������ļ�.
 * @param [in] path      �ļ���·��.
 * @param [in] recursive �Ƿ�ݹ����.
 * @return �ļ��б�.
 * @throw std::filesystem_error ����ʧ��.
 */
std::vector<std::string> glob(const std::string& pattern, const bool& recursive = false);

/**
 * @brief ��ȡ��ǰʱ����ַ���.
 */
std::string getCurrentTime();
std::string getCurrentTime06lld();
/**
 * @brief ��ʽ��ʱ���.
 */
std::string formatMilliseconds(double ms);
}
