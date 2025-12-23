#include "../include/utils.h"
#include <iostream>
#include <filesystem>
#include <sstream>   // stringstream头文件
#include <iomanip>   // put_time头文件
#include <chrono>    // 时间相关头文件
#include <algorithm>
#include <iterator>
#include <cctype>
#include <regex>
#include <functional>

namespace fs = std::filesystem;

bool utils::generateNewFolder(const std::string& path)
{
    if (path.empty())
    {
        std::cerr << "Error: Input path is empty." << std::endl;
        return false;
    }

    try
    {
        fs::path targetPath = fs::absolute(fs::path(path));
        fs::path parentPath = targetPath.parent_path();

        // 若路径是文件, 仅创建父目录
        bool isFilePath = false;
        if (fs::exists(targetPath))
        {
            isFilePath = fs::is_regular_file(targetPath);
        }
        else  // 排除如"folder.endswithdot."的情况
        {
            isFilePath = targetPath.has_extension() && !targetPath.filename().string().ends_with(".");
        }
            
        fs::path finalTargetPath = isFilePath ? parentPath : targetPath;

        // 步骤1: 创建父目录
        bool parentCreated = true;
        if (!parentPath.empty() && !fs::exists(parentPath))
        {
            parentCreated = fs::create_directories(parentPath);
            if (parentCreated)
            {
                std::cout << "Generated parent folder: \"" << parentPath.string() << "\"." << std::endl;
            }
            else
            {
                std::cerr << "Failed to generate parent folder: \"" << parentPath.string() << "\"." << std::endl;
                return false;
            }
        }

        // 步骤2: 创建目标目录(仅当目标路径≠父路径且不存在时)
        if (finalTargetPath != parentPath && !fs::exists(finalTargetPath)) 
        {
            bool targetCreated = fs::create_directories(finalTargetPath);
            if (targetCreated)
            {
                std::cout << "Generated target folder: \"" << finalTargetPath.string() << "\"." << std::endl;
                return true;
            }
            else
            {
                std::cerr << "Failed to generate target folder: \"" << finalTargetPath.string() << "\"." << std::endl;
                return false;
            }
        }

        // 路径已存在的情况
        if (finalTargetPath == parentPath)
        {
            std::cout << "Parent folder already exists: \"" << parentPath.string() << "\"." << std::endl;
        }           
        else
        {
            std::cout << "Target folder already exists: \"" << finalTargetPath.string() << "\"." << std::endl;
        }          
        
        return true;      
    }
    catch (const fs::filesystem_error& e)
    {
        std::stringstream ss;
        ss << "Filesystem error: " << e.what()<< " (Error code: " << e.code().value() << ")";
        std::cerr << ss.str() << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "General error: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unknown error occurred while generating folder." << std::endl;
    }
    return false;
}

static std::string globToRegex(const std::string& pattern)
{
    std::string regexPattern;
    for (char c : pattern)
    {
        switch (c)
        {
        case '*': regexPattern += ".*"; break;  // 匹配任意字符（0次或多次）
        case '?': regexPattern += "."; break;   // 匹配任意单个字符
        case '.': regexPattern += "\\."; break; // 匹配点字符（需要转义）
        default: regexPattern += c; break;      // 其他字符直接添加
        }
    }
    return regexPattern;
}

std::vector<std::string> utils::glob(const std::string& pattern, const bool& recursive)
{
    std::vector<std::string> result;
    size_t lastSlashPos = pattern.find_last_of("/\\");
    std::string directory = (lastSlashPos != std::string::npos) ? pattern.substr(0, lastSlashPos + 1) : ".";
    std::string filePattern = (lastSlashPos != std::string::npos) ? pattern.substr(lastSlashPos + 1) : pattern;

    std::regex regexPattern(globToRegex(filePattern), std::regex_constants::ECMAScript);

    std::function<void(const std::string&)> searchFiles = [&](const std::string& dir)
        {
            try
            {
                //std::filesystem::path wdir = std::filesystem::u8path(dir);
                for (const auto& entry : fs::directory_iterator(dir))
                {
                    std::string filename = entry.path().filename()./*u8*/string();
                    if (entry.is_regular_file() && std::regex_match(filename, regexPattern))
                    {
                        result.push_back(entry.path()./*u8*/string());
                    }
                    if (recursive && entry.is_directory())
                    {
                        searchFiles(entry.path()./*u8*/string());
                    }
                }
            }
            catch (const fs::filesystem_error& e)
            {
                // 错误处理...
            }
        };
    searchFiles(directory);
    return result;
}

std::string utils::getCurrentTime()
{
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm timeinfo;
    localtime_s(&timeinfo, &time_t_now);
    std::stringstream ss;
    ss << std::put_time(&timeinfo, "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

std::string utils::getCurrentTimeMS()
{
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm timeinfo;
    localtime_s(&timeinfo, &time_t_now);
    auto ms_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    int milliseconds = static_cast<int>(ms_since_epoch.count() % 1000);
    std::stringstream ss;
    ss << std::put_time(&timeinfo, "%Y%m%d_%H%M%S");
    ss << "_" << std::setw(3) << std::setfill('0') << milliseconds;
    return ss.str();
}

std::string utils::formatMilliseconds(double ms)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << ms;
    return oss.str();
}
