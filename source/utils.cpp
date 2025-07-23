#include "../include/utils.h"
#include "../include/logger.h"
#include <chrono>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <regex>
#include <functional>

namespace fs = std::filesystem;

void utils::generateNewFolder(const std::string& path)
{
    try
    {
        fs::path target_path = fs::path(path);  // ����·��
        fs::path parent_path = target_path.parent_path();  // ����·���ĸ�Ŀ¼

        //std::cout << "Target path: " << target_path.string() << "\n";
        //std::cout << "Parent path: " << parent_path.string() << "\n";

        // ���Ŀ��·����һ���ļ���������չ��������ֻ�����丸Ŀ¼
        if (target_path.has_extension())
        {
            //std::cout << "The input path is a file path. Creating parent folder instead.\n";
            target_path = parent_path;  // ��Ŀ��·����Ϊ��Ŀ¼
        }

        // �����Ŀ¼�����ڣ����Դ�����Ŀ¼
        if (!fs::exists(parent_path))
        {
            if (fs::create_directories(parent_path))
                LOG_INFO("Generated folder: \"" + parent_path.string() + "\".");
            else
                LOG_ERROR("Cannot generate folder: \"" + parent_path.string() + "\".");
        }
        else
        {
            LOG_DEBUG("Parent folder already exists: \"" + parent_path.string() + "\".");
        }

        // ���Ŀ��·�����ļ����Ҳ����ڣ����Դ���Ŀ��·��
        if (target_path != parent_path && !fs::exists(target_path))
        {
            if (fs::create_directories(target_path))
                LOG_INFO("Generated folder: \"" + target_path.string() + "\".");
            else
                LOG_ERROR("Cannot generate folder: \"" + target_path.string() + "\".");
        }
        else if (target_path != parent_path)
        {
            LOG_DEBUG("Target folder already exists: \"" + target_path.string() + "\".");
        }
    }
    catch (const fs::filesystem_error& e)
    {
        std::stringstream ss;
        ss << "Error: " << e.what() << " (Error code: " << e.code().value() << ").";
        LOG_ERROR(ss.str());
    }
}

static std::string globToRegex(const std::string& pattern)
{
    std::string regexPattern;
    for (char c : pattern)
    {
        switch (c)
        {
        case '*': regexPattern += ".*"; break;  // ƥ�������ַ���0�λ��Σ�
        case '?': regexPattern += "."; break;   // ƥ�����ⵥ���ַ�
        case '.': regexPattern += "\\."; break; // ƥ����ַ�����Ҫת�壩
        default: regexPattern += c; break;      // �����ַ�ֱ�����
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
                // ������...
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

std::string utils::formatMilliseconds(double ms)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << ms;
    return oss.str();
}
