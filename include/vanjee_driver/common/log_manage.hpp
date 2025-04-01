#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <vector>
#include <algorithm>
#include <sys/stat.h>
#include <dirent.h>

namespace vanjee
{
    namespace lidar
    {
        const size_t MAX_FILE_SIZE = 100 * 1024 * 1024; // 100MB
        const size_t MAX_DIR_SIZE = 1024 * 1024 * 1024; // 1GB
        const std::string LOG_DIR = (std::string)PROJECT_PATH + "/Log/";

        class Logger {
        private:
            std::ofstream logFile;
            std::string currentFileName;
            size_t currentFileSize = 0;

            void openNewFile() {
                if (logFile.is_open()) {
                    logFile.close();
                }
                currentFileName = generateFileName();
                logFile.open(LOG_DIR + currentFileName, std::ios::app);
                currentFileSize = getFileSize(LOG_DIR + currentFileName);
            }

            std::string generateFileName() {
                auto t = std::time(nullptr);
                auto tm = *std::localtime(&t);
                std::ostringstream oss;
                oss << std::put_time(&tm, "%Y-%m-%d %H.%M.%S") << ".log";
                return oss.str();
            }

            size_t getFileSize(const std::string& fileName) {
                struct stat stat_buf;
                int rc = stat(fileName.c_str(), &stat_buf);
                return rc == 0 ? stat_buf.st_size : 0;
            }

            void checkAndCleanDirectory() {
                size_t totalSize = 0;
                std::vector<std::string> files;

                DIR* dirp = opendir(LOG_DIR.c_str());
                struct dirent* dp;
                while ((dp = readdir(dirp)) != nullptr) {
                    if (dp->d_type == DT_REG) { // Only regular files
                        std::string filePath = LOG_DIR + dp->d_name;
                        files.push_back(filePath);
                        totalSize += getFileSize(filePath);
                    }
                }
                closedir(dirp);

                if (totalSize > MAX_DIR_SIZE) {
                    std::sort(files.begin(), files.end(), [](const std::string& a, const std::string& b) {
                        struct stat a_stat, b_stat;
                        stat(a.c_str(), &a_stat);
                        stat(b.c_str(), &b_stat);
                        return a_stat.st_mtime < b_stat.st_mtime;
                    });

                    while (totalSize > MAX_DIR_SIZE && !files.empty()) {
                        totalSize -= getFileSize(files.front());
                        remove(files.front().c_str());
                        files.erase(files.begin());
                    }
                }
            }

        public:
            Logger() {
                struct stat st = {0};
                if (stat(LOG_DIR.c_str(), &st) == -1) {
                    mkdir(LOG_DIR.c_str(), 0700);
                }
                openNewFile();
                checkAndCleanDirectory();
            }

            ~Logger() {
                if (logFile.is_open()) {
                    logFile.close();
                }
            }

            void log(const std::string& message, const std::string& level) {
                if (!logFile.is_open() || currentFileSize >= MAX_FILE_SIZE) {
                    openNewFile();
                }

                auto t = std::time(nullptr);
                auto tm = *std::localtime(&t);
                std::ostringstream oss;
                oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << " [" << level << "] " << message << std::endl;
                std::string logEntry = oss.str();

                logFile << logEntry;
                logFile.flush();
                currentFileSize += logEntry.size();
            }
        };
    }  
}  
