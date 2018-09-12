#ifndef _MYLOGGER_HPP__
#define _MYLOGGER_HPP__

#include <cstdio>
#include <string>

// C文件读写实现
class MyLogger {
    FILE *log_file;
public:
    // 提供类printf接口
    int log(const char *fmt, ...);

    // 将n个c风格字符串写入文件
    void log(int n, ...);

    void setLogFilePath(const std::string &fileDir, const std::string &fileName);

    MyLogger();

    MyLogger(const std::string &fileDir, const std::string &fileName);

    ~MyLogger();
};

#endif /*_MYLOGGER_HPP__*/
