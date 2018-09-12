#include <cstdarg>
#include <cassert>
#include "MyLogger.hpp"
#include "mytimer.hpp"

int MyLogger::log(const char *fmt, ...) {
    assert(log_file != NULL);
    va_list args;
    int n;

    va_start(args, fmt);
    n = vfprintf(log_file, fmt, args);
    va_end(args);

    return n;
}

void MyLogger::log(int n, ...) {
    assert(log_file != NULL);
    assert(n >= 1);

    va_list args;
    va_start(args, n);

    while (--n)
        fprintf(log_file, "%s ", va_arg(args, const char*));

    fprintf(log_file, "%s\n", va_arg(args, const char*));

    va_end(args);
}

void MyLogger::setLogFilePath(const std::string &fileDir, const std::string &fileName) {
    log_file = fopen((fileDir + fileName).c_str(), "w");
    assert(log_file != NULL);

    const struct timeval tv = mytimer::getBeijingTimeval();
    mytimer::YYYYMMDDCStr curDate;
    mytimer::extractYYYYMMDD(&tv, curDate);
    mytimer::HHMMSSUSCStr curTime;
    mytimer::extractHHMMSSUS(&tv, curTime);

    fprintf(log_file, "%s %s\n", curDate, curTime);
}

MyLogger::MyLogger() : log_file(NULL) {}

MyLogger::MyLogger(const std::string &fileDir, const std::string &fileName) {
    setLogFilePath(fileDir, fileName);
}

MyLogger::~MyLogger() {
    if (log_file != NULL)
        fclose(log_file);
}
