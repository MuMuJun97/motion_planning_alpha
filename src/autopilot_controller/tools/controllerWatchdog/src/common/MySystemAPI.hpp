#ifndef _MYSYSTEMAPI_HPP__
#define _MYSYSTEMAPI_HPP__

#include <string>

namespace MySystemAPI {

// 类似system()，但可通过管道将shell输出返回为string
    std::string exec_pipe(const char *cmd);

// 类似system()，但不阻塞父进程，且返回子进程pid
    pid_t exec(const std::string &path, char *const argv[]);

// file system
    bool createDir(const std::string &DirName);

    std::string getPWD();

    int setMOD(const std::string &path, mode_t mode);

    std::string getFileNameFromPath(const std::string &path);

    std::string getFileNameWithExtensionFromPath(const std::string &path);

    std::string getFileExtensionFromPath(const std::string &path);

    std::string getFileDirFromPath(const std::string &path);

}

#endif /*_MYSYSTEMAPI_HPP__*/
