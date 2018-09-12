#include "MySystemAPI.hpp"
#include <iostream>

#include <cstdio>
#include <cstring>

#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <signal.h>

std::string MySystemAPI::exec_pipe(const char *cmd) {
    char buff[256];
    FILE *pp = NULL;
    std::string shellReturnStr("");

    pp = popen(cmd, "r");
    if (!pp)
        return NULL;

    while (fgets(buff, sizeof(buff), pp) != NULL) {
        shellReturnStr.append(buff, strlen(buff));
    }

    if (pp)
        pclose(pp);

    return shellReturnStr;
}

pid_t MySystemAPI::exec(const std::string &path, char *const argv[]) {
    const pid_t child_pid = fork();
    if (child_pid < 0) {
        std::cerr << "fork error" << std::endl;
        return -1;
    } else if (child_pid == 0) { /*child*/
        execv(path.c_str(), argv);
    } else {                     /*father*/
        signal(SIGCHLD, SIG_IGN);
        // waitpid(child_pid, NULL, 0);  // act like system() if call waitpid
    }
    return child_pid;
}

bool MySystemAPI::createDir(const std::string &DirName) {
    return mkdir(DirName.c_str(), 0777) == 0;   // can not make a dir if return none-zero
}

std::string MySystemAPI::getPWD() {
    char cwd[256];
    return getcwd(cwd, 256) == NULL ? "" : std::string(cwd);
}

int MySystemAPI::setMOD(const std::string &path, mode_t mode) {
    return chmod(path.c_str(), mode);
}

std::string MySystemAPI::getFileNameFromPath(const std::string &path) {
    int posBegin = path.find_last_of('/');
    int posEnd = path.find_last_of('.');
    return std::string(path.substr(posBegin + 1, posEnd - posBegin - 1));
}

std::string MySystemAPI::getFileNameWithExtensionFromPath(const std::string &path) {
    int posBegin = path.find_last_of('/');
    return std::string(path.substr(posBegin + 1));
}

std::string MySystemAPI::getFileExtensionFromPath(const std::string &path) {
    int posBegin = path.find_last_of('.');
    return std::string(path.substr(posBegin));
}

std::string MySystemAPI::getFileDirFromPath(const std::string &path) {
    int posEnd = path.find_last_of('/');
    return std::string(path.substr(0, posEnd + 1));
}
