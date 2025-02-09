#ifndef FILES_WRAP
#define FILES_WRAP
#include "MinWin.h"
#include <string>
#include <vector>

namespace filesWrap{
    bool pathExist(std::string path){
        return GetFileAttributesA(path.c_str()) != INVALID_FILE_ATTRIBUTES;
    }

    bool isDirectory(std::string path){
        return (GetFileAttributesA(path.c_str()) & FILE_ATTRIBUTE_DIRECTORY) != 0;
    }

    bool createDirectory(std::string filename){
        return CreateDirectoryA(filename.c_str(),nullptr);
    }

    bool writeFile(std::string filename, uint8_t* data, uint64_t size){
        uint64_t type = pathExist(filename) ? TRUNCATE_EXISTING : CREATE_NEW;
        HANDLE file = CreateFileA(filename.c_str(),GENERIC_WRITE,FILE_SHARE_DELETE | FILE_SHARE_READ | FILE_SHARE_WRITE,NULL,type,FILE_ATTRIBUTE_NORMAL,nullptr);
        DWORD err = GetLastError();
        if(err != 0) return err;
        DWORD written = 0;
        WriteFile(file,data,size,&written,nullptr);
        CloseHandle(file);
        if(written != size) return false;
        return true;
    }

    std::vector<uint8_t>* readFile(std::string filename){
        if(!pathExist(filename)) return nullptr;
        HANDLE file = CreateFileA(filename.c_str(),GENERIC_READ,FILE_SHARE_DELETE | FILE_SHARE_READ | FILE_SHARE_WRITE,NULL,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr);
        if(GetLastError()) return nullptr;
        
        uint32_t maxSize = GetFileSize(file,nullptr);
        std::vector<uint8_t>* data = new std::vector<uint8_t>(maxSize,0);
        ReadFile(file,data->data(),maxSize,nullptr, nullptr);
        return data;
    }

    std::string getEnvVar(std::string name){
        std::vector<char> buffer(32767,0);
        uint32_t readCount = GetEnvironmentVariableA(name.c_str(),buffer.data(),buffer.size());
        buffer.resize(readCount,0);
        std::string out(buffer.data());
        return out;
    }

    bool removeFile(std::string filename){
        if(isDirectory(filename)){
            return RemoveDirectoryA(filename.c_str());
        }else{
            return DeleteFileA(filename.c_str());
        }
    }
}


#endif