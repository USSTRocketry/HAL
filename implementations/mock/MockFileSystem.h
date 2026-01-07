#pragma once

#include "abstractions/IStorage.h"
#include "implementations/mock/MockFile.h"
#include <unordered_map>
#include <memory>

namespace HAL {

class MockFileSystem : public IFileSystem {
public:
    explicit MockFileSystem(const std::string& basePath = "./data");
    ~MockFileSystem() override;

    IFile* OpenFile(const char* path, const char* mode = "w") override;
    Result RemoveFile(const char* path) override;
    bool Exists(const char* path) override;
    std::vector<std::string> ListFiles(const char* dir = "/") override;
    bool Initialize() override;

private:
    std::string basePath_;
    std::unordered_map<std::string, std::unique_ptr<MockFile>> openFiles_;

    std::string GetFullPath(const char* path);
};

} // namespace HAL
