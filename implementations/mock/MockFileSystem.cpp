#include "implementations/mock/MockFileSystem.h"
#include <filesystem>

namespace HAL {

MockFileSystem::MockFileSystem(const std::string& basePath)
    : basePath_(basePath) {}

MockFileSystem::~MockFileSystem() {
    openFiles_.clear();
}

bool MockFileSystem::Initialize() {
    if (!std::filesystem::exists(basePath_)) {
        return std::filesystem::create_directories(basePath_);
    }
    return true;
}

IFile* MockFileSystem::OpenFile(const char* path, const char* mode) {
    std::string fullPath = GetFullPath(path);

    auto it = openFiles_.find(path);
    if (it != openFiles_.end()) {
        openFiles_.erase(it);
    }

    std::filesystem::path filePath(fullPath);
    if (filePath.has_parent_path()) {
        std::filesystem::create_directories(filePath.parent_path());
    }

    auto file = std::make_unique<MockFile>(fullPath, mode);
    if (!file->IsOpen()) {
        return nullptr;
    }

    IFile* filePtr = file.get();
    openFiles_[path] = std::move(file);
    return filePtr;
}

Result MockFileSystem::RemoveFile(const char* path) {
    std::string fullPath = GetFullPath(path);

    auto it = openFiles_.find(path);
    if (it != openFiles_.end()) {
        openFiles_.erase(it);
    }

    std::error_code ec;
    bool removed = std::filesystem::remove(fullPath, ec);
    return removed ? Result::Success : Result::Fail;
}

bool MockFileSystem::Exists(const char* path) {
    return std::filesystem::exists(GetFullPath(path));
}

std::vector<std::string> MockFileSystem::ListFiles(const char* dir) {
    std::vector<std::string> files;
    std::string fullPath = GetFullPath(dir);

    if (!std::filesystem::exists(fullPath)) {
        return files;
    }

    for (const auto& entry : std::filesystem::directory_iterator(fullPath)) {
        if (entry.is_regular_file()) {
            auto relPath = std::filesystem::relative(entry.path(), basePath_);
            files.push_back(relPath.string());
        }
    }

    return files;
}

std::string MockFileSystem::GetFullPath(const char* path) {
    std::filesystem::path fullPath(basePath_);
    std::string pathStr(path);
    if (!pathStr.empty() && pathStr[0] == '/') {
        pathStr = pathStr.substr(1);
    }
    fullPath /= pathStr;
    return fullPath.string();
}

} // namespace HAL
