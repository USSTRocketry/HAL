#pragma once

#include "abstractions/IStorage.h"
#include "implementations/arduino/ArduinoSDFile.h"
#include <vector>
#include <string>

namespace HAL {

// Placeholder Arduino SD Filesystem implementation
class ArduinoSDFileSystem : public IFileSystem {
public:
    ArduinoSDFileSystem() = default;
    ~ArduinoSDFileSystem() override = default;

    IFile* OpenFile(const char*, const char* = "w") override { return nullptr; }
    Result RemoveFile(const char*) override { return Result::Fail; }
    bool Exists(const char*) override { return false; }
    std::vector<std::string> ListFiles(const char* = "/") override { return {}; }
    bool Initialize() override { return false; }
};

} // namespace HAL
