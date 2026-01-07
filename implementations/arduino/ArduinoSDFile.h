#pragma once

#include "abstractions/IStorage.h"

namespace HAL {

// Placeholder Arduino SD File implementation
class ArduinoSDFile : public IFile {
public:
    ArduinoSDFile() = default;
    ~ArduinoSDFile() override = default;

    Result Write(std::span<const std::byte>) override { return Result::Fail; }
    Result Flush() override { return Result::Fail; }
    Result Read(std::span<std::byte>, size_t& bytesRead) override { bytesRead = 0; return Result::Fail; }
    Result Seek(size_t) override { return Result::Fail; }
    size_t Tell() const override { return 0; }
    size_t Size() const override { return 0; }
    bool IsOpen() const override { return false; }
    void Close() override {}
};

} // namespace HAL
