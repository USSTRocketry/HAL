#pragma once

#include "abstractions/IStorage.h"
#include <fstream>
#include <string>

namespace HAL {

class MockFile : public IFile {
public:
    MockFile(const std::string& path, const char* mode);
    ~MockFile() override;
    
    Result Write(std::span<const std::byte> data) override;
    Result Flush() override;
    Result Read(std::span<std::byte> buffer, size_t& bytesRead) override;
    Result Seek(size_t pos) override;
    size_t Tell() const override;
    size_t Size() const override;
    bool IsOpen() const override;
    void Close() override;
    
private:
    std::fstream file_;
    std::string path_;
    size_t size_;
    
    void UpdateSize();
};

} // namespace HAL
