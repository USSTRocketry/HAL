#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace HAL {

enum class Result {
    Fail,
    Partial,
    Success
};

/// @brief Base interface for write-only operations
struct IWriter {
    virtual ~IWriter() = default;
    
    /// Write data to storage
    virtual Result Write(std::span<const std::byte> data) = 0;
    
    /// Flush buffered data (optional, no-op for unbuffered storage)
    virtual Result Flush() { return Result::Success; }
};

/// @brief File interface with read/write/seek operations
struct IFile : IWriter {
    virtual ~IFile() = default;
    
    /// Read data from file
    virtual Result Read(std::span<std::byte> buffer, size_t& bytesRead) = 0;
    
    /// Seek to position in file
    virtual Result Seek(size_t pos) = 0;
    
    /// Get current position in file
    virtual size_t Tell() const = 0;
    
    /// Get file size
    virtual size_t Size() const = 0;
    
    /// Check if file is open
    virtual bool IsOpen() const = 0;
    
    /// Close the file
    virtual void Close() = 0;
};

/// @brief Filesystem interface for managing multiple files
struct IFileSystem {
    virtual ~IFileSystem() = default;
    
    /// Open or create a file
    /// @param path File path
    /// @param mode "r" = read, "w" = write (truncate), "a" = append
    virtual IFile* OpenFile(const char* path, const char* mode = "w") = 0;
    
    /// Remove a file
    virtual Result RemoveFile(const char* path) = 0;
    
    /// Check if file exists
    virtual bool Exists(const char* path) = 0;
    
    /// List all files in directory
    virtual std::vector<std::string> ListFiles(const char* dir = "/") = 0;
    
    /// Initialize the filesystem
    virtual bool Initialize() = 0;
};

} // namespace HAL
