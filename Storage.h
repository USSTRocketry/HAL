#pragma once

#include "abstractions/IStorage.h"
#include <memory>

// Platform-specific filesystem implementations
#if defined(USST_PLATFORM_MOCK)
    #include "implementations/mock/MockFileSystem.h"
#elif defined(USST_PLATFORM_ARDUINO)
    // Arduino SD card support (placeholder)
    #include "implementations/arduino/ArduinoSDFileSystem.h"
#endif

namespace HAL
{

/**
 * @brief Storage manager providing unified access to filesystem
 * 
 * This class provides platform-agnostic storage access using the IFileSystem
 * interface. On desktop/mock platforms, it uses std::filesystem. On Arduino,
 * it uses SD card access (to be implemented).
 */
class Storage
{
public:
    using Result = HAL::Result;

    static Storage& Instance()
    {
        static Storage instance;
        return instance;
    }

    /**
     * @brief Initialize the storage system
     * @return true if successful
     */
    bool Initialize()
    {
        if (!fileSystem_) {
            return false;
        }
        return fileSystem_->Initialize();
    }

    /**
     * @brief Get the filesystem interface
     * @return Pointer to filesystem, or nullptr if not available
     */
    IFileSystem* GetFileSystem()
    {
        return fileSystem_.get();
    }

    /**
     * @brief Open a file for reading or writing
     * @param path File path (relative to storage base)
     * @param mode "r" = read, "w" = write (truncate), "a" = append
     * @return Pointer to file interface, or nullptr on failure
     */
    IFile* OpenFile(const char* path, const char* mode = "w")
    {
        if (!fileSystem_) {
            return nullptr;
        }
        return fileSystem_->OpenFile(path, mode);
    }

    /**
     * @brief Check if a file exists
     */
    bool FileExists(const char* path)
    {
        if (!fileSystem_) {
            return false;
        }
        return fileSystem_->Exists(path);
    }

    /**
     * @brief Delete a file
     */
    Result DeleteFile(const char* path)
    {
        if (!fileSystem_) {
            return Result::Fail;
        }
        return fileSystem_->RemoveFile(path);
    }

private:
    Storage()
    {
#if defined(USST_PLATFORM_MOCK)
    fileSystem_ = std::make_unique<MockFileSystem>("./data");
#elif defined(USST_PLATFORM_ARDUINO)
    // Placeholder Arduino SD filesystem
    fileSystem_ = std::make_unique<ArduinoSDFileSystem>();
#endif
    }

    std::unique_ptr<IFileSystem> fileSystem_;
};

} // namespace HAL

