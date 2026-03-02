#include "implementations/mock/MockFile.h"
#include <filesystem>
#include <cstring>

namespace HAL {

File::File(const std::string& path, const char* mode)
    : path_(path), size_(0) {
    std::ios_base::openmode flags = std::ios::binary;

    if (strcmp(mode, "r") == 0) {
        flags |= std::ios::in;
    } else if (strcmp(mode, "w") == 0) {
        flags |= std::ios::out | std::ios::trunc;
    } else if (strcmp(mode, "a") == 0) {
        flags |= std::ios::out | std::ios::app;
    } else {
        flags |= std::ios::in | std::ios::out;
    }

    file_.open(path_, flags);
    if (file_.is_open()) {
        UpdateSize();
    }
}

File::~File() {
    Close();
}

Result File::Write(std::span<const std::byte> data) {
    if (!file_.is_open()) {
        return Result::Fail;
    }

    file_.write(reinterpret_cast<const char*>(data.data()), data.size());
    if (file_.fail()) {
        return Result::Fail;
    }

    UpdateSize();
    return Result::Success;
}

Result File::Flush() {
    if (!file_.is_open()) {
        return Result::Fail;
    }
    file_.flush();
    return file_.fail() ? Result::Fail : Result::Success;
}

Result File::Read(std::span<std::byte> buffer, size_t& bytesRead) {
    if (!file_.is_open()) {
        bytesRead = 0;
        return Result::Fail;
    }

    file_.read(reinterpret_cast<char*>(buffer.data()), buffer.size());
    bytesRead = file_.gcount();

    if (file_.bad()) {
        return Result::Fail;
    }

    if (file_.eof() && bytesRead > 0) {
        file_.clear();
        return Result::Success;
    }

    return bytesRead > 0 ? Result::Success : Result::Fail;
}

Result File::Seek(size_t pos) {
    if (!file_.is_open()) {
        return Result::Fail;
    }
    file_.seekg(pos);
    file_.seekp(pos);
    return file_.fail() ? Result::Fail : Result::Success;
}

size_t File::Tell() const {
    if (!file_.is_open()) {
        return 0;
    }
    return const_cast<std::fstream&>(file_).tellg();
}

size_t File::Size() const {
    return size_;
}

bool File::IsOpen() const {
    return file_.is_open();
}

void File::Close() {
    if (file_.is_open()) {
        file_.close();
    }
}

void File::UpdateSize() {
    if (std::filesystem::exists(path_)) {
        size_ = std::filesystem::file_size(path_);
    }
}

} // namespace HAL
