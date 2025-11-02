#pragma once

#include <span>

// Redo this class, see StorageDesign.md
namespace ra::hal
{
class Storage
{
public:
    enum class Result
    {
        Fail,
        Partial,
        Success
    };

public:
    static Storage& Instance()
    {
        static Storage S;
        return S;
    }
    // We only support 1 stream of binary writes for now
    Result Write(std::span<std::byte>) { /* TODO : */ return Result::Success; }

    Result Flush() { /* TODO: */ return Result::Success; }

private:
    Storage() = default;
};
} // namespace ra::hal
