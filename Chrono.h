#pragma once
#if defined(ARDUINO)
    #include <Arduino.h>
#else
    #include <chrono>
#endif

#include <stdint.h>

namespace ra::hal
{
#if defined(ARDUINO)
inline unsigned long SysUptimeMs() { return ::millis(); }
#else
inline unsigned long SysUptimeMs()
{
    static const auto start = std::chrono::steady_clock::now();
    return static_cast<unsigned long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());
}
#endif

class Tick
{
    using TickType = uint32_t;
    using TickId   = uint32_t;

public:
    // Immutable tick value
    class TickPoint
    {
    public:
        enum class CompareStatus
        {
            Invalid,
            MismatchedId,
            Equal,
            Less,
            Greater,
        };

    public:
        TickType Raw() const { return m_Tick; }
        TickId Id() const { return m_Id; }

        // Comparison operators (only valid if same TickId)
        CompareStatus Compare(const TickPoint& Other) const
        {
            if (m_Id == InvalidId) { return CompareStatus::Invalid; }
            if (m_Id != Other.m_Id) { return CompareStatus::MismatchedId; }

            if (m_Tick == Other.m_Tick) { return CompareStatus::Equal; }
            else if (m_Tick < Other.m_Tick) { return CompareStatus::Less; }
            else {
                return CompareStatus::Greater;
            }
        }

        TickPoint(const TickPoint&)            = default;
        TickPoint& operator=(const TickPoint&) = default;

    private:
        constexpr static auto InvalidId = 0;
        constexpr explicit TickPoint(TickId Id, TickType Tick) : m_Id(Id), m_Tick(Tick) {}

    private:
        TickId m_Id;
        TickType m_Tick;

        // Only Tick can construct TickPoint
        friend class Tick;
    };

public:
    explicit Tick(TickId Id = TickPoint::InvalidId) : m_Id(Id), m_CurrentTick(0) {}

    TickPoint Advance()
    {
        ++m_CurrentTick;
        return TickPoint(m_Id, m_CurrentTick);
    }

    TickPoint Now() const { return TickPoint(m_Id, m_CurrentTick); }
    TickId Id() const { return m_Id; }

    static constexpr TickPoint Invalid() { return TickPoint {TickPoint::InvalidId, 0}; }

private:
    TickId m_Id;
    TickType m_CurrentTick;
};
} // namespace ra::hal
