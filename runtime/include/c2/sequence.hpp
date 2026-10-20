#pragma once
#include "c2/protocol.hpp"
namespace c2 {
// Forward distances less than half the uint32 range are newer, including wrap.
// Exactly half a range is ambiguous and must be rejected.
inline bool newer_sequence(std::uint32_t candidate, std::uint32_t current) noexcept {
    const auto distance=static_cast<std::uint32_t>(candidate-current);
    return distance!=0&&distance<0x80000000U;
}
inline bool same_sequence_stream(const MessageHeader& a,const MessageHeader& b) noexcept {
    return a.asset_id==b.asset_id&&a.session_id==b.session_id&&a.source_id==b.source_id;
}
}
