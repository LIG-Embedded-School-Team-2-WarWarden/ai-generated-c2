#pragma once
#include "c2/udp_transport.hpp"
#include <functional>
#include <span>
namespace c2 {
using DatagramSender = std::function<void(std::span<const std::byte>, const Endpoint&)>;
} // namespace c2
