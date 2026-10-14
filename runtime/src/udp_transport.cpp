#include "c2/udp_transport.hpp"

#include <winsock2.h>
#include <ws2tcpip.h>

#include <array>
#include <stdexcept>
#include <utility>

#pragma comment(lib, "Ws2_32.lib")

namespace c2 {
namespace {
SOCKET native_socket(const std::uintptr_t value) noexcept {
    return static_cast<SOCKET>(value);
}

sockaddr_in parse_endpoint(const Endpoint& endpoint) {
    if (endpoint.port == 0) throw std::invalid_argument("UDP destination port must be non-zero");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(endpoint.port);
    if (inet_pton(AF_INET, endpoint.address.c_str(), &address.sin_addr) != 1)
        throw std::invalid_argument("UDP endpoint must contain a valid IPv4 address");
    return address;
}

sockaddr_in parse_bind_address(const UdpConfig& config) {
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(config.listen_port);
    if (inet_pton(AF_INET, config.bind_address.c_str(), &address.sin_addr) != 1)
        throw std::invalid_argument("UDP bind address must be a valid IPv4 address");
    return address;
}

Endpoint endpoint_from(const sockaddr_in& address) {
    std::array<char, INET_ADDRSTRLEN> text{};
    if (inet_ntop(AF_INET, &address.sin_addr, text.data(), text.size()) == nullptr)
        return {};
    return {text.data(), ntohs(address.sin_port)};
}
}  // namespace

UdpTransport::UdpTransport(UdpConfig config, ReceiveHandler handler)
    : config_(std::move(config)), handler_(std::move(handler)) {
    if (!handler_) throw std::invalid_argument("UDP receive handler must be set");
}

UdpTransport::~UdpTransport() { stop(); }

void UdpTransport::start() {
    std::lock_guard lock(lifecycle_mutex_);
    if (running_ || receiver_.joinable())
        throw std::logic_error("UDP transport must be stopped before starting");

    WSADATA data{};
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0)
        throw std::runtime_error("failed to initialize Winsock");
    winsock_initialized_ = true;

    SOCKET created = INVALID_SOCKET;
    try {
        const auto bind_address = parse_bind_address(config_);
        created = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (created == INVALID_SOCKET) throw std::runtime_error("failed to create UDP socket");

        constexpr DWORD timeout_ms = 100;
        if (setsockopt(
                created, SOL_SOCKET, SO_RCVTIMEO,
                reinterpret_cast<const char*>(&timeout_ms), sizeof(timeout_ms)) == SOCKET_ERROR)
            throw std::runtime_error("failed to configure UDP receive timeout");

        if (::bind(
                created, reinterpret_cast<const sockaddr*>(&bind_address),
                sizeof(bind_address)) == SOCKET_ERROR)
            throw std::runtime_error("failed to bind UDP socket");

        {
            std::lock_guard socket_lock(socket_mutex_);
            socket_.store(static_cast<std::uintptr_t>(created));
        }
        running_.store(true);
        receiver_ = std::thread(&UdpTransport::receive_loop, this);
    } catch (...) {
        running_.store(false);
        socket_.store(invalid_socket);
        if (created != INVALID_SOCKET) closesocket(created);
        WSACleanup();
        winsock_initialized_ = false;
        throw;
    }
}

void UdpTransport::stop() noexcept {
    std::lock_guard lifecycle_lock(lifecycle_mutex_);
    running_.store(false);
    {
        std::lock_guard socket_lock(socket_mutex_);
        const auto socket_value = socket_.exchange(invalid_socket);
        if (socket_value != invalid_socket) {
            const auto socket = native_socket(socket_value);
            shutdown(socket, SD_BOTH);
            closesocket(socket);
        }
    }

    if (receiver_.joinable()) receiver_.join();

    if (winsock_initialized_) {
        WSACleanup();
        winsock_initialized_ = false;
    }
}

void UdpTransport::send(
    const std::span<const std::byte> bytes, const Endpoint& destination) {
    if (bytes.size() > max_udp_datagram_size)
        throw std::length_error("payload exceeds the IPv4 UDP datagram limit");
    const auto address = parse_endpoint(destination);

    std::lock_guard lock(socket_mutex_);
    const auto socket_value = socket_.load();
    if (!running_ || socket_value == invalid_socket)
        throw std::logic_error("UDP transport is not running");
    const auto sent = sendto(
        native_socket(socket_value), reinterpret_cast<const char*>(bytes.data()),
        static_cast<int>(bytes.size()), 0, reinterpret_cast<const sockaddr*>(&address),
        sizeof(address));
    if (sent == SOCKET_ERROR || static_cast<std::size_t>(sent) != bytes.size()) {
        ++send_errors_;
        last_socket_error_.store(sent == SOCKET_ERROR ? WSAGetLastError() : 0);
        throw std::runtime_error("failed to send UDP datagram");
    }
    ++sent_datagrams_;
    sent_bytes_ += bytes.size();
}

Endpoint UdpTransport::local_endpoint() const {
    std::lock_guard lock(socket_mutex_);
    const auto socket_value = socket_.load();
    if (!running_ || socket_value == invalid_socket)
        throw std::logic_error("UDP transport is not running");

    sockaddr_in address{};
    int size = sizeof(address);
    if (getsockname(
            native_socket(socket_value), reinterpret_cast<sockaddr*>(&address), &size) ==
        SOCKET_ERROR)
        throw std::runtime_error("failed to read UDP local endpoint");
    return endpoint_from(address);
}

void UdpTransport::receive_loop() noexcept {
    std::array<std::byte, max_udp_datagram_size> buffer{};
    while (running_) {
        const auto socket_value = socket_.load();
        if (socket_value == invalid_socket) break;

        sockaddr_in source_address{};
        int source_size = sizeof(source_address);
        const auto received = recvfrom(
            native_socket(socket_value), reinterpret_cast<char*>(buffer.data()),
            static_cast<int>(buffer.size()), 0,
            reinterpret_cast<sockaddr*>(&source_address), &source_size);
        if (received == SOCKET_ERROR) {
            if (!running_) break;
            const auto error = WSAGetLastError();
            if (error == WSAETIMEDOUT || error == WSAEWOULDBLOCK) continue;
            ++receive_errors_;
            last_socket_error_.store(error);
            // UDP ICMP port-unreachable and oversized packets are per-datagram
            // failures, not a reason to kill a long-lived receive socket.
            if (error == WSAECONNRESET || error == WSAEMSGSIZE) continue;
            break;
        }

        ++received_datagrams_;
        received_bytes_ += received;
        try {
            std::vector<std::byte> datagram(buffer.begin(), buffer.begin() + received);
            handler_(std::move(datagram), endpoint_from(source_address));
        } catch (...) {
            ++handler_errors_;
            // A consumer failure must not terminate the process or the receive thread.
        }
    }
    running_.store(false);
}

UdpTransportStats UdpTransport::stats() const noexcept {
    return {received_datagrams_.load(), received_bytes_.load(), sent_datagrams_.load(),
        sent_bytes_.load(), receive_errors_.load(), send_errors_.load(),
        handler_errors_.load(), last_socket_error_.load()};
}
}  // namespace c2
