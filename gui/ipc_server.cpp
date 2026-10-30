#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include "ipc_protocol.hpp"
#include "ipc_server.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using namespace c2::gui;
struct Handle {
    HANDLE value{};
    explicit Handle(HANDLE h = nullptr) : value(h) {}
    ~Handle() { if (value && value != INVALID_HANDLE_VALUE) CloseHandle(value); }
    Handle(const Handle&) = delete;
    Handle& operator=(const Handle&) = delete;
};
void require(bool ok, const char* operation) {
    if (!ok) throw std::runtime_error(std::string(operation) + " (Win32 " + std::to_string(GetLastError()) + ")");
}
struct View {
    void* value{};
    ~View() { if (value) UnmapViewOfFile(value); }
};
struct Server {
    Handle stop_event, pipe, mapping, frame_mutex;
    View view;
    std::mutex gate;
    std::condition_variable changed;
    std::optional<PipeCommand> command;
    std::optional<PipeResponse> response;
    std::uint64_t ticket{};
    bool delivered{};
    std::atomic<bool> closing{}, healthy{true};
    std::thread worker;

    Server(const wchar_t* memory, const wchar_t* mutex, const wchar_t* path)
        : stop_event(CreateEventW(nullptr, TRUE, FALSE, nullptr)),
          pipe(CreateNamedPipeW(path, PIPE_ACCESS_DUPLEX | FILE_FLAG_OVERLAPPED | FILE_FLAG_FIRST_PIPE_INSTANCE,
               PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT | PIPE_REJECT_REMOTE_CLIENTS,
               1, 4096, 4096, 0, nullptr)) {
        require(stop_event.value != nullptr, "CreateEvent");
        require(pipe.value != INVALID_HANDLE_VALUE, "CreateNamedPipe (another GUI server may be running)");
        frame_mutex.value = CreateMutexW(nullptr, FALSE, mutex);
        require(frame_mutex.value != nullptr, "CreateMutex");
        mapping.value = CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0, frame_size, memory);
        const auto mapping_error = GetLastError();
        require(mapping.value != nullptr, "CreateFileMapping");
        if (mapping_error == ERROR_ALREADY_EXISTS)
            throw std::runtime_error("Shared memory already exists; stop the other publisher or close its readers");
        view.value = MapViewOfFile(mapping.value, FILE_MAP_WRITE, 0, 0, frame_size);
        require(view.value != nullptr, "MapViewOfFile");
        Frame initial;
        initial.header.timestamp_ms = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
        require(publish(initial) == 1, "Publish initial frame");
        worker = std::thread([this] {
            try { serve(); }
            catch (...) { healthy = false; }
        });
    }
    ~Server() {
        closing = true;
        SetEvent(stop_event.value);
        changed.notify_all();
        if (worker.joinable()) worker.join();
    }
    bool await(OVERLAPPED& op, DWORD timeout, DWORD& count) {
        HANDLE waits[]{stop_event.value, op.hEvent};
        if (WaitForMultipleObjects(2, waits, FALSE, timeout) != WAIT_OBJECT_0 + 1) {
            CancelIoEx(pipe.value, &op);
            GetOverlappedResult(pipe.value, &op, &count, TRUE); // Drain before op leaves the stack.
            return false;
        }
        return GetOverlappedResult(pipe.value, &op, &count, FALSE) != FALSE;
    }
    bool connect() {
        Handle event(CreateEventW(nullptr, TRUE, FALSE, nullptr));
        require(event.value != nullptr, "CreateEvent");
        OVERLAPPED op{}; op.hEvent = event.value;
        if (ConnectNamedPipe(pipe.value, &op)) return true;
        const auto error = GetLastError();
        if (error == ERROR_PIPE_CONNECTED) return true;
        if (error != ERROR_IO_PENDING) return false;
        DWORD ignored{};
        return await(op, INFINITE, ignored);
    }
    bool transfer(void* bytes, DWORD size, bool writing) {
        Handle event(CreateEventW(nullptr, TRUE, FALSE, nullptr));
        require(event.value != nullptr, "CreateEvent");
        DWORD offset{};
        while (offset < size && !closing) {
            ResetEvent(event.value);
            OVERLAPPED op{}; op.hEvent = event.value;
            DWORD count{};
            auto* position = static_cast<std::uint8_t*>(bytes) + offset;
            const auto ok = writing ? WriteFile(pipe.value, position, size - offset, &count, &op)
                                    : ReadFile(pipe.value, position, size - offset, &count, &op);
            // A connected GUI may pause polling while its UI thread is busy.
            // Only partial requests and response writes have a two-second limit;
            // an idle client remains connected until data, disconnect or shutdown.
            const DWORD timeout = !writing && offset == 0 ? INFINITE : 2000;
            if (!ok && (GetLastError() != ERROR_IO_PENDING || !await(op, timeout, count))) return false;
            if (count == 0) return false;
            offset += count;
        }
        return offset == size;
    }
    void serve() {
        while (!closing) {
            if (!connect()) {
                if (closing) break;
                DisconnectNamedPipe(pipe.value);
                continue;
            }
            while (!closing) {
                std::array<std::uint8_t, command_size> bytes{};
                if (!transfer(bytes.data(), static_cast<DWORD>(bytes.size()), false)) break;
                PipeCommand request{};
                PipeResponse reply{};
                const bool valid = deserialize(bytes, request);
                reply.request_id = request.request_id; reply.command = request.command;
                reply.fcs_id = request.fcs_id;
                reply.result = static_cast<std::uint32_t>(Result::invalid_request);
                if (valid) {
                    std::unique_lock lock(gate);
                    ++ticket;
                    command = request; delivered = false; response.reset();
                    changed.wait_for(lock, std::chrono::seconds(2), [this] { return response.has_value() || closing; });
                    if (closing) break;
                    if (response) reply = *response;
                    else reply.result = static_cast<std::uint32_t>(Result::device_error);
                    command.reset(); response.reset();
                }
                auto output = serialize(reply);
                if (!transfer(output.data(), static_cast<DWORD>(output.size()), true)) break;
            }
            DisconnectNamedPipe(pipe.value);
        }
    }
    int publish(const Frame& frame) {
        auto bytes = serialize(frame);
        Frame checked;
        if (!deserialize(bytes, checked)) return false;
        const auto result = WaitForSingleObject(frame_mutex.value, 250);
        // A busy/preempted reader can retain the mutex during UI inspection.
        // Skip this display frame; the next one is current and self-contained.
        if (result == WAIT_TIMEOUT) return 2;
        if (result != WAIT_OBJECT_0 && result != WAIT_ABANDONED) return false;
        // The only raw copy here is an already serialized byte buffer, never a struct.
        CopyMemory(view.value, bytes.data(), bytes.size());
        ReleaseMutex(frame_mutex.value);
        return true;
    }
};
thread_local std::string last_error;
} // namespace

// Opaque transport handle. All wire encoding is field-wise in ipc_protocol.cpp.
#define API extern "C"
API const char* c2_gui_error() { return last_error.c_str(); }
API void* c2_gui_create(const wchar_t* memory, const wchar_t* mutex, const wchar_t* path) {
    try { return new Server(memory, mutex, path); }
    catch (const std::exception& e) { last_error = e.what(); return nullptr; }
    catch (...) { last_error = "Cannot create GUI IPC server"; return nullptr; }
}
API void c2_gui_destroy(void* handle) { delete static_cast<Server*>(handle); }
API int c2_gui_healthy(void* handle) { auto* server = static_cast<Server*>(handle); return server && server->healthy; }
API int c2_gui_publish(void* handle, const Frame* frame) {
    auto* server = static_cast<Server*>(handle);
    try { return server && frame ? server->publish(*frame) : 0; }
    catch (...) { return 0; }
}
API int c2_gui_poll(void* handle, PipeCommand* request, std::uint64_t* ticket) {
    auto* server = static_cast<Server*>(handle);
    if (!server || !request || !ticket) return 0;
    std::lock_guard lock(server->gate);
    if (!server->command || server->delivered) return 0;
    *request = *server->command; *ticket = server->ticket; server->delivered = true;
    return 1;
}
API int c2_gui_reply(void* handle, const PipeResponse* response, std::uint64_t ticket) {
    auto* server = static_cast<Server*>(handle);
    if (!server || !response) return 0;
    std::lock_guard lock(server->gate);
    if (!server->command || !server->delivered || server->response || server->ticket != ticket ||
        response->request_id != server->command->request_id || response->command != server->command->command) return 0;
    server->response = *response; server->changed.notify_one(); return 1;
}
