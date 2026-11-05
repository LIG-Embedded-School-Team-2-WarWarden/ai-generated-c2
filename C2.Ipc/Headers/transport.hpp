#pragma once
#include "protocol.hpp"
extern "C" {
const char* c2_ipc_error();
void* c2_ipc_create(const wchar_t*, const wchar_t*, const wchar_t*);
void* c2_ipc_create_commands(const wchar_t*);
void c2_ipc_destroy(void*);
int c2_ipc_healthy(void*);
int c2_ipc_publish(void*, const c2::ipc::Frame*);
int c2_ipc_poll(void*, c2::ipc::PipeCommand*, std::uint64_t*);
int c2_ipc_reply(void*, const c2::ipc::PipeResponse*, std::uint64_t);
}
