#pragma once
#include "ipc_protocol.hpp"
extern "C" {
const char* c2_gui_error();
void* c2_gui_create(const wchar_t*, const wchar_t*, const wchar_t*);
void c2_gui_destroy(void*);
int c2_gui_healthy(void*);
int c2_gui_publish(void*, const c2::gui::Frame*);
int c2_gui_poll(void*, c2::gui::PipeCommand*, std::uint64_t*);
int c2_gui_reply(void*, const c2::gui::PipeResponse*, std::uint64_t);
}
