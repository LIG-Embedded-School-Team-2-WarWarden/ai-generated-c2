#pragma once
#include "c2mvp/domain.hpp"
#include "ipc_protocol.hpp"

namespace c2mvp {
// Called on the engine's existing serialized network/CLI thread.
class GuiBridge {
    Engine& engine;
    void* transport{};
    std::string effector, observer_session, assigned_session;
    std::map<std::string, std::uint32_t> ids;
    std::map<std::uint32_t, std::string> visible;
    std::uint32_t next_id{1001}, sequence{}, target{}, assignment{}, fire{};
    std::uint64_t laser_sequence{};
    bool cancelling{};
    double last_publish{-1};
    void refresh();
  public:
    explicit GuiBridge(Engine&, bool open_transport = true);
    ~GuiBridge();
    GuiBridge(const GuiBridge&) = delete;
    GuiBridge& operator=(const GuiBridge&) = delete;
    c2::gui::Frame frame();
    c2::gui::PipeResponse command(const c2::gui::PipeCommand&);
    void tick();
};
}
