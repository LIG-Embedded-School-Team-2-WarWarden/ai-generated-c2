#include <bit>
#include "c2/diagnostic_log.hpp"
#include "c2/fcs_client.hpp"
#include "c2/rdr_client.hpp"
#include "rdr_control.hpp"
#include "lidar_interlock.hpp"
#include "manual_sequence.hpp"
#include "batch_sequence.hpp"
#include "observation_retention.hpp"
#include "rdr_startup.hpp"
#include "rdr_live_diagnostics.hpp"
#include "target_position_filter.hpp"
#include "rdr_coordinate_recorder.hpp"
#include "transport.hpp"
#include "assignment_health.hpp"
#include "assignment_position_guard.hpp"
#include "fcs_angles.hpp"
#include "installation_config.hpp"
#include <winsock2.h>
#include <ws2tcpip.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <charconv>
#include <condition_variable>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <memory>
#include <numbers>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <deque>
#include <sstream>
#include <iterator>
#include <windows.h>

using Clock = std::chrono::steady_clock;
namespace gui = c2::ipc;
namespace fcs = c2::fcs;
static volatile std::sig_atomic_t quitting{};
static void quit(int) { quitting = 1; }
static const char* result_name(gui::Result result) {
    switch(result) {
    case gui::Result::success: return "success";
    case gui::Result::invalid_request: return "invalid_request";
    case gui::Result::asset_unavailable: return "asset_unavailable";
    case gui::Result::not_assigned: return "not_assigned";
    case gui::Result::already_assigned: return "already_assigned";
    case gui::Result::not_supported: return "not_supported";
    case gui::Result::device_error: return "device_error";
    case gui::Result::status_pending: return "status_pending";
    case gui::Result::target_pending: return "target_pending";
    case gui::Result::target_changed: return "target_changed";
    case gui::Result::device_fault: return "device_fault";
    case gui::Result::alignment_required: return "alignment_required";
    case gui::Result::device_busy: return "device_busy";
    case gui::Result::response_timeout: return "response_timeout";
    case gui::Result::device_rejected: return "device_rejected";
    case gui::Result::lidar_unsupported: return "lidar_unsupported";
    case gui::Result::lidar_unknown: return "lidar_unknown";
    case gui::Result::lidar_recovery_required: return "lidar_recovery_required";
    case gui::Result::lidar_stop_warning_unconfirmed: return "lidar_stop_warning_unconfirmed";
    case gui::Result::lidar_thermal_fault: return "lidar_thermal_fault";
    case gui::Result::lidar_stop_unconfirmed: return "lidar_stop_unconfirmed";
    case gui::Result::lidar_disconnected: return "lidar_disconnected";
    case gui::Result::lidar_busy: return "lidar_busy";
    case gui::Result::lidar_resume_unconfirmed: return "lidar_resume_unconfirmed";
    case gui::Result::fcs_off_unconfirmed: return "fcs_off_unconfirmed";
    case gui::Result::rdr_start_unconfirmed: return "rdr_start_unconfirmed";
    case gui::Result::result_unconfirmed: return "result_unconfirmed";
    case gui::Result::pause_timeout: return "pause_timeout";
    }
    return "unknown";
}
static uint64_t unix_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}
static void put(std::vector<std::byte>& out, uint64_t value, unsigned size) {
    for (unsigned i = size; i > 0; --i) out.push_back(std::byte((value >> ((i - 1) * 8)) & 255));
}
static uint64_t get(std::span<const std::byte> bytes, size_t at, unsigned size) {
    if (at + size > bytes.size()) throw std::invalid_argument("truncated RDR field");
    uint64_t value{};
    for (unsigned i = 0; i < size; ++i) value = (value << 8) | std::to_integer<unsigned>(bytes[at+i]);
    return value;
}
static int signed16(std::span<const std::byte> bytes, size_t at) {
    auto value = int(get(bytes, at, 2)); return value >= 32768 ? value - 65536 : value;
}
struct Target { uint32_t id; double east, north, up; };
struct RdrState {
    bool connected{}, streaming{}, scanning{};
    std::vector<Target> targets;
    Clock::time_point received{}, frame_received{};
    uint32_t frame{};
    uint64_t generation{};
};
static uint16_t parse_port(const std::string& value, bool automatic = false) {
    unsigned port{};
    const auto [end, error] = std::from_chars(value.data(), value.data()+value.size(), port);
    if (error != std::errc{} || end != value.data()+value.size() || port > 65535 || (!automatic && !port))
        throw std::invalid_argument("UDP port must be " + std::string(automatic ? "0" : "1") + "..65535");
    return uint16_t(port);
}
static void validate_ipv4(const std::string& value) {
    in_addr address{};
    if (inet_pton(AF_INET, value.c_str(), &address) != 1)
        throw std::invalid_argument("network address must be a valid IPv4 address");
}

class ConsoleInput {
    std::thread thread; std::mutex mutex; std::deque<std::string> lines;
    std::atomic<bool> closed{false}, stopping{false};
public:
    explicit ConsoleInput(bool headless) {
        if (!headless) thread=std::thread([this] {
            std::string line; while(!stopping && std::getline(std::cin,line)) { std::lock_guard lock(mutex);lines.push_back(line); }
            closed=true;
        });
    }
    ~ConsoleInput() { stopping=true; if(thread.joinable()) { CancelSynchronousIo(thread.native_handle());thread.join(); } }
    std::optional<std::string> next() {std::lock_guard lock(mutex);if(lines.empty())return {};auto line=lines.front();lines.pop_front();return line;}
    bool ended() {std::lock_guard lock(mutex);return closed&&lines.empty();}
};
static nlohmann::json rdr_json(const c2::rdr::State& state,
                               const std::vector<c2::rdr::CorrectedTarget>& corrected,
                               const std::vector<c2::rdr::CorrectedTarget>& displayed) {
    nlohmann::json targets=nlohmann::json::array();
    for(const auto& point:corrected) {
        const auto id=point.id;const auto& t=state.targets.at(id);
        nlohmann::json row={{"id",id},{"raw_x",t.x},{"raw_y",t.y},{"raw_z",t.z},
        {"observation_pan",t.pan},{"observation_tilt",t.tilt},{"detected",t.detected},
        {"track_state",t.track_state},{"status",t.status},{"world_valid",point.valid},
        {"world_xyz",nullptr},{"gui_xyz",nullptr}};
        if(point.valid) {
            row["world_xyz"]={point.world.east,point.world.north,point.world.up};
        }
        const auto live=std::find_if(displayed.begin(),displayed.end(),
            [id](const auto& value){return value.id==id;});
        if(live!=displayed.end() && live->valid)
            row["gui_xyz"]={float(live->world.east),float(live->world.north),float(live->world.up)};
        targets.push_back(std::move(row));
    }
    return {{"connected",state.connected},{"mode",state.mode},{"reporting",state.reporting},{"result",state.last_result},{"targets",targets}};
}

int main(int argc, char** argv) {
    void* ipc{};
    void* rdr_control{};
    std::filesystem::path event_log_file;
    std::ofstream events;
    try {
        fcs::ClientConfig config;
        config.peer = {"192.168.0.103", 2000}; config.local = {"192.168.0.102", 2000};
        // Leave 20 ms of the 100 ms polling period for loop processing after a timeout.
        config.response_timeout = std::chrono::milliseconds(80);
        std::filesystem::path stop_file, status_file, installation_file;
        bool standalone_installation{};
        std::filesystem::path rdr_record_file;
        double rdr_record_seconds=30;
        InstallationConfig installation;
        c2::rdr::TargetPositionFilterConfig position_filter_config;
        auto laser_deadline = Clock::time_point{};
        unsigned rdr_port=1000, rdr_local_port=1000;
        std::string rdr_ip="192.168.0.101", rdr_local_ip;
        std::wstring ipc_suffix;
        bool original_gui=true, real_rdr_mode=true, rdr_only=false, print_network=false, headless=false;
        for (int i=1; i<argc; ++i) {
            std::string option = argv[i];
            auto next = [&]() -> std::string { if (++i>=argc) throw std::invalid_argument("missing argument"); return argv[i]; };
            if (option == "--help") {
                std::cout << "C2 binary ICD server --config FILE [--ipc] [--headless --stop-file FILE] [--print-network]\n"
                    "Event log: --event-log FILE (default logs/events.log; --status-file uses adjacent events.log)\n"
                    "Positions: persisted in --config installation; without --config, installation.json beside executable\n"
                    "Coordinate recording: --rdr-record FILE.csv [--rdr-record-seconds 30]\n"
                    "RDR: rdr connect|status|targets|live on|off|report on|off|start|stop|exclude ID|resume ID\n"
                    "FCS: fcs connect|status|point ELEVATION AZIMUTH|laser on|off; quit\n";
                return 0;
            }
            else if (option == "--ipc") { /* server transport, no GUI application */ }
            else if (option == "--headless") headless=true;
            else if (option == "--rdr-real") real_rdr_mode=true;
            else if (option == "--rdr-only") real_rdr_mode=rdr_only=true;
            else if (option == "--print-network") print_network=true;
            else if (option == "--bind") config.local.bind_address = next();
            else if (option == "--fcs-ip") config.peer.address = next();
            else if (option == "--fcs-port") config.peer.port=parse_port(next());
            else if (option == "--fcs-local-port") config.local.listen_port=parse_port(next(),true);
            else if (option == "--rdr-ip") rdr_ip=next();
            else if (option == "--rdr-local-ip" || option == "--rdr-bind") rdr_local_ip=next();
            else if (option == "--stop-file") stop_file = next();
            else if (option == "--status-file") status_file = next();
            else if (option == "--event-log") event_log_file = next();
            else if (option == "--rdr-record") rdr_record_file=next();
            else if (option == "--rdr-record-seconds") {
                const auto value=next();
                const auto [end,error]=std::from_chars(value.data(),value.data()+value.size(),rdr_record_seconds);
                if(error!=std::errc{} || end!=value.data()+value.size() || !std::isfinite(rdr_record_seconds) ||
                   rdr_record_seconds<=0 || rdr_record_seconds>3600)
                    throw std::invalid_argument("invalid rdr-record-seconds");
            }
            else if (option == "--config") {
                installation_file = std::filesystem::absolute(next());
                std::ifstream file(installation_file); if(!file) throw std::invalid_argument("configuration missing");
                nlohmann::json json; file>>json;
                const auto& radar=json.at("rdr"); const auto& fcs_config=json.at("fcs");
                if(radar.at("protocol")!="edde24" || fcs_config.at("protocol")!="effe") throw std::invalid_argument("unsupported binary ICD");
                rdr_local_ip=radar.at("local").at("ip"); rdr_local_port=parse_port(radar.at("local").at("port").dump(),true);
                rdr_ip=radar.at("peer").at("ip"); rdr_port=parse_port(radar.at("peer").at("port").dump());
                config.local.bind_address=fcs_config.at("local").at("ip"); config.local.listen_port=parse_port(fcs_config.at("local").at("port").dump(),true);
                config.peer.address=fcs_config.at("peer").at("ip"); config.peer.port=parse_port(fcs_config.at("peer").at("port").dump());
                rdr_only=!fcs_config.value("enabled",true);
                position_filter_config=c2::rdr::parse_target_position_filter(
                    radar.value("gui_kalman",nlohmann::json::object()));
            }
            else if (option == "--rdr-port" || option == "--rdr-real-port") rdr_port=parse_port(next());
            else if (option == "--rdr-local-port" || option == "--rdr-real-local-port") rdr_local_port=parse_port(next());
            else if (option == "--ipc-suffix") { auto s=next(); ipc_suffix.assign(s.begin(),s.end()); }
            else if (option == "--gui-extended") original_gui=false;
            else throw std::invalid_argument("unknown console option");
        }
        c2::rdr::TargetPositionFilter position_filter(position_filter_config);
        const auto position_filter_parameters=c2::rdr::target_position_filter_json(position_filter_config);
        if (rdr_local_ip.empty()) rdr_local_ip=config.local.bind_address;
        for (const auto& address : {config.local.bind_address,config.peer.address,rdr_ip,rdr_local_ip})
            validate_ipv4(address);
        if (!rdr_only && config.local.bind_address==rdr_local_ip && config.local.listen_port &&
            config.local.listen_port==rdr_local_port)
            throw std::invalid_argument("FCS and RDR local UDP endpoints must differ");
        const auto endpoint_json=[](const c2::Endpoint& endpoint) {
            return nlohmann::json{{"address",endpoint.address},{"port",endpoint.port}};
        };
        if (print_network) {
            std::cout << nlohmann::json{
                {"fcs_local",endpoint_json({config.local.bind_address,config.local.listen_port})},
                {"fcs_peer",endpoint_json(config.peer)},
                {"rdr_local",endpoint_json({rdr_local_ip,uint16_t(rdr_local_port)})},
                {"rdr_peer",endpoint_json({rdr_ip,uint16_t(rdr_port)})},
                {"rdr_mode",real_rdr_mode?"real24":"draft"},{"fcs_enabled",!rdr_only}}.dump() << '\n';
            return 0;
        }
        if(event_log_file.empty()) event_log_file=status_file.empty()?
            std::filesystem::path("logs/events.log"):status_file.parent_path()/"events.log";
        event_log_file=std::filesystem::absolute(event_log_file);
        std::filesystem::create_directories(event_log_file.parent_path());
        const auto trace_file=event_log_file.parent_path()/(event_log_file.stem().string()+".devices.jsonl");
        c2::DiagnosticLog::instance().start(trace_file,
            std::to_string(unix_ms())+"-"+std::to_string(GetCurrentProcessId()));
        events.open(event_log_file,std::ios::app);
        if(!events) throw std::runtime_error("cannot open event log: "+event_log_file.string());
        bool event_log_error_reported{};
        auto log=[&](const std::string& message) {
            const auto line=std::to_string(unix_ms())+" "+message;
            c2::diagnostic("ECS","event",{{"message",message}});
            std::cout<<line<<std::endl;
            events<<line<<std::endl;
            if(!events && !event_log_error_reported) {
                std::cerr<<"Event log write failed: "<<event_log_file.string()<<std::endl;
                event_log_error_reported=true;
            }
        };
        log("runtime_start "+nlohmann::json{{"executable",std::filesystem::absolute(argv[0]).string()},
            {"build",__DATE__ " " __TIME__},{"pid",GetCurrentProcessId()},
            {"event_log",event_log_file.string()},{"device_log",trace_file.string()},
            {"fcs_timeout_ms",config.response_timeout.count()},{"fcs_missed_status_limit",config.missed_status_limit},
            {"position_filter",position_filter_parameters}}.dump());
        std::unique_ptr<c2::rdr::CoordinateRecorder> recorder;
        if(!rdr_record_file.empty()) recorder=std::make_unique<c2::rdr::CoordinateRecorder>(
            rdr_record_file,rdr_record_seconds,nlohmann::json{
                {"executable",std::filesystem::absolute(argv[0]).string()},
                {"rdr_peer",endpoint_json({rdr_ip,uint16_t(rdr_port)})},
                {"filter",position_filter_parameters}});
        bool recording_finished_logged{};
        std::filesystem::file_time_type installation_seen{};
        if (installation_file.empty()) {
            standalone_installation = true;
            installation_file = std::filesystem::absolute(argv[0]).parent_path() / "installation.json";
        }
        installation = load_installation_settings(installation_file, standalone_installation);
        if (std::filesystem::exists(installation_file)) {
            installation_seen = std::filesystem::last_write_time(installation_file);
        }
        std::signal(SIGINT, quit); std::signal(SIGTERM, quit);
        std::unique_ptr<fcs::Client> client;

        std::unique_ptr<c2::rdr::Client> real_rdr;
        if (!rdr_only) { client=std::make_unique<fcs::Client>(config); client->start(); }
        real_rdr=std::make_unique<c2::rdr::Client>(c2::UdpConfig{rdr_local_ip,uint16_t(rdr_local_port)},
                c2::Endpoint{rdr_ip,uint16_t(rdr_port)},true);
        const auto network=nlohmann::json{
            {"fcs_local",endpoint_json(client?client->local_endpoint():c2::Endpoint{})},
            {"fcs_peer",endpoint_json(config.peer)},
            {"rdr_local",endpoint_json(real_rdr->local_endpoint())},
            {"rdr_peer",endpoint_json({rdr_ip,uint16_t(rdr_port)})}};
        c2::diagnostic("ECS","network",network);
        ipc = c2_ipc_create((std::wstring(gui::mapping_name)+ipc_suffix).c_str(),
            (std::wstring(gui::mutex_name)+ipc_suffix).c_str(), (std::wstring(gui::pipe_path)+ipc_suffix).c_str());
        if (!ipc) throw std::runtime_error(c2_ipc_error());
        rdr_control=c2_ipc_create_commands((std::wstring(L"\\\\.\\pipe\\ECS_RdrControl")+ipc_suffix).c_str());
        if (!rdr_control) throw std::runtime_error(c2_ipc_error());
        std::cout << "LIVE ECS / configured FCS and RDR peers / manual Big Endian codecs\n"
                  << "Network: " << network.dump() << '\n'
                  << "GUI FCS-1 = " << config.peer.address << ':' << config.peer.port << '\n'
                  << "Installation settings: " << installation_file.string() << '\n'
                  << "Position edits update map and FCS assignment coordinates; reassign after edits.\n"
                  << "Assign sends EF03; Active sends EF05 built-in demo; Cancel sends EF04 OFF.\n"
                  << "FCS and RDR connect automatically; RDR report/start require commands.\n"
                  << "RDR controls: rdr connect / rdr report on / rdr start / rdr stop\n"
                  << "Read-only world XYZ monitor: rdr live on / rdr live off (200 ms)\n"
                  << "Manual result after visual verification: sequence confirm-hit TARGET_ID\n"
                  << "Target position Kalman=" << (position_filter_config.enabled?"on":"off")
                  << " measurement_stddev_m=" << position_filter_config.measurement_stddev_m
                  << " process_stddev_m_sqrt_s=" << position_filter_config.process_stddev_m_sqrt_s << '\n'
                  << "Ctrl+C closes this station.\n" << std::flush;
        std::unique_ptr<RdrControl<c2::rdr::Client>> real_control;
        if (real_rdr) {

            real_control=std::make_unique<RdrControl<c2::rdr::Client>>(*real_rdr,log,c2::rdr::seconds());
            log(std::string("RDR mode=real24 ED/DE; FCS=")+(client?"active":"inactive")+
                "; automatic Connect/reconnect; explicit Report ON and Start required");
        }
        auto observation_snapshot=[&](const c2::rdr::State& state,
                                     const std::vector<c2::rdr::CorrectedTarget>& targets) {
            RdrState out;
            out.generation=state.generation;
            out.connected=state.connected;out.streaming=state.reporting;out.scanning=real_control->operating(state);
            for (const auto& target : targets)
                if (out.scanning && target.valid)
                    out.targets.push_back({target.id,target.world.east,target.world.north,target.world.up});
            return out;
        };
        auto command_result=[&](const char* name,const std::optional<fcs::Packet>& reply) {
            log(std::string("FCS ")+name+(reply ? " reply="+std::to_string(unsigned(reply->result))+
                " sequence="+std::to_string(reply->reply_to) : " response_timeout outcome_unknown"));
            if(!reply) return gui::Result::response_timeout;
            switch(reply->result) {
            case fcs::Result::accepted: return gui::Result::success;
            case fcs::Result::busy: return gui::Result::device_busy;
            case fcs::Result::device_error: return gui::Result::device_fault;
            default: return gui::Result::device_rejected;
            }
        };
        uint32_t target_id{}, active{}, frame_sequence{};
        AssignmentHealth health; auto& assignment=health.state;
        LidarInterlock lidar_interlock;
        ManualSequence manual;
        BatchSequence batch;
        RdrStartup rdr_startup;
        c2::rdr::ObservationRetention observation_retention;
        c2::rdr::HitRetention hit_retention;
        auto laser_off=[&]() {
            // Keep retrying even when an explicit CLI/GUI OFF fails or throws.
            laser_deadline=Clock::now()+std::chrono::seconds(1);
            auto reply=client->laser(false);
            lidar_interlock.off_reply(reply,Clock::now());
            if(reply&&reply->result==fcs::Result::accepted&&reply->laser_state==0)
                laser_deadline={};
            return reply;
        };
        gui::Result output_block_result=gui::Result::lidar_stop_unconfirmed;
        auto prepare_output=[&]() {
            output_block_result=gui::Result::lidar_stop_unconfirmed;
            if(rdr_startup.blocks_recovery()) {
                output_block_result=gui::Result::lidar_recovery_required;
                log("LIDAR output blocked: RDR startup incomplete");return false;
            }
            const auto before=real_rdr->snapshot();
            if(before.thermal_fault()){output_block_result=gui::Result::lidar_thermal_fault;return false;}
            if(!client||!lidar_interlock.begin(before,Clock::now())) {
                switch(lidar_interlock.block_reason(before)) {
                case LidarInterlock::Block::disconnected: output_block_result=gui::Result::lidar_disconnected; break;
                case LidarInterlock::Block::unsupported: output_block_result=gui::Result::lidar_unsupported; break;
                case LidarInterlock::Block::unknown: output_block_result=gui::Result::lidar_unknown; break;
                case LidarInterlock::Block::recovery_required: output_block_result=gui::Result::lidar_recovery_required; break;
                case LidarInterlock::Block::busy: output_block_result=gui::Result::lidar_busy; break;
                default: break;
                }
                log(std::string("LIDAR output blocked: ")+result_name(output_block_result)+
                    " supported="+std::to_string(before.lidar_supported)+" known="+std::to_string(before.lidar_known)+
                    " inhibited="+std::to_string(before.lidar_inhibited)+" last_result="+std::to_string(before.last_result)+
                    " phase="+std::to_string(unsigned(lidar_interlock.phase())));
                return false;
            }
            const c2::rdr::BasePoint pause_origin{-installation.rdr_position[1],installation.rdr_position[0],installation.rdr_position[2]};
            const auto pause_state=real_rdr->snapshot();
            observation_retention.begin(pause_state,position_filter.update(pause_state,pause_origin),pause_origin,c2::rdr::seconds());
            position_filter.reset();
            health.observation_paused();
            try {
                if(real_rdr->command(0xED07,0)>1) {lidar_interlock.fail();return false;}
                log("LIDAR ED07 OFF requested; waiting for matching DE07 STOP completion");
                const auto deadline=Clock::now()+std::chrono::duration<double>(c2::rdr::lidar_command_timeout_seconds);
                while(!quitting&&Clock::now()<deadline) {
                    real_rdr->tick();
                    const auto r=real_rdr->snapshot();
                    if(lidar_interlock.stopped(r)) {
                        // Recheck FCS immediately before enabling output; do not use a
                        // status snapshot taken before the RDR stop handshake.
                        auto status=client->query();
                        if(!status||!(status->flags&1)||(status->flags&30)||
                           !lidar_interlock.stopped(real_rdr->snapshot()))break;
                        lidar_interlock.emitted(Clock::now());
                        log(r.lidar_known?"LIDAR physical OFF confirmed; FCS output permitted":"LIDAR STOP command completed under thermal warning; optical state unknown; FCS output permitted");return true;
                    }
                    if(!r.connected||r.last_result>1)break;
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                real_rdr->tick(); // Expire the transport operation at the same deadline.
                lidar_interlock.fail();log("LIDAR stop unconfirmed; FCS output blocked");return false;
            } catch(...) {lidar_interlock.fail();throw;}
        };
        uint64_t assigned_rdr_generation{};
        unsigned logged_state=0, logged_reason=0;
        double commanded_pan{}, commanded_tilt{};
        AssignmentPositionGuard position_guard(c2::rdr::gui_noise_axes(
            position_filter_config.measurement_stddev_xyz_m,position_filter_config.measurement_stddev_m));
        bool demo_requested{}, demo_seen{};
        auto invalidate_installation_alignment=[&]() {
            batch.fail(gui::Result::target_changed);
            if (!target_id) return;
            if(manual.active())manual.fail(gui::Result::target_changed);
            health.fail(gui::Result::target_changed);
            active=0;demo_requested=demo_seen=false;
            log("Installation or calibration changed: existing alignment invalid; reassign required");
        };
        Clock::time_point next_poll{}, next_log{}, move_deadline{}, next_installation_check{}, next_rdr_retry{};
        auto logged_lidar_phase=lidar_interlock.phase();
        bool thermal_output_stopped=false;
        Clock::time_point last_assignment_cancel{};
        Clock::time_point last_calibration_move{};
        InstallationConfig geometry_snapshot{};
        FcsGeometry geometry_pending{};
        uint32_t geometry_token{},geometry_parts{};
        Clock::time_point geometry_deadline{};
        const auto same_geometry_basis=[](const InstallationConfig& a,const InstallationConfig& b) {
            return a.rdr_position==b.rdr_position && a.eff_position==b.eff_position &&
                a.fcs_angle_offset_deg==b.fcs_angle_offset_deg && a.fcs_geometry==b.fcs_geometry;
        };
        Clock::time_point last_fcs_status_sample{};
        bool have_fcs_busy_sample{};
        bool last_fcs_busy{};
        const auto diagnostic_start = Clock::now();
        const auto diagnostic_us = [diagnostic_start](Clock::time_point time) {
            return std::chrono::duration_cast<std::chrono::microseconds>(time - diagnostic_start).count();
        };
        bool installation_missing{};
        ConsoleInput console(headless);
        bool rdr_live{};
        Clock::time_point next_rdr_live{};
        std::cout << nlohmann::json{{"ready",true},{"runtime","C++"},{"network",network},{"rdr_protocol","edde24"},{"fcs_protocol","effe"}}.dump()<<std::endl;
        while (!quitting && (stop_file.empty() || !std::filesystem::exists(stop_file))) {
            enum class RdrReply { none, state, targets };
            auto rdr_reply=RdrReply::none;
            if(auto line=console.next()) {
                try {
                    std::istringstream input(*line);std::string device,command,arg;input>>device>>command;
                    std::istringstream validation(*line);
                    const std::vector<std::string> words{std::istream_iterator<std::string>{validation}, {}};
                    size_t expected=0;
                    if(device=="quit" || device=="exit")expected=1;
                    else if(device=="sequence"&&command=="confirm-hit")expected=3;
                    else if(device=="rdr") {
                        if(command=="connect"||command=="status"||command=="targets"||command=="start"||command=="stop")expected=2;
                        else if(command=="report"||command=="exclude"||command=="resume"||command=="live")expected=3;
                    } else if(device=="fcs") {
                        if(command=="connect"||command=="status")expected=2;
                        else if(command=="point")expected=4;
                        else if(command=="laser")expected=3;
                    }
                    if((manual.active()||batch.active())&&((device=="fcs"&&command=="point")||
                        (device=="fcs"&&command=="laser"&&words.back()=="on")||
                        (device=="rdr"&&command!="status"&&command!="targets"&&command!="live")))
                        throw std::invalid_argument("Manual sequence owns devices; cancel it before independent control");
                    if(!expected || words.size()!=expected)throw std::invalid_argument("invalid command or argument count");
                    if(device=="quit" || device=="exit")break;
                    if(device=="sequence"&&command=="confirm-hit") {
                        const auto id=parse_port(words[2]);
                        const auto r=real_rdr->snapshot();const auto f=client?client->snapshot():fcs::Snapshot{};
                        if(!manual.active()||manual.target!=id||!manual.result_started||
                           !(manual.phase==ManualSequence::Phase::observing||
                             (manual.phase==ManualSequence::Phase::failed&&manual.reason==gui::Result::result_unconfirmed))||
                           r.generation!=manual.generation||!ManualSequence::resumed(r)||
                           !f.connected||!f.status||Clock::now()-f.status_received_at>std::chrono::seconds(2)||
                           (f.status->flags&30))throw std::invalid_argument("Manual result confirmation unavailable");
                        manual.enter(ManualSequence::Phase::hit,Clock::now());
                        hit_retention.confirm(uint16_t(manual.target));
                        log("MANUAL operator_confirmed_hit target="+std::to_string(id)+" source=console_visual_confirmation");
                        std::cout<<nlohmann::json{{"manual_result","hit"},{"target",id},{"source","operator"}}.dump()<<std::endl;
                    } else if(device=="rdr" && command=="live") {
                        input>>arg;
                        if(arg!="on" && arg!="off")throw std::invalid_argument("live on|off");
                        rdr_live=arg=="on";
                        next_rdr_live={};
                        std::cout<<nlohmann::json{{"rdr_live",rdr_live},{"interval_ms",200}}.dump()<<std::endl;
                    } else if(device=="rdr") {
                        if(command=="connect")real_rdr->connect();
                        else if(command=="start")real_rdr->command(0xED06,1);
                        else if(command=="stop") {real_control->cancel_start();real_rdr->command(0xED06,0);}
                        else if(command=="report") {input>>arg;if(arg!="on"&&arg!="off")throw std::invalid_argument("report on|off");real_rdr->command(0xED03,arg=="on");}
                        else if(command=="exclude"||command=="resume") {input>>arg;auto id=parse_port(arg);real_rdr->command(0xED04,id,command=="resume");}
                        else if(command!="status"&&command!="targets")throw std::invalid_argument("unknown RDR command");
                        rdr_reply=command=="targets"?RdrReply::targets:RdrReply::state;
                    } else if(device=="fcs" && client) {
                        std::optional<fcs::Packet> response;
                        if(command=="connect")response=client->connect();
                        else if(command=="status")response=client->query();
                        else if(command=="point") {
                            auto angle=[](const std::string& text) {double number{};
                                auto [end,error]=std::from_chars(text.data(),text.data()+text.size(),number);
                                if(error!=std::errc{}||end!=text.data()+text.size())throw std::invalid_argument("point requires degrees in 0.1 degree increments");
                                (void)fcs::angle_to_wire(number);
                                return number;
                            };
                            response=client->point(angle(words[2]),angle(words[3]));
                        }
                        else if(command=="laser") {
                            input>>arg;if(arg!="on"&&arg!="off")throw std::invalid_argument("laser on|off");
                            if(arg=="on") {
                                if(!prepare_output())throw std::runtime_error("LIDAR stop not confirmed; output blocked (use fcs laser off to recover)");
                                laser_deadline=Clock::now()+std::chrono::seconds(1);
                                response=client->laser(true);
                            } else {response=laser_off();if(manual.active())manual.fail(gui::Result::device_error);batch.fail(gui::Result::device_error);}
                        }
                        else throw std::invalid_argument("unknown FCS command");
                        auto snapshot=client->snapshot();
                        auto report=nlohmann::json{{"fcs",command},{"confirmed",bool(response)},{"result",response?int(response->result):-1},{"connected",snapshot.connected}};
                        if(snapshot.status) {report["elevation"]=snapshot.status->elevation;report["azimuth"]=snapshot.status->azimuth;report["flags"]=snapshot.status->flags;}
                        std::cout<<report.dump()<<std::endl;
                    } else throw std::invalid_argument("unknown device command");
                } catch(const std::exception& e) {std::cout<<nlohmann::json{{"rejected",e.what()}}.dump()<<std::endl;}
            }
            if(!headless&&console.ended()&&rdr_reply==RdrReply::none)break;
            if(client&&laser_deadline!=Clock::time_point{}&&Clock::now()>=laser_deadline) {
                try {
                    (void)laser_off();
                }catch(const std::exception& error){log(std::string("FCS OFF retry: ")+error.what());}
            }
            if (!c2_ipc_healthy(ipc) || !c2_ipc_healthy(rdr_control)) throw std::runtime_error("IPC stopped");
            if (!installation_file.empty() && Clock::now() >= next_installation_check) {
                next_installation_check = Clock::now()+std::chrono::milliseconds(500);
                std::error_code error;
                auto modified = std::filesystem::last_write_time(installation_file,error);
                if (error) {
                    if (!installation_missing) log("display_config unavailable; retaining last valid positions");
                    installation_missing = true;
                } else if (installation_missing || modified != installation_seen) {
                    installation_missing = false;
                    installation_seen = modified;
                    try {
                        auto updated = load_installation_settings(installation_file);
                        if(updated.rdr_position!=installation.rdr_position || updated.eff_position!=installation.eff_position ||
                            updated.fcs_angle_offset_deg!=installation.fcs_angle_offset_deg || updated.fcs_geometry!=installation.fcs_geometry)
                            invalidate_installation_alignment();
                        installation = std::move(updated);
                        log("display_config reloaded");
                    } catch (const std::exception& failure) {
                        log(std::string("display_config rejected; retaining last valid positions: ")+failure.what());
                    }
                }
            }
            if (Clock::now() >= next_rdr_retry) {
                try {
                    // connect() preserves any outstanding ED01; tick() handles its retries.
                    if (real_rdr && !real_rdr->snapshot().connected) (void)real_rdr->connect();
                    if (real_control) real_control->tick(c2::rdr::seconds());

                }
                catch (const std::exception& error) {
                    log(std::string("RDR network retry: ")+error.what());
                    next_rdr_retry=Clock::now()+std::chrono::seconds(1);
                }
            }
            if (client && Clock::now() >= next_poll) {
                // Schedule from request start so response latency is not added to the period.
                next_poll = Clock::now()+std::chrono::milliseconds(100);
                bool connected=client->snapshot().connected;
                try {
                    auto reply=connected?client->query():client->connect();
                    if(!reply) log(std::string("FCS ")+(connected?"EF02":"EF01")+" response_timeout");
                } catch (const std::exception& error) {
                    log(std::string("FCS network retry: ")+error.what());
                }
            }
            auto state = client?client->snapshot():fcs::Snapshot{};
            if(rdr_startup.active()) {
                const auto previous=rdr_startup.phase;
                try {rdr_startup.tick(*real_rdr,*real_control,client.get(),Clock::now());}
                catch(const std::exception& error) {
                    rdr_startup.fail(*real_control,gui::Result::fcs_off_unconfirmed);
                    log(std::string("RDR startup exception: ")+error.what());
                }
                if(previous!=rdr_startup.phase) {
                    log(std::string("RDR startup phase=")+rdr_startup.name()+" result="+result_name(rdr_startup.result));
                    if(rdr_startup.phase==RdrStartup::Phase::complete)lidar_interlock=LidarInterlock{};
                    else if(rdr_startup.phase==RdrStartup::Phase::failed)lidar_interlock.fail();
                }
            }
            if(client&&!manual.active()&&!batch.active()&&!rdr_startup.blocks_recovery()&&lidar_interlock.update(real_rdr->snapshot(),state,Clock::now())) {
                try {
                    const auto result=real_rdr->command(0xED07,1);
                    log("LIDAR ED07 resume requested result="+std::to_string(result));
                    if(result>1)lidar_interlock.fail();
                }catch(const std::exception& error){lidar_interlock.fail();log(std::string("LIDAR resume failed: ")+error.what());}
            }
            if(logged_lidar_phase!=lidar_interlock.phase()) {
                static constexpr const char* names[]={"idle","stopping","held","restoring","fault"};
                logged_lidar_phase=lidar_interlock.phase();
                log(std::string("LIDAR interlock=")+names[unsigned(logged_lidar_phase)]+(logged_lidar_phase==LidarInterlock::Phase::fault?
                    "; check device, then acknowledge fcs laser off before recovery":""));
            }
            const auto rdr_state=real_rdr->snapshot();
            if(rdr_state.thermal_fault()&&!thermal_output_stopped){
                thermal_output_stopped=true;lidar_interlock.fail();
                log("LIDAR thermal_protective_stop; requesting FCS OFF; automatic recovery disabled");
                if(client)try{(void)laser_off();}catch(const std::exception& error){log(error.what());}
            }
            if(rdr_state.connected&&!rdr_state.thermal_fault())thermal_output_stopped=false;
            // Installation storage is (forward,left,up); corrected coordinates
            // and GUI XYZ are (east,north,up). Rotate raw DE03 coordinates once.
            const c2::rdr::BasePoint rdr_origin{-installation.rdr_position[1],installation.rdr_position[0],installation.rdr_position[2]};
            if (rdr_live && Clock::now()>=next_rdr_live) {
                std::cout<<c2::rdr::live_diagnostics(rdr_state,rdr_origin,unix_ms()).dump()<<std::endl;
                next_rdr_live=Clock::now()+std::chrono::milliseconds(200);
            }
            auto corrected_targets=c2::rdr::correct_targets(rdr_state,rdr_origin);
            // Domain coordinates are computed before either consumer runs.
            // GUI serialization never feeds coordinates back into control.
            if (!real_control->operating(rdr_state)) position_filter.reset();
            auto filtered_targets=position_filter.update(rdr_state,rdr_origin);
            std::set<uint16_t> retained_assignments;
            if(manual.active())retained_assignments.insert(uint16_t(manual.target));
            for(const auto& [id,row]:batch.rows())retained_assignments.insert(uint16_t(id));
            hit_retention.observe(filtered_targets,retained_assignments);
            auto display_targets=observation_retention.update(rdr_state,filtered_targets,rdr_origin,
                c2::rdr::seconds(),lidar_interlock.phase()==LidarInterlock::Phase::idle,
                manual.phase==ManualSequence::Phase::fired||manual.phase==ManualSequence::Phase::resuming||
                manual.phase==ManualSequence::Phase::observing);
            auto observation = observation_snapshot(rdr_state,filtered_targets);
            bool current = state.connected && state.status &&
                Clock::now()-state.status_received_at < std::chrono::seconds(2);
            uint16_t flags = current ? state.status->flags : 0;
            if (!current) have_fcs_busy_sample = false;
            if (current && (!have_fcs_busy_sample || state.status_received_at != last_fcs_status_sample)) {
                const bool busy = (flags & 10) != 0;
                if (!have_fcs_busy_sample || busy != last_fcs_busy) {
                    log("FCS busy_observed="+std::to_string(busy)+
                        " observation="+(have_fcs_busy_sample?"change":"initial")+
                        " status_rx_us="+std::to_string(diagnostic_us(state.status_received_at))+
                        " previous_status_rx_us="+std::to_string(have_fcs_busy_sample?
                            diagnostic_us(last_fcs_status_sample):-1)+" flags="+std::to_string(flags));
                }
                last_fcs_status_sample = state.status_received_at;
                last_fcs_busy = busy;
                have_fcs_busy_sample = true;
            }
            bool ready = current && !(flags&16) && state.status->azimuth != -32768 && state.status->elevation != -32768;
            auto assigned_now = std::find_if(observation.targets.begin(), observation.targets.end(),
                [&](const Target& t) { return t.id == target_id; });
            const bool target_consistent=target_id && assigned_now!=observation.targets.end() &&
                position_guard.consistent({assigned_now->east,assigned_now->north,assigned_now->up},
                    rdr_state.targets.at(uint16_t(assigned_now->id)).seen);
            bool aligned=current && fcs_pan_error(state.status->azimuth,commanded_pan)<=1 &&
                std::abs(state.status->elevation-commanded_tilt)<=1;
            if (real_rdr && target_id && observation.generation!=assigned_rdr_generation)
                health.fail(gui::Result::target_changed);
            if(!manual.active())health.update(current,ready,assigned_now!=observation.targets.end(),target_consistent,
                aligned,(flags&2)!=0,demo_requested,Clock::now()>move_deadline);
            if(manual.active()) {
                const auto before=manual.phase;
                manual.tick(rdr_state,state,target_consistent,Clock::now(),c2::rdr::seconds());
                if(before!=manual.phase) {
                    if(manual.phase==ManualSequence::Phase::hit)hit_retention.confirm(uint16_t(manual.target));
                    log("MANUAL target="+std::to_string(manual.target)+" state="+std::to_string(manual.state())+
                        " reason="+result_name(manual.reason));
                    if(manual.phase==ManualSequence::Phase::observing)lidar_interlock=LidarInterlock{};
                }
                if(manual.phase==ManualSequence::Phase::failed&&manual.reason!=gui::Result::result_unconfirmed&&client&&!manual.recovery_requested) {
                    manual.recovery_requested=true;
                    log("MANUAL result unconfirmed; explicit cancellation required; requesting FCS OFF");
                    try{(void)laser_off();}catch(const std::exception& e){log(e.what());}
                }
            }
            auto batch_context=[&]() {
                return BatchSequence::Context{rdr_state,state,filtered_targets,installation,Clock::now(),
                    c2::rdr::seconds(),observation.scanning,manual.active()||target_id||rdr_startup.blocks_recovery(),
                    lidar_interlock.phase()==LidarInterlock::Phase::idle};
            };
            {
                const auto before=batch.phase();
                std::map<uint32_t,unsigned> previous;
                for(const auto& [id,row]:batch.rows())previous[id]=row.sequence.state();
                batch.tick(batch_context());
                for(const auto& [id,row]:batch.rows())
                    if(previous[id]!=108&&row.sequence.phase==ManualSequence::Phase::hit)hit_retention.confirm(uint16_t(id));
                if(before==BatchSequence::Phase::resuming&&
                   (batch.phase()==BatchSequence::Phase::observing||batch.phase()==BatchSequence::Phase::idle))
                    lidar_interlock=LidarInterlock{};
                for(const auto& [id,row]:batch.rows())if(previous[id]!=row.sequence.state())
                    log("BATCH target="+std::to_string(id)+" state="+std::to_string(row.sequence.state())+
                        " reason="+result_name(row.sequence.reason));
                if(client&&batch.take_off_request()) {
                    log("BATCH failed; explicit cancellation required; requesting FCS OFF");
                    try{(void)laser_off();}catch(const std::exception& e){log(e.what());}
                    next_poll={};
                }
            }
            auto assignment_details=[&](uint32_t inspected_target) {
                const bool diagnostic_aligned=current && fcs_pan_error(state.status->azimuth,commanded_pan)<=1 &&
                    std::abs(state.status->elevation-commanded_tilt)<=1;
                nlohmann::json detail={{"target",inspected_target},{"fcs_connected",state.connected},
                    {"status_current",current},{"device_ready",ready},{"flags",flags},
                    {"status_age_ms",state.status?std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now()-state.status_received_at).count():-1},
                    {"rdr_connected",observation.connected},{"rdr_reporting",observation.streaming},
                    {"rdr_operating",observation.scanning},{"rdr_generation",observation.generation},
                    {"sensor_state",rdr_state.sensor_state},{"sensor_health_valid",rdr_state.sensor_health_valid},
                    {"thermal_protective_stop",rdr_state.thermal_protective_stop},{"lidar_supported",rdr_state.lidar_supported},{"lidar_known",rdr_state.lidar_known},
                    {"lidar_on",rdr_state.lidar_on},{"lidar_inhibited",rdr_state.lidar_inhibited},
                    {"lidar_request",rdr_state.lidar_request},{"lidar_result",rdr_state.lidar_result},
                    {"lidar_interlock_phase",unsigned(lidar_interlock.phase())},
                    {"rdr_startup_phase",rdr_startup.name()},{"manual_state",manual.state()},
                    {"manual_target",manual.target},{"manual_reason",result_name(manual.reason)},
                    {"assigned_generation",assigned_rdr_generation},{"aligned",diagnostic_aligned},
                    {"commanded_pan",commanded_pan},{"commanded_tilt",commanded_tilt},
                    {"fcs_angle_offset_deg",installation.fcs_angle_offset_deg},
                    {"actual_pan",nullptr},{"actual_tilt",nullptr},
                    {"filtered_xyz",nullptr},{"reference_xyz",nullptr},{"delta_xyz",nullptr},
                    {"tolerance_xyz_m",position_guard.tolerance()},{"outside_samples",position_guard.outside_samples()}};
                if(state.status) {detail["actual_pan"]=state.status->azimuth;detail["actual_tilt"]=state.status->elevation;}
                const auto now=std::find_if(observation.targets.begin(),observation.targets.end(),
                    [&](const Target& t){return t.id==inspected_target;});
                detail["target_present"]=now!=observation.targets.end();
                if(now!=observation.targets.end()) detail["filtered_xyz"]={now->east,now->north,now->up};
                if(inspected_target==target_id && position_guard.reference()) {
                    const auto& ref=*position_guard.reference();
                    detail["reference_xyz"]={ref.east,ref.north,ref.up};
                    if(now!=observation.targets.end())
                        detail["delta_xyz"]={now->east-ref.east,now->north-ref.north,now->up-ref.up};
                }
                return detail.dump();
            };
            if(logged_state!=assignment || logged_reason!=unsigned(health.reason)) {
                log("assignment="+std::to_string(assignment)+" reason="+std::to_string(unsigned(health.reason))+" reason_name="+result_name(health.reason)+
                    " target="+std::to_string(target_id)+" flags="+std::to_string(flags)+
                    " status_age_ms="+std::to_string(state.status?std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now()-state.status_received_at).count():-1)+
                    " target_present="+std::to_string(assigned_now!=observation.targets.end())+
                    " target_consistent="+std::to_string(target_consistent)+" aligned="+std::to_string(aligned)+
                    " detail="+assignment_details(target_id));
                logged_state=assignment; logged_reason=unsigned(health.reason);
            }
            if (demo_requested && current) {
                if (flags&8) demo_seen=true;
                if (demo_seen && ready && !(flags&14)) { active=1; demo_requested=false; }
            }
            gui::PipeCommand request; uint64_t ticket;
            void* request_ipc=ipc;
            bool has_request=c2_ipc_poll(ipc,&request,&ticket)!=0;
            if (!has_request) { request_ipc=rdr_control; has_request=c2_ipc_poll(rdr_control,&request,&ticket)!=0; }
            if (has_request) {
                const auto request_received = Clock::now();
                if (request.command == 1 || request.command == 1001) {
                    log("GUI assignment_received request_id="+std::to_string(request.request_id)+
                        " command="+std::to_string(request.command)+
                        " requested_target="+std::to_string(request.target_id)+
                        " request_rx_us="+std::to_string(diagnostic_us(request_received))+
                        " status_rx_us="+std::to_string(state.status?diagnostic_us(state.status_received_at):-1)+
                        " busy_observed="+(current?((flags&10)?"1":"0"):"unknown")+
                        " detail="+assignment_details(request.target_id));
                }
                gui::PipeResponse response;
                response.request_id = request.request_id; response.command = request.command; response.fcs_id=1;
                const bool stop_manual_capture=request.command==uint32_t(gui::Command::stop_manual_capture);
                const bool manual_command=(request.command>=1001&&request.command<=1009)||stop_manual_capture;
                if(gui::is_batch_command(request.command)) {
                    auto result=gui::Result::invalid_request;
                    if(request.request_id&&request_ipc==ipc&&request.rdr_id==1&&request.fcs_id==1) {
                        BatchSequence::Actions actions;
                        actions.stop=[&]() {
                            if(!lidar_interlock.begin(real_rdr->snapshot(),Clock::now()))return gui::Result::lidar_recovery_required;
                            observation_retention.begin(rdr_state,filtered_targets,rdr_origin,c2::rdr::seconds());
                            position_filter.reset();
                            return real_rdr->command(0xED07,0)<=1?gui::Result::success:gui::Result::lidar_stop_unconfirmed;
                        };
                        actions.resume=[&]() {
                            return real_rdr->command(0xED07,1)<=1?gui::Result::success:gui::Result::lidar_resume_unconfirmed;
                        };
                        actions.point=[&](double tilt,double pan) {
                            next_poll={};return client?command_result("EF03_BATCH",client->point(tilt,pan)):gui::Result::asset_unavailable;
                        };
                        actions.capture=[&]() {
                            next_poll={};return client?command_result("EF05_BATCH",client->demo()):gui::Result::asset_unavailable;
                        };
                        actions.off=[&]() {
                            if(!client)return gui::Result::asset_unavailable;
                            const auto reply=laser_off();next_poll={};last_assignment_cancel=Clock::now();
                            const auto result=command_result("EF04_OFF_BATCH",reply);
                            return result==gui::Result::success&&reply->laser_state!=0?gui::Result::fcs_off_unconfirmed:result;
                        };
                        try {
                            std::optional<FcsPointAngles> aim_override;
                            if(request.command==uint32_t(gui::Command::aim_batch_target))
                                aim_override=gui_fcs_point_override(request.position_x,request.position_y,request.position_z);
                            result=batch.command(request.command,request.target_id,batch_context(),actions,aim_override);
                        }
                        catch(const std::invalid_argument& error){result=gui::Result::invalid_request;log(error.what());}
                        catch(const std::exception& error){result=gui::Result::device_error;log(error.what());}
                    }
                    response.result=uint32_t(result);response.target_id=request.target_id;
                    response.assignment=batch.state(request.target_id);response.active=batch.flags(batch_context());
                    response.connected=state.connected;
                    c2_ipc_reply(request_ipc,&response,ticket);
                    if(request.command!=1200)log("BATCH command="+std::to_string(request.command)+
                        " request_id="+std::to_string(request.request_id)+" target="+std::to_string(request.target_id)+
                        " state="+std::to_string(response.assignment)+" flags="+std::to_string(response.active)+
                        " result="+result_name(result)+" reason="+result_name(batch.reason(request.target_id)));
                } else if(request.command==uint32_t(gui::Command::get_rdr_sensor_health)) {
                    const auto h=real_rdr->snapshot();response.connected=h.connected;
                    response.target_id=255;
                    if(!request.request_id||request_ipc!=ipc||request.rdr_id!=1||request.target_id)
                        response.result=uint32_t(gui::Result::invalid_request);
                    else if(!h.connected)response.result=uint32_t(gui::Result::asset_unavailable);
                    else {response.target_id=h.sensor_state;response.assignment=h.sensor_health_valid;response.active=h.thermal_protective_stop;}
                    c2_ipc_reply(request_ipc,&response,ticket);
                } else if(request.command==uint32_t(gui::Command::get_rdr_lidar_state)) {
                    const auto optical=real_rdr->snapshot();
                    response.connected=optical.connected;
                    if(!request.request_id||request_ipc!=ipc||request.rdr_id!=1||request.target_id)
                        response.result=uint32_t(gui::Result::invalid_request);
                    else if(!optical.connected)response.result=uint32_t(gui::Result::asset_unavailable);
                    else {response.assignment=optical.lidar_known?1:0;response.active=optical.lidar_known&&optical.lidar_on?1:0;}
                    c2_ipc_reply(request_ipc,&response,ticket);
                } else if(request.command==uint32_t(gui::Command::get_manual_status)) {
                    if(!request.request_id||request_ipc!=ipc||request.rdr_id!=1||request.fcs_id!=1)
                        response.result=uint32_t(gui::Result::invalid_request);
                    else if(!manual.active())response.result=uint32_t(gui::Result::not_supported);
                    else {
                        response.result=uint32_t(manual.active()?manual.reason:health.reason);
                        response.target_id=target_id;
                        response.assignment=manual.active()?manual.state():assignment;
                        response.active=manual.active()?manual.completed_step():0;
                        response.connected=state.connected;
                    }
                    c2_ipc_reply(request_ipc,&response,ticket);
                } else if (real_control && request.command>=10 && request.command<1000 && request.command!=18) {
                    if((manual.active()||batch.active())&&request.command>=10&&request.command<=16) {
                        response.result=uint32_t(gui::Result::device_busy);
                        response.target_id=manual.target;response.assignment=manual.state();response.connected=rdr_state.connected;
                    } else response=real_control->handle(request,c2::rdr::seconds());
                    c2_ipc_reply(request_ipc,&response,ticket);
                } else {
                auto result = gui::Result::success;
                FcsPointAngles calibration_angles{};
                std::array<double,3> geometry_values{};
                bool geometry_enabled{};
                try {
                    if (!request.request_id) result=gui::Result::invalid_request;
                    else if(batch.active()&&(request.command==1||request.command==2||request.command==3||
                        request.command==5||request.command==6||request.command==9||
                        manual_command))result=gui::Result::device_busy;
                    else if(request.command>=1110 && request.command<=1114) {
                        if(request_ipc!=ipc || request.rdr_id!=1 || request.fcs_id!=1 || !request.target_id)
                            result=gui::Result::invalid_request;
                        else if(request.command==1110) {
                            geometry_snapshot=installation;geometry_token=request.target_id;geometry_parts=0;
                            geometry_values=geometry_snapshot.fcs_geometry.translation;geometry_enabled=geometry_snapshot.fcs_geometry.enabled;
                        } else if(request.target_id!=geometry_token)result=gui::Result::invalid_request;
                        else if(request.command==1111) {
                            geometry_values=geometry_snapshot.fcs_geometry.rotation;geometry_enabled=geometry_snapshot.fcs_geometry.enabled;
                        } else if(!same_geometry_basis(installation,geometry_snapshot))result=gui::Result::target_changed;
                        else if(manual.active() || batch.active() || target_id || (flags&14) || lidar_interlock.phase()!=LidarInterlock::Phase::idle)
                            result=gui::Result::device_busy;
                        else if(!current || !ready)result=gui::Result::asset_unavailable;
                        else if(request.command==1112) {
                            geometry_parts=0; // A malformed replacement must invalidate older staged values.
                            geometry_pending=FcsGeometry{};
                            geometry_pending.translation=camera_triple(nlohmann::json::array({request.position_x,request.position_y,request.position_z}));
                            geometry_pending.rdr_anchor=installation.rdr_position;geometry_pending.fcs_anchor=installation.eff_position;
                            geometry_parts=1;geometry_deadline=Clock::now()+std::chrono::seconds(30);
                        } else if(!geometry_parts || Clock::now()>geometry_deadline)result=gui::Result::invalid_request;
                        else if(request.command==1113) {
                            geometry_parts=0;
                            geometry_pending.rotation=camera_triple(nlohmann::json::array({request.position_x,request.position_y,request.position_z}));
                            geometry_parts=3;
                        } else if(geometry_parts!=3 || (request.position_x!=0 && request.position_x!=1) || request.position_y!=0 || request.position_z!=0)
                            result=gui::Result::invalid_request;
                        else {
                            auto saved=load_installation_settings(installation_file,standalone_installation);
                            if(!same_geometry_basis(saved,geometry_snapshot))result=gui::Result::target_changed;
                            else {
                                geometry_pending.enabled=request.position_x==1;saved.fcs_geometry=geometry_pending;
                                validate_camera_settings(saved);
                                save_installation_settings(installation_file,saved,standalone_installation);
                                invalidate_installation_alignment();installation=saved;geometry_snapshot=saved;geometry_parts=0;
                                log("FCS 3D calibration saved; no motion requested");
                            }
                        }
                    }
                    else if(request.command>=1100 && request.command<=1102) {
                        // Calibration uses only EF03 positioning, never capture/output commands.
                        if(request_ipc!=ipc || request.rdr_id!=1 || request.fcs_id!=1)
                            result=gui::Result::invalid_request;
                        else if(!client || !current || !ready) result=gui::Result::asset_unavailable;
                        else if(state.status_received_at<=last_calibration_move) result=gui::Result::status_pending;
                        else if(manual.active() || batch.active() || target_id || (flags&14) ||
                            lidar_interlock.phase()!=LidarInterlock::Phase::idle)
                            result=gui::Result::device_busy;
                        else if(request.command==1100) {
                            const double pan=request.position_x, tilt=request.position_y;
                            if(request.target_id || !std::isfinite(pan) || !std::isfinite(tilt) ||
                                request.position_z!=0 || pan<0 || pan>=360 || tilt< -30 || tilt>90 ||
                                fcs_pan_error(pan,state.status->azimuth)>30.01 ||
                                std::abs(tilt-state.status->elevation)>30.01)
                                result=gui::Result::invalid_request;
                            else { result=command_result("EF03_CALIBRATION",client->point(
                                    std::round(tilt*10)/10,std::fmod(std::round(pan*10)/10,360.)));
                                last_calibration_move=Clock::now(); next_poll={}; }
                        } else if(request.command==1102) {
                            if(request.target_id) result=gui::Result::invalid_request;
                            else calibration_angles={state.status->elevation,state.status->azimuth};
                        } else {
                            const auto found=std::find_if(observation.targets.begin(),observation.targets.end(),
                                [&](const Target& t){return t.id==request.target_id;});
                            if(!request.target_id || !observation.scanning || !observation.streaming ||
                                found==observation.targets.end()) result=gui::Result::target_pending;
                            else calibration_angles=fcs_point_angles(found->east+installation.eff_position[1],
                                found->north-installation.eff_position[0],found->up-installation.eff_position[2]);
                        }
                    }
                    else if(manual_command) {
                        using P=ManualSequence::Phase;
                        const auto now=Clock::now();
                        if(request_ipc!=ipc||request.rdr_id!=1||request.fcs_id!=1||!request.target_id||request.target_id>65535)
                            result=gui::Result::invalid_request;
                        else if(request.command==1002||request.command==1006||request.command==1008)
                            result=gui::Result::not_supported;
                        else if(rdr_state.thermal_fault())result=gui::Result::lidar_thermal_fault;
                        else if(!client||!current||!ready)result=gui::Result::asset_unavailable;
                        else if(request.command==1001) {
                            const auto found=std::find_if(observation.targets.begin(),observation.targets.end(),
                                [&](const Target& t){return t.id==request.target_id;});
                            if(manual.active()&&manual.target==request.target_id&&manual.phase==P::registered){}
                            else if(target_id||manual.active())result=gui::Result::already_assigned;
                            else if(!observation.scanning||!observation.streaming||found==observation.targets.end()||
                                    rdr_state.lidar_inhibited) {
                                result=gui::Result::target_pending;
                                nlohmann::json blockers=nlohmann::json::array();
                                if(!observation.scanning)blockers.push_back("rdr_not_operating");
                                if(!observation.streaming)blockers.push_back("rdr_not_reporting");
                                if(found==observation.targets.end())blockers.push_back("target_not_present");
                                if(rdr_state.lidar_inhibited)blockers.push_back("lidar_inhibited");
                                log("GUI assignment_blocked request_id="+std::to_string(request.request_id)+
                                    " requested_target="+std::to_string(request.target_id)+" blockers="+blockers.dump());
                            }
                            else if((flags&14)||lidar_interlock.phase()!=LidarInterlock::Phase::idle||rdr_startup.blocks_recovery())
                                result=gui::Result::device_busy;
                            else {
                                manual=ManualSequence{};manual.target=target_id=request.target_id;
                                manual.generation=assigned_rdr_generation=rdr_state.generation;
                                manual.enter(P::registered,now);health.reset();active=0;
                            }
                        } else if(!manual.active()||manual.target!=request.target_id)result=gui::Result::not_assigned;
                        else if(request.command==1003) {
                            if(manual.phase==P::aiming||manual.phase==P::aimed){}
                            else if(manual.phase!=P::registered)result=gui::Result::invalid_request;
                            else if(assigned_now==observation.targets.end()||!observation.scanning||!observation.streaming||
                                rdr_state.lidar_inhibited||(flags&14))result=gui::Result::target_pending;
                            else {
                                const auto aim_override=gui_fcs_point_override(request.position_x,request.position_y,request.position_z);
                                const auto angles=aim_override?*aim_override:fcs_calibrated_angles(assigned_now->east+installation.eff_position[1],
                                    assigned_now->north-installation.eff_position[0],assigned_now->up-installation.eff_position[2],
                                    installation.fcs_angle_offset_deg[0],installation.fcs_angle_offset_deg[1],installation.fcs_geometry);
                                manual.pan=commanded_pan=angles.azimuth;manual.tilt=commanded_tilt=angles.elevation;
                                auto reply=client->point(commanded_tilt,commanded_pan);
                                result=command_result("EF03_MANUAL",reply);
                                if(result==gui::Result::success){
                                    manual.capture_reference=c2::rdr::sensor_to_base(rdr_state.targets.at(uint16_t(target_id)));
                                    position_guard.start({assigned_now->east,assigned_now->north,assigned_now->up},
                                        rdr_state.targets.at(uint16_t(target_id)).seen);
                                    manual.enter(P::aiming,Clock::now());next_poll={};
                                } else if(!reply)manual.fail(result);
                            }
                        } else if(request.command==1004) {
                            if(manual.phase==P::pausing||manual.phase==P::paused){}
                            else if(manual.phase!=P::aimed||!target_consistent||!aligned||(flags&14))result=gui::Result::alignment_required;
                            else if(!lidar_interlock.begin(rdr_state,now))result=gui::Result::lidar_recovery_required;
                            else {
                                observation_retention.begin(rdr_state,filtered_targets,rdr_origin,c2::rdr::seconds());
                                position_filter.reset();
                                manual.paused_at=now;manual.enter(P::pausing,now);
                                if(real_rdr->command(0xED07,0)>1){manual.fail(gui::Result::lidar_stop_unconfirmed);result=manual.reason;}
                            }
                        } else if(request.command==1005) {
                            if(manual.phase==P::firing||manual.phase==P::stopping_capture||manual.phase==P::fired){}
                            else if(manual.phase!=P::paused)result=gui::Result::invalid_request;
                            else if(!ManualSequence::stopped(real_rdr->snapshot())||!aligned||(flags&14))result=gui::Result::lidar_stop_unconfirmed;
                            else {
                                // Existing EF05 capture command; legacy demo() wire API is unchanged.
                                manual.enter(P::firing,Clock::now());manual.output_seen=false;
                                const auto reply=client->demo();result=command_result("EF05_MANUAL",reply);
                                if(result!=gui::Result::success)manual.fail(result);
                                next_poll={};
                            }
                        } else if(stop_manual_capture) {
                            if(manual.phase==P::stopping_capture||manual.phase==P::fired){}
                            else if(manual.phase!=P::firing)result=gui::Result::invalid_request;
                            else {
                                try {
                                    const auto reply=laser_off();
                                    result=command_result("EF04_STOP_CAPTURE_MANUAL",reply);
                                    if(result==gui::Result::success&&reply->laser_state!=0)
                                        result=gui::Result::fcs_off_unconfirmed;
                                } catch(const std::exception& error) {
                                    log(error.what());result=gui::Result::fcs_off_unconfirmed;
                                }
                                if(result==gui::Result::success)manual.enter(P::stopping_capture,Clock::now());
                                else manual.fail(result);
                                next_poll={};
                            }
                        } else if(request.command==1007) {
                            if(manual.phase==P::resuming||manual.phase==P::observing||manual.phase==P::alive||manual.phase==P::hit){}
                            else if(manual.phase!=P::fired)result=gui::Result::invalid_request;
                            // Phase::fired already requires a fresh OFF observation after activity.
                            // Do not require another poll after publishing completion to the GUI.
                            else if(flags&14)result=gui::Result::fcs_off_unconfirmed;
                            else {
                                manual.enter(P::resuming,now);
                                if(real_rdr->command(0xED07,1)>1){manual.fail(gui::Result::lidar_resume_unconfirmed);result=manual.reason;}
                            }
                        } else if(request.command==1009) {
                            if(manual.phase!=P::alive)result=gui::Result::invalid_request;
                            else if(assigned_now==observation.targets.end()||!ManualSequence::resumed(rdr_state)||(flags&14))
                                result=gui::Result::target_pending;
                            else {manual.enter(P::registered,now);position_guard.reset();manual.output_seen=false;
                                manual.result_started=false;manual.recovery_requested=false;active=0;}
                        }
                    }
                    else if(manual.active()&&(request.command==1||request.command==3||request.command==5||
                            request.command==6||request.command==9))result=gui::Result::device_busy;
                    else if(request.command==18) {
                        result=request.rdr_id==1&&request.fcs_id==1?rdr_startup.result:gui::Result::invalid_request;
                    }
                    else if (request.command == 8 || request.command == 9) {
                        if (request.rdr_id || request.fcs_id || request.target_id<2 || request.target_id>4)
                            throw std::invalid_argument("Invalid camera settings group");
                        if (request.command==9) {
                            // Merge with disk to preserve unrelated settings and edits.
                            auto saved=load_installation_settings(installation_file,standalone_installation);
                            auto& triple=request.target_id==2?saved.ranges:
                                request.target_id==3?saved.rdr_fov:saved.fcs_angle_offset_deg;
                            triple={request.position_x,request.position_y,request.position_z};
                            validate_camera_settings(saved);
                            save_installation_settings(installation_file,saved,standalone_installation);
                            const bool rdr_changed=installation.rdr_position!=saved.rdr_position;
                            if(rdr_changed || installation.eff_position!=saved.eff_position ||
                                installation.fcs_angle_offset_deg!=saved.fcs_angle_offset_deg || installation.fcs_geometry!=saved.fcs_geometry)
                                invalidate_installation_alignment();
                            installation=saved;
                            if(rdr_changed) {
                                const auto position=installation.rdr_position;
                                const c2::rdr::BasePoint origin{-position[1],position[0],position[2]};
                                position_filter.reset();
                                corrected_targets=c2::rdr::correct_targets(rdr_state,origin);
                                filtered_targets=position_filter.update(rdr_state,origin);
                                display_targets=observation_retention.update(rdr_state,filtered_targets,origin,
                                    c2::rdr::seconds(),lidar_interlock.phase()==LidarInterlock::Phase::idle);
                                observation=observation_snapshot(rdr_state,filtered_targets);
                            }
                        }
                    }
                    else if (request.command == 5) {
                        const bool rdr_position=request.rdr_id==1 && request.fcs_id==0;
                        const bool fcs_position=request.fcs_id==1 && request.rdr_id==0;
                        if ((!rdr_position && !fcs_position) || !std::isfinite(request.position_x) ||
                            !std::isfinite(request.position_y) || !std::isfinite(request.position_z))
                            result=gui::Result::invalid_request;
                        else {
                            auto& position=rdr_position?installation.rdr_position:installation.eff_position;
                            // GUI (right,forward,up) -> installation (forward,left,up).
                            const std::array<double,3> updated{request.position_y,-double(request.position_x),request.position_z};
                            auto saved = load_installation_settings(installation_file, standalone_installation);
                            if((rdr_position?saved.rdr_position:saved.eff_position)!=updated)saved.fcs_geometry.enabled=false;
                            (rdr_position?saved.rdr_position:saved.eff_position) = updated;
                            save_installation_settings(installation_file, saved, standalone_installation);
                            std::error_code timestamp_error;
                            installation_seen = std::filesystem::last_write_time(installation_file, timestamp_error);
                            installation_missing = false;
                            installation.fcs_geometry=saved.fcs_geometry;
                            if (position!=updated) {
                                invalidate_installation_alignment();
                                position=updated;
                                if (rdr_position) {
                                    const auto calibrated=installation.rdr_position;
                                    corrected_targets=c2::rdr::correct_targets(rdr_state,
                                        {-calibrated[1],calibrated[0],calibrated[2]});
                                    filtered_targets=position_filter.update(rdr_state,
                                        {-calibrated[1],calibrated[0],calibrated[2]});
                                    display_targets=observation_retention.update(rdr_state,filtered_targets,
                                        {-calibrated[1],calibrated[0],calibrated[2]},c2::rdr::seconds(),
                                        lidar_interlock.phase()==LidarInterlock::Phase::idle);
                                    observation=observation_snapshot(rdr_state,filtered_targets);
                                }
                            }
                            log(std::string("GUI position saved: ")+(rdr_position?"RDR":"FCS")+
                                " XYZ="+std::to_string(request.position_x)+","+
                                std::to_string(request.position_y)+","+std::to_string(request.position_z));
                        }
                    }
                    else if (request.command == 6 && real_control) {
                        if(request.rdr_id!=1||request.fcs_id!=1||request.target_id>1)result=gui::Result::invalid_request;
                        else if(request.target_id==0) {
                            result=static_cast<gui::Result>(real_control->set_operating(request).result);
                            if(result==gui::Result::success) {
                                if(rdr_startup.active())lidar_interlock.fail();
                                rdr_startup=RdrStartup{};
                            }
                        } else if(rdr_startup.active()||target_id||demo_requested||laser_deadline!=Clock::time_point{})result=gui::Result::lidar_busy;
                        else if(!rdr_state.connected)result=gui::Result::lidar_disconnected;
                        else if(!rdr_state.lidar_supported)result=gui::Result::lidar_unsupported;
                        else if(rdr_state.last_result==1)result=gui::Result::lidar_busy;
                        else {
                            real_control->confirm_start(false);
                            rdr_startup.begin(request,rdr_state,Clock::now());
                            if(!client||!state.connected)log("RDR startup: FCS disconnected; skipping FCS laser OFF confirmation");
                            log("RDR startup phase=fcs_off (request accepted; not yet operating)");
                            next_poll={};
                        }
                        if (!real_control->operating(rdr_state)) position_filter.reset();
                    }
                    else if (request.fcs_id != 1 || request.rdr_id != 1) result=gui::Result::invalid_request;
                    else if (request.command == 0) { result=manual.active()?manual.reason:health.reason; }
                    else if (request.command == uint32_t(gui::Command::get_fcs_busy)) {
                        if (!current || state.status_received_at < last_assignment_cancel)
                            result=gui::Result::status_pending;
                        else if (!ready) result=gui::Result::device_fault;
                    }
                    else if (request.command == 4) result=request.target_id<=900?gui::Result::not_supported:gui::Result::invalid_request;
                    else if (request_ipc==rdr_control) result=gui::Result::not_supported;
                    else if (!client) result=gui::Result::not_supported;
                    else if (request.command == 1) {
                        auto found = std::find_if(observation.targets.begin(), observation.targets.end(),
                            [&](const Target& t) { return t.id == request.target_id; });
                        if (!observation.scanning || !observation.streaming) result=gui::Result::asset_unavailable;
                        else if (!current) result=gui::Result::status_pending;
                        else if (!ready) result=gui::Result::device_fault;
                        else if (flags&10) result=gui::Result::device_busy;
                        else if (found==observation.targets.end()) result=gui::Result::target_pending;
                        else if (target_id && (target_id!=request.target_id || assignment!=3 || demo_requested)) result=gui::Result::already_assigned;
                        else {
                            // Consume the shared filtered world point before GUI serialization.
                            const double east=found->east+installation.eff_position[1];
                            const double north=found->north-installation.eff_position[0];
                            const double up=found->up-installation.eff_position[2];
                            const auto angles=fcs_calibrated_angles(east,north,up,
                                installation.fcs_angle_offset_deg[0],installation.fcs_angle_offset_deg[1],installation.fcs_geometry);
                            commanded_pan=angles.azimuth;commanded_tilt=angles.elevation;
                            log("FCS point target="+std::to_string(request.target_id)+
                                " corrected_xyz="+std::to_string(found->east)+","+std::to_string(found->north)+","+std::to_string(found->up)+
                                " relative_xyz="+std::to_string(east)+","+std::to_string(north)+","+std::to_string(up)+
                                " pan="+std::to_string(commanded_pan)+" tilt="+std::to_string(commanded_tilt));
                            auto reply=client->point(commanded_tilt,commanded_pan);
                            result=command_result("EF03",reply);
                            if(result==gui::Result::success) { target_id=request.target_id; position_guard.start({found->east,found->north,found->up},rdr_state.targets.at(uint16_t(found->id)).seen); assigned_rdr_generation=observation.generation; health.start(); active=0; demo_requested=demo_seen=false; move_deadline=Clock::now()+std::chrono::seconds(60); next_poll={}; }
                        }
                    } else if (request.command == 2) {
                        // OFF/recovery is independent of target assignment.
                        auto reply=laser_off();
                        result=command_result("EF04_OFF",reply);
                        if(result==gui::Result::success&&reply->laser_state!=0)result=gui::Result::device_fault;
                        if(result==gui::Result::success) { manual=ManualSequence{};target_id=active=0; health.reset(); position_guard.reset(); demo_requested=demo_seen=false; last_assignment_cancel=Clock::now(); next_poll={}; }
                    } else if (request.command == 3) {
                        if (!target_id || target_id!=request.target_id) result=gui::Result::not_assigned;
                        else if (assignment!=2) result=health.reason!=gui::Result::success?health.reason:gui::Result::alignment_required;
                        else if (!current) result=gui::Result::status_pending;
                        else if (!ready) result=gui::Result::device_fault;
                        else if (!target_consistent) result=gui::Result::target_pending;
                        else if ((flags&10) || demo_requested) result=gui::Result::device_busy;
                        else {
                            if(!prepare_output()) result=output_block_result;
                            else {
                                auto reply=client->demo();
                                result=command_result("EF05",reply);
                                if(result==gui::Result::success) { active=0; demo_requested=true; demo_seen=false; next_poll={}; }
                                else if(!reply) { demo_requested=true; health.fail(gui::Result::response_timeout); }
                            }
                        }
                    } else result=gui::Result::not_supported;
                } catch (const std::invalid_argument& error) {
                    result=gui::Result::invalid_request; log(std::string("invalid_request ")+error.what());
                } catch (const std::exception& error) {
                    result=gui::Result::device_error; log(std::string("device_exception ")+error.what());
                }
                response.result=uint32_t(result); response.target_id=target_id;
                response.assignment=manual.active()?manual.state():assignment;
                response.active=manual.active()?unsigned(manual.phase==ManualSequence::Phase::fired):active;
                response.connected=state.connected;
                 if (request.command == uint32_t(gui::Command::get_fcs_busy))
                     response.active=result==gui::Result::success && (flags&10)!=0;
                if(request.command==5 || request.command==6) response.connected=real_rdr->snapshot().connected;
                // Retain legacy mapping for existing reasons. Updated GUI also
                // accepts additive optical-interlock diagnostics (15..20).
                if(original_gui && request_ipc==ipc) {
                    if(response.assignment==4) response.assignment=1;
                    if(request.command==0 && result!=gui::Result::invalid_request) response.result=0;
                    else if(request.command!=18 && !(request.command==1009 && result==gui::Result::target_pending) && response.result>=7 && response.result<15) response.result=
                        (result==gui::Result::response_timeout || result==gui::Result::device_rejected ||
                         result==gui::Result::device_fault)?6:2;
                }
                if (request.command==8 && result==gui::Result::success) {
                    const auto& values=request.target_id==2?installation.ranges:
                        request.target_id==3?installation.rdr_fov:installation.fcs_angle_offset_deg;
                    response.fcs_id=std::bit_cast<uint32_t>(float(values[0]));
                    response.target_id=std::bit_cast<uint32_t>(float(values[1]));
                    response.assignment=std::bit_cast<uint32_t>(float(values[2]));
                    response.active=0; response.connected=1;
                }
                if((request.command==1101 || request.command==1102) && result==gui::Result::success) {
                    response.fcs_id=std::bit_cast<uint32_t>(float(calibration_angles.azimuth));
                    response.target_id=std::bit_cast<uint32_t>(float(calibration_angles.elevation));
                    response.assignment=0; response.active=0;
                }
                if((request.command==1110 || request.command==1111) && result==gui::Result::success) {
                    response.fcs_id=std::bit_cast<uint32_t>(float(geometry_values[0]));
                    response.target_id=std::bit_cast<uint32_t>(float(geometry_values[1]));
                    response.assignment=std::bit_cast<uint32_t>(float(geometry_values[2]));
                    response.active=geometry_enabled;
                }
                c2_ipc_reply(request_ipc,&response,ticket);
                if(request.command && request.command!=8) log("GUI command="+std::to_string(request.command)+
                    " request_id="+std::to_string(request.request_id)+" result="+std::to_string(response.result)+
                    " internal_result="+std::to_string(unsigned(result))+" reason_name="+result_name(result)+
                    " requested_target="+std::to_string(request.target_id)+" target="+std::to_string(target_id)+
                    " assignment="+std::to_string(assignment)+" detail="+assignment_details(request.target_id));
                }
            }
            gui::Frame frame; frame.header.sequence=++frame_sequence; frame.header.timestamp_ms=unix_ms();
            auto& p=frame.payload; p.rdr_count=p.fcs_count=p.selected_rdr=p.selected_fcs=1;
            p.rdr.id=p.fcs.id=1; p.rdr.connected=p.rdr.enabled=observation.connected;
            // Publish actual installation positions shared by the map and pointing calculation.
            p.rdr.position_x=float(-installation.rdr_position[1]);
            p.rdr.position_y=float(installation.rdr_position[0]);
            p.rdr.position_z=float(installation.rdr_position[2]);
            p.fcs.position_x=float(-installation.eff_position[1]);
            p.fcs.position_y=float(installation.eff_position[0]);
            p.fcs.position_z=float(installation.eff_position[2]);
            p.fcs.connected=state.connected; p.fcs.enabled=ready; p.fcs.operating=ready;
            p.fcs.laser_on=current && (flags&4);
            p.fcs.azimuth=current && state.status->azimuth!=-32768 ?
                fcs_motor_to_gui_azimuth(state.status->azimuth) : 135;
            p.fcs.elevation=current && state.status->elevation!=-32768 ? float(state.status->elevation) : 0;
            if (real_control) {
                // A command above can start the pause in this same frame.
                display_targets=observation_retention.update(real_rdr->snapshot(),filtered_targets,
                    {-installation.rdr_position[1],installation.rdr_position[0],installation.rdr_position[2]},
                    c2::rdr::seconds(),lidar_interlock.phase()==LidarInterlock::Phase::idle,
                    manual.phase==ManualSequence::Phase::fired||manual.phase==ManualSequence::Phase::resuming||
                    manual.phase==ManualSequence::Phase::observing);
                real_control->populate(p,rdr_state,hit_retention.merge(display_targets));
                if (rdr_reply!=RdrReply::none) {
                    const auto reply=rdr_json(rdr_state,corrected_targets,filtered_targets);
                    std::cout<<(rdr_reply==RdrReply::targets?reply.at("targets"):reply).dump()<<std::endl;
                }
                if(recorder) {
                    recorder->append(rdr_state,
                        {-installation.rdr_position[1],installation.rdr_position[0],installation.rdr_position[2]},
                        filtered_targets,real_control->operating(rdr_state),c2::rdr::seconds(),unix_ms());
                    if(recorder->finished() && !recording_finished_logged) {
                        log("RDR coordinate recording finished: "+rdr_record_file.string());
                        recording_finished_logged=true;
                    }
                }
                // Legacy GUI does not recognize unknown-angle sentinels.
                if(p.rdr.azimuth==-32768)p.rdr.azimuth=135;
                if(p.rdr.elevation==-32768)p.rdr.elevation=0;
                if (!client) { p.fcs={};p.fcs_count=p.selected_fcs=0; }
            }
            // Publish the saved display FOV after device population so both IPC
            // readers use the same setting. This does not change sensor operation.
            p.rdr.field_of_view = static_cast<float>(installation.rdr_fov[0]);
            if (!c2_ipc_publish(ipc,&frame)) throw std::runtime_error("GUI frame publish failed");
            if (Clock::now()>=next_log) {
                std::cout << "FCS connected=" << state.connected << " ready=" << ready << " flags=" << flags
                          << " RDR connected=" << observation.connected << " targets=" << p.target_count << '\n' << std::flush;
                if (!status_file.empty()) {
                    std::ofstream out(status_file);
                    const auto real_state=real_rdr?real_rdr->snapshot():c2::rdr::State{};
                    out << "{\"runtime\":\"C2.Console\",\"rdr_mode\":\"" << (real_rdr?"real24":"draft")
                        << "\",\"fcs_enabled\":" << (client?"true":"false")
                        << ",\"connected\":" << (observation.connected?"true":"false")
                        << ",\"mode\":" << real_state.mode << ",\"reporting\":" << (observation.streaming?"true":"false")
                        << ",\"result\":" << real_state.last_result
                        << ",\"frame\":\"" << (real_rdr?"SENSOR_RAW":"bench") << "\""
                        << ",\"fcs_connected\":" << (state.connected?"true":"false")
                        << ",\"fcs_ready\":" << (ready?"true":"false") << ",\"flags\":" << flags
                        << ",\"rdr_connected\":" << (observation.connected?"true":"false")
                        << ",\"rdr_scanning\":" << (observation.scanning?"true":"false")
                        << ",\"targets\":" << p.target_count << ",\"assignment\":" << assignment
                        << ",\"assignment_reason\":" << unsigned(health.reason) << ",\"assigned_target\":" << target_id
                        << ",\"display_positions\":{\"rdr\":[" << installation.rdr_position[0] << ',' << installation.rdr_position[1] << ',' << installation.rdr_position[2]
                        << "],\"eff\":[" << installation.eff_position[0] << ',' << installation.eff_position[1] << ',' << installation.eff_position[2] << "]}"
                        << ",\"network\":" << network.dump()
                        << ",\"gui_kalman\":" << position_filter_parameters.dump()
                        << ",\"rdr_received_datagrams\":" << real_rdr->transport_stats().received_datagrams
                        << ",\"timestamp_ms\":" << unix_ms() << "}";
                }
                next_log=Clock::now()+std::chrono::seconds(2);
            }
            const auto loop_wakeup = Clock::now()+std::chrono::milliseconds(50);
            std::this_thread::sleep_until(client ? std::min(loop_wakeup,next_poll) : loop_wakeup);
        }
        if (real_rdr && real_rdr->snapshot().connected) {
            real_rdr->command(0xED06,0);
            const auto deadline=c2::rdr::seconds()+6;
            while (c2::rdr::seconds()<deadline) {
                real_rdr->tick();const auto state=real_rdr->snapshot();
                if (!state.connected || state.mode==0) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
        if(client && client->snapshot().connected) { try { (void)client->laser(false); } catch(...) {} }
        log("runtime_stop graceful=1");
        c2_ipc_destroy(rdr_control); rdr_control=nullptr;
        c2_ipc_destroy(ipc); ipc=nullptr; if (client) client->stop();
        return 0;
    } catch(const std::exception& error) {
        if (rdr_control) c2_ipc_destroy(rdr_control);
        if (ipc) c2_ipc_destroy(ipc);
        if(events) events<<unix_ms()<<" runtime_error "<<error.what()<<std::endl;
        std::cerr << "Live station error: " << error.what() << '\n'; return 1;
    }
}
