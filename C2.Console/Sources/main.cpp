#include "c2/fcs_client.hpp"
#include "c2/rdr_client.hpp"
#include "rdr_control.hpp"
#include "rdr_live_diagnostics.hpp"
#include "transport.hpp"
#include "assignment_health.hpp"
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
static nlohmann::json rdr_json(const c2::rdr::State& state, c2::rdr::BasePoint origin,
                               c2::rdr::WorldTargetStore& world_targets) {
    nlohmann::json targets=nlohmann::json::array();
    const auto corrected=world_targets.update(state,origin);
    const auto displayed=c2::rdr::correct_targets(state,origin);
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
    try {
        fcs::ClientConfig config;
        config.peer = {"192.168.0.103", 2000}; config.local = {"192.168.0.102", 2000};
        config.response_timeout = std::chrono::milliseconds(200);
        std::filesystem::path stop_file, status_file, installation_file;
        InstallationConfig installation;
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
            else if (option == "--config") {
                std::ifstream file(next()); if(!file) throw std::invalid_argument("configuration missing");
                nlohmann::json json; file>>json;
                const auto& radar=json.at("rdr"); const auto& fcs_config=json.at("fcs");
                if(radar.at("protocol")!="edde24" || fcs_config.at("protocol")!="effe") throw std::invalid_argument("unsupported binary ICD");
                rdr_local_ip=radar.at("local").at("ip"); rdr_local_port=parse_port(radar.at("local").at("port").dump(),true);
                rdr_ip=radar.at("peer").at("ip"); rdr_port=parse_port(radar.at("peer").at("port").dump());
                config.local.bind_address=fcs_config.at("local").at("ip"); config.local.listen_port=parse_port(fcs_config.at("local").at("port").dump(),true);
                config.peer.address=fcs_config.at("peer").at("ip"); config.peer.port=parse_port(fcs_config.at("peer").at("port").dump());
                rdr_only=!fcs_config.value("enabled",true);
            }
            else if (option == "--rdr-port" || option == "--rdr-real-port") rdr_port=parse_port(next());
            else if (option == "--rdr-local-port" || option == "--rdr-real-local-port") rdr_local_port=parse_port(next());
            else if (option == "--ipc-suffix") { auto s=next(); ipc_suffix.assign(s.begin(),s.end()); }
            else if (option == "--gui-extended") original_gui=false;
            else throw std::invalid_argument("unknown console option");
        }
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
        std::filesystem::file_time_type installation_seen{};
        if (!installation_file.empty()) {
            installation_file = std::filesystem::absolute(installation_file);
            installation = load_installation_config(installation_file);
            installation_seen = std::filesystem::last_write_time(installation_file);
        }
        std::signal(SIGINT, quit); std::signal(SIGTERM, quit);
        std::unique_ptr<fcs::Client> client;

        std::unique_ptr<c2::rdr::Client> real_rdr;
        if (!rdr_only) { client=std::make_unique<fcs::Client>(config); client->start(); }
        real_rdr=std::make_unique<c2::rdr::Client>(c2::UdpConfig{rdr_local_ip,uint16_t(rdr_local_port)},
                c2::Endpoint{rdr_ip,uint16_t(rdr_port)});
        const auto network=nlohmann::json{
            {"fcs_local",endpoint_json(client?client->local_endpoint():c2::Endpoint{})},
            {"fcs_peer",endpoint_json(config.peer)},
            {"rdr_local",endpoint_json(real_rdr->local_endpoint())},
            {"rdr_peer",endpoint_json({rdr_ip,uint16_t(rdr_port)})}};
        ipc = c2_ipc_create((std::wstring(gui::mapping_name)+ipc_suffix).c_str(),
            (std::wstring(gui::mutex_name)+ipc_suffix).c_str(), (std::wstring(gui::pipe_path)+ipc_suffix).c_str());
        if (!ipc) throw std::runtime_error(c2_ipc_error());
        rdr_control=c2_ipc_create_commands((std::wstring(L"\\\\.\\pipe\\ECS_RdrControl")+ipc_suffix).c_str());
        if (!rdr_control) throw std::runtime_error(c2_ipc_error());
        std::cout << "LIVE ECS / configured FCS and RDR peers / manual Big Endian codecs\n"
                  << "Network: " << network.dump() << '\n'
                  << "GUI FCS-1 = " << config.peer.address << ':' << config.peer.port << '\n'
                  << "Installation display config: " << (installation_file.empty()?"common origin defaults":installation_file.string()) << '\n'
                  << "Position edits update map and FCS assignment coordinates; reassign after edits.\n"
                  << "Assign sends EF03; Fire sends EF05 built-in demo; Cancel sends EF04 OFF.\n"
                  << "FCS and RDR connect automatically; RDR report/start require commands.\n"
                  << "RDR controls: rdr connect / rdr report on / rdr start / rdr stop\n"
                  << "Read-only world XYZ monitor: rdr live on / rdr live off (200 ms)\n"
                  << "Ctrl+C closes this station.\n" << std::flush;
        std::ofstream events;
        if (!status_file.empty()) events.open(status_file.parent_path()/"events.log",std::ios::app);
        auto log=[&](const std::string& message) {
            auto line=std::to_string(unix_ms())+" "+message;
            std::cout<<line<<std::endl;
            if(events) events<<line<<std::endl;
        };
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
        uint32_t target_id{}, fire{}, frame_sequence{};
        uint64_t assigned_rdr_generation{};
        AssignmentHealth health; auto& assignment=health.state;
        unsigned logged_state=0, logged_reason=0;
        double commanded_pan{}, commanded_tilt{};
        Target assigned_target{};
        bool demo_requested{}, demo_seen{};
        auto invalidate_installation_alignment=[&]() {
            if (!target_id) return;
            health.fail(gui::Result::target_changed);
            fire=0;demo_requested=demo_seen=false;
            log("Installation positions changed: existing alignment invalid; reassign required");
        };
        Clock::time_point next_poll{}, next_log{}, move_deadline{}, next_installation_check{}, next_rdr_retry{};
        bool installation_missing{};
        ConsoleInput console(headless);
        c2::rdr::WorldTargetStore world_targets;
        bool rdr_live{};
        Clock::time_point next_rdr_live{};
        std::cout << nlohmann::json{{"ready",true},{"runtime","C++"},{"network",network},{"rdr_protocol","edde24"},{"fcs_protocol","effe"}}.dump()<<std::endl;
        while (!quitting && (stop_file.empty() || !std::filesystem::exists(stop_file))) {
            if(auto line=console.next()) {
                try {
                    std::istringstream input(*line);std::string device,command,arg;input>>device>>command;
                    std::istringstream validation(*line);
                    const std::vector<std::string> words{std::istream_iterator<std::string>{validation}, {}};
                    size_t expected=0;
                    if(device=="quit" || device=="exit")expected=1;
                    else if(device=="rdr") {
                        if(command=="connect"||command=="status"||command=="targets"||command=="start"||command=="stop")expected=2;
                        else if(command=="report"||command=="exclude"||command=="resume"||command=="live")expected=3;
                    } else if(device=="fcs") {
                        if(command=="connect"||command=="status")expected=2;
                        else if(command=="point")expected=4;
                        else if(command=="laser")expected=3;
                    }
                    if(!expected || words.size()!=expected)throw std::invalid_argument("invalid command or argument count");
                    if(device=="quit" || device=="exit")break;
                    if(device=="rdr" && command=="live") {
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
                        auto state=rdr_json(real_rdr->snapshot(),
                            {-installation.rdr_position[1],installation.rdr_position[0],installation.rdr_position[2]},world_targets);
                        std::cout<<(command=="targets"?state["targets"]:state).dump()<<std::endl;
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
                        else if(command=="laser") {input>>arg;if(arg!="on"&&arg!="off")throw std::invalid_argument("laser on|off");response=client->laser(arg=="on");laser_deadline=arg=="on"?Clock::now()+std::chrono::seconds(1):Clock::time_point{};}
                        else throw std::invalid_argument("unknown FCS command");
                        auto snapshot=client->snapshot();
                        auto report=nlohmann::json{{"fcs",command},{"confirmed",bool(response)},{"result",response?int(response->result):-1},{"connected",snapshot.connected}};
                        if(snapshot.status) {report["elevation"]=snapshot.status->elevation;report["azimuth"]=snapshot.status->azimuth;report["flags"]=snapshot.status->flags;}
                        std::cout<<report.dump()<<std::endl;
                    } else throw std::invalid_argument("unknown device command");
                } catch(const std::exception& e) {std::cout<<nlohmann::json{{"rejected",e.what()}}.dump()<<std::endl;}
            }
            if(!headless&&console.ended())break;
            if(client&&laser_deadline!=Clock::time_point{}&&Clock::now()>=laser_deadline) {(void)client->laser(false);laser_deadline=Clock::time_point{};}
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
                        auto updated = load_installation_config(installation_file);
                        if(updated.rdr_position!=installation.rdr_position || updated.eff_position!=installation.eff_position)
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
                bool connected=client->snapshot().connected;
                try {
                    auto reply=connected?client->query():client->connect();
                    if(!reply) log(std::string("FCS ")+(connected?"EF02":"EF01")+" response_timeout");
                } catch (const std::exception& error) {
                    log(std::string("FCS network retry: ")+error.what());
                }
                next_poll = Clock::now()+std::chrono::seconds(1);
            }
            auto state = client?client->snapshot():fcs::Snapshot{};
            const auto rdr_state=real_rdr->snapshot();
            // Installation storage is (forward,left,up); corrected coordinates
            // and GUI XYZ are (east,north,up). Rotate raw DE03 coordinates once.
            const c2::rdr::BasePoint rdr_origin{-installation.rdr_position[1],installation.rdr_position[0],installation.rdr_position[2]};
            if (rdr_live && Clock::now()>=next_rdr_live) {
                std::cout<<c2::rdr::live_diagnostics(rdr_state,rdr_origin,unix_ms()).dump()<<std::endl;
                next_rdr_live=Clock::now()+std::chrono::milliseconds(200);
            }
            auto corrected_targets=world_targets.update(rdr_state,rdr_origin);
            auto observation = observation_snapshot(rdr_state,corrected_targets);
            bool current = state.connected && state.status &&
                Clock::now()-state.status_received_at < std::chrono::seconds(2);
            uint16_t flags = current ? state.status->flags : 0;
            bool ready = current && !(flags&16) && state.status->azimuth != -32768 && state.status->elevation != -32768;
            auto assigned_now = std::find_if(observation.targets.begin(), observation.targets.end(),
                [&](const Target& t) { return t.id == target_id; });
            bool target_current = target_id && assigned_now != observation.targets.end() &&
                std::abs(assigned_now->east-assigned_target.east)<.01 &&
                std::abs(assigned_now->north-assigned_target.north)<.01 &&
                std::abs(assigned_now->up-assigned_target.up)<.01;
            bool aligned=current && std::abs(state.status->azimuth-commanded_pan)<=1 &&
                std::abs(state.status->elevation-commanded_tilt)<=1;
            if (real_rdr && target_id && observation.generation!=assigned_rdr_generation)
                health.fail(gui::Result::target_changed);
            health.update(current,ready,assigned_now!=observation.targets.end(),target_current,
                aligned,(flags&2)!=0,demo_requested,Clock::now()>move_deadline);
            if(logged_state!=assignment || logged_reason!=unsigned(health.reason)) {
                log("assignment="+std::to_string(assignment)+" reason="+std::to_string(unsigned(health.reason))+
                    " target="+std::to_string(target_id)+" flags="+std::to_string(flags)+
                    " status_age_ms="+std::to_string(state.status?std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now()-state.status_received_at).count():-1)+
                    " target_present="+std::to_string(assigned_now!=observation.targets.end())+
                    " target_unchanged="+std::to_string(target_current)+" aligned="+std::to_string(aligned));
                logged_state=assignment; logged_reason=unsigned(health.reason);
            }
            if (demo_requested && current) {
                if (flags&8) demo_seen=true;
                if (demo_seen && ready && assignment==2 && !(flags&14)) { fire=1; demo_requested=false; }
            }
            gui::PipeCommand request; uint64_t ticket;
            void* request_ipc=ipc;
            bool has_request=c2_ipc_poll(ipc,&request,&ticket)!=0;
            if (!has_request) { request_ipc=rdr_control; has_request=c2_ipc_poll(rdr_control,&request,&ticket)!=0; }
            if (has_request) {
                gui::PipeResponse response;
                response.request_id = request.request_id; response.command = request.command; response.fcs_id=1;
                if (real_control && request.command>=10) {
                    response=real_control->handle(request,c2::rdr::seconds());
                    c2_ipc_reply(request_ipc,&response,ticket);
                } else {
                auto result = gui::Result::success;
                try {
                    if (!request.request_id) result=gui::Result::invalid_request;
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
                            if (position!=updated) {
                                invalidate_installation_alignment();
                                position=updated;
                                if (rdr_position) {
                                    corrected_targets=world_targets.update(rdr_state,
                                        {-position[1],position[0],position[2]});
                                    observation=observation_snapshot(rdr_state,corrected_targets);
                                }
                            }
                            log(std::string("GUI position updated: ")+(rdr_position?"RDR":"FCS")+
                                " XYZ="+std::to_string(request.position_x)+","+
                                std::to_string(request.position_y)+","+std::to_string(request.position_z));
                        }
                    }
                    else if (request.command == 6 && real_control) {
                        result=static_cast<gui::Result>(real_control->set_operating(request).result);
                    }
                    else if (request.fcs_id != 1 || request.rdr_id != 1) result=gui::Result::invalid_request;
                    else if (request.command == 0) { result=health.reason; }
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
                            // Use the same corrected installation point published to the GUI.
                            const double east=found->east+installation.eff_position[1];
                            const double north=found->north-installation.eff_position[0];
                            const double up=found->up-installation.eff_position[2];
                            const auto angles=fcs_point_angles(east,north,up);
                            commanded_pan=angles.azimuth;commanded_tilt=angles.elevation;
                            log("FCS point target="+std::to_string(request.target_id)+
                                " corrected_xyz="+std::to_string(found->east)+","+std::to_string(found->north)+","+std::to_string(found->up)+
                                " relative_xyz="+std::to_string(east)+","+std::to_string(north)+","+std::to_string(up)+
                                " pan="+std::to_string(commanded_pan)+" tilt="+std::to_string(commanded_tilt));
                            auto reply=client->point(commanded_tilt,commanded_pan);
                            result=command_result("EF03",reply);
                            if(result==gui::Result::success) { target_id=request.target_id; assigned_target=*found; assigned_rdr_generation=observation.generation; health.start(); fire=0; demo_requested=demo_seen=false; move_deadline=Clock::now()+std::chrono::seconds(60); next_poll={}; }
                        }
                    } else if (request.command == 2) {
                        if (!target_id) result=gui::Result::not_assigned;
                        else {
                            auto reply=client->laser(false);
                            result=command_result("EF04_OFF",reply);
                            if(result==gui::Result::success) { target_id=fire=0; health.reset(); demo_requested=demo_seen=false; next_poll={}; }
                        }
                    } else if (request.command == 3) {
                        if (!target_id || target_id!=request.target_id) result=gui::Result::not_assigned;
                        else if (assignment!=2) result=health.reason!=gui::Result::success?health.reason:gui::Result::alignment_required;
                        else if (!current) result=gui::Result::status_pending;
                        else if (!ready) result=gui::Result::device_fault;
                        else if (!target_current) result=gui::Result::target_pending;
                        else if ((flags&10) || demo_requested) result=gui::Result::device_busy;
                        else {
                            auto reply=client->demo();
                            result=command_result("EF05",reply);
                            if(result==gui::Result::success) { fire=0; demo_requested=true; demo_seen=false; next_poll={}; }
                            else if(!reply) { demo_requested=true; health.fail(gui::Result::response_timeout); }
                        }
                    } else result=gui::Result::not_supported;
                } catch (const std::invalid_argument& error) {
                    result=gui::Result::invalid_request; log(std::string("invalid_request ")+error.what());
                } catch (const std::exception& error) {
                    result=gui::Result::device_error; log(std::string("device_exception ")+error.what());
                }
                response.result=uint32_t(result); response.target_id=target_id;
                response.assignment=assignment; response.fire=fire; response.connected=state.connected;
                if(request.command==5 || request.command==6) response.connected=real_rdr->snapshot().connected;
                // Upstream GUI accepts result 0..6 and assignment 0..3 only.
                // Keep detailed reasons internally and in events.log.
                if(original_gui && request_ipc==ipc) {
                    if(response.assignment==4) response.assignment=1;
                    if(request.command==0 && result!=gui::Result::invalid_request) response.result=0;
                    else if(response.result>=7) response.result=
                        (result==gui::Result::response_timeout || result==gui::Result::device_rejected ||
                         result==gui::Result::device_fault)?6:2;
                }
                c2_ipc_reply(request_ipc,&response,ticket);
                if(request.command) log("GUI command="+std::to_string(request.command)+" result="+std::to_string(response.result)+" target="+std::to_string(target_id));
                }
            }
            gui::Frame frame; frame.header.sequence=++frame_sequence; frame.header.timestamp_ms=unix_ms();
            auto& p=frame.payload; p.rdr_count=p.fcs_count=p.selected_rdr=p.selected_fcs=1;
            p.rdr.id=p.fcs.id=1; p.rdr.connected=p.rdr.enabled=observation.connected;
            // Use the same installation positions as the FCS assignment calculation.
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
                // Display the latest observation for each ID, without the held-position cache.
                // Read installation positions here so same-frame position edits are reflected.
                const auto display_targets=c2::rdr::correct_targets(rdr_state,
                    {-installation.rdr_position[1],installation.rdr_position[0],installation.rdr_position[2]});
                real_control->populate(p,rdr_state,display_targets);
                // Legacy GUI does not recognize unknown-angle sentinels.
                if(p.rdr.azimuth==-32768)p.rdr.azimuth=135;
                if(p.rdr.elevation==-32768)p.rdr.elevation=0;
                if (!client) { p.fcs={};p.fcs_count=p.selected_fcs=0; }
            }
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
                        << ",\"rdr_received_datagrams\":" << real_rdr->transport_stats().received_datagrams
                        << ",\"timestamp_ms\":" << unix_ms() << "}";
                }
                next_log=Clock::now()+std::chrono::seconds(2);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
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
        c2_ipc_destroy(rdr_control); rdr_control=nullptr;
        c2_ipc_destroy(ipc); ipc=nullptr; if (client) client->stop();
        return 0;
    } catch(const std::exception& error) {
        if (rdr_control) c2_ipc_destroy(rdr_control);
        if (ipc) c2_ipc_destroy(ipc);
        std::cerr << "Live station error: " << error.what() << '\n'; return 1;
    }
}
