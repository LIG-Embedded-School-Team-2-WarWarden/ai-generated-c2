#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include "live_host.hpp"
#include <sstream>
#include <memory>
#include <vector>

namespace {
const char* connection(c2::AssetConnectionState state) {
    switch (state) {
    case c2::AssetConnectionState::connected: return "CONNECTED";
    case c2::AssetConnectionState::awaiting_heartbeat: return "AWAITING HEARTBEAT";
    case c2::AssetConnectionState::disconnected: return "DISCONNECTED";
    case c2::AssetConnectionState::lease_expired: return "LEASE EXPIRED";
    case c2::AssetConnectionState::unregistered: return "UNREGISTERED";
    }
    return "UNKNOWN";
}
const char* outcome(c2::CommandTerminalState state) {
    switch (state) {
    case c2::CommandTerminalState::completed: return "COMPLETED";
    case c2::CommandTerminalState::rejected: return "REJECTED";
    case c2::CommandTerminalState::failed: return "FAILED";
    case c2::CommandTerminalState::expired: return "EXPIRED";
    case c2::CommandTerminalState::delivery_exhausted: return "DELIVERY EXHAUSTED";
    case c2::CommandTerminalState::completion_timeout: return "COMPLETION TIMEOUT";
    case c2::CommandTerminalState::session_ended: return "SESSION ENDED";
    case c2::CommandTerminalState::cancelled_before_send: return "CANCELLED";
    }
    return "UNKNOWN";
}
class Dashboard {
    ai_c2::LiveHost host_;
    HWND assets_{}, tracks_{}, history_{}, status_{}, metrics_{};
    HFONT font_{};
    std::vector<std::uint64_t> asset_ids_, track_ids_;
    std::string previous_history_;
    HWND control(HWND window, const char* type, const char* text, DWORD style,
                 int x, int y, int w, int h, int id = 0) {
        auto handle = CreateWindowExA(0, type, text, WS_CHILD | WS_VISIBLE | style,
            x, y, w, h, window, reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),
            GetModuleHandleA(nullptr), nullptr);
        SendMessageA(handle, WM_SETFONT, reinterpret_cast<WPARAM>(font_), TRUE);
        return handle;
    }
    static int selected(HWND list) { return static_cast<int>(SendMessageA(list, LB_GETCURSEL, 0, 0)); }
    static void row(HWND list, const std::string& value) {
        SendMessageA(list, LB_ADDSTRING, 0, reinterpret_cast<LPARAM>(value.c_str()));
    }
    template<class Result> void dispatch(const Result& result) {
        if (auto error = std::get_if<c2::DispatchError>(&result)) {
            const char* message = "Rejected: invalid command or state";
            if (*error == c2::DispatchError::connection_unavailable) message = "Rejected: connection unavailable";
            if (*error == c2::DispatchError::pose_resynchronization_required) message = "Rejected: pose resynchronization required";
            if (*error == c2::DispatchError::delivery_uncertain) message = "Delivery uncertain - check pending commands and outcomes";
            SetWindowTextA(status_, message);
        } else SetWindowTextA(status_, "Sent - waiting for device ACK. See command outcomes below.");
    }
public:
    explicit Dashboard(c2::ServerOptions options) : host_(options) {}
    ~Dashboard() { if (font_) DeleteObject(font_); }
    void build(HWND window) {
        font_ = CreateFontA(-16, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
            OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH, "Segoe UI");
        control(window, "STATIC", "WARWARDEN / AI C2 / LIVE UDP + PROTOBUF v3", 0, 24, 20, 920, 30);
        control(window, "STATIC", "Real asset connection / no simulated data / command results require ACK",
            0, 24, 56, 920, 26);
        metrics_ = control(window, "STATIC", "Waiting for registration...", 0, 24, 90, 920, 30);
        control(window, "STATIC", "Assets: ID / role / connection / command endpoint / pose", 0, 24, 137, 930, 24);
        assets_ = control(window, "LISTBOX", "", WS_BORDER | WS_VSCROLL | WS_HSCROLL | LBS_NOTIFY | WS_TABSTOP,
            24, 165, 930, 135, 101);
        control(window, "STATIC", "Tracks: global ID / observer ID / project position (metres)", 0, 24, 315, 930, 24);
        tracks_ = control(window, "LISTBOX", "", WS_BORDER | WS_VSCROLL | LBS_NOTIFY | WS_TABSTOP,
            24, 345, 930, 100, 102);
        control(window, "BUTTON", "Assign + point", WS_TABSTOP, 24, 465, 170, 38, 110);
        control(window, "BUTTON", "Stop selected", WS_TABSTOP, 208, 465, 170, 38, 111);
        control(window, "BUTTON", "ESTOP all", WS_TABSTOP, 392, 465, 170, 38, 112);
        control(window, "BUTTON", "Scan observer", WS_TABSTOP, 576, 465, 170, 38, 113);
        control(window, "BUTTON", "Stop observer", WS_TABSTOP, 760, 465, 194, 38, 114);
        status_ = control(window, "STATIC", "Ready. Register assets on the configured UDP asset port.",
            0, 24, 520, 930, 42);
        control(window, "STATIC", "Assignments / pending delivery / ACK outcomes / rejected packets", 0, 24, 575, 930, 25);
        history_ = control(window, "EDIT", "", WS_BORDER | WS_VSCROLL | ES_MULTILINE | ES_READONLY,
            24, 605, 930, 180, 103);
        SetTimer(window, 1, 250, nullptr); refresh();
    }
    void refresh() {
        auto& runtime = host_.runtime(); const auto now = ai_c2::now_us();
        const auto old_a = selected(assets_), old_t = selected(tracks_);
        const auto a_id = old_a >= 0 ? asset_ids_.at(old_a) : 0;
        const auto t_id = old_t >= 0 ? track_ids_.at(old_t) : 0;
        auto assets = runtime.assets(now); auto tracks = runtime.tracks(now);
        SendMessageA(assets_, WM_SETREDRAW, FALSE, 0); SendMessageA(tracks_, WM_SETREDRAW, FALSE, 0);
        SendMessageA(assets_, LB_RESETCONTENT, 0, 0); SendMessageA(tracks_, LB_RESETCONTENT, 0, 0);
        asset_ids_.clear(); track_ids_.clear();
        for (const auto& a : assets) {
            std::ostringstream line;
            line << a.asset_id << " / " << (a.role == c2::AssetRole::observation ? "OBSERVER" : "EFFECTOR")
                 << " / " << connection(a.connection_state) << " / " << a.command_endpoint.address
                 << ':' << a.command_endpoint.port << " / pose=" << (a.pose_synchronized ? "OK" : "REQUIRED")
                 << " / status=" << (a.status_current ? "CURRENT" : "STALE") << " / session=" << a.session_id;
            row(assets_, line.str()); asset_ids_.push_back(a.asset_id);
            if (a.asset_id == a_id) SendMessageA(assets_, LB_SETCURSEL, asset_ids_.size() - 1, 0);
        }
        for (const auto& t : tracks) {
            std::ostringstream line;
            line << t.track_id << " / observer " << t.observation_asset_id << " / ("
                 << t.measurement.x_m << ", " << t.measurement.y_m << ", " << t.measurement.z_m << ')';
            row(tracks_, line.str()); track_ids_.push_back(t.track_id);
            if (t.track_id == t_id) SendMessageA(tracks_, LB_SETCURSEL, track_ids_.size() - 1, 0);
        }
        SendMessageA(assets_, WM_SETREDRAW, TRUE, 0); SendMessageA(tracks_, WM_SETREDRAW, TRUE, 0);
        InvalidateRect(assets_, nullptr, TRUE); InvalidateRect(tracks_, nullptr, TRUE);
        const auto stats = host_.stats(); std::ostringstream metrics;
        metrics << "Assets " << assets.size() << " | Tracks " << tracks.size()
                << " | RX packets " << stats.received_datagrams << " | Pending ACK " << runtime.pending_command_count()
                << " | Worker errors " << host_.worker_errors();
        SetWindowTextA(metrics_, metrics.str().c_str());
        std::ostringstream history;
        for (const auto& a : runtime.assignments()) history << "ASSIGN track=" << a.track_id
            << " asset=" << a.effector_asset_id << " state=" << static_cast<int>(a.state) << "\r\n";
        for (const auto& p : runtime.pending_commands()) history << "PENDING asset=" << p.key.asset_id
            << " command=" << p.key.command_id << " attempts=" << p.attempts << "\r\n";
        auto outcomes = runtime.command_outcomes();
        const auto start = outcomes.size() > 30 ? outcomes.size() - 30 : 0;
        for (auto i = start; i < outcomes.size(); ++i) { const auto& o = outcomes[i];
            history << "ACK asset=" << o.key.asset_id << " command=" << o.key.command_id << ' ' << outcome(o.state) << "\r\n";
        }
        auto rejected = runtime.inbound_rejections();
        if (!rejected.empty()) { const auto& r = rejected.back(); history << "REJECTED packet asset=" << r.asset_id
            << " category=" << static_cast<int>(r.category) << " reason=" << static_cast<int>(r.reason) << "\r\n"; }
        auto text = history.str();
        if (text != previous_history_) { SetWindowTextA(history_, text.c_str()); previous_history_ = std::move(text); }
    }
    void command(int id) {
        auto& runtime = host_.runtime(); const auto now = ai_c2::now_us();
        const auto a = selected(assets_), t = selected(tracks_);
        if (id == 112) {
            const auto result = runtime.emergency_stop_all(now);
            const auto text = "ESTOP sent to " + std::to_string(result.assets) + " known effectors; wait for ACK";
            SetWindowTextA(status_, text.c_str()); refresh(); return;
        }
        if (id < 110 || id > 114) return;
        if (a < 0) { SetWindowTextA(status_, "Select an asset first."); return; }
        const auto asset = asset_ids_.at(a);
        if (id == 110) {
            if (t < 0) { SetWindowTextA(status_, "Select a track first."); return; }
            const auto result = runtime.assign(track_ids_.at(t), asset, now);
            if (result.result == c2::AssignmentResult::assigned)
                dispatch(runtime.point_effector(track_ids_.at(t), now));
            else SetWindowTextA(status_, "Assignment rejected: check role, pose, state, limits and connection");
        } else if (id == 111) dispatch(runtime.stop_effector(asset, now));
        else if (id == 113) dispatch(runtime.command_observation(asset, c2::ObservationTurretCommandType::scan, 0, 0, now));
        else if (id == 114) dispatch(runtime.command_observation(asset, c2::ObservationTurretCommandType::stop, 0, 0, now));
        refresh();
    }
};
std::unique_ptr<Dashboard> dashboard;
LRESULT CALLBACK window_proc(HWND window, UINT message, WPARAM wparam, LPARAM lparam) {
    try {
        if (message == WM_CREATE) { dashboard->build(window); return 0; }
        if (message == WM_TIMER) { dashboard->refresh(); return 0; }
        if (message == WM_COMMAND) { if (HIWORD(wparam) == BN_CLICKED) dashboard->command(LOWORD(wparam)); return 0; }
        if (message == WM_DESTROY) { KillTimer(window, 1); PostQuitMessage(0); return 0; }
    } catch (const std::exception& e) { MessageBoxA(window, e.what(), "C2 runtime error", MB_ICONERROR); }
    return DefWindowProcA(window, message, wparam, lparam);
}
}
int WINAPI WinMain(HINSTANCE instance, HINSTANCE, LPSTR command_line, int show) {
    try {
        std::istringstream input(command_line); std::vector<std::string> tokens;
        for (std::string token; input >> token;) tokens.push_back(token);
        std::vector<std::string_view> arguments; for (const auto& t : tokens) arguments.push_back(t);
        const auto options = c2::parse_server_options(arguments);
        dashboard = std::make_unique<Dashboard>(options);
        SetProcessDPIAware(); WNDCLASSA cls{};
        cls.lpfnWndProc = window_proc; cls.hInstance = instance; cls.lpszClassName = "AiC2LiveDashboard";
        cls.hCursor = LoadCursor(nullptr, IDC_ARROW); cls.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
        if (!RegisterClassA(&cls)) throw std::runtime_error("Cannot register window class");
        RECT rect{0, 0, 980, 810}; constexpr DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
        AdjustWindowRect(&rect, style, FALSE);
        const auto title = "WarWarden AI C2 / LIVE / " + options.bind_address + ':' + std::to_string(options.asset_port);
        auto window = CreateWindowA(cls.lpszClassName, title.c_str(), style, CW_USEDEFAULT, CW_USEDEFAULT,
            rect.right - rect.left, rect.bottom - rect.top, nullptr, nullptr, instance, nullptr);
        if (!window) throw std::runtime_error("Cannot create dashboard window");
        ShowWindow(window, show); UpdateWindow(window); MSG message{};
        while (GetMessageA(&message, nullptr, 0, 0) > 0) {
            if (!IsDialogMessageA(window, &message)) { TranslateMessage(&message); DispatchMessageA(&message); }
        }
        dashboard.reset(); return static_cast<int>(message.wParam);
    } catch (const std::exception& e) {
        MessageBoxA(nullptr, e.what(), "Cannot start live C2", MB_ICONERROR); dashboard.reset(); return 1;
    }
}
