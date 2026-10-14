#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include "c2/demo_adapters.hpp"
#include <chrono>
#include <sstream>
#include <memory>

namespace {
std::uint64_t now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}
struct Dashboard {
    c2::DemoLidarProcessor lidar;
    c2::DemoPointingSolver solver;
    std::ostringstream output;
    c2::ConsoleTransport transport{output};
    c2::ControlCenter center{lidar, solver, transport};
    HWND assets{}, targets{}, history{}, status{}, simulation{};
    HFONT font{};
    std::vector<std::uint64_t> asset_ids, track_ids;
    std::uint64_t last_sample{}, displayed_event{};
    Dashboard() {
        auto now = now_us();
        center.register_asset({201, {}, 0, true, now});
        center.register_asset({202, {5, 0, 0}, 90, true, now});
    }
    ~Dashboard() { if (font) DeleteObject(font); }
    HWND control(HWND parent, const char* type, const char* text, DWORD style,
                 int x, int y, int w, int h, int id = 0) {
        auto handle = CreateWindowExA(0, type, text, WS_CHILD | WS_VISIBLE | style,
            x, y, w, h, parent, reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),
            GetModuleHandleA(nullptr), nullptr);
        SendMessageA(handle, WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);
        return handle;
    }
    void build(HWND window) {
        font = CreateFontA(-17, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
            DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
            DEFAULT_PITCH, "Segoe UI");
        control(window, "STATIC", "WARWARDEN / AI C2", 0, 24, 20, 400, 30);
        control(window, "STATIC", "SIMULATION ONLY - no hardware connection - fixed demo angles 15 / 5 deg",
            0, 24, 54, 880, 26);
        simulation = control(window, "BUTTON", "Run simulated observations and heartbeats",
            BS_AUTOCHECKBOX | WS_TABSTOP, 24, 88, 500, 30, 100);
        SendMessageA(simulation, BM_SETCHECK, BST_CHECKED, 0);
        control(window, "STATIC", "Assets (select one)", 0, 24, 135, 420, 25);
        control(window, "STATIC", "Tracks (select one)", 0, 470, 135, 420, 25);
        assets = control(window, "LISTBOX", "", WS_BORDER | WS_VSCROLL | LBS_NOTIFY | WS_TABSTOP,
            24, 165, 420, 165, 101);
        targets = control(window, "LISTBOX", "", WS_BORDER | WS_VSCROLL | LBS_NOTIFY | WS_TABSTOP,
            470, 165, 420, 165, 102);
        control(window, "BUTTON", "Assign + point", BS_PUSHBUTTON | WS_TABSTOP, 24, 350, 170, 38, 110);
        control(window, "BUTTON", "Stop selected", BS_PUSHBUTTON | WS_TABSTOP, 210, 350, 170, 38, 111);
        control(window, "BUTTON", "Stop all", BS_PUSHBUTTON | WS_TABSTOP, 396, 350, 170, 38, 112);
        status = control(window, "STATIC", "Ready", 0, 24, 410, 865, 48);
        control(window, "STATIC", "Recent events / queued means local acceptance, not device ACK",
            0, 24, 470, 865, 24);
        history = control(window, "EDIT", "", WS_BORDER | WS_VSCROLL | ES_MULTILINE | ES_READONLY,
            24, 500, 865, 200, 103);
        SetTimer(window, 1, 250, nullptr);
        refresh();
    }
    static int selection(HWND list) { return static_cast<int>(SendMessageA(list, LB_GETCURSEL, 0, 0)); }
    static void row(HWND list, const std::string& text) {
        SendMessageA(list, LB_ADDSTRING, 0, reinterpret_cast<LPARAM>(text.c_str()));
    }
    void refresh() {
        const auto selected_asset = selection(assets);
        const auto selected_track = selection(targets);
        const auto asset_id = selected_asset >= 0 ? asset_ids.at(selected_asset) : 0;
        const auto track_id = selected_track >= 0 ? track_ids.at(selected_track) : 0;
        SendMessageA(assets, WM_SETREDRAW, FALSE, 0);
        SendMessageA(targets, WM_SETREDRAW, FALSE, 0);
        SendMessageA(assets, LB_RESETCONTENT, 0, 0);
        SendMessageA(targets, LB_RESETCONTENT, 0, 0);
        asset_ids.clear(); track_ids.clear();
        for (const auto& [id, a] : center.assets()) {
            std::ostringstream line;
            line << id << "   " << (a.connected ? "CONNECTED" : "OFFLINE");
            if (auto it = center.assignments().find(id); it != center.assignments().end())
                line << "   track " << it->second;
            row(assets, line.str()); asset_ids.push_back(id);
            if (id == asset_id) SendMessageA(assets, LB_SETCURSEL, asset_ids.size() - 1, 0);
        }
        for (const auto& [id, t] : center.tracks()) {
            std::ostringstream line;
            line << id << "   observer " << t.observer_id << "   (" << t.project_position.x
                 << ", " << t.project_position.y << ", " << t.project_position.z << ") m";
            row(targets, line.str()); track_ids.push_back(id);
            if (id == track_id) SendMessageA(targets, LB_SETCURSEL, track_ids.size() - 1, 0);
        }
        SendMessageA(assets, WM_SETREDRAW, TRUE, 0);
        SendMessageA(targets, WM_SETREDRAW, TRUE, 0);
        InvalidateRect(assets, nullptr, TRUE); InvalidateRect(targets, nullptr, TRUE);
        if (!center.events().empty() && displayed_event != center.events().back().sequence) {
            std::ostringstream text;
            for (const auto& e : center.events())
                text << e.sequence << " / " << e.action << " / asset " << e.asset_id
                     << " / track " << e.track_id << " / " << e.result << "\r\n";
            SetWindowTextA(history, text.str().c_str());
            SendMessageA(history, EM_SETSEL, static_cast<WPARAM>(-1), static_cast<LPARAM>(-1));
            SendMessageA(history, EM_SCROLLCARET, 0, 0);
            displayed_event = center.events().back().sequence;
        }
    }
    void tick() {
        const auto now = now_us();
        if (SendMessageA(simulation, BM_GETCHECK, 0, 0) == BST_CHECKED && now - last_sample >= 500'000) {
            for (const auto& [id, a] : center.assets()) center.heartbeat(id, now, now);
            center.ingest({101, now, {{10, 20, 2}}}, now);
            center.ingest({102, now, {{20, 10, 3}}}, now);
            last_sample = now;
        }
        center.tick(now); refresh();
        // ConsoleTransport only prints simulation commands; do not retain an unbounded stream.
        output.str(""); output.clear();
    }
    void command(int id) {
        const auto a = selection(assets), t = selection(targets);
        if (id == 110) {
            if (a < 0 || t < 0) { SetWindowTextA(status, "Select an asset and a track first."); return; }
            center.assign(asset_ids.at(a), track_ids.at(t), now_us());
        } else if (id == 111) {
            if (a < 0) { SetWindowTextA(status, "Select an asset first."); return; }
            center.stop(asset_ids.at(a));
        } else if (id == 112) {
            std::size_t failures = 0;
            for (const auto& [asset_id, asset] : center.assets()) if (!center.stop(asset_id)) ++failures;
            const auto text = "Stop all / local queue failures: " + std::to_string(failures);
            SetWindowTextA(status, text.c_str()); refresh(); return;
        } else return;
        SetWindowTextA(status, center.last_result().c_str()); refresh();
    }
};
std::unique_ptr<Dashboard> dashboard;
LRESULT CALLBACK window_proc(HWND window, UINT message, WPARAM wparam, LPARAM lparam) {
    switch (message) {
    case WM_CREATE: dashboard = std::make_unique<Dashboard>(); dashboard->build(window); return 0;
    case WM_TIMER: dashboard->tick(); return 0;
    case WM_COMMAND:
        if (HIWORD(wparam) == BN_CLICKED) dashboard->command(LOWORD(wparam));
        return 0;
    case WM_DESTROY: KillTimer(window, 1); dashboard.reset(); PostQuitMessage(0); return 0;
    }
    return DefWindowProcA(window, message, wparam, lparam);
}
}
int WINAPI WinMain(HINSTANCE instance, HINSTANCE, LPSTR, int show) {
    SetProcessDPIAware();
    WNDCLASSA cls{}; cls.lpfnWndProc = window_proc; cls.hInstance = instance;
    cls.lpszClassName = "AiC2Dashboard"; cls.hCursor = LoadCursor(nullptr, IDC_ARROW);
    cls.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    if (!RegisterClassA(&cls)) return 1;
    RECT bounds{0, 0, 920, 725};
    constexpr DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
    AdjustWindowRect(&bounds, style, FALSE);
    auto window = CreateWindowA(cls.lpszClassName, "WarWarden AI C2 / Simulation", style,
        CW_USEDEFAULT, CW_USEDEFAULT, bounds.right - bounds.left, bounds.bottom - bounds.top,
        nullptr, nullptr, instance, nullptr);
    if (!window) return 1;
    ShowWindow(window, show); UpdateWindow(window);
    MSG message{};
    while (GetMessageA(&message, nullptr, 0, 0) > 0) {
        if (!IsDialogMessageA(window, &message)) { TranslateMessage(&message); DispatchMessageA(&message); }
    }
    return static_cast<int>(message.wParam);
}
