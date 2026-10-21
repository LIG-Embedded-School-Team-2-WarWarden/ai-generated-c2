#include "c2mvp/domain.hpp"
#include <csignal>
#include <fstream>
#include <iostream>
#ifndef _WIN32
#include <cerrno>
#include <poll.h>
#include <unistd.h>
#endif
using namespace c2mvp;
namespace {
volatile std::sig_atomic_t interrupted = 0;
void signal_handler(int) {
    interrupted = 1;
}
// Poll stdin on POSIX so SIGINT also exits while no line is being typed.
// Read bytes ourselves to avoid mixing fd readiness with iostream read-ahead.
class ConsoleInput {
#ifndef _WIN32
    std::string buffered;
    bool eof = false;
#endif
  public:
    std::optional<std::string> next() {
#ifdef _WIN32
        std::string line;
        if (!interrupted && std::getline(std::cin, line))
            return line;
        return {};
#else
        while (!interrupted) {
            auto end = buffered.find('\n');
            if (end != std::string::npos || eof) {
                if (buffered.empty())
                    return {};
                auto line = buffered.substr(0, end);
                if (end == std::string::npos)
                    buffered.clear();
                else
                    buffered.erase(0, end + 1);
                if (!line.empty() && line.back() == '\r')
                    line.pop_back();
                return line;
            }
            pollfd fd{STDIN_FILENO, POLLIN, 0};
            int rc = ::poll(&fd, 1, 50);
            if (rc < 0) {
                if (errno == EINTR)
                    continue;
                throw std::runtime_error("stdin poll failed");
            }
            if (!rc)
                continue;
            char bytes[1024];
            auto n = ::read(STDIN_FILENO, bytes, sizeof bytes);
            if (n > 0)
                buffered.append(bytes, static_cast<size_t>(n));
            else if (n == 0)
                eof = true;
            else if (errno != EINTR)
                throw std::runtime_error("stdin read failed");
        }
        return {};
#endif
    }
};
} // namespace
int main(int argc, char **argv) {
    try {
        std::string config_file = "configs/team-demo.json", bind = "127.0.0.1";
        int port = 5000;
        for (int i = 1; i < argc; ++i) {
            std::string a = argv[i];
            if (a == "--help") {
                std::cout << "c2_mvp_console --config FILE --bind IPv4 --asset-port PORT\n";
                return 0;
            }
            require(i + 1 < argc, "option value missing");
            std::string v = argv[++i];
            if (a == "--config")
                config_file = v;
            else if (a == "--bind")
                bind = v;
            else if (a == "--asset-port")
                port = std::stoi(v);
            else
                throw std::invalid_argument("unknown option: " + a);
        }
        std::ifstream file(config_file);
        require(file.good(), "cannot open config: " + config_file);
        auto config = Config::parse(Json::parse(file));
        SystemClock clock;
        UdpTransport transport(bind, port);
        LidarDetector detector;
        RelativePointingSolver solver;
        Engine engine(std::move(config), clock, transport, detector, solver);
        std::mutex mutex;
        std::atomic<bool> running = true;
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);
        std::thread network([&] {
            while (running) {
                try {
                    auto packet = transport.receive();
                    std::lock_guard lock(mutex);
                    if (packet)
                        try {
                            engine.receive(*packet);
                        } catch (const std::exception &e) {
                            engine.log("rejected", e.what());
                        }
                    engine.tick();
                } catch (const std::exception &e) {
                    std::lock_guard lock(mutex);
                    engine.log("runtime_error", e.what());
                    try {
                        engine.effector.stop_all("RUNTIME_ERROR");
                    } catch (...) {
                    }
                }
            }
        });
        std::cout << Json({{"ready", true},
                           {"runtime", "C++"},
                           {"port", transport.address().port},
                           {"commands", "status targets point aim stop collect obs events "
                                        "clock-load clock-profile quit"}})
                         .dump()
                  << std::endl;
        try {
            ConsoleInput input;
            while (auto line = input.next()) {
                if (*line == "quit" || *line == "exit")
                    break;
                try {
                    std::lock_guard lock(mutex);
                    auto result = engine.command(*line);
                    if (!result.is_null())
                        std::cout << result.dump() << std::endl;
                } catch (const std::exception &e) {
                    std::cout << Json({{"rejected", e.what()}}).dump() << std::endl;
                }
            }
        } catch (...) {
            interrupted = 1;
        }
        {
            std::lock_guard lock(mutex);
            try {
                if (engine.observer.busy())
                    engine.observer.stop();
                engine.effector.stop_all("SHUTDOWN");
            } catch (const std::exception &e) {
                engine.log("shutdown_error", e.what());
            }
        }
        auto wait_until = [&](double seconds, const std::function<bool()> &done) {
            double end = clock.monotonic() + seconds;
            while (clock.monotonic() < end) {
                {
                    std::lock_guard lock(mutex);
                    if (done())
                        return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        };
        wait_until(2, [&] { return engine.effector.stopped() && !engine.observer.busy(); });
        {
            std::lock_guard lock(mutex);
            try {
                auto it = engine.devices.peers.find(engine.config.observer);
                if (it != engine.devices.peers.end() && it->second.collecting)
                    engine.observer.collect(false, true);
            } catch (const std::exception &e) {
                engine.log("shutdown_error", e.what());
            }
        }
        wait_until(1, [&] { return engine.requests.size() == 0; });
        running = false;
        network.join();
        std::cout << engine.snapshot().dump() << std::endl;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
