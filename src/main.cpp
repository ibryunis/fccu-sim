// Entry point: HTTP server (cpp-httplib) + SSE telemetry + REST commands.
//
// This is the only translation unit that knows about HTTP, JSON or strings
// for sensor/failure names. The sim core never includes any of it.
//
// Transport choice: telemetry is strictly one-way at 20 Hz, commands are
// request/response - so Server-Sent Events over plain HTTP, not WebSocket.
// EventSource in the browser reconnects automatically for free.

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <format>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include "fccu/runtime.hpp"
#include "httplib.h"

namespace {

using namespace fccu;

// --- string <-> enum maps (UI boundary only) --------------------------------

std::optional<SensorId> sensor_from(const std::string& name) {
    for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
        if (name == SENSOR_NAMES[i]) return static_cast<SensorId>(i);
    }
    return std::nullopt;
}

std::optional<FailureMode> mode_from(const std::string& name) {
    if (name == "stuck") return FailureMode::stuck;
    if (name == "disconnected") return FailureMode::disconnected;
    if (name == "out_of_range") return FailureMode::out_of_range;
    return std::nullopt;
}

const char* mode_name(FailureMode m) {
    switch (m) {
    case FailureMode::stuck: return "stuck";
    case FailureMode::disconnected: return "disconnected";
    case FailureMode::out_of_range: return "out_of_range";
    case FailureMode::none: break;
    }
    return "none";
}

// --- JSON ---------------------------------------------------------------

// NaN is not valid JSON: a disconnected sensor becomes null
std::string num(double v, int decimals = 3) {
    if (std::isnan(v)) return "null";
    return std::format("{:.{}f}", v, decimals);
}

std::string to_json(const Snapshot& s) {
    std::ostringstream j;
    j << "{\"t\":" << num(s.t, 2)
      << ",\"state\":\"" << to_string(s.state) << '"'
      << ",\"latched\":" << (s.latched ? "true" : "false")
      << ",\"fault\":";
    if (s.fault) {
        j << '"' << s.fault->describe() << '"';
    } else {
        j << "null";
    }
    j << ",\"fault_history\":[";
    for (std::size_t i = 0; i < s.fault_history.size(); ++i) {
        j << (i ? "," : "") << '"' << s.fault_history[i].describe() << '"';
    }
    j << "],\"overruns\":" << s.overruns
      << ",\"demand_pct\":" << num(s.demand_pct, 0)
      << ",\"current_setpoint\":" << num(s.current_setpoint, 1)
      << ",\"power_kw\":" << num(s.power_kw, 2)
      << ",\"readings\":{";
    for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
        j << (i ? "," : "") << '"' << SENSOR_NAMES[i]
          << "\":" << num(s.readings.*READING_FIELDS[i]);
    }
    j << "},\"actuators\":{\"h2_valve\":" << num(s.h2_valve)
      << ",\"compressor\":" << num(s.compressor)
      << ",\"recirc_pump\":" << num(s.recirc_pump)
      << ",\"cooling\":" << num(s.cooling)
      << "},\"cooling_blocked\":" << (s.cooling_blocked ? "true" : "false")
      << ",\"sensor_failures\":{";
    bool first = true;
    for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
        if (s.failures[i] == FailureMode::none) continue;
        j << (first ? "" : ",") << '"' << SENSOR_NAMES[i] << "\":\""
          << mode_name(s.failures[i]) << '"';
        first = false;
    }
    j << "}}";
    return j.str();
}

// --- static dashboard -----------------------------------------------------

std::string load_dashboard(const char* argv0) {
    namespace fs = std::filesystem;
    fs::path exe_dir = fs::path(argv0).parent_path();
    for (const fs::path& base : {fs::path("."), exe_dir, exe_dir / ".."}) {
        fs::path candidate = base / "ui" / "static" / "index.html";
        std::ifstream f(candidate, std::ios::binary);
        if (f) {
            std::ostringstream ss;
            ss << f.rdbuf();
            return ss.str();
        }
    }
    return {};
}

} // namespace

int main(int, char** argv) {
    Simulation sim(std::filesystem::path("logs"));
    sim.start_thread();
    std::printf("logging to %s\n", sim.log_path().string().c_str());

    std::string dashboard = load_dashboard(argv[0]);
    if (dashboard.empty()) {
        std::fprintf(stderr, "ui/static/index.html not found; run from repo root\n");
        return 1;
    }

    httplib::Server svr;
    // each SSE connection occupies a worker for its lifetime; a bigger pool
    // keeps command endpoints responsive with many dashboard tabs open
    svr.new_task_queue = [] { return new httplib::ThreadPool(32); };

    svr.Get("/", [&dashboard](const httplib::Request&, httplib::Response& res) {
        res.set_content(dashboard, "text/html");
    });

    // SSE telemetry, 20 Hz. Lock discipline: sim.snapshot() locks only for
    // the struct copy; JSON serialization and the network write happen here,
    // outside the sim lock. A slow client stalls only this handler thread.
    svr.Get("/api/events", [&sim](const httplib::Request&, httplib::Response& res) {
        res.set_header("Cache-Control", "no-cache");
        res.set_chunked_content_provider(
            "text/event-stream",
            [&sim](std::size_t, httplib::DataSink& sink) {
                std::string frame = "data: " + to_json(sim.snapshot()) + "\n\n";
                if (!sink.write(frame.data(), frame.size())) return false;
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                return true;
            });
    });

    svr.Post("/api/demand", [&sim](const httplib::Request& req, httplib::Response& res) {
        try {
            // stod("nan") parses without throwing - reject non-finite here too
            double v = std::stod(req.get_param_value("value"));
            if (!std::isfinite(v)) throw std::invalid_argument("non-finite");
            sim.set_demand(v);
        } catch (const std::exception&) {
            res.status = 400;
        }
    });

    svr.Post("/api/command", [&sim](const httplib::Request& req, httplib::Response& res) {
        switch (sim.command(req.get_param_value("name"))) {
        case Simulation::CommandResult::ok: break;
        case Simulation::CommandResult::refused: res.status = 409; break;
        case Simulation::CommandResult::unknown: res.status = 400; break;
        }
    });

    svr.Post("/api/inject", [&sim](const httplib::Request& req, httplib::Response& res) {
        const std::string kind = req.get_param_value("kind");
        if (kind == "pressure_spike") {
            sim.inject(InjectKind::pressure_spike);
        } else if (kind == "overheat") {
            sim.inject(InjectKind::overheat);
        } else if (kind == "clear_overheat") {
            sim.inject(InjectKind::clear_overheat);
        } else if (kind == "sensor_fail" || kind == "sensor_repair") {
            auto sensor = sensor_from(req.get_param_value("sensor"));
            auto mode = mode_from(req.get_param_value("mode"));
            if (!sensor || (kind == "sensor_fail" && !mode)) {
                res.status = 400;
                return;
            }
            if (kind == "sensor_fail") {
                sim.inject(InjectKind::sensor_fail, *sensor, *mode);
            } else {
                sim.inject(InjectKind::sensor_repair, *sensor);
            }
        } else {
            res.status = 400;
        }
    });

    std::printf("FCCU simulator: http://localhost:8000\n");
    if (!svr.listen("127.0.0.1", 8000)) {
        std::fprintf(stderr, "failed to bind port 8000 (already in use?)\n");
        return 1;
    }
    return 0;
}
