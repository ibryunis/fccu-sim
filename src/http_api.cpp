#include "http_api.hpp"

#include <chrono>
#include <cmath>
#include <format>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "httplib.h"

namespace fccu {
namespace {

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

// NaN is not valid JSON: a disconnected sensor becomes null
std::string num(double v, int decimals = 3) {
    if (std::isnan(v)) return "null";
    return std::format("{:.{}f}", v, decimals);
}

} // namespace

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

void register_api(httplib::Server& svr, Simulation& sim,
                  const std::string& dashboard) {
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
}

} // namespace fccu
