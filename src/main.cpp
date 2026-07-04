// Entry point: HTTP server (cpp-httplib) + SSE telemetry + REST commands.
//
// The sim core never includes any HTTP/JSON code; the boundary lives in
// http_api.cpp and is exercised end-to-end by tests/test_http.cpp.
//
// Transport choice: telemetry is strictly one-way at 20 Hz, commands are
// request/response - so Server-Sent Events over plain HTTP, not WebSocket.
// EventSource in the browser reconnects automatically for free.

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "fccu/runtime.hpp"
#include "http_api.hpp"
#include "httplib.h"

#ifdef _WIN32
#include <timeapi.h>
#endif

namespace {

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
#ifdef _WIN32
    // default Windows timer granularity is ~15.6 ms; a 100 Hz loop using
    // sleep_until needs 1 ms resolution or most deadlines are missed
    timeBeginPeriod(1);
#endif
    fccu::Simulation sim(std::filesystem::path("logs"));
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
    fccu::register_api(svr, sim, dashboard);

    std::printf("FCCU simulator: http://localhost:8000\n");
    if (!svr.listen("127.0.0.1", 8000)) {
        std::fprintf(stderr, "failed to bind port 8000 (already in use?)\n");
        return 1;
    }
    return 0;
}
