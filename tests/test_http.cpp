// End-to-end HTTP test: the real server wired by register_api, the real
// Simulation running its real-time thread, exercised through a real client.
// Proves the API contract (status codes, SSE frame shape), not just the sim.
#include <chrono>
#include <string>
#include <thread>

#include "catch_amalgamated.hpp"
#include "http_api.hpp"
#include "httplib.h"

using namespace fccu;

TEST_CASE("HTTP API end to end") {
    Simulation sim;  // no CSV logging
    httplib::Server svr;
    const std::string dashboard = "<html>test stub</html>";
    register_api(svr, sim, dashboard);

    int port = svr.bind_to_any_port("127.0.0.1");
    REQUIRE(port > 0);
    std::thread server([&svr] { svr.listen_after_bind(); });
    svr.wait_until_ready();
    sim.start_thread();

    httplib::Client cli("127.0.0.1", port);

    // dashboard served at /
    auto root = cli.Get("/");
    REQUIRE(root);
    CHECK(root->status == 200);
    CHECK(root->body == dashboard);

    // valid commands accepted
    CHECK(cli.Post("/api/command?name=start")->status == 200);
    CHECK(cli.Post("/api/demand?value=42")->status == 200);

    // invalid input rejected at the boundary
    CHECK(cli.Post("/api/demand?value=nan")->status == 400);
    CHECK(cli.Post("/api/demand?value=abc")->status == 400);
    CHECK(cli.Post("/api/command?name=bogus")->status == 400);
    CHECK(cli.Post("/api/inject?kind=bogus")->status == 400);
    CHECK(cli.Post("/api/inject?kind=sensor_fail&sensor=nope&mode=stuck")->status
          == 400);

    // one SSE frame arrives and is shaped like telemetry
    std::string frame;
    cli.Get("/api/events", [&frame](const char* data, std::size_t n) {
        frame.append(data, n);
        return frame.find("\n\n") == std::string::npos;  // stop after one frame
    });
    REQUIRE(frame.rfind("data: {", 0) == 0);
    CHECK(frame.find("\"state\":\"") != std::string::npos);
    CHECK(frame.find("\"readings\":{") != std::string::npos);
    CHECK(frame.find("\"tank_pressure\":") != std::string::npos);

    // latch a fault for real (disconnected sensor, 500 ms persistence),
    // then verify a refused reset maps to HTTP 409
    CHECK(cli.Post("/api/inject?kind=sensor_fail&sensor=tank_pressure"
                   "&mode=disconnected")->status == 200);
    std::this_thread::sleep_for(std::chrono::milliseconds(900));
    REQUIRE(sim.safety().latched());
    CHECK(cli.Post("/api/command?name=reset")->status == 409);

    svr.stop();
    server.join();
}
