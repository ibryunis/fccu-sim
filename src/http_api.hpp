// HTTP/JSON boundary, shared by main.cpp and the end-to-end HTTP test.
#pragma once

#include <string>

#include "fccu/runtime.hpp"

namespace httplib {
class Server;
}

namespace fccu {

std::string to_json(const Snapshot& s);

// wires all REST + SSE endpoints onto the server; dashboard is served at /
void register_api(httplib::Server& svr, Simulation& sim,
                  const std::string& dashboard);

} // namespace fccu
