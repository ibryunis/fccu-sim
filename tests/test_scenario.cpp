// The autotest scenario runner, driven headless. Seeds cover different
// random fault draws; each full run exercises startup, demand tracking,
// fault latch, refused reset, drain, accepted reset and restart.
#include "catch_amalgamated.hpp"
#include "fccu/runtime.hpp"
#include "scenario/scenario_runner.hpp"

using namespace fccu;

namespace {
ScenarioRunner::Report run_scenario(std::uint32_t seed) {
    Simulation sim;
    REQUIRE(sim.start_autotest(seed));
    for (long i = 0; i < 400L * 100; ++i) {  // 400 s sim budget
        sim.step();
        sim.advance_autotest();
        if (sim.autotest().report().done) break;
    }
    return sim.autotest().report();
}
} // namespace

TEST_CASE("scenario runner passes across random seeds") {
    for (std::uint32_t seed : {1u, 2u, 7u}) {
        auto rep = run_scenario(seed);
        INFO("seed " << seed);
        for (const auto& l : rep.lines) {
            INFO((l.pass ? "PASS " : "FAIL ") << l.text);
        }
        REQUIRE(rep.done);
        CHECK(rep.pass);
    }
}

TEST_CASE("second autotest refused while one is running") {
    Simulation sim;
    REQUIRE(sim.start_autotest(5));
    CHECK_FALSE(sim.start_autotest(6));
}

TEST_CASE("manual commands rejected while autotest runs") {
    Simulation sim;
    REQUIRE(sim.start_autotest(3));
    sim.step();
    sim.advance_autotest();
    CHECK(sim.command("stop") == Simulation::CommandResult::refused);
    CHECK_FALSE(sim.set_demand(50.0));
    CHECK_FALSE(sim.inject(InjectKind::overheat));
    // the runner itself is not locked out
    CHECK(sim.set_demand(10.0, Simulation::Auth::autotest));
}
