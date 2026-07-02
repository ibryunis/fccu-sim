#include "catch_amalgamated.hpp"
#include "fccu/state_machine.hpp"

using namespace fccu;

namespace {
constexpr double DT = 0.01;

struct Inputs {
    double anode_pressure;
    double stack_temp;
    bool latched;
};
constexpr Inputs NOMINAL{2.0, 65.0, false};
constexpr Inputs COLD{1.0, 25.0, false};

State run(StateMachine& fsm, double seconds, const Inputs& in) {
    for (int i = 0; i < static_cast<int>(seconds / DT); ++i) {
        fsm.update(DT, in.anode_pressure, in.stack_temp, in.latched);
    }
    return fsm.state();
}

StateMachine running_fsm() {
    StateMachine fsm;
    fsm.request_start();
    run(fsm, 3.0, NOMINAL);
    REQUIRE(fsm.state() == State::running);
    return fsm;
}
} // namespace

TEST_CASE("startup sequence") {
    StateMachine fsm;
    CHECK(fsm.state() == State::off);

    SECTION("off ignores time") {
        CHECK(run(fsm, 5.0, COLD) == State::off);
    }
    SECTION("start enters purge") {
        fsm.request_start();
        fsm.update(DT, COLD.anode_pressure, COLD.stack_temp, false);
        CHECK(fsm.state() == State::purge);
    }
    SECTION("purge lasts two seconds") {
        fsm.request_start();
        run(fsm, 1.9, COLD);
        CHECK(fsm.state() == State::purge);
        run(fsm, 0.2, COLD);
        CHECK(fsm.state() == State::pressurize);
    }
    SECTION("pressurize exits on pressure") {
        fsm.request_start();
        run(fsm, 2.1, COLD);
        fsm.update(DT, 1.9, 25.0, false);
        CHECK(fsm.state() == State::warmup);
    }
    SECTION("pressurize timeout faults") {
        fsm.request_start();
        run(fsm, 2.1 + 10.1, COLD);
        CHECK(fsm.state() == State::fault);
        REQUIRE(fsm.timeout_reason().has_value());
        CHECK(fsm.timeout_reason()->find("pressurize") != std::string::npos);
    }
    SECTION("warmup exits on temperature") {
        fsm.request_start();
        run(fsm, 2.1, COLD);
        fsm.update(DT, 2.0, 25.0, false);
        CHECK(fsm.state() == State::warmup);
        fsm.update(DT, 2.0, 36.0, false);
        CHECK(fsm.state() == State::running);
    }
    SECTION("full sequence to running") {
        fsm.request_start();
        run(fsm, 2.1, COLD);
        CHECK(fsm.state() == State::pressurize);
        run(fsm, 1.0, NOMINAL);
        CHECK(fsm.state() == State::running);
    }
}

TEST_CASE("stop behaviour") {
    SECTION("stop from running goes through shutdown to off") {
        StateMachine fsm = running_fsm();
        fsm.request_stop();
        fsm.update(DT, 2.0, 65.0, false);
        CHECK(fsm.state() == State::shutdown);
        run(fsm, 3.1, NOMINAL);
        CHECK(fsm.state() == State::off);
    }
    SECTION("stop aborts startup") {
        StateMachine fsm;
        fsm.request_start();
        fsm.update(DT, 1.0, 25.0, false);
        fsm.request_stop();
        fsm.update(DT, 1.0, 25.0, false);
        CHECK(fsm.state() == State::shutdown);
    }
}

TEST_CASE("fault latch behaviour") {
    SECTION("latch forces fault from running") {
        StateMachine fsm = running_fsm();
        fsm.update(DT, 2.0, 65.0, true);
        CHECK(fsm.state() == State::fault);
    }
    SECTION("latch forces fault from startup") {
        StateMachine fsm;
        fsm.request_start();
        fsm.update(DT, 1.0, 25.0, false);
        fsm.update(DT, 1.0, 25.0, true);
        CHECK(fsm.state() == State::fault);
    }
    SECTION("cannot start while latched") {
        StateMachine fsm;
        fsm.request_start();
        fsm.update(DT, 1.0, 25.0, true);
        CHECK(fsm.state() == State::fault);
    }
    SECTION("reset ignored while latched") {
        StateMachine fsm = running_fsm();
        fsm.update(DT, 2.0, 65.0, true);
        fsm.request_reset();
        fsm.update(DT, 2.0, 65.0, true);
        CHECK(fsm.state() == State::fault);
    }
    SECTION("reset works after latch cleared") {
        StateMachine fsm = running_fsm();
        fsm.update(DT, 2.0, 65.0, true);
        fsm.request_reset();
        fsm.update(DT, 2.0, 65.0, false);
        CHECK(fsm.state() == State::off);
    }
    SECTION("fault does not auto-clear") {
        StateMachine fsm = running_fsm();
        fsm.update(DT, 2.0, 65.0, true);
        CHECK(run(fsm, 10.0, NOMINAL) == State::fault);
    }
}
