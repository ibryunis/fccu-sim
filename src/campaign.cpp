// Monte Carlo verification campaign: run the randomized acceptance scenario
// N times in parallel (each worker owns its whole Simulation - no shared
// state), then aggregate pass rate and latch-latency statistics per fault
// class. Any failing seed reproduces exactly via the AUTO TEST button or
// fccu_campaign --runs 1 --seed <seed>.
//
//   fccu_campaign [--runs N] [--jobs J] [--seed BASE]

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "fccu/runtime.hpp"
#include "scenario/scenario_runner.hpp"

namespace {

using namespace fccu;

constexpr const char* FAULT_NAMES[] = {"overheat", "pressure spike",
                                       "sensor disconnect", "sensor stuck"};

struct RunResult {
    std::uint32_t seed = 0;
    bool pass = false;
    int fault_kind = -1;
    long latch_ticks = -1;
    long ticks = 0;
};

RunResult run_one(std::uint32_t seed) {
    Simulation sim;  // logging disabled
    sim.start_autotest(seed);
    for (long guard = 400L * 100; guard > 0; --guard) {
        sim.step();
        sim.advance_autotest();
        if (sim.autotest().report().done) break;
    }
    auto rep = sim.autotest().report();
    return {seed, rep.done && rep.pass, rep.fault_kind, rep.latch_ticks,
            rep.total_ticks};
}

long pct_ms(std::vector<long>& v, double p) {
    if (v.empty()) return 0;
    auto k = static_cast<std::size_t>(p * static_cast<double>(v.size() - 1));
    std::nth_element(v.begin(), v.begin() + k, v.end());
    return v[k] * 10;  // ticks -> ms
}

} // namespace

int main(int argc, char** argv) {
    long runs = 1000;
    unsigned jobs = std::max(1u, std::thread::hardware_concurrency());
    std::uint32_t base_seed = 1;
    for (int i = 1; i + 1 < argc; i += 2) {
        if (!std::strcmp(argv[i], "--runs")) runs = std::atol(argv[i + 1]);
        else if (!std::strcmp(argv[i], "--jobs")) jobs = std::atoi(argv[i + 1]);
        else if (!std::strcmp(argv[i], "--seed")) base_seed = std::atoi(argv[i + 1]);
    }

    std::printf("campaign: %ld runs, %u workers, seeds %u..%u\n",
                runs, jobs, base_seed, base_seed + static_cast<unsigned>(runs) - 1);

    std::vector<RunResult> results(static_cast<std::size_t>(runs));
    std::atomic<std::size_t> next{0};
    std::atomic<std::size_t> completed{0};
    auto t0 = std::chrono::steady_clock::now();

    {
        std::vector<std::jthread> pool;
        for (unsigned j = 0; j < jobs; ++j) {
            pool.emplace_back([&] {
                std::size_t i;
                while ((i = next.fetch_add(1)) < results.size()) {
                    results[i] = run_one(base_seed + static_cast<std::uint32_t>(i));
                    auto done = completed.fetch_add(1) + 1;
                    if (done % 200 == 0) {
                        std::printf("  %zu/%ld done\n", done, runs);
                    }
                }
            });
        }
    }  // jthreads join

    double wall_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();

    long passed = 0, total_ticks = 0;
    std::vector<long> latency[4];
    std::vector<std::uint32_t> failed_seeds;
    for (const auto& r : results) {
        passed += r.pass;
        total_ticks += r.ticks;
        if (!r.pass) failed_seeds.push_back(r.seed);
        if (r.fault_kind >= 0 && r.latch_ticks >= 0) {
            latency[r.fault_kind].push_back(r.latch_ticks);
        }
    }
    double sim_hours = static_cast<double>(total_ticks) / 100.0 / 3600.0;

    std::printf("\n==== campaign report ====\n");
    std::printf("scenarios      %ld\n", runs);
    std::printf("passed         %ld (%.2f%%)\n", passed,
                100.0 * static_cast<double>(passed) / static_cast<double>(runs));
    std::printf("sim time       %.1f h (%ld ticks)\n", sim_hours, total_ticks);
    std::printf("wall time      %.1f s  (%.0fx realtime, %.1f Mticks/s)\n",
                wall_s, static_cast<double>(total_ticks) / 100.0 / wall_s,
                static_cast<double>(total_ticks) / wall_s / 1e6);
    std::printf("\nlatch latency from injection (ms, sim time)\n");
    std::printf("%-18s %6s %8s %8s %8s\n", "fault", "n", "p50", "p95", "max");
    for (int k = 0; k < 4; ++k) {
        auto& v = latency[k];
        if (v.empty()) continue;
        long mx = *std::max_element(v.begin(), v.end()) * 10;
        std::printf("%-18s %6zu %8ld %8ld %8ld\n", FAULT_NAMES[k], v.size(),
                    pct_ms(v, 0.5), pct_ms(v, 0.95), mx);
    }
    if (!failed_seeds.empty()) {
        std::printf("\nFAILED seeds (reproduce with --runs 1 --seed <s>):");
        for (auto s : failed_seeds) std::printf(" %u", s);
        std::printf("\n");
    }

    std::ofstream json("campaign_report.json");
    json << "{\"runs\":" << runs << ",\"passed\":" << passed
         << ",\"wall_s\":" << wall_s << ",\"total_ticks\":" << total_ticks
         << ",\"failed_seeds\":[";
    for (std::size_t i = 0; i < failed_seeds.size(); ++i) {
        json << (i ? "," : "") << failed_seeds[i];
    }
    json << "]}\n";
    std::printf("\nreport written to campaign_report.json\n");

    return failed_seeds.empty() ? 0 : 1;
}
