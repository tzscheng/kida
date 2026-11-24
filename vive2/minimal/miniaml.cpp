#include "ultimate_tracker.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt_handler(int) { interrupted = 1; }

const char* state_name(vive::TrackerState state) {
    switch (state) {
    case vive::Syncing: return "Syncing";
    case vive::BuildMap: return "BuildMap";
    case vive::Ready: return "Ready";
    default: return "Other";
    }
}

struct Sample {
    std::string serial;
    vive::TrackerData pose{};
    bool updated = false;
};

class Tracker final : public vive::ITrackerSubscriber {
public:
    Tracker() : samples_(vive::GetNumTrackers()) {}

    void ServerStart() override {}
    void ServerStop() override { stopped.store(true); }

    void Add(uint32_t id, const char* serial) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (id < samples_.size()) {
            samples_[id] = {};
            samples_[id].serial = serial ? serial : "";
        }
    }

    void Update(const vive::TrackerData& pose) override {
        // Runs on the SDK thread: only copy data; do work on the main thread.
        std::lock_guard<std::mutex> lock(mutex_);
        if (pose.Id < samples_.size()) {
            samples_[pose.Id].pose = pose;
            samples_[pose.Id].updated = true;
            ++received;
        }
    }

    void Click(uint32_t, int, int clicks) override {
        if (clicks >= 3) stopped.store(true);
    }

    void Remove(uint32_t id) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (id < samples_.size()) samples_[id] = {};
    }

    void CommandResponse(const vive::TrackerCommandResponse&) override {}

    std::vector<Sample> take_latest() {
        std::lock_guard<std::mutex> lock(mutex_);
        auto result = samples_;
        for (auto& sample : samples_) sample.updated = false;
        return result;
    }

    std::atomic<bool> stopped{false};
    std::atomic<unsigned long long> received{0};

private:
    std::mutex mutex_;
    std::vector<Sample> samples_;
};
} // namespace

int main(int argc, char** argv) {
    double seconds = 0;
    if (argc != 1) {
        if (argc == 2 && std::strcmp(argv[1], "--help") == 0) {
            std::puts("Usage: minimal [--seconds N]\n"
                      "Stop with Ctrl+C or a connected tracker's triple-click.");
            return 0;
        }
        char* end = nullptr;
        if (argc == 3 && std::strcmp(argv[1], "--seconds") == 0)
            seconds = std::strtod(argv[2], &end);
        if (!end || *end || !std::isfinite(seconds) || seconds <= 0) {
            std::fputs("Usage: minimal [--seconds N], N > 0\n", stderr);
            return 2;
        }
    }

    // Register before starting the server; keep alive until after it stops.
    Tracker tracker;
    if (!vive::StartTrackerServer()) {
        std::fputs("Tracker server failed to start. Check the dongle, USB access, "
                   "and whether another tracker server is already running.\n", stderr);
        return 1;
    }
    std::signal(SIGINT, interrupt_handler);
    std::signal(SIGTERM, interrupt_handler);

    std::fprintf(stderr, "SDK %s; server started. Turn on your paired trackers.\n"
                         "Receiving through direct callbacks; printing latest samples at 20 Hz.\n",
                 vive::GetVersion());
    std::puts("id,serial,timestamp_us,state,pose_state,rotation_only,setup_required,"
              "x_m,y_m,z_m,qx,qy,qz,qw");

    const auto start = std::chrono::steady_clock::now();
    while (!interrupted && !tracker.stopped.load()) {
        if (seconds > 0 && std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count() >= seconds) break;

        for (const auto& sample : tracker.take_latest()) {
            if (!sample.updated) continue;
            const auto& p = sample.pose;
            std::printf("%u,%s,%lld,%s,%u,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                p.Id, sample.serial.c_str(), static_cast<long long>(p.Timestamp_us),
                state_name(p.State), p.Flags & vive::TrackerFlag_PoseStateMask,
                (p.Flags & vive::TrackerFlag_RotationOnly) != 0,
                (p.Flags & vive::TrackerFlag_SetupRequired) != 0,
                p.Location[0], p.Location[1], p.Location[2],
                p.Rotation[0], p.Rotation[1], p.Rotation[2], p.Rotation[3]);
        }
        std::fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    vive::StopTrackerServer();
    std::fprintf(stderr, "Server stopped; received %llu pose updates.\n", tracker.received.load());
    return 0;
}
