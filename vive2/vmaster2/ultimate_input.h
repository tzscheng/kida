#pragma once
#include "ultimate_tracker.h"
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace ultimate {
using Clock = std::chrono::steady_clock;
constexpr double kStaleSeconds = .3;

struct Sample {
    uint32_t id = 0;
    std::string serial;
    vive::TrackerData data{};
    Clock::time_point received{};
    uint64_t generation = 0;
    bool connected = false, has_pose = false;
};

inline bool valid_pose(const vive::TrackerData& p) {
    if (p.State != vive::Ready || (p.Flags & vive::TrackerFlag_PoseStateMask) != 2 ||
        (p.Flags & (vive::TrackerFlag_RotationOnly | vive::TrackerFlag_SetupRequired))) return false;
    for (float v : p.Location) if (!std::isfinite(v)) return false;
    double norm = 0;
    for (float v : p.Rotation) { if (!std::isfinite(v)) return false; norm += double(v)*v; }
    return norm > .1;
}

inline const char* health(const Sample& s, Clock::time_point now = Clock::now()) {
    if (!s.connected) return "NOT CONNECTED";
    if (!s.has_pose) return "WAITING FOR POSE";
    if (std::chrono::duration<double>(now-s.received).count() > kStaleSeconds) return "STALE DATA";
    const auto& p = s.data;
    if (p.State == vive::Syncing) return "SYNCING MAP";
    if (p.State == vive::BuildMap) return "BUILDING MAP";
    if (p.Flags & vive::TrackerFlag_SetupRequired) return "MAP SETUP REQUIRED";
    if (p.Flags & vive::TrackerFlag_RotationOnly) return "ROTATION ONLY";
    if ((p.Flags & vive::TrackerFlag_PoseStateMask) == 3) return "LOCALIZING";
    return valid_pose(p) ? "TRACKING" : "INVALID / NOT TRACKING";
}
inline bool usable(const Sample& s) { return std::string(health(s)) == "TRACKING"; }

// SDK slots choose a device once. Pin its serial for the rest of this run so
// another tracker cannot silently take over an arm after a reconnect.
struct Selection {
    uint32_t id = 0;
    std::string serial;
};

class Trackers final : public vive::ITrackerSubscriber {
public:
    Trackers() : samples_(vive::GetNumTrackers()) {
        for (uint32_t i=0;i<samples_.size();++i) samples_[i].id=i;
    }
    ~Trackers() override { stop(); }
    bool start() { running_ = vive::StartTrackerServer(); return running_; }
    void stop() { if (running_) { vive::StopTrackerServer(); running_=false; } }
    void ServerStart() override {}
    void ServerStop() override { stopped.store(true); }
    void Add(uint32_t id,const char* serial) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (id>=samples_.size()) return;
        samples_[id]={};
        auto& s=samples_[id]; s.id=id; s.serial=serial?serial:"";
        s.connected=true; s.generation=++generation_;
    }
    void Remove(uint32_t id) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (id>=samples_.size()) return;
        samples_[id]={}; samples_[id].id=id; samples_[id].generation=++generation_;
    }
    void Update(const vive::TrackerData& p) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (p.Id>=samples_.size()) return;
        auto& s=samples_[p.Id]; auto now=Clock::now();
        // Observe even brief relocalization/loss events between UI polls.
        if (s.has_pose && (valid_pose(s.data)!=valid_pose(p) ||
            std::chrono::duration<double>(now-s.received).count()>kStaleSeconds))
            s.generation=++generation_;
        s.data=p; s.received=now; s.has_pose=true;
    }
    void Click(uint32_t,int,int clicks) override { if (clicks>=3) stopped.store(true); }
    void CommandResponse(const vive::TrackerCommandResponse&) override {}

    Sample select(Selection& selection) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (selection.serial.empty() && selection.id<samples_.size()) {
            auto s=samples_[selection.id];
            if (s.connected && !s.serial.empty()) selection.serial=s.serial;
            return s;
        }
        for (const auto& s : samples_)
            if (s.connected && s.serial==selection.serial) return s;
        Sample missing; missing.id=selection.id; missing.serial=selection.serial;
        return missing;
    }
    std::atomic<bool> stopped{false};
private:
    bool running_=false;
    uint64_t generation_=0;
    std::mutex mutex_;
    std::vector<Sample> samples_;
};

// Ultimate coordinates already match KIDA: X forward, Y left, Z up.
// Quaternion is SDK (x,y,z,w); the returned transform is row-major.
inline std::array<double,16> pose_matrix(const vive::TrackerData& p) {
    double x=p.Rotation[0],y=p.Rotation[1],z=p.Rotation[2],w=p.Rotation[3];
    double n=std::sqrt(x*x+y*y+z*z+w*w);
    x/=n; y/=n; z/=n; w/=n;
    return {{1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),p.Location[0],
             2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),p.Location[1],
             2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y),p.Location[2],
             0,0,0,1}};
}
} // namespace ultimate
