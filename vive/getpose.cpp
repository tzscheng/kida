// Minimal OpenVR tracker-pose reader.
//
// Build from kida/vive:
//   OPENVR_LIB_DIR=${OPENVR_LIB_DIR:-$HOME/.local/share/Steam/steamapps/common/SteamVR/bin/linux64}
//   g++ -O2 -std=c++17 -Wall -Wextra -I. getpose.cpp -o getpose -L"$OPENVR_LIB_DIR" -lopenvr_api -Wl,-rpath,"$OPENVR_LIB_DIR"
//
// SteamVR must already be running.  Stop with Ctrl-C.

#include "openvr/openvr.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

std::atomic<bool> running{true};

void Stop(int)
{
    running.store(false);
}

std::string GetSerial(vr::IVRSystem* system, vr::TrackedDeviceIndex_t index)
{
    vr::ETrackedPropertyError error = vr::TrackedProp_Success;
    const uint32_t size = system->GetStringTrackedDeviceProperty(
        index, vr::Prop_SerialNumber_String, nullptr, 0, &error);
    if (size == 0 || error != vr::TrackedProp_Success) return "unknown";

    std::vector<char> buffer(size);
    system->GetStringTrackedDeviceProperty(
        index, vr::Prop_SerialNumber_String, buffer.data(), size, &error);
    return error == vr::TrackedProp_Success ? std::string(buffer.data()) : "unknown";
}

// Convert the rotation part of OpenVR's 3x4 pose matrix to x,y,z,w quaternion.
void MatrixToQuaternion(const vr::HmdMatrix34_t& m,
                        double& qx, double& qy, double& qz, double& qw)
{
    const double trace = m.m[0][0] + m.m[1][1] + m.m[2][2];
    if (trace > 0.0) {
        const double s = 2.0 * std::sqrt(trace + 1.0);
        qw = 0.25 * s;
        qx = (m.m[2][1] - m.m[1][2]) / s;
        qy = (m.m[0][2] - m.m[2][0]) / s;
        qz = (m.m[1][0] - m.m[0][1]) / s;
    } else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2]) {
        const double s = 2.0 * std::sqrt(1.0 + m.m[0][0] - m.m[1][1] - m.m[2][2]);
        qw = (m.m[2][1] - m.m[1][2]) / s;
        qx = 0.25 * s;
        qy = (m.m[0][1] + m.m[1][0]) / s;
        qz = (m.m[0][2] + m.m[2][0]) / s;
    } else if (m.m[1][1] > m.m[2][2]) {
        const double s = 2.0 * std::sqrt(1.0 + m.m[1][1] - m.m[0][0] - m.m[2][2]);
        qw = (m.m[0][2] - m.m[2][0]) / s;
        qx = (m.m[0][1] + m.m[1][0]) / s;
        qy = 0.25 * s;
        qz = (m.m[1][2] + m.m[2][1]) / s;
    } else {
        const double s = 2.0 * std::sqrt(1.0 + m.m[2][2] - m.m[0][0] - m.m[1][1]);
        qw = (m.m[1][0] - m.m[0][1]) / s;
        qx = (m.m[0][2] + m.m[2][0]) / s;
        qy = (m.m[1][2] + m.m[2][1]) / s;
        qz = 0.25 * s;
    }
}

// Extrinsic XYZ roll-pitch-yaw, matching vmaster's RotationToEulerXYZ().
void MatrixToRollPitchYaw(const vr::HmdMatrix34_t& m,
                          double& roll, double& pitch, double& yaw)
{
    const double r20 = std::max(-1.0, std::min(1.0, static_cast<double>(m.m[2][0])));
    pitch = std::asin(-r20);
    if (std::abs(r20) < 0.999999) {
        roll = std::atan2(m.m[2][1], m.m[2][2]);
        yaw  = std::atan2(m.m[1][0], m.m[0][0]);
    } else {
        roll = std::atan2(-m.m[1][2], m.m[1][1]);
        yaw  = 0.0;
    }
}

}  // namespace

int main(int argc, char** argv)
{
    bool quaternion_output = false;
    if (argc == 2 && std::string(argv[1]) == "-q") {
        quaternion_output = true;
    } else if (argc != 1) {
        std::fprintf(stderr, "usage: %s [-q]\n", argv[0]);
        return 64;
    }

    std::signal(SIGINT, Stop);
    std::signal(SIGTERM, Stop);

    vr::EVRInitError error = vr::VRInitError_None;
    vr::IVRSystem* system = vr::VR_Init(&error, vr::VRApplication_Background);
    if (error != vr::VRInitError_None || system == nullptr) {
        std::fprintf(stderr, "VR_Init failed: %s\n",
                     vr::VR_GetVRInitErrorAsEnglishDescription(error));
        return 1;
    }

    std::vector<std::pair<std::string, vr::TrackedDeviceIndex_t>> trackers;
    for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i) {
        if (system->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_GenericTracker)
            trackers.emplace_back(GetSerial(system, i), i);
    }
    std::sort(trackers.begin(), trackers.end());

    if (trackers.empty()) {
        std::fprintf(stderr, "No Vive tracker found.\n");
        vr::VR_Shutdown();
        return 2;
    }

    if (quaternion_output)
        std::printf("Standing-space raw tracker pose (metres, quaternion x y z w)\n");
    else
        std::printf("Standing-space raw tracker pose (metres, roll-pitch-yaw radians)\n");
    for (size_t i = 0; i < trackers.size(); ++i)
        std::printf("tracker[%zu]: device=%u serial=%s\n",
                    i, trackers[i].second, trackers[i].first.c_str());

    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};
    while (running.load()) {
        system->GetDeviceToAbsoluteTrackingPose(
            vr::TrackingUniverseStanding, 0.0f,
            poses, vr::k_unMaxTrackedDeviceCount);

        for (size_t i = 0; i < trackers.size(); ++i) {
            const auto device = trackers[i].second;
            const auto& pose = poses[device];
            if (!pose.bDeviceIsConnected || !pose.bPoseIsValid ||
                pose.eTrackingResult != vr::TrackingResult_Running_OK) {
                std::printf("[%zu] %-16s tracking unavailable\n",
                            i, trackers[i].first.c_str());
                continue;
            }

            const auto& m = pose.mDeviceToAbsoluteTracking;
            if (quaternion_output) {
                double qx, qy, qz, qw;
                MatrixToQuaternion(m, qx, qy, qz, qw);
                std::printf("[%zu] %-16s p=(% .4f % .4f % .4f) "
                            "q=(% .5f % .5f % .5f % .5f)\n",
                            i, trackers[i].first.c_str(),
                            m.m[0][3], m.m[1][3], m.m[2][3], qx, qy, qz, qw);
            } else {
                double roll, pitch, yaw;
                MatrixToRollPitchYaw(m, roll, pitch, yaw);
                std::printf("[%zu] %-16s p=(% .4f % .4f % .4f) "
                            "rpy=(% .5f % .5f % .5f)\n",
                            i, trackers[i].first.c_str(),
                            m.m[0][3], m.m[1][3], m.m[2][3], roll, pitch, yaw);
            }
        }
        std::fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    vr::VR_Shutdown();
    return 0;
}
