// SPDX-FileCopyrightText: Copyright (c) 2026 HTC Corporation. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef VIVE_TRACKER_H
#define VIVE_TRACKER_H

#include <stdint.h>

// Linux with GCC/Clang
#if defined(VIVETRACKER_BUILD_SHARED) && defined(__GNUC__) && __GNUC__ >= 4
  #define VIVETRACKER_API __attribute__((visibility("default")))
#else
  #define VIVETRACKER_API
#endif

namespace vive {

enum VIVETRACKER_API TrackerState : uint8_t {
  Unpaired = 0, NA = 0, // invisible
  Pairing = 1,
  Paired = 2,
  Connecting = 3,   // likly the tracker is off, press button to wake up.
  Connected = 4,    // tracker data received
  Initialized = 5,  // init but not start

  //
  // tracker subscribers and ipc clients can only see trackers of the following states
  Syncing = 6,      // the client is syncing map from host, or host has no map (setup required).
  BuildMap = 7,     // the host is building map (setup)

  // running with map ready
  Ready = 8
};

constexpr uint32_t TrackerFlag_PoseStateMask = 0x07; // pose state mask
constexpr uint32_t TrackerFlag_Charging      = 1<<3; // since it's not charging when in use, this flag is not useful.
constexpr uint32_t TrackerFlag_PoseStable    = 1<<4; // tracker is stable (completely still, not moving)
constexpr uint32_t TrackerFlag_RotationOnly  = 1<<5; // location isn't updated (lost tracking?)
constexpr uint32_t TrackerFlag_IsHostTracker = 1<<6; // flag for the host tracker
constexpr uint32_t TrackerFlag_SetupRequired = 1<<7; // need to run setup process(build map)

struct VIVETRACKER_API TrackerData {
  //
  // coordinate system follows isaac sim, i.e.
  //   X-axis: Forward
  //   Y-axis: Left
  //   Z-axis: Up
  float Location[3];        // x, y, z in meter
  float Rotation[4];        // quaternion qx, qy, qz, qw
  float Velocity[3];        // m/s
  float AngularVelocity[3]; // rad/s

  int64_t Timestamp_us;     // timestamp in microseconds, i.e. elapsed_time = GetTimestamp() - Timestamp_us
  uint32_t Id;              // tracker index [0, NumTrackers-1]
  uint32_t Flags;           // use TrackerFlag_Mask to get properties
  TrackerState State;       // Syncing, BuildMap or Ready. need setup if first 2s
  uint8_t Button;           // button press=1, button release=0
  uint8_t Clicks;           // button click count, e.g. double-click=2, tripple-click=3, ...
  uint8_t Battery;          // battery power %
};

//
// response to command sent via ITrackerSubscriber::CommandResponse(cmd), or ITrackerIPCClient::CommandResponse(cmd)
struct VIVETRACKER_API TrackerCommandResponse {
  char const* Command;
  char const* Result;

  // all trackers' state resport.
  TrackerState const* TrackerStates; // size = NumTrackers
  uint32_t NumTrackers; // GetMaxTotalTrackers();
};

//
// start and stop server thread. argv/argc for further useage
VIVETRACKER_API bool StartTrackerServer(int argc=0, char** argv=nullptr);
VIVETRACKER_API void StopTrackerServer();

// get current timestamp in microseconds. cf. TrackerData.Timestamp_us
VIVETRACKER_API int64_t GetTimestamp();

// total trackers you can connect to.
VIVETRACKER_API uint32_t GetNumTrackers();

// version string
VIVETRACKER_API char const* GetVersion();

//
// ITrackerSubscriber - Direct server-side callbacks
//
// Callbacks are invoked automatically from the tracker server thread.
// This is the highest priority method for receiving tracker updates.
//
// Usage: (refer ./vivetracker_viewer/tracker_viewer_standalone.cpp for an example.)
//   class MyTracker : public vive::ITrackerSubscriber {
//     void ServerStart() override { ... }
//     void ServerStop() override { ... }
//     void Add(uint32_t id, char const* name) override { ... }
//     void Update(TrackerData const& data) override { ... }
//     void Click(uint32_t id, int button, int clicks) override { ... }
//     void Remove(uint32_t id) override { ... }
//     void CommandResponse(TrackerCommandResponse const& resp) override { ... }
//   };
//
//   int main() {
//     MyTracker tracker;
//     if (vive::StartTrackerServer()) {
//       while (running) {
//         // do some works, callbacks happen automatically
//       }
//       vive::StopTrackerServer();
//     }
//   }
//
class VIVETRACKER_API ITrackerSubscriber {
protected:
  ITrackerSubscriber();

public:
  virtual ~ITrackerSubscriber();

  // server start/stop
  virtual void ServerStart() = 0;
  virtual void ServerStop() = 0;

  // new tracker added
  virtual void Add(uint32_t tracker_id, char const* name) = 0;

  // update tracker status
  virtual void Update(TrackerData const& TrackerData) = 0;

  // tracker button clicks, e.g. a double-click action triggers 4 Click_() calls...
  //  #1 clicks = 1 --> when the 1st click occurs
  //  #2 clicks = 0 --> when the 1st click release
  //  #3 clicks = 2 --> when the 2nd click occurs (short than 400ms since last release)
  //  #4 clicks = 0 --> when the 2nd click release
  virtual void Click(uint32_t tracker_id, int button_id, int clicks) = 0;

  // tracker has beed removed from list (timeout)
  virtual void Remove(uint32_t tracker_id) = 0;

  //
  // tracker command is composed of case insensity c-string + optional 0-based id list.
  // all commands (include unrecognized commands) return tracker states, state before the changes made.
  //  1) Pairing <id-list>
  //  2) Unpair <id-list>
  //  3) Restart <id-list>
  //  4) PowerOff <id-list>
  //  5) Echo [id-list] default: all
  //  6) Setup [id] default: any
  //
  // e.g.
  //  1) "Pairing 0"
  //  2) "Unpair 0, 1, 2, 3, 4",
  //  3) "Restart all"
  bool SendCommand(char const* cmd, int len=-1); // non blocking
  virtual void CommandResponse(TrackerCommandResponse const&) = 0;
};

//
// ITrackerIPCClient - IPC client with manual polling
//
// Connects to tracker server via Unix Domain Socket.
// Requires calling Poll() repeatedly to receive updates.
// Callbacks are invoked from the Poll() function (same thread).
//
// Usage: (refer ./vivetracker_viewer/tracker_viewer.cpp for an example.)
//   class MyTrackerClient : public vive::ITrackerIPCClient {
//     bool server_stop_ {false};
//
//     void ServerStart() override { ... }
//     void ServerStop() override { server_stop=true; ... }
//     void Add(uint32_t id, char const* name) override { ... }
//     void Update(TrackerData const& data) override { ... }
//     void Click(uint32_t id, int button, int clicks) override { ... }
//     void Remove(uint32_t id) override { ... }
//     void CommandResponse(TrackerCommandResponse const& resp) override { ... }
//
//   public:
//     bool Stop() const { return server_stop_; }
//   };
//
//   int main() {
//     MyTrackerClient client;
//     while (!client.Stop()) {
//       client.Poll(); // call every frame
//
//       // do other works or take a nap...
//       std::this_thread::sleep_for(std::chrono::milliseconds(16));
//     }
//   }
//
class VIVETRACKER_API ITrackerIPCClient {
  int socket_{-1};

protected:
  ITrackerIPCClient() = default;
  ITrackerIPCClient(ITrackerIPCClient const&) = delete;
  ITrackerIPCClient& operator=(ITrackerIPCClient const&) = delete;

public:
  virtual ~ITrackerIPCClient() { Disconnect(); };

  // server start/stop
  virtual void ServerStart() = 0;
  virtual void ServerStop() = 0;

  // new tracker added
  virtual void Add(uint32_t tracker_id, char const* name) = 0;

  // update tracker status
  virtual void Update(TrackerData const& TrackerData) = 0;

  // tracker button clicks
  virtual void Click(uint32_t tracker_id, int button_id, int clicks) = 0;

  // tracker has beed removed from list (timeout)
  virtual void Remove(uint32_t tracker_id) = 0;

  // Poll for updates - call every frame. return false if it's not connecting.
  bool Poll();

  // disconnect
  void Disconnect();

  // tracker command and response
  bool SendCommand(char const* cmd, int len=-1);
  virtual void CommandResponse(TrackerCommandResponse const&) = 0;
};

}

#endif
