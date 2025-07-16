#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/exception.h>
#include <iostream>
#include <array>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <zmq.hpp>

// ===== CONFIGURATION =====
const std::string robot_ip    = "10.0.2.103";
const std::string pos_cmd_sub = "tcp://10.0.2.105:2098";
const std::string torque_pub  = "tcp://*:3087";
const std::string state_pub   = "tcp://*:3099";
// =========================

int main() {
  try {
    // 1) Initialize Robot and Gripper interfaces
    franka::Robot robot(robot_ip);
    franka::Gripper gripper(robot_ip);  // thread-safe according to libfranka docs :contentReference[oaicite:1]{index=1}

    // 2) Set collision behavior thresholds
    robot.setCollisionBehavior(
        {{20,20,20,20,20,20,20}},
        {{20,20,20,20,20,20,20}},
        {{20,20,20,20,20,20}},
        {{20,20,20,20,20,20}}
    );

    // 3) Setup ZeroMQ
    zmq::context_t ctx(1);
    zmq::socket_t sub(ctx, ZMQ_SUB);
    sub.connect(pos_cmd_sub);
    sub.set(zmq::sockopt::subscribe, "");
    zmq::socket_t pub_tau(ctx, ZMQ_PUB);
    pub_tau.bind(torque_pub);
    zmq::socket_t pub_state(ctx, ZMQ_PUB);
    pub_state.bind(state_pub);

    // Shared state variables
    std::array<double,7> latest_leader_q{};
    std::atomic<double> latest_leader_gripper{0.0};
    std::atomic<bool> have_leader_data{false};
    std::atomic<bool> stop_thread{false};

    // 4) Gripper control in separate thread
    std::thread gripper_thread([&]() {
      double last_cmd = -1.0;
      while (!stop_thread.load()) {
        if (have_leader_data.load()) {
          double cmd = latest_leader_gripper.load();
          if (std::abs(cmd - last_cmd) > 1e-4) {
            bool ok = false;
            try {
              ok = gripper.move(cmd, 0.1);
            } catch (const franka::Exception &e) {
              std::cerr << "[GRIPPER ERROR] " << e.what() << std::endl;
            }
            if (ok) {
              std::cout << "[GRIPPER] Moved to " << cmd << " m" << std::endl;
              last_cmd = cmd;
            } else {
              std::cerr << "[GRIPPER WARNING] Move failed or returned false" << std::endl;
            }
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    });

    std::cout << "[INFO] Starting torque control callback..." << std::endl;

    // 5) Real-time torque control loop
    robot.control([&](const franka::RobotState& state, franka::Duration) {
      // Receive leader commands (non-blocking)
      zmq::message_t msg;
      if (sub.recv(msg, zmq::recv_flags::dontwait) &&
          msg.size() == sizeof(double)*8) {
        std::array<double,8> buf;
        std::memcpy(buf.data(), msg.data(), sizeof(buf));
        std::copy(buf.begin(), buf.begin()+7, latest_leader_q.begin());
        latest_leader_gripper.store(buf[7]);
        have_leader_data.store(true);
      }

      // PD control for arm joints
      const std::array<double,7> Kp{30,30,30,30,15,15,15};
      const std::array<double,7> Kd{0.5,0.5,0.5,0.5,0.25,0.25,0.25};
      const std::array<double,7> max_tau{100,100,100,100,100,100,100};
      std::array<double,7> tau{};
      if (have_leader_data.load()) {
        for (size_t i = 0; i < 7; ++i) {
          double pos_err = latest_leader_q[i] - state.q[i];
          double vel_err = -state.dq[i];
          tau[i] = std::clamp(Kp[i]*pos_err + Kd[i]*vel_err, -max_tau[i], max_tau[i]);
        }
      } else {
        tau.fill(0.0);
      }

      // Publish torque command and joint state
      pub_tau.send(zmq::buffer(tau), zmq::send_flags::dontwait);
      pub_state.send(zmq::buffer(state.q), zmq::send_flags::dontwait);
      return franka::Torques{tau};
    });

    // Cleanup
    stop_thread.store(true);
    gripper_thread.join();
    return 0;

  } catch (const franka::Exception& e) {
    std::cerr << "[ERROR] Franka exception: " << e.what() << std::endl;
    return -1;
  }
}
