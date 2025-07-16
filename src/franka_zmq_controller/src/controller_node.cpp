#include <franka/robot.h>
#include <franka/exception.h>
#include <iostream>
#include <array>
#include <string>
#include <zmq.hpp>

// ===== CONFIGURATION =====
const std::string robot_ip    = "10.0.2.103";         // follower robot IP
const std::string pos_cmd_sub = "tcp://10.0.2.105:2098"; // leader q subscriber
const std::string torque_pub  = "tcp://*:3087";       // follower torque publisher
const std::string state_pub   = "tcp://*:3099";       // follower state publisher
// =========================

int main() {
  try {
    // 1) Connect to Franka Robot
    franka::Robot robot(robot_ip);

    // 2) Set collision behavior thresholds (safety)
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}
    );

    // 3) ZeroMQ Context + Sockets
    zmq::context_t ctx(1);

    // --- Subscriber: Leader joint angles ---
    zmq::socket_t sub(ctx, ZMQ_SUB);
    sub.connect(pos_cmd_sub);
    sub.set(zmq::sockopt::subscribe, "");

    // --- Publisher: Torque command ---
    zmq::socket_t pub_tau(ctx, ZMQ_PUB);
    pub_tau.bind(torque_pub);

    // --- Publisher: Follower joint state ---
    zmq::socket_t pub_state(ctx, ZMQ_PUB);
    pub_state.bind(state_pub);

    // Buffer for latest leader q
    std::array<double, 7> latest_leader_q{};
    bool have_leader_data = false;

    std::cout << "[INFO] Starting torque control loop..." << std::endl;

    // 4) Torque control loop
    robot.control([&](const franka::RobotState& state,
                      franka::Duration /*period*/) -> franka::Torques {

      // ---- Try receive latest leader q (non-blocking) ----
      zmq::message_t msg;
      zmq::recv_result_t res = sub.recv(msg, zmq::recv_flags::dontwait);
      if (res && msg.size() == sizeof(double) * 7) {
        std::memcpy(latest_leader_q.data(), msg.data(), sizeof(double) * 7);
        have_leader_data = true;
      }

      // ---- PD Controller Gains ----
      std::vector<double> Kp = {30, 30, 30, 30, 15, 15, 15};
      std::vector<double> Kd = {0.5, 0.5, 0.5, 0.5, 0.25, 0.25, 0.25};

      std::array<double,7> tau{};

      if (have_leader_data) {
        // ---- Compute torque command ----
        for (size_t i = 0; i < 7; i++) {
          double pos_error = latest_leader_q[i] - state.q[i];   // leader - follower
          double vel_error = -state.dq[i];                      // want zero velocity
          tau[i] = Kp[i] * pos_error + Kd[i] * vel_error;
        }
      } else {
        // No leader data yet → hold current position (zero torque)
        tau.fill(0.0);
      }

      // ---- Publish torque command ----
      pub_tau.send(zmq::buffer(tau), zmq::send_flags::dontwait);

      // ---- Publish follower current joint state ----
      pub_state.send(zmq::buffer(state.q), zmq::send_flags::dontwait);

      return tau;
    });

    return 0;

  } catch (const franka::Exception& e) {
    std::cerr << "Franka Exception: " << e.what() << std::endl;
    return -1;
  }
}
