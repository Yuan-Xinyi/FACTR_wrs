#include <franka/robot.h>
#include <franka/exception.h>
#include <iostream>
#include <array>
#include <string>
#include <zmq.hpp>
#include <thread>
#include <chrono>
#include <iomanip>  // for std::setprecision

const std::string robot_ip = "10.0.2.102";
const std::string pos_cmd_sub_address = "tcp://10.0.2.105:2098";  // 接收位置指令
const std::string torque_pub_address   = "tcp://*:3087";          // 发布 torque
const std::string state_pub_address    = "tcp://*:3099";          // 发布 joint state（q, dq）

int main() {
  try {
    franka::Robot robot(robot_ip);
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}
    );

    zmq::context_t context(1);

    // 订阅来自 Python 的位置指令
    zmq::socket_t pos_sub(context, ZMQ_SUB);
    pos_sub.connect(pos_cmd_sub_address);
    pos_sub.set(zmq::sockopt::subscribe, "");

    // 发布外部力矩
    zmq::socket_t torque_pub(context, ZMQ_PUB);
    torque_pub.bind(torque_pub_address);

    // 发布 joint 状态（q 和 dq）
    zmq::socket_t state_pub(context, ZMQ_PUB);
    state_pub.bind(state_pub_address);

    std::cout << "[INFO] Franka follower started.\n"
              << "  Subscribing position  @ " << pos_cmd_sub_address << "\n"
              << "  Publishing torque     @ " << torque_pub_address << "\n"
              << "  Publishing jointstate @ " << state_pub_address << std::endl;

    // 初始位置设为当前状态
    franka::RobotState initial_state = robot.readOnce();
    std::array<double, 7> latest_position = initial_state.q;

    robot.control([&](const franka::RobotState& state, franka::Duration) -> franka::JointPositions {
      // === 接收位置指令 ===
      zmq::message_t msg;
      if (pos_sub.recv(msg, zmq::recv_flags::dontwait)) {
        if (msg.size() == sizeof(double) * 7) {
          std::memcpy(latest_position.data(), msg.data(), sizeof(double) * 7);
          std::cout << "[RECV] Position: ";
          for (double q : latest_position)
            std::cout << std::fixed << std::setprecision(4) << q << " ";
          std::cout << std::endl;
        } else {
          std::cerr << "[WARN] Received message with wrong size: " << msg.size() << std::endl;
        }
      }

      // === 发布力矩 ===
      zmq::message_t torque_msg(sizeof(double) * 7);
      std::memcpy(torque_msg.data(), state.tau_ext_hat_filtered.data(), sizeof(double) * 7);
      torque_pub.send(torque_msg, zmq::send_flags::dontwait);

      std::cout << "[SEND] Torque: ";
      for (double tau : state.tau_ext_hat_filtered)
        std::cout << std::fixed << std::setprecision(4) << tau << " ";
      std::cout << std::endl;

      // === 发布 joint 状态（q, dq） ===
      zmq::message_t state_msg(sizeof(double) * 14);
      std::memcpy(state_msg.data(), state.q.data(), sizeof(double) * 7);
      std::memcpy(static_cast<char*>(state_msg.data()) + sizeof(double) * 7,
                  state.dq.data(), sizeof(double) * 7);
      state_pub.send(state_msg, zmq::send_flags::dontwait);

      std::cout << "[SEND] JointState (q, dq): ";
      for (int i = 0; i < 7; ++i)
        std::cout << std::fixed << std::setprecision(4) << state.q[i] << "/" << state.dq[i] << " ";
      std::cout << std::endl;

      return franka::JointPositions(latest_position);
    });

  } catch (const franka::Exception& e) {
    std::cerr << "[ERROR] LibFranka Exception: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}
