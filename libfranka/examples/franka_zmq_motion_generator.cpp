#include <franka/robot.h>
#include <franka/exception.h>
#include <iostream>
#include <array>
#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <chrono>
#include <iomanip>
#include <zmq.hpp>
#include "examples_common.h"

const std::string robot_ip = "10.0.2.103";
const std::string pos_cmd_sub_address = "tcp://10.0.2.105:2098";
const std::string torque_pub_address   = "tcp://*:3087";
const std::string state_pub_address    = "tcp://*:3099";

int main() {
  try {
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    zmq::context_t context(1);
    zmq::socket_t pos_sub(context, ZMQ_SUB);
    pos_sub.connect(pos_cmd_sub_address);
    pos_sub.set(zmq::sockopt::subscribe, "");

    zmq::socket_t torque_pub(context, ZMQ_PUB);
    torque_pub.bind(torque_pub_address);

    zmq::socket_t state_pub(context, ZMQ_PUB);
    state_pub.bind(state_pub_address);

    std::cout << "[INFO] Franka Motion Server started.\n"
              << "  Subscribing position  @ " << pos_cmd_sub_address << "\n"
              << "  Publishing torque     @ " << torque_pub_address << "\n"
              << "  Publishing jointstate @ " << state_pub_address << std::endl;

    std::queue<std::array<double, 7>> position_queue;
    std::mutex queue_mutex;
    std::mutex robot_control_mutex;
    const size_t max_queue_size = 20;

    std::thread receiver_thread([&]() {
      while (true) {
        zmq::message_t msg;
        if (pos_sub.recv(msg, zmq::recv_flags::none)) {
          if (msg.size() == sizeof(double) * 7) {
            std::array<double, 7> new_position;
            std::memcpy(new_position.data(), msg.data(), sizeof(double) * 7);
            {
              std::lock_guard<std::mutex> lock(queue_mutex);
              if (position_queue.size() < max_queue_size) {
                position_queue.push(new_position);
              } else {
                std::cout << "[DROP] Queue full. Dropping incoming command." << std::endl;
              }
            }
            std::cout << "[RECV] Enqueued Position: ";
            for (double q : new_position)
              std::cout << std::fixed << std::setprecision(4) << q << " ";
            std::cout << std::endl;
          } else {
            std::cerr << "[WARN] Invalid joint command size: " << msg.size() << std::endl;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });

    while (true) {
      std::array<double, 7> target_position;
      bool has_target = false;

      {
        std::lock_guard<std::mutex> lock(queue_mutex);
        if (!position_queue.empty()) {
          target_position = position_queue.front();
          position_queue.pop();
          has_target = true;
        }
      }

      if (has_target) {
        std::lock_guard<std::mutex> lock(robot_control_mutex);
        try {
          std::cout << "[EXEC] Executing MotionGenerator...\n";
          MotionGenerator motion_generator(0.5, target_position);
          robot.control(motion_generator);
          std::cout << "[DONE] Motion completed.\n";

          franka::RobotState state = robot.readOnce();

          zmq::message_t torque_msg(sizeof(double) * 7);
          std::memcpy(torque_msg.data(), state.tau_ext_hat_filtered.data(), sizeof(double) * 7);
          torque_pub.send(torque_msg, zmq::send_flags::dontwait);

          zmq::message_t state_msg(sizeof(double) * 14);
          std::memcpy(state_msg.data(), state.q.data(), sizeof(double) * 7);
          std::memcpy(static_cast<char*>(state_msg.data()) + sizeof(double) * 7,
                      state.dq.data(), sizeof(double) * 7);
          state_pub.send(state_msg, zmq::send_flags::dontwait);

        } catch (const std::exception& e) {
          std::cerr << "[ERROR] MotionGenerator threw exception: " << e.what() << std::endl;

          if (std::string(e.what()).find("Reflex") != std::string::npos) {
            std::cerr << "[WARN] Reflex mode detected. Trying automatic recovery..." << std::endl;
            try {
              robot.automaticErrorRecovery();
              std::cerr << "[INFO] Recovery successful. Clearing queue.\n";

              std::lock_guard<std::mutex> lock(queue_mutex);
              std::queue<std::array<double, 7>> empty;
              std::swap(position_queue, empty);
            } catch (const std::exception& recovery_error) {
              std::cerr << "[ERROR] Recovery failed: " << recovery_error.what() << std::endl;
            }
          }
        } catch (...) {
          std::cerr << "[ERROR] Unknown exception in robot.control(MotionGenerator)" << std::endl;
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }

    receiver_thread.join();

  } catch (const franka::Exception& e) {
    std::cerr << "[ERROR] LibFranka Exception: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}
