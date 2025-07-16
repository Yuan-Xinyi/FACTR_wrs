#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/active_motion_generator.h>

#include <iostream>
#include <array>
#include <string>
#include <zmq.hpp>
#include <iomanip>

const std::string robot_ip         = "10.0.2.103";
const std::string pos_cmd_sub      = "tcp://10.0.2.105:2098";
const std::string torque_pub       = "tcp://*:3087";
const std::string state_pub        = "tcp://*:3099";

int main() {
  try {
    franka::Robot robot(robot_ip);
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}
    );

    zmq::context_t ctx(1);
    zmq::socket_t sub(ctx, ZMQ_SUB);
    sub.connect(pos_cmd_sub);
    sub.set(zmq::sockopt::subscribe, "");

    zmq::socket_t pub_tau(ctx, ZMQ_PUB);
    pub_tau.bind(torque_pub);
    zmq::socket_t pub_state(ctx, ZMQ_PUB);
    pub_state.bind(state_pub);

    std::array<double, 7> latest_q{}, smoothed_q{};
    const double alpha = 0.02;
    const double max_speed = 0.001;

    auto motion_cb = [&](const franka::RobotState& state, franka::Duration period)
        -> franka::JointPositions {
      double dt = std::max(period.toSec(), 0.001);
      static bool init = false;
      if (!init) {
        latest_q = smoothed_q = state.q;
        init = true;
      }

      zmq::message_t msg;
      if (sub.recv(msg, zmq::recv_flags::dontwait) && msg.size() == sizeof(latest_q)) {
        std::memcpy(latest_q.data(), msg.data(), sizeof(latest_q));
      }

      for (int i = 0; i < 7; ++i) {
        smoothed_q[i] = (1 - alpha) * smoothed_q[i] + alpha * latest_q[i];
        double delta = smoothed_q[i] - state.q[i];
        double limit = max_speed * dt;
        if (std::abs(delta) > limit) {
          delta = std::copysign(limit, delta);
        }
        smoothed_q[i] = state.q[i] + delta;
      }

      // Publish external torque
      zmq::message_t tau_msg(sizeof(double) * 7);
      std::memcpy(tau_msg.data(), state.tau_ext_hat_filtered.data(), sizeof(double) * 7);
      pub_tau.send(tau_msg, zmq::send_flags::dontwait);

      // Publish joint state (q + dq)
      zmq::message_t state_msg(sizeof(double) * 14);
      std::memcpy(state_msg.data(), state.q.data(), sizeof(double) * 7);
      std::memcpy(static_cast<char*>(state_msg.data()) + sizeof(double) * 7,
                  state.dq.data(), sizeof(double) * 7);
      pub_state.send(state_msg, zmq::send_flags::dontwait);

      return franka::JointPositions(smoothed_q);
    };

    // ✅ 使用 ActiveMotionGenerator 包装 motion_cb
    franka::ActiveMotionGenerator<franka::JointPositions> motion_generator(motion_cb);

    // ✅ 启动 motion generator 控制循环
    robot.control(moti-on_generator);

  } catch (const franka::Exception& e) {
    std::cerr << "[ERROR] " << e.what() << std::endl;
    return -1;
  }
  return 0;
}
