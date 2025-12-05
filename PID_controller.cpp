#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "franka_custom_controller/examples_common.h"

// TORQUE SATURATION CHECK
void saturatedTorque(std::array<double, 7>& tau_cmd, const std::array<double, 7>& limits) {
    for (size_t i = 0; i < 7; ++i) {
        if (tau_cmd[i] > limits[i]) {
            tau_cmd[i] = limits[i];
        } else if (tau_cmd[i] < -limits[i]) {
            tau_cmd[i] = -limits[i];
        }
    }
}

// MAIN CONTROL LOOP
int main() {
  auto robot_ip = "169.254.243.49"; // may need to change this

  std::default_random_engine generator;
  std::uniform_real_distribution<double> radii(0.1,1.0);
  std::uniform_int_distribution<int> radii(20,80);
  const double radius_t = radii(generator);
  const double period_t = periods(generator);
  double time{0.0};
  const double MAX_TIME = 10.0; 
  const double alpha{0.05};

  Eigen::VectorXd tau_d_prev = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd tau_d_filtered = Eigen::VectorXd::Zero(7);
  Eigen::Matrix<double, 6, 1> error_sum;

  struct LogEntry {
      std::array<double, 7> torque;
      Eigen::Vector3d ee_position;
      Eigen::Quaterniond ee_orientation;
      std::array<double, 7> joint_positions;
  };
  std::vector<LogEntry> log;

  Eigen::MatrixXd kp(6, 6), kd(6, 6), ki(6,6);
  kp.setZero();
  kp.topLeftCorner(3,3) << (MatrixXd::Random(3,3)+MatrixXd::eye(3,3))*20 + MatrixXd::eye(3,3)*40 
  kp.bottomRightCorner(3,3) << (MatrixXd::Random(3,3)+MatrixXd::eye(3,3))*20 + MatrixXd::eye(3,3)*40 

  kd.setZero();
  kd.topLeftCorner(3,3) << (MatrixXd::Random(3,3)+MatrixXd::eye(3,3))*20 + MatrixXd::eye(3,3)*40 
  kd.bottomRightCorner(3,3) << (MatrixXd::Random(3,3)+MatrixXd::eye(3,3))*20 + MatrixXd::eye(3,3)*40 

  ki.setZero();
  ki.topLeftCorner(3,3) << (MatrixXd::Random(3,3)+MatrixXd::eye(3,3))*20 + MatrixXd::eye(3,3)*40 
  ki.bottomRightCorner(3,3) << (MatrixXd::Random(3,3)+MatrixXd::eye(3,3))*20 + MatrixXd::eye(3,3)*40 

  try {
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();

    constexpr std::array<double, 7> joint_position_lower = {
        -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
    constexpr std::array<double, 7> joint_position_upper = {
        2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973};
    constexpr std::array<double, 7> joint_effort_limits = {
        87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};

    std::array<double, 7> joint_middle{};
    for (size_t i = 0; i < 7; ++i) {
        joint_middle[i] = (joint_position_lower[i] + joint_position_upper[i]) / 2.0;
    }
    joint_middle[6] = 0.785398;

    MotionGenerator motion_generator(0.2, joint_middle);
    
    std::cout << "WARNING: This example will move the robot! "
                  << "Please make sure to have the user stop button at hand!" << std::endl
                  << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl
                << "Press Enter to start the task..." << std::endl;
    std::cin.ignore();

    franka::RobotState initial_state = robot.readOnce();

    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Vector3d position_d_start(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.rotation());

    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration period /*duration*/) -> franka::Torques {
        std::array<double, 7> coriolis_array = model.coriolis(robot_state);
        std::array<double, 42> jacobian_array =
            model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
        double dt = period.toSec();
        time += dt;
        int itr = (time / dt)-1;
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.rotation());

        switch (task)
        {
        case "VS":
            position_d(0) = position_d_start(0) + sin(itr / period_t) * radius_t;
            position_d(1) = position_d_start(1) + cos(itr / period_t) * radius_t;
            position_d(2) = position_d_start(2) - 0.1 + sign * itr / num_itr;
            break;
        
        case "FS":
            position_d(0) = position_d_start(0) + sin(itr / period_t) * radius_t;
            position_d(1) = position_d_start(1) + cos(itr / period_t) * radius_t;
            position_d(2) = position_d_start(2) - 0.1 + 0.2 * itr / num_itr;
            break;

        case "FC":
            position_d(0) = position_d_start(0)
            position_d(1) = position_d_start(1) + sin(itr / period_t) * radius_t;
            position_d(2) = position_d_start(2) + cos(itr / period_t) * radius_t;
            break;

        case "PULL":
            position_d(0) = position_d_start(0)
            position_d(1) = position_d_start(1)
            position_d(2) = position_d_start(2)
            break;

        case "PUSH":
            position_d(0) = position_d_start(0)
            position_d(1) = position_d_start(1)
            position_d(2) = position_d_start(2)
            break;

        default:
            position_d(0) = position_d_start(0)
            position_d(1) = position_d_start(1) + sin(itr / period_t) * radius_t;
            position_d(2) = position_d_start(2) + cos(itr / period_t) * radius_t;
            break;
        }

        // POS ERROR
        Eigen::Matrix<double, 6, 1> error;
        error.head(3) << position_d - position;
        error_sum.head(3) += position_d - position;

        // ORN ERROR
        if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
        }
        Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        error.tail(3) << -transform.rotation() * error.tail(3);

        // TOTAL ERROR
        Eigen::VectorXd tau_task(7), tau_d(7);

        // PID CONTROLLER
        tau_task << jacobian.transpose() * (kp * error + kd * error / dt + ki * error_sum * dt);
        tau_d << tau_task + coriolis;
        tau_d_filtered = (1.0 - alpha) * tau_d_prev + alpha * tau_d;    
        tau_d_prev = tau_d_filtered;
        std::array<double, 7> tau_d_array{};
        Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d_filtered;
        saturatedTorque(tau_d_array, joint_effort_limits);
        log.push_back(LogEntry{tau_d_array, position, orientation, robot_state.q});

        if (time > MAX_TIME) {
            return franka::MotionFinished(franka::Torques(tau_d_array));
        }
        return tau_d_array;
    };

    robot.control(impedance_control_callback);

    try {
        std::ostringstream filename;
        filename << "log" << task << task_params << PID_params <<".csv"

        std::ofstream file(filename.str());
        if (!file.is_open()) {
            throw std::runtime_error("Can't open the file");
        }

        file << "tau0,tau1,tau2,tau3,tau4,tau5,tau6,"
            << "px,py,pz,"
            << "qx,qy,qz,qw,"
            << "q0,q1,q2,q3,q4,q5,q6\n";

        for (const auto& entry : log) {
            for (double tau : entry.torque) file << tau << ",";
            file << entry.ee_position(0) << ",";
            file << entry.ee_position(1) << ",";
            file << entry.ee_position(2) << ",";
            file << entry.ee_orientation.x() << ","
                << entry.ee_orientation.y() << ","
                << entry.ee_orientation.z() << ","
                << entry.ee_orientation.w() << ",";
            for (double q : entry.joint_positions) file << q << ",";
            file.seekp(-1, std::ios_base::cur); 
            file << "\n";
        }

        file.close();
        std::cout << "File saved.\n";
    } catch (const std::exception& e) {
        std::cerr << "Error during the file save: " << e.what() << std::endl;
        return -1;
    }

  } catch (const franka::Exception& ex) {
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
