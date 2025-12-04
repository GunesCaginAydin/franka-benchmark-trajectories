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

void saturatedTorque(std::array<double, 7>& tau_cmd, const std::array<double, 7>& limits) {
    for (size_t i = 0; i < 7; ++i) {
        if (tau_cmd[i] > limits[i]) {
            tau_cmd[i] = limits[i];
        } else if (tau_cmd[i] < -limits[i]) {
            tau_cmd[i] = -limits[i];
        }
    }
}

int main() {
  auto robot_ip = "169.254.243.49";
  // Task related params
  const double radius_t{0.2};
  const double period_t{1500.0};
  double time{0.0};
  Eigen::VectorXd tau_d_prev = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd tau_d_filtered = Eigen::VectorXd::Zero(7);
  const double alpha{0.05}; //smorzamento della coppia iniziale dato che rileva salto
  Eigen::Matrix<double, 6, 1> error_sum;
  const double MAX_TIME = 10.0; //5[s] since we use toSec()
  /* Log struct
  --> this will contain the next torque (applied at time t+1) and the actual end effector pos, orn and joint positions (at time t),
      so the position of the torque applied will be recorded at the following line in the log file
  */
  struct LogEntry {
      std::array<double, 7> torque;
      Eigen::Vector3d ee_position;
      Eigen::Quaterniond ee_orientation;
      std::array<double, 7> joint_positions;
  };
  std::vector<LogEntry> log; //array where I save the states

  // Compliance parameters
  const double translational_stiffness{150.0};
  const double rotational_stiffness{10.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 4.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);
  const double ki{0.35}; //between 0.2-0.5 --> 0.35

  try {
    // connect to robot
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    // setting limits (taken from https://github.com/frankarobotics/franka_ros/blob/develop/franka_description/robots/panda/joint_limits.yaml, same as URDF)
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

    // moving to mids configuration, used https://github.com/frankarobotics/libfranka/blob/main/examples/examples_common.cpp to recreate MotionGenerator
    MotionGenerator motion_generator(0.2, joint_middle); //moving at 20% max speed to joint_middle
    
    std::cout << "WARNING: This example will move the robot! "
                  << "Please make sure to have the user stop button at hand!" << std::endl
                  << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator); //blocking call, until the pose is not reached it will not proceed
    std::cout << "Finished moving to initial joint configuration." << std::endl
                << "Press Enter to start the task..." << std::endl;
    std::cin.ignore();

    franka::RobotState initial_state = robot.readOnce();

    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Vector3d position_d_start(initial_transform.translation());
    // position_d_start(2) = position_d_start(2) - 0.2; // VS UP only
    Eigen::Quaterniond orientation_d(initial_transform.rotation());

    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration period /*duration*/) -> franka::Torques {
        // get state variables
        std::array<double, 7> coriolis_array = model.coriolis(robot_state);
        std::array<double, 42> jacobian_array =
            model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
        double dt = period.toSec();
        time += dt;
        int itr = (time / dt)-1;

        // convert to Eigen
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.rotation());

        // compute error to desired equilibrium pose FC
        // position error
        position_d(0) = position_d_start(0) // x
        position_d(1) = position_d_start(1) + sin(itr / period_t) * radius_t;  // y
        position_d(2) = position_d_start(2) + cos(itr / period_t) * radius_t;  // z

        Eigen::Matrix<double, 6, 1> error;
        error.head(3) << position - position_d;
        error_sum.head(3) += position - position_d;

        // orientation error
        // "difference" quaternion
        if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
        }
        // "difference" quaternion
        Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        // Transform to base frame
        error.tail(3) << -transform.rotation() * error.tail(3);

        // compute control
        Eigen::VectorXd tau_task(7), tau_d(7);

        // PID Control Law
        tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq) + ki * error_sum * dt);
        tau_d << tau_task + coriolis;

        tau_d_filtered = (1.0 - alpha) * tau_d_prev + alpha * tau_d;
                    
        tau_d_prev = tau_d_filtered;

        std::array<double, 7> tau_d_array{};
        Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d_filtered;

        // clipping torque inside the limits
        saturatedTorque(tau_d_array, joint_effort_limits);

        log.push_back(LogEntry{tau_d_array, position, orientation, robot_state.q});

        if (time > MAX_TIME) {
            return franka::MotionFinished(franka::Torques(tau_d_array)); //stops the callback loop
        }

        return tau_d_array;
    };

    robot.control(impedance_control_callback);

    //end of control loop, saving the log
    try {
        std::ostringstream filename;
        filename << "log_VS_radius" << radius_t << "_period" << period_t 
        << "_alpha" << alpha << "_durationS" << MAX_TIME 
        << "_stiffTransl" << translational_stiffness << "_stiffRot" << rotational_stiffness << "_ki" << ki <<".csv" ;

        std::ofstream file(filename.str());
        if (!file.is_open()) {
            throw std::runtime_error("Can't open the file");
        }

        // header
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
            file.seekp(-1, std::ios_base::cur);  // removes the last ',' added on the last joint pos
            file << "\n";
        }

        file.close();
        std::cout << "File saved.\n";
    } catch (const std::exception& e) {
        std::cerr << "Error during the file save: " << e.what() << std::endl;
        return -1;
    }

  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
