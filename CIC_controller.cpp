#include <array>
#include <cmath>
#include <random>
#include <chrono>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>

// ignore error warnings if eigen is present
#include <Eigen/Dense> 
// ignore error warnings if libfranka is present
#include <franka/duration.h> 
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
// ignore error warnings if include folder has examples_common.h, examples_common.cpp
#include "include/examples_common.h" 

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
int main(int argc, char** argv) {
    int num_trials = std::stoi(argv[2]);
    std::string task_type = argv[1];
    for (int i = 0; i < num_trials; ++i) {
        std::cout << "Iteration: " << i << std::endl;
        auto robot_ip = "169.254.243.49"; // may need to change this

        std::default_random_engine generator;
        generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<double> radii(0.1,1.0);
        std::uniform_int_distribution<int> periods(20,80);
        std::bernoulli_distribution sign(0.5);
        const double radius_t = radii(generator);
        const double period_t = periods(generator);
        const int sign_t = sign(generator) ? 1 : -1;
        double time{0.0};
        const double MAX_TIME = 4.0; 
        const double alpha{0.05};
        const int num_itr = 4000;

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

        Eigen::VectorXd translational_stiffness_array(3, 1), rotational_stiffness_array(3, 1);
        Eigen::MatrixXd stiffness(6, 6), damping(6,6);

        translational_stiffness_array = Eigen::VectorXd::Random(3, 1)*10 + Eigen::VectorXd::Constant(3, 40.0);
        rotational_stiffness_array = Eigen::VectorXd::Random(3, 1)*2.5 + Eigen::VectorXd::Constant(3, 7.5);    

        stiffness.topLeftCorner(3, 3) = translational_stiffness_array.asDiagonal();
        stiffness.bottomRightCorner(3, 3) = rotational_stiffness_array.asDiagonal();

        damping.topLeftCorner(3, 3) = (4.0 * translational_stiffness_array.array().sqrt()).matrix().asDiagonal();
        damping.bottomRightCorner(3, 3) = (2.0 * rotational_stiffness_array.array().sqrt()).matrix().asDiagonal();  

        const double ki{0.35};

        std::cout
        << "The randomized controller parameters are:\n"
        << "Nominal Translational Stiffness: " << translational_stiffness_array.transpose() << "\n"
        << "Nominal Rotational Stiffness: " << rotational_stiffness_array.transpose() << "\n"
        << "Nominal Integral Gain: " << ki << "\n"
        << "Stiffness Matrix:\n" << stiffness << "\n"
        << "Damping Matrix:\n" << damping <<
        std::endl;

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
                joint_middle[i] = (joint_position_lower[i] + joint_position_upper[i]) / 2.0
                + 0.2 * (joint_position_upper[i] - joint_position_lower[i])*((double)generator()/generator.max() - 0.5);
            }
            // joint_middle[6] = 0.785398 ;

            std::cout << "Initial Joint Configuration:" << std::endl;
            for (const auto& joint : joint_middle) {
                std::cout << joint << " ";
            }
            std::cout << std::endl;
            MotionGenerator motion_generator(0.2, joint_middle);
            
            std::cout << "Moving the robot to initial joint configuration." << std::endl;
            robot.control(motion_generator);
            std::cout << "Finished moving to initial joint configuration." << std::endl;

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

                if (task_type == "VS") {
                    position_d(0) = position_d_start(0) + sin(itr / period_t) * radius_t;
                    position_d(1) = position_d_start(1) + cos(itr / period_t) * radius_t;
                    position_d(2) = position_d_start(2) - 0.1 + sign_t * itr / num_itr;
                }
                else if (task_type == "FS") {
                    position_d(0) = position_d_start(0) + sin(itr / period_t) * radius_t;
                    position_d(1) = position_d_start(1) + cos(itr / period_t) * radius_t;
                    position_d(2) = position_d_start(2) - 0.1 + 0.2 * itr / num_itr;
                }
                else if (task_type == "FC") {
                    position_d(0) = position_d_start(0);
                    position_d(1) = position_d_start(1) + sin(itr / period_t) * radius_t;
                    position_d(2) = position_d_start(2) + cos(itr / period_t) * radius_t;
                }
                else if (task_type == "PULL") { // NOT YET IMPLEMENTED
                    std::cout << "PULL task is not yet implemented." << std::endl;
                }
                else if (task_type == "PUSH") { // NOT YET IMPLEMENTED
                    std::cout << "PUSH task is not yet implemented." << std::endl;
                }
                else { // FC by default
                    position_d(0) = position_d_start(0);
                    position_d(1) = position_d_start(1) + sin(itr / period_t) * radius_t;
                    position_d(2) = position_d_start(2) + cos(itr / period_t) * radius_t;
                }

                // POS ERROR
                Eigen::Matrix<double, 6, 1> error;
                error.head(3) << position_d - position;
                error_sum.head(3) += error.head(3);

                // ORN ERROR
                if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
                    orientation.coeffs() << -orientation.coeffs();
                }
                Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
                error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
                error.tail(3) << transform.rotation() * error.tail(3);

                // TOTAL ERROR
                Eigen::VectorXd tau_task(7), tau_d(7);

                // PI + Joint Velocity CONTROLLER
                tau_task << jacobian.transpose() * (stiffness * error + damping * (jacobian * dq) + ki * error_sum * dt);
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

        } catch (const franka::Exception& ex) {
            std::cout << ex.what() << std::endl;
        }

        try {

        std::string period_str = std::to_string(period_t);
        std::string radius_str = std::to_string(radius_t);
        std::string kpt_str = std::to_string(translational_stiffness_array.mean());
        std::string kpr_str = std::to_string(rotational_stiffness_array.mean());
        std::string ki_str = std::to_string(ki);

        std::ostringstream filename;
        filename << "data/" << "log" << argv[1] << i << "_period" << period_t << "_radius" << radius_t << "_Kp" 
                << kpt_str << "_Kr" << kpr_str << "_Ki" << ki_str <<".csv";

        std::ofstream file(filename.str());
        file.open(filename.str());
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

    }
  return 0;
}
