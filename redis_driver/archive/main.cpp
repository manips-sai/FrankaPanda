// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
// #include <thread>
// #include <signal.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "RedisClient.h"

#include "ButterworthFilter.h"

// redis keys
// - read:
std::string JOINT_TORQUES_COMMANDED_KEY;
// - write:
std::string JOINT_ANGLES_KEY; 
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

// safety
const std::array<double, 7> joint_position_max_default = {2.89, 1.76, 2.89, -0.06, 2.89, 3.75, 2.89};
const std::array<double, 7> joint_position_min_default = {-2.89, -1.76, -2.89, -3.07, -2.89, -0.01, -2.89};
const std::array<double, 7> joint_velocity_limits_default = {2.17, 2.17, 2.17, 2.17, 2.61, 2.61, 2.61};
const std::array<double, 7> joint_torques_limits_default = {87, 87, 87, 87, 12, 12, 12};

// original safety set
std::array<double, 7> joint_position_max = {2.7, 1.6, 2.7, -0.2, 2.7, 3.6, 2.7};
std::array<double, 7> joint_position_min = {-2.7, -1.6, -2.7, -3.0, -2.7, 0.2, -2.7};
std::array<double, 7> joint_velocity_limits = {2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5};
std::array<double, 7> joint_torques_limits = {85, 85, 85, 85, 10, 10, 10};

// // override with new safety set 
// double default_sf = 0.98;  // max violation safety factor 
// for (int i = 0; i < 7; ++i) {
//     joint_position_max[i] = default_sf * joint_position_max_default[i];
//     joint_position_min[i] = default_sf * joint_position_min_default[i];
//     joint_velocity_limits[i] = default_sf * joint_velocity_limits_default[i];
//     joint_torques_limits[i] = default_sf * joint_torques_limits_default[i];
// }

const Eigen::Vector3d monitoring_point_ee_frame = Eigen::Vector3d(0.0, 0.0, 0.15);
const double safety_plane_z_coordinate = 0.28;
const double safety_cylinder_radius = 0.28;
const double safety_cylinder_height = 0.53;

bool safety_mode_flag = false;
bool safety_enabled = false;  // if safety is enabled
int safety_controller_count = 200;
const double kv_safety = 20; 

Eigen::MatrixXd MassMatrix;
std::array<double, 7> tau_cmd_array{};
std::array<double, 7> q_array{};
std::array<double, 7> dq_array{};
std::array<double, 7> tau_sensed_array{};
std::array<double, 7> gravity_vector{};
std::array<double, 7> coriolis{};
std::array<double, 49> M_array{}; 
std::vector<std::array<double, 7>> sensor_feedback;
std::vector<string> key_names;
// bool fDriverRunning = true;
// void sighandler(int sig)
// { fDriverRunning = false; }

unsigned long long counter = 0;

// void redis_transfer(CDatabaseRedisClient* redis_client)
// {
//   while(fDriverRunning)
//   {
//     Eigen::Map<const Eigen::Matrix<double, 7, 7> > MassMatrix(M_array.data());
//     redis_client->setGetBatchCommands(key_names, tau_cmd_array, MassMatrix, sensor_feedback);
//   }
// }

enum Limit {
    SAFE = 0,
    MIN_SOFT,  // soft lower limit (velocity damping only)
    MIN_HARD,  // hard lower limit (velocity damping + position holding)
    MAX_SOFT,  // soft upper limit (velocity damping only)
    MAX_HARD  // hard upper limit (velocity damping + position holding)
};

int main(int argc, char** argv) {

    if(argc < 2)
    {
        std::cout << "Please enter the robot ip as an argument" << std::endl;
        return -1;
    }
    std::string robot_ip = argv[1];

    if(robot_ip == "172.16.0.10")
    {
        JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Clyde::actuators::fgc";
        JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Clyde::sensors::q";
        JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Clyde::sensors::dq";
        JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::Clyde::sensors::torques";
        MASSMATRIX_KEY = "sai2::FrankaPanda::Clyde::sensors::model::massmatrix";
        CORIOLIS_KEY = "sai2::FrankaPanda::Clyde::sensors::model::coriolis";
        ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::Clyde::sensors::model::robot_gravity";    
    }
    else if(robot_ip == "172.16.0.11")
    {
        JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Bonnie::actuators::fgc";
        JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Bonnie::sensors::q";
        JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Bonnie::sensors::dq";
        JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::Bonnie::sensors::torques";
        MASSMATRIX_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::massmatrix";
        CORIOLIS_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::coriolis";
        ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::robot_gravity";       
    }
    else
    {
        JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::actuators::fgc";
        JOINT_ANGLES_KEY  = "sai2::FrankaPanda::sensors::q";
        JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::sensors::dq";
        JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::sensors::torques";
        MASSMATRIX_KEY = "sai2::FrankaPanda::sensors::model::massmatrix";
        CORIOLIS_KEY = "sai2::FrankaPanda::sensors::model::coriolis";
        ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::sensors::model::robot_gravity";        
    }

    // start redis client
    CDatabaseRedisClient* redis_client;
    HiredisServerInfo info;
    info.hostname_ = "127.0.0.1";
    info.port_ = 6379;
    info.timeout_ = { 1, 500000 }; // 1.5 seconds
    redis_client = new CDatabaseRedisClient();
    redis_client->serverIs(info);

    // // set up signal handler
    // signal(SIGABRT, &sighandler);
    // signal(SIGTERM, &sighandler);
    // signal(SIGINT, &sighandler);

    // debug_function(redis_client);

    for(int i=0; i<7; i++)
    {
        tau_cmd_array[i] = 0;
    }

    redis_client->setDoubleArray(JOINT_TORQUES_COMMANDED_KEY, tau_cmd_array, 7);
    // Safety to detect if controller is already running : wait 50 milliseconds
    usleep(50000);
    redis_client->getDoubleArray(JOINT_TORQUES_COMMANDED_KEY, tau_cmd_array, 7);
    for(int i=0; i<7; i++)
    {
        if(tau_cmd_array[i] != 0)
        {
            std::cout << "Stop the controller before runnung the driver\n" << std::endl;
            return -1;
        }
    }

    // prepare batch command
    key_names.push_back(JOINT_TORQUES_COMMANDED_KEY);
    key_names.push_back(MASSMATRIX_KEY);
    key_names.push_back(JOINT_ANGLES_KEY);
    key_names.push_back(JOINT_VELOCITIES_KEY);
    key_names.push_back(JOINT_TORQUES_SENSED_KEY);
    key_names.push_back(ROBOT_GRAVITY_KEY);
    key_names.push_back(CORIOLIS_KEY);

    sensor_feedback.push_back(q_array);
    sensor_feedback.push_back(dq_array);
    sensor_feedback.push_back(tau_sensed_array);
    sensor_feedback.push_back(gravity_vector);
    sensor_feedback.push_back(coriolis);

    // create velocity filter
    Eigen::VectorXd _vel_raw = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd _vel_filtered = Eigen::VectorXd::Zero(7);
    sai::ButterworthFilter _filter;
    _filter.setDimension(7);
    _filter.setCutoffFrequency(0.001);

    // setup joint limit avoidance 
    // int n_samples = 100;  // t_window = n_samples / 1000 
    std::vector<int> _limit_flag {7, SAFE};
    // std::vector<Eigen::VectorXd> _tau_buffer {n_samples, Eigen::VectorXd::Zero(7)};
    // std::vector<Eigen::VectorXd> _ddq_buffer {n_samples, Eigen::VectorXd::Zero(7)};
    Eigen::VectorXd _ddq = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd _dq_estimate = Eigen::VectorXd::Zero(7);
    double _dq_tol = 1e-1;  // velocity threshold to switch out of safety 
    double _tau_tol = 1e0;  // tau threshold to switch out of safety 
    Eigen::MatrixXd _J_s;  // constraint task jacobian for nullspace projection (n_limited x dof) 
    Eigen::MatrixXd _N_s;  // nullspace matrix with the constraint task jacobian 
    Eigen::MatrixXd _Lambda_s;  // constraint task mass matrix (n_limited x n_limited)
    bool _tau_written = false;
    Eigen::VectorXi _limited_joints(7);  // 1 or 0 depending on whether the joint is limited 
    Eigen::VectorXd _tau_unit_limited(7);
    Eigen::VectorXd _tau_result(7);  // resultant torque from everything (RHS of M \ddot{q} = \tau equation)

    // override with new safety set 
    double default_sf = 0.98;  // max violation safety factor 
    for (int i = 0; i < 7; ++i) {
        joint_position_max[i] = default_sf * joint_position_max_default[i];
        joint_position_min[i] = default_sf * joint_position_min_default[i];
        joint_velocity_limits[i] = default_sf * joint_velocity_limits_default[i];
        joint_torques_limits[i] = default_sf * joint_torques_limits_default[i];
    }

    // zone definitions
    double soft_sf = 0.95;  // pure damping zone 
    double hard_sf = 0.96;  // high spring stiffness + damping zone 

    double kv_soft_scalar = 40;
    double kp_soft_scalar = (kv_soft_scalar / 2) * (kv_soft_scalar / 2);
    Eigen::VectorXd kp_soft(7);
    kp_soft << kp_soft_scalar, kp_soft_scalar, kp_soft_scalar, kp_soft_scalar, kp_soft_scalar, kp_soft_scalar, kp_soft_scalar;
    Eigen::VectorXd kv_soft(7);
    kv_soft << kv_soft_scalar, kv_soft_scalar, kv_soft_scalar, kv_soft_scalar, kv_soft_scalar, kv_soft_scalar, kv_soft_scalar;

    double kv_hard_scalar = 100;
    double kp_hard_scalar = (kv_hard_scalar / 2) * (kv_hard_scalar / 2);
    Eigen::VectorXd kv_hard(7);
    kv_hard << kv_hard_scalar, kv_hard_scalar, kv_hard_scalar, kv_hard_scalar, kv_hard_scalar, kv_hard_scalar, kv_hard_scalar * sqrt(10);
    Eigen::VectorXd kp_hard(7);
    kp_hard << kp_hard_scalar, kp_hard_scalar, kp_hard_scalar, kp_hard_scalar, kp_hard_scalar, kp_hard_scalar, kp_hard_scalar * 10;

    Eigen::VectorXd soft_min_angles(7);
    Eigen::VectorXd hard_min_angles(7);
    Eigen::VectorXd soft_max_angles(7);
    Eigen::VectorXd hard_max_angles(7);

    for (int i = 0; i < 7; ++i) {
        soft_min_angles(i) = soft_sf * joint_position_min_default[i];
        hard_min_angles(i) = hard_sf * joint_position_min_default[i];
        soft_max_angles(i) = soft_sf * joint_position_max_default[i];
        hard_max_angles(i) = hard_sf * joint_position_max_default[i];
    }

    // manual offsets
    soft_max_angles(3) -= 0.05;
    hard_max_angles(3) -= 0.05;
    soft_min_angles(5) += 0.05;
    hard_min_angles(5) += 0.05;
    soft_min_angles(6) += 0.05;
    hard_min_angles(6) += 0.05;

    // timing 
    std::clock_t start;
    double duration;

    try {
    // start the redis thread first
    // std::thread redis_thread(redis_transfer, redis_client);

    // connect to robot and gripper
    franka::Robot robot(robot_ip);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    auto torque_control_callback = [&](const franka::RobotState& robot_state,
                                        franka::Duration period) -> franka::Torques 
    {
        // filter velocities 
        for (int i = 0; i < 7; ++i) {
            _vel_raw(i) = robot_state.dq[i];
        }
        _vel_filtered = _filter.update(_vel_raw);
        for (int i = 0; i < 7; ++i) {
            dq_array[i] = _vel_filtered[i];
        }

        // start = std::clock();
        sensor_feedback[0] = robot_state.q;
        sensor_feedback[1] = robot_state.dq;
        // sensor_feedback[1] = dq_array;  // filtered velocities
        sensor_feedback[2] = robot_state.tau_J;
        sensor_feedback[3] = model.gravity(robot_state);
        sensor_feedback[4] = model.coriolis(robot_state);

        M_array = model.mass(robot_state);
        Eigen::Map<const Eigen::Matrix<double, 7, 7> > MassMatrix(M_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> _tau(tau_cmd_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> _sensed_torques(sensor_feedback[2].data());  // sensed torques 
        Eigen::Map<Eigen::Matrix<double, 7, 1>> _coriolis(sensor_feedback[4].data());
        Eigen::MatrixXd MassMatrixInverse = MassMatrix.llt().solve(Eigen::MatrixXd::Identity(7, 7));

        redis_client->setGetBatchCommands(key_names, tau_cmd_array, MassMatrix, sensor_feedback);

        // joint limit avoidance 
        // std::cout << "Classify\n";
        _tau_written = false;
        _limited_joints.setZero();
        _tau_result = _tau + _sensed_torques - _coriolis;
        for (int i = 0; i < 7; ++i) {
            if (robot_state.q[i] > soft_min_angles(i) && robot_state.q[i] < soft_max_angles(i)) {
                _limit_flag[i] = SAFE;
                _dq_estimate(i) = 0;
            } else if (robot_state.q[i] < hard_min_angles(i)) {
                // std::cout << "Hard Min Angle " << i << "\n";
                _limit_flag[i] = MIN_HARD;
                _limited_joints(i) = 1;
                if (_tau_written == false) {
                    _ddq = MassMatrixInverse * _tau_result;
                    _tau_written = true;
                }
            } else if (robot_state.q[i] < soft_min_angles(i)) {
                _limit_flag[i] = MIN_SOFT;
                // std::cout << "Soft Min Angle " << i << "\n";
                _limited_joints(i) = 1;
                if (_tau_written == false) {
                    _ddq = MassMatrixInverse * _tau_result;
                    _tau_written = true;
                } 
            } else if (robot_state.q[i] > hard_max_angles(i)) {
                _limit_flag[i] = MAX_HARD;
                // std::cout << "Hard Max Angle " << i << "\n";
                _limited_joints(i) = 1;
                if (_tau_written == false) {
                    _ddq = MassMatrixInverse * _tau_result;
                    _tau_written = true;
                }
            } else if (robot_state.q[i] > soft_max_angles(i)) {
                _limit_flag[i] = MAX_SOFT;
                // std::cout << "Soft Max Angle " << i << "\n";
                _limited_joints(i) = 1;
                if (_tau_written == false) {
                    _ddq = MassMatrixInverse * _tau_result;
                    _tau_written = true;
                }               
            }
        }

        // std::cout << "Update velocity\n";
        // update velocity estimate buffer and conditional acceptance of input torque 
        for (int i = 0; i < 7; ++i) {
            if (_limit_flag[i] != SAFE) {
                // _dq_estimate(i) += _ddq(i) * (1. / 1000);
                // _dq_estimate(i) = _vel_filtered(i) + _ddq(i) * (1. / 1000);
                _dq_estimate(i) = _vel_filtered(i);
                if (_dq_estimate(i) > _dq_tol && (_limit_flag[i] == MIN_SOFT || _limit_flag[i] == MIN_HARD) && _tau_result(i) > _tau_tol) {
                    // accept torque commands from the controller if velocity estimate is pushing away from lower limit with a minimum magnitude
                    _limit_flag[i] = SAFE;
                    _limited_joints(i) = 0;
                } else if (_dq_estimate(i) < -_dq_tol && (_limit_flag[i] == MAX_SOFT || _limit_flag[i] == MAX_HARD) && _tau_result(i) < -_tau_tol) {
                    // accept torque commands from the controller if velocity estimate is pushing away from upper limit with a minimum magnitude
                    _limit_flag[i] = SAFE;
                    _limited_joints(i) = 0;
                } 
            }
        }

        // std::cout << "Compute unit mass torque\n";
        int n_limited_joints = _limited_joints.sum();
        _tau_unit_limited = Eigen::VectorXd::Zero(n_limited_joints);
        int cnt = 0;
        for (int i = 0; i < 7; ++i) {
            // overwrite with holding torque (highest level) based on zone 
            if (_limit_flag[i] == MIN_SOFT || _limit_flag[i] == MAX_SOFT) {
                _tau_unit_limited(cnt) = - kv_soft(i) * _vel_filtered(i);
                cnt++;
            } else if (_limit_flag[i] == MAX_HARD) {
                _tau_unit_limited(cnt) = - kp_hard(i) * (robot_state.q[i] - hard_max_angles(i)) - kv_hard(i) * _vel_filtered(i);
                cnt++;
            } else if (_limit_flag[i] == MIN_HARD) {
                _tau_unit_limited(cnt) = - kp_hard(i) * (robot_state.q[i] - hard_min_angles(i)) - kv_hard(i) * _vel_filtered(i);
                cnt++;
            }
        }

        // std::cout << "Compute constraint matrices\n";
        // compute revised torques 
        _J_s = Eigen::MatrixXd::Zero(n_limited_joints, 7);
        cnt = 0;
        for (int i = 0; i < 7; ++i) {
            if (_limited_joints(i)) {
                _J_s(cnt, i) = 1;
                cnt++;
            }
        }
        _Lambda_s = (_J_s * MassMatrixInverse * _J_s.transpose()).llt().solve(Eigen::MatrixXd::Identity(n_limited_joints, n_limited_joints));
        _N_s = Eigen::MatrixXd::Identity(7, 7) - MassMatrixInverse * _J_s.transpose() * _Lambda_s * _J_s;  // I - J_bar * J
        _tau = _J_s.transpose() * _Lambda_s * _tau_unit_limited + _N_s.transpose() * _tau;  // revised torque command using nullspace projection (not including coriolis)
        // _tau = _J_s.transpose() * _tau_unit_limited + _N_s.transpose() * _tau;  // revised torque command using nullspace projection (not including coriolis)
        for (int i = 0; i < 7; ++i) {
            tau_cmd_array[i] = _tau(i);
        }

        // safety checks
        // joint torques, velocity and positions
        for(int i=0 ; i<7 ; i++)
        {
        // torque saturation
        if(tau_cmd_array[i] > joint_torques_limits[i])
        {
            std::cout << "WARNING : Torque commanded on joint " << i << " too high (" << tau_cmd_array[i];
            std::cout << "), saturating to max value(" << joint_torques_limits[i] << ")" << std::endl;
            tau_cmd_array[i] = joint_torques_limits[i];
        }
        if(tau_cmd_array[i] < -joint_torques_limits[i])
        {
            std::cout << "WARNING : Torque commanded on joint " << i << " too low (" << tau_cmd_array[i];
            std::cout << "), saturating to min value(" << -joint_torques_limits[i] << ")" << std::endl;
            tau_cmd_array[i] = -joint_torques_limits[i];
        }
        // position limit
        if(robot_state.q[i] > joint_position_max[i])
        {
            safety_mode_flag = true;
            if(safety_controller_count == 200)
            {
            std::cout << "WARNING : Soft joint upper limit violated on joint " << i << ", engaging safety mode" << std::endl;
            }
        }
        if(robot_state.q[i] < joint_position_min[i])
        {
            safety_mode_flag = true;
            if(safety_controller_count == 200)
            {
            std::cout << "WARNING : Soft joint lower limit violated on joint " << i << ", engaging safety mode" << std::endl;
            }
        }
        // velocity limit
        if(abs(robot_state.dq[i]) > joint_velocity_limits[i])
        {
            safety_mode_flag = true;
            if(safety_controller_count == 200)
            {
            std::cout << "WARNING : Soft velocity limit violated on joint " << i << ", engaging safety mode" << std::endl;
            }
        }
        }

        if (safety_enabled) {
        // cartesian checks
        Eigen::Affine3d T_EE(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d pos_monitoring_point = T_EE*monitoring_point_ee_frame;
        double radius_square = pos_monitoring_point(0)*pos_monitoring_point(0) + pos_monitoring_point(1)*pos_monitoring_point(1);
        double z_ee = pos_monitoring_point(2);

        // lower plane
        if(z_ee < safety_plane_z_coordinate)
        {
            safety_mode_flag = true;
            if(safety_controller_count == 200)
            {
            std::cout << "WARNING : End effector too low, engaging safety mode" << std::endl;
            std::cout << "position of monitoring point : " << pos_monitoring_point.transpose() << std::endl;
            }       
        }
        // cylinder
        if(z_ee < safety_cylinder_height && radius_square < safety_cylinder_radius*safety_cylinder_radius)
        {
            safety_mode_flag = true;
            if(safety_controller_count == 200)
            {
            std::cout << "WARNING : End effector too close to center of workspace, engaging safety mode" << std::endl;
            std::cout << "position of monitoring point : " << pos_monitoring_point.transpose() << std::endl;
            }       
        }
        // std::cout << pos_monitoring_point.transpose() << std::endl;
        }

        // safety mode
        if(safety_mode_flag)
        {
        for(int i=0 ; i<7 ; i++)
        {
            tau_cmd_array[i] = -kv_safety*robot_state.dq[i];
        }

        if(safety_controller_count == 0)
        {
            throw std::runtime_error("Stopping driver due to safety violation");
        }

        safety_controller_count--;
        }

        counter++;
        return tau_cmd_array;
    };

    // start real-time control loop
    robot.control(torque_control_callback);
    // robot.control(torque_control_callback, false, franka::kMaxCutoffFrequency);
    // stop redis
    // fDriverRunning = false;
    // redis_thread.join();

    } catch (const std::exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
    std::cout << "counter : " << counter << std::endl;
    // stop redis
    // fDriverRunning = false;
    // redis_thread.join();
    }


  return 0;
}
