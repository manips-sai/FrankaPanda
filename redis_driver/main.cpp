// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
// #include <thread>
// #include <signal.h>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "RedisClient.h"

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
// const std::array<double, 7> joint_position_max = {2.89, 1.76, 2.89, -0.06, 2.89, 3.75, 2.89};
// const std::array<double, 7> joint_position_min = {-2.89, -1.76, -2.89, -3.07, -2.89, -0.01, -2.89};
// const std::array<double, 7> joint_velocity_limits = {2.17, 2.17, 2.17, 2.17, 2.61, 2.61, 2.61};
// const std::array<double, 7> joint_torques_limits = {87, 87, 87, 87, 12, 12, 12};
const std::array<double, 7> joint_position_max = {2.8, 1.7, 2.85, -0.1, 2.8, 3.7, 2.87};
const std::array<double, 7> joint_position_min = {-2.8, -1.7, -2.85, -3.0, -2.8, 0.05, -2.87};
const std::array<double, 7> joint_velocity_limits = {2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5};
const std::array<double, 7> joint_torques_limits = {85, 85, 85, 85, 10, 10, 10};

const Eigen::Vector3d monitoring_point_ee_frame = Eigen::Vector3d(0.0, 0.0, 0.05);
const double safety_plane_z_coordinate = 0.35;
const double safety_cylinder_radius = 0.28;
const double safety_cylinder_height = 0.53;

bool safety_mode_flag = false;
int safety_controller_count = 200;
const std::array<double, 7> kv_safety = {20.0, 20.0, 20.0, 15.0, 10.0, 10.0, 5.0};

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
    // robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
    //                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
    //                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
    //                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{200.0, 200.0, 200.0, 100.0, 100.0, 100.0}},
                               {{200.0, 200.0, 200.0, 100.0, 100.0, 100.0}});

    auto torque_control_callback = [&](const franka::RobotState& robot_state,
                                      franka::Duration period) -> franka::Torques 
    {
      // start = std::clock();
      sensor_feedback[0] = robot_state.q;
      sensor_feedback[1] = robot_state.dq;
      sensor_feedback[2] = robot_state.tau_J;
      sensor_feedback[3] = model.gravity(robot_state);
      sensor_feedback[4] = model.coriolis(robot_state);

      M_array = model.mass(robot_state);
      Eigen::Map<const Eigen::Matrix<double, 7, 7> > MassMatrix(M_array.data());

      redis_client->setGetBatchCommands(key_names, tau_cmd_array, MassMatrix, sensor_feedback);

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

      // safety mode
      if(safety_mode_flag)
      {
        for(int i=0 ; i<7 ; i++)
        {
          tau_cmd_array[i] = -kv_safety[i]*robot_state.dq[i];
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
