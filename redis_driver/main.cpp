// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <functional>
#include <iostream>

#include <Eigen/Core>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "RedisClient.h"

// redis keys
// - read:
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::actuators::fgc";
// - write:
const std::string JOINT_ANGLES_KEY  = "sai2::FrankaPanda::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::sensors::dq";
const std::string JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::sensors::torques";
const std::string MASSMATRIX_KEY = "sai2::FrankaPanda::sensors::model::massmatrix";
const std::string CORIOLIS_KEY = "sai2::FrankaPanda::sensors::model::coriolis";
const std::string ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::sensors::model::robot_gravity";
Eigen::MatrixXd MassMatrix;

unsigned long long counter = 0;

void debug_function(CDatabaseRedisClient &redis_client)
{
  std::array<double, 7> tau_cmd_array{};
  std::array<double, 7> tau_sensed_array{};
  std::array<double, 7> q_array{};
  std::array<double, 7> dq_array{}; 
  for(int i=0; i<7; i++)
  {
    tau_cmd_array[i] = 0;
    tau_sensed_array[i] = 0;
    q_array[i] = 0;
    dq_array[i] = 0;
  }

        q_array[3] = 1.5;
    std::clock_t start;
    double duration;


    // redis_client.setDoubleArray(JOINT_ANGLES_KEY, q_array, 7);
    // redis_client.setDoubleArray(JOINT_VELOCITIES_KEY, dq_array, 7);
    // redis_client.setDoubleArray(JOINT_TORQUES_SENSED_KEY, tau_sensed_array, 7);   
    redis_client.getDoubleArray(JOINT_TORQUES_COMMANDED_KEY, tau_cmd_array, 7);

    std::vector<string> key_names;
    key_names.push_back(JOINT_ANGLES_KEY);
    key_names.push_back(JOINT_VELOCITIES_KEY);
    key_names.push_back(JOINT_TORQUES_SENSED_KEY);

    std::vector<std::array<double, 7>> sensor_feedback;
    sensor_feedback.push_back(q_array);
    sensor_feedback.push_back(dq_array);
    sensor_feedback.push_back(tau_sensed_array);

    // redis_client.setCommandBatch(key_names, sensor_feedback, 3);


  while(true)
  {
      start = std::clock();

      sensor_feedback.clear();
      sensor_feedback.push_back(q_array);
      sensor_feedback.push_back(dq_array);
      sensor_feedback.push_back(tau_sensed_array);
      
      redis_client.setCommandBatch(key_names, sensor_feedback, 3);
      // redis_client.setDoubleArray(JOINT_ANGLES_KEY, q_array, 7);
      // redis_client.setDoubleArray(JOINT_VELOCITIES_KEY, dq_array, 7);
      // redis_client.setDoubleArray(JOINT_TORQUES_SENSED_KEY, tau_sensed_array, 7);   
      redis_client.getDoubleArray(JOINT_TORQUES_COMMANDED_KEY, tau_cmd_array, 7);
      
      duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      if(duration > 0.0003)
      {
        std::cout << "duration : "<< duration <<'\n';
        std::cout << "counter : " << counter << std::endl << std::endl;
      }

      if(counter == 1000)
      {
        q_array[0] = 1.5;
        // sensor_feedback[0][0] = 1.5;
        // std::cout << q_array[0] << std::endl;
      }
      // if(counter % 100 == 0)
      // {
      //   std::cout << q_array[0] << std::endl;
      // }

      counter++;
      // std::cout << counter << std::endl;
  }

}

int main(int argc, char** argv) {
  // start redis client
  CDatabaseRedisClient redis_client;
  HiredisServerInfo info;
  info.hostname_ = "127.0.0.1";
  info.port_ = 6379;
  info.timeout_ = { 1, 500000 }; // 1.5 seconds
  redis_client = CDatabaseRedisClient();
  redis_client.serverIs(info);

  // debug_function(redis_client);

  std::array<double, 7> tau_cmd_array{};
  for(int i=0; i<7; i++)
  {
    tau_cmd_array[i] = 0;
  }

  std::array<double, 7> q_array{};
  std::array<double, 7> dq_array{};
  std::array<double, 7> tau_sensed_array{};
  std::array<double, 7> gravity_vector{};
  std::array<double, 7> coriolis{};

  std::array<double, 49> M_array{};  
  MassMatrix = Eigen::MatrixXd::Identity(7,7);

  redis_client.setDoubleArray(JOINT_TORQUES_COMMANDED_KEY, tau_cmd_array, 7);
  // Safety to detect if controller is already running : wait 50 milliseconds
  usleep(50000);
  redis_client.getDoubleArray(JOINT_TORQUES_COMMANDED_KEY, tau_cmd_array, 7);
  for(int i=0; i<7; i++)
  {
    if(tau_cmd_array[i] != 0)
    {
      std::cout << "Stop the controller before runnung the driver\n" << std::endl;
      return -1;
    }
  }

  // prepare batch command
  std::vector<string> key_names;
  key_names.push_back(JOINT_TORQUES_COMMANDED_KEY);
  key_names.push_back(MASSMATRIX_KEY);
  key_names.push_back(JOINT_ANGLES_KEY);
  key_names.push_back(JOINT_VELOCITIES_KEY);
  key_names.push_back(JOINT_TORQUES_SENSED_KEY);
  key_names.push_back(ROBOT_GRAVITY_KEY);
  key_names.push_back(CORIOLIS_KEY);

  std::vector<std::array<double, 7>> sensor_feedback;
  sensor_feedback.push_back(q_array);
  sensor_feedback.push_back(dq_array);
  sensor_feedback.push_back(tau_sensed_array);
  sensor_feedback.push_back(gravity_vector);
  sensor_feedback.push_back(coriolis);

  std::clock_t start;
  double duration;

  try {
    // connect to robot and gripper
    franka::Robot robot("172.16.0.10");
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
      // start = std::clock();
      sensor_feedback[0] = robot_state.q;
      sensor_feedback[1] = robot_state.dq;
      sensor_feedback[2] = robot_state.tau_J;
      sensor_feedback[3] = model.gravity(robot_state);
      sensor_feedback[4] = model.coriolis(robot_state);

      M_array = model.mass(robot_state);
      Eigen::Map<const Eigen::Matrix<double, 7, 7> > MassMatrix(M_array.data());
      // duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      // std::cout << "duration 1 : "<< duration <<'\n';

      // start = std::clock();
      redis_client.setGetBatchCommands(key_names, tau_cmd_array, MassMatrix, sensor_feedback);
      // duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      // std::cout << "duration 2 : "<< duration <<'\n' << std::endl;

      // if(counter % 100 == 0)
      // {
      //   std::cout << "joint angles : ";
      //   for(int i=0; i<7; i++)
      //   {
      //     std::cout << " " << robot_state.q[i] << " ";
      //   }
      //   std::cout << std::endl;
      //   std::cout << "joint velocities : ";
      //   for(int i=0; i<7; i++)
      //   {
      //     std::cout << " " << robot_state.dq[i] << " ";
      //   }
      //   std::cout << std::endl;
      //   std::cout << "joint torques sensed : ";
      //   for(int i=0; i<7; i++)
      //   {
      //     std::cout << " " << robot_state.tau_J[i] << " ";
      //   }
      //   std::cout << std::endl;
      //   std::cout << "torque commanded : ";
      //   for(int i=0; i<7; i++)
      //   {
      //     std::cout << " " << tau_cmd_array[i] << " ";
      //   }
      //   std::cout << std::endl;
      //   std::cout << std::endl;
      // }

      // for(int i=0; i<7; i++)
      // {
      //   tau_cmd_array[i] = 0;
      // }

      counter++;
      return tau_cmd_array;
    };

    // start real-time control loop
    robot.control(torque_control_callback);
    // robot.control(torque_control_callback, false, franka::kMaxCutoffFrequency);

  } catch (const std::exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
    std::cout << "counter : " << counter << std::endl;
  }



  return 0;
}
