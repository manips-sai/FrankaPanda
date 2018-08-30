// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <functional>
#include <iostream>

#include <Eigen/Core>

#include <franka/exception.h>
#include <franka/gripper.h>

#include "RedisClient.h"

// - gripper
const std::string GRIPPER_MODE_KEY  = "sai2::FrankaPanda::gripper::mode"; // m for move and g for graps
const std::string GRIPPER_MAX_WIDTH_KEY  = "sai2::FrankaPanda::gripper::max_width";
const std::string GRIPPER_CURRENT_WIDTH_KEY  = "sai2::FrankaPanda::gripper::current_width";
const std::string GRIPPER_DESIRED_WIDTH_KEY  = "sai2::FrankaPanda::gripper::desired_width";
const std::string GRIPPER_DESIRED_SPEED_KEY  = "sai2::FrankaPanda::gripper::desired_speed";
const std::string GRIPPER_DESIRED_FORCE_KEY  = "sai2::FrankaPanda::gripper::desired_force";

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}
unsigned long long counter = 0;


int main(int argc, char** argv) {

  if(argc < 2)
  {
    std::cout << "Please enter the robot ip as an argument" << std::endl;
    return -1;
  }
  std::string robot_ip = argv[1];

  // start redis client
  CDatabaseRedisClient redis_client;
  HiredisServerInfo info;
  info.hostname_ = "127.0.0.1";
  info.port_ = 6379;
  info.timeout_ = { 1, 500000 }; // 1.5 seconds
  redis_client = CDatabaseRedisClient();
  redis_client.serverIs(info);

  // set up signal handler
  signal(SIGABRT, &sighandler);
  signal(SIGTERM, &sighandler);
  signal(SIGINT, &sighandler);

  // setup variables for gripper control
  double gripper_desired_width, gripper_desired_width_tmp;
  double gripper_desired_speed, gripper_desired_speed_tmp;
  double gripper_desired_force, gripper_desired_force_tmp;
  double gripper_max_width;
  string gripper_mode = "m";
  string gripper_mode_tmp;

  bool flag_command_changed = false;

  try {
    // connect to gripper
    franka::Gripper gripper(robot_ip);

    // home the gripper
    // gripper.homing();

    franka::GripperState gripper_state = gripper.readOnce();
    gripper_max_width = gripper_state.max_width;
    redis_client.setCommandIs(GRIPPER_MAX_WIDTH_KEY, std::to_string(gripper_max_width));
    redis_client.setCommandIs(GRIPPER_MODE_KEY, gripper_mode);

    // gripper_desired_width = 0.5*gripper_max_width;
    gripper_desired_width = gripper_state.width;
    gripper_desired_speed = 0.07;
    gripper_desired_force = 0.0;
    gripper.move(gripper_desired_width, gripper_desired_speed);

    redis_client.setCommandIs(GRIPPER_DESIRED_WIDTH_KEY, std::to_string(gripper_desired_width));
    redis_client.setCommandIs(GRIPPER_DESIRED_SPEED_KEY, std::to_string(gripper_desired_speed));
    redis_client.setCommandIs(GRIPPER_DESIRED_FORCE_KEY, std::to_string(gripper_desired_force));

    runloop = true;
    while(runloop)
    {
      gripper_state = gripper.readOnce();
      redis_client.setCommandIs(GRIPPER_CURRENT_WIDTH_KEY, std::to_string(gripper_state.width));

      redis_client.getCommandIs(GRIPPER_DESIRED_WIDTH_KEY, gripper_desired_width_tmp);
      redis_client.getCommandIs(GRIPPER_DESIRED_SPEED_KEY, gripper_desired_speed_tmp);
      redis_client.getCommandIs(GRIPPER_DESIRED_FORCE_KEY, gripper_desired_force_tmp);
      redis_client.getCommandIs(GRIPPER_MODE_KEY, gripper_mode_tmp);
      if(gripper_desired_width_tmp > gripper_max_width)
      {
        gripper_desired_width_tmp = gripper_max_width;
        redis_client.setCommandIs(GRIPPER_DESIRED_WIDTH_KEY, std::to_string(gripper_max_width));
        std::cout << "WARNING : Desired gripper width higher than max width. saturating to max width\n" << std::endl;
      }
      if(gripper_desired_width_tmp < 0)
      {
        gripper_desired_width_tmp = 0;
        redis_client.setCommandIs(GRIPPER_DESIRED_WIDTH_KEY, std::to_string(0));
        std::cout << "WARNING : Desired gripper width lower than 0. saturating to max 0\n" << std::endl;
      }
      if(gripper_desired_speed_tmp < 0)
      {
        gripper_desired_speed_tmp = 0;
        redis_client.setCommandIs(GRIPPER_DESIRED_SPEED_KEY, std::to_string(0));
        std::cout << "WARNING : Desired gripper speed lower than 0. saturating to max 0\n" << std::endl;
      } 
      if(gripper_desired_force_tmp < 0)
      {
        gripper_desired_force_tmp = 0;
        redis_client.setCommandIs(GRIPPER_DESIRED_FORCE_KEY, std::to_string(0));
        std::cout << "WARNING : Desired gripper speed lower than 0. saturating to max 0\n" << std::endl;
      } 

      if(gripper_desired_width != gripper_desired_width_tmp ||
         gripper_desired_speed != gripper_desired_speed_tmp ||
         gripper_desired_force != gripper_desired_force_tmp ||
         gripper_mode != gripper_mode_tmp)
      {
        flag_command_changed = true;
        gripper_desired_width = gripper_desired_width_tmp;
        gripper_desired_speed = gripper_desired_speed_tmp;
        gripper_desired_force_tmp = gripper_desired_force_tmp;
        gripper_mode = gripper_mode_tmp;
      }

      if(flag_command_changed)
      {
        if(gripper_mode == "m")
        {
          gripper.move(gripper_desired_width, gripper_desired_speed);
        }
        else if(gripper_mode == "g")
        {
          gripper.grasp(gripper_desired_width, gripper_desired_speed, gripper_desired_force);
        }
        else
        {
          std::cout << "WARNING : gripper mode not regognized. Use g for grasp and m for move\n"<< std::endl;
        }
        flag_command_changed = false;
      }

      // if(counter % 500 == 0)
      // {
        // std::cout << "counter : " << counter << std::endl;
      // }
      counter++;
    }

    gripper.stop();


  } catch (const std::exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }



  return 0;
}
