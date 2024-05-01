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
#include <thread>
#include <atomic>
#include <mutex>

// - gripper
std::string GRIPPER_MODE_KEY; // m for move and g for grasp
std::string GRIPPER_MAX_WIDTH_KEY;
std::string GRIPPER_CURRENT_WIDTH_KEY;
std::string GRIPPER_DESIRED_WIDTH_KEY;
std::string GRIPPER_DESIRED_SPEED_KEY;
std::string GRIPPER_DESIRED_FORCE_KEY;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}
unsigned long long counter = 0;

std::atomic<int> begin_move_flag(1);
// std::mutex mtx;
// bool flag_blocking_call_executed = false;
auto start_time = std::chrono::high_resolution_clock::now();
void stopGripper(franka::Gripper& gripper) {
  while(1) {
    // std::lock_guard<std::mutex> lock(mtx);
    int local_begin_move_flag = begin_move_flag.load();
    std::cout << local_begin_move_flag << "\n";
    if (local_begin_move_flag) {
      // std::cout << "reset start time\n";
      start_time = std::chrono::high_resolution_clock::now();
      begin_move_flag.store(0);
    }
    auto end_time = std::chrono::high_resolution_clock::now();  
    std::chrono::duration<double> duration = end_time - start_time;
    double elapsed_time = duration.count();
    std::cout << elapsed_time << "\n";
    if (elapsed_time > 5) {
      std::cout << "stopped gripper\n";
      gripper.stop(); 
      // begin_move_flag.store(0);
    }
  }
}

int main(int argc, char** argv) {

  if(argc < 2)
  {
    std::cout << "Please enter the robot ip as an argument" << std::endl;
    return -1;
  }
  std::string robot_ip = argv[1];

  if(robot_ip == "172.16.0.10")
  {
    GRIPPER_MODE_KEY  = "sai2::FrankaPanda::Romeo::gripper::mode"; 
    GRIPPER_MAX_WIDTH_KEY  = "sai2::FrankaPanda::Romeo::gripper::max_width";
    GRIPPER_CURRENT_WIDTH_KEY  = "sai2::FrankaPanda::Romeo::gripper::current_width";
    GRIPPER_DESIRED_WIDTH_KEY  = "sai2::FrankaPanda::Romeo::gripper::desired_width";
    GRIPPER_DESIRED_SPEED_KEY  = "sai2::FrankaPanda::Romeo::gripper::desired_speed";
    GRIPPER_DESIRED_FORCE_KEY  = "sai2::FrankaPanda::Romeo::gripper::desired_force";   
  }
  else if(robot_ip == "172.16.0.11")
  {
    GRIPPER_MODE_KEY  = "sai2::FrankaPanda::Juliet::gripper::mode"; 
    GRIPPER_MAX_WIDTH_KEY  = "sai2::FrankaPanda::Juliet::gripper::max_width";
    GRIPPER_CURRENT_WIDTH_KEY  = "sai2::FrankaPanda::Juliet::gripper::current_width";
    GRIPPER_DESIRED_WIDTH_KEY  = "sai2::FrankaPanda::Juliet::gripper::desired_width";
    GRIPPER_DESIRED_SPEED_KEY  = "sai2::FrankaPanda::Juliet::gripper::desired_speed";
    GRIPPER_DESIRED_FORCE_KEY  = "sai2::FrankaPanda::Juliet::gripper::desired_force";    
  }
  else
  {
    GRIPPER_MODE_KEY  = "sai2::FrankaPanda::gripper::mode"; 
    GRIPPER_MAX_WIDTH_KEY  = "sai2::FrankaPanda::gripper::max_width";
    GRIPPER_CURRENT_WIDTH_KEY  = "sai2::FrankaPanda::gripper::current_width";
    GRIPPER_DESIRED_WIDTH_KEY  = "sai2::FrankaPanda::gripper::desired_width";
    GRIPPER_DESIRED_SPEED_KEY  = "sai2::FrankaPanda::gripper::desired_speed";
    GRIPPER_DESIRED_FORCE_KEY  = "sai2::FrankaPanda::gripper::desired_force";
  }

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
  string gripper_mode_tmp = "m";

  bool flag_command_changed = false;

  try {
    // connect to gripper
    franka::Gripper gripper(robot_ip);

    // setup stop thread 
    begin_move_flag.store(0);
    // std::thread stopGripperThread(stopGripper, std::ref(gripper));

    // create timer
    auto start_time = std::chrono::high_resolution_clock::now();

    // home the gripper
    // gripper.homing();

    franka::GripperState gripper_state = gripper.readOnce();
    gripper_max_width = gripper_state.max_width;
    redis_client.setCommandIs(GRIPPER_MAX_WIDTH_KEY, std::to_string(gripper_max_width));
    redis_client.setCommandIs(GRIPPER_MODE_KEY, gripper_mode);

    // gripper_desired_width = 0.5*gripper_max_width;
    // gripper_desired_width = gripper_state.width;
    gripper_desired_width = gripper_state.max_width;
    // gripper_desired_speed = 0.07;
    gripper_desired_speed = 0.1;
    gripper_desired_force = 50.0;
    gripper.move(gripper_desired_width, gripper_desired_speed);

    redis_client.setCommandIs(GRIPPER_DESIRED_WIDTH_KEY, std::to_string(gripper_desired_width));
    redis_client.setCommandIs(GRIPPER_DESIRED_SPEED_KEY, std::to_string(gripper_desired_speed));
    redis_client.setCommandIs(GRIPPER_DESIRED_FORCE_KEY, std::to_string(gripper_desired_force));

    int failed_flag = 0;

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
        redis_client.setCommandIs(GRIPPER_DESIRED_WIDTH_KEY, std::to_string(0.08));
        std::cout << "WARNING : Desired gripper width higher than max width. saturating to max width\n" << std::endl;
      }
      if(gripper_desired_width_tmp < 0)
      {
        gripper_desired_width_tmp = 0;
        redis_client.setCommandIs(GRIPPER_DESIRED_WIDTH_KEY, std::to_string(0));
        std::cout << "WARNING : Desired gripper width lower than 0. saturating to min (0.0)\n" << std::endl;
      }
      if(gripper_desired_speed_tmp < 0)
      {
        gripper_desired_speed_tmp = 0.01;
        redis_client.setCommandIs(GRIPPER_DESIRED_SPEED_KEY, std::to_string(0));
        std::cout << "WARNING : Desired gripper speed lower than 0. saturating to min (0.01)\n" << std::endl;
      } 
      if(gripper_desired_force_tmp < 0)
      {
        gripper_desired_force_tmp = 0.1;
        redis_client.setCommandIs(GRIPPER_DESIRED_FORCE_KEY, std::to_string(0));
        std::cout << "WARNING : Desired gripper force lower than 0. saturating to min (0.1)\n" << std::endl;
      } 

      if(gripper_desired_width != gripper_desired_width_tmp ||
         gripper_desired_speed != gripper_desired_speed_tmp ||
         gripper_desired_force != gripper_desired_force_tmp ||
         gripper_mode != gripper_mode_tmp)
      {
        flag_command_changed = true;
        gripper_desired_width = gripper_desired_width_tmp;
        gripper_desired_speed = gripper_desired_speed_tmp;
        gripper_desired_force = gripper_desired_force_tmp;
        gripper_mode = gripper_mode_tmp;
      }

      if(flag_command_changed || failed_flag)
      // if(flag_command_changed)
      {
        // gripper.stop();
        if(gripper_mode == "m")
        {
          // begin_move_flag.store(1);
          bool move_successful = gripper.move(gripper_desired_width, gripper_desired_speed);   
          if (!move_successful) {
            failed_flag = 1;
            gripper_desired_width = gripper_state.width + 0.01;
            redis_client.setCommandIs(GRIPPER_DESIRED_WIDTH_KEY, std::to_string(gripper_desired_width));
          } else {
            failed_flag = 0;
          }
        }
        else if(gripper_mode == "g")
        {
          // try {
          bool grasp_successful = gripper.grasp(0.0, gripper_desired_speed, gripper_desired_force, 1.0, 1.0);
            // begin_move_flag.store(1);
            // bool grasp_successful = gripper.grasp(gripper_desired_width, gripper_desired_speed, gripper_desired_force, 1.0, 1.0);
            // bool grasp_successful = gripper.grasp(0, gripper_desired_speed, gripper_desired_force, 0, 0);
          std::cout << "grasp status: " << grasp_successful << "\n";
          if (!grasp_successful) {
            failed_flag = 1;            
          } else {
            failed_flag = 0;
          }
          // } catch(...) {
            // std::cout << "catch\n";
            // failed_flag = 1;  
            // if (!grasp_successful) {
            //   gripper_desired_width = gripper_state.width + 0.01;
            //   failed_flag = 1;
            //   gripper.grasp(0.0, gripper_desired_speed, gripper_desired_force, 1.0, 1.0);
            // } else {
            //   begin_move_flag.store(0);
            //   failed_flag = 0;
            // }
          // }
          // std::cout << "grasp succesful : " << grasp_successful << "  force : " << gripper_desired_force << std::endl;
          // std::lock_guard<std::mutex> lock(mtx);
          // flag_blocking_call_executed = true;                    
        }
        else if (gripper_mode == "o") {
          bool move_successful = gripper.move(gripper_state.max_width, gripper_desired_speed);   
          if (!move_successful) {
            failed_flag = 1;
            gripper_desired_width = gripper_state.width + 0.01;
            redis_client.setCommandIs(GRIPPER_DESIRED_WIDTH_KEY, std::to_string(gripper_desired_width));
          } else {
            failed_flag = 0;
          }
        } else 
        {
          std::cout << "WARNING : gripper mode not regognized. Use g for grasp and m for move\n"<< std::endl;
        }
        flag_command_changed = false;
      }

      // std::cout << counter << "\n";

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
