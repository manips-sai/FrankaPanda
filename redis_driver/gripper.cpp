// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <exception>
#include <functional>
#include <iostream>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <thread>

#include <Eigen/Core>

#include <franka/exception.h>
#include <franka/gripper.h>

#include "SaiFrankaRedisClientLocal.h"
#include "SaiFrankaDriverConfig.h"

// - gripper
std::string GRIPPER_MODE_KEY; // m for move and g for grasp
std::string GRIPPER_MAX_WIDTH_KEY;
std::string GRIPPER_CURRENT_WIDTH_KEY;
std::string GRIPPER_DESIRED_WIDTH_KEY;
std::string GRIPPER_DESIRED_SPEED_KEY;
std::string GRIPPER_DESIRED_FORCE_KEY;
std::string GRIPPER_GRASP_SUCCESS_KEY;
std::string GRIPPER_READY_KEY;

// Minimum finger width (m) after a force-close for an object to count as
// "held". libfranka's grasp() returns true even on an empty full-close given
// the wide epsilon window below, so we AND it with this width check: an empty
// close ends near 0 m, while a held object keeps the fingers apart.
const double GRIPPER_GRASP_MIN_WIDTH = 0.003;
// Published to GRIPPER_GRASP_SUCCESS_KEY while a grasp is in flight.
const char* GRIPPER_GRASP_PENDING = "pending";
// Tolerance (m) on the desired-width-over-max saturation check. std::to_string
// rounds max_width UP at 6 decimals, so writing it back reads as fractionally
// over max and re-triggers the saturation warning every loop. A small slack
// stops that spam without letting genuinely-too-wide commands through.
const double GRIPPER_MAX_WIDTH_SLACK = 1e-3;

struct GripperRedisExchangeData {
  double current_width = 0.0;
  double max_width = 0.0;
  double desired_width = 0.0;
  double desired_speed = 0.0;
  double desired_force = 0.0;
  double desired_width_to_write = 0.0;
  std::string mode = "m";
  std::string grasp_success = "0";
  bool has_state = false;
  bool desired_width_write_requested = false;
};

volatile std::sig_atomic_t runloop = 0;
void sighandler(int){runloop = 0;}
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

	std::string config_file = "default_config.xml";

    if (argc > 1) {
        config_file = argv[1];
    }
	std::string config_file_path = std::string(CONFIG_FOLDER) + "/" + config_file;

	Sai::Franka::DriverConfig config = Sai::Franka::loadConfig(config_file_path);
	std::string redis_prefix = config.redis_prefix.empty() ? "" : config.redis_prefix + "::";

  GRIPPER_MODE_KEY  = redis_prefix + config.robot_name + "::gripper::mode"; 
  GRIPPER_MAX_WIDTH_KEY  = redis_prefix + config.robot_name + "::gripper::max_width";
  GRIPPER_CURRENT_WIDTH_KEY  = redis_prefix + config.robot_name + "::gripper::current_width";
  GRIPPER_DESIRED_WIDTH_KEY  = redis_prefix + config.robot_name + "::gripper::desired_width";
  GRIPPER_DESIRED_SPEED_KEY  = redis_prefix + config.robot_name + "::gripper::desired_speed";
  GRIPPER_DESIRED_FORCE_KEY  = redis_prefix + config.robot_name + "::gripper::desired_force";   
  GRIPPER_GRASP_SUCCESS_KEY  = redis_prefix + config.robot_name + "::gripper::grasp_success";
  GRIPPER_READY_KEY          = redis_prefix + config.robot_name + "::gripper::ready";

  // start redis client
  Sai::Franka::CDatabaseRedisClient redis_client;
  Sai::Franka::HiredisServerInfo info;
  info.hostname_ = "127.0.0.1";
  info.port_ = 6379;
  info.timeout_ = { 1, 500000 }; // 1.5 seconds
  redis_client = Sai::Franka::CDatabaseRedisClient();
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
  std::string gripper_mode = "m";
  std::string gripper_mode_tmp = "m";

  bool flag_command_changed = false;

  GripperRedisExchangeData redis_exchange_data;
  std::shared_mutex redis_exchange_mutex;
  std::atomic_bool redis_thread_running{false};
  std::atomic_bool redis_thread_failed{false};
  std::exception_ptr redis_thread_exception = nullptr;
  std::mutex redis_thread_exception_mutex;

  auto redis_transfer = [&]() {
    double local_current_width = 0.0;
    double local_max_width = 0.0;
    double local_desired_width = 0.0;
    double local_desired_speed = 0.0;
    double local_desired_force = 0.0;
    double local_desired_width_to_write = 0.0;
    std::string local_mode = "m";
    std::string local_grasp_success = "0";
    std::string last_grasp_success_written;
    bool has_state = false;
    bool desired_width_write_requested = false;

    try {
      while(redis_thread_running.load(std::memory_order_relaxed)) {
        {
          std::unique_lock<std::shared_mutex> lock(redis_exchange_mutex);
          has_state = redis_exchange_data.has_state;
          if(has_state) {
            local_current_width = redis_exchange_data.current_width;
            local_max_width = redis_exchange_data.max_width;
            local_grasp_success = redis_exchange_data.grasp_success;
          }
          desired_width_write_requested = redis_exchange_data.desired_width_write_requested;
          local_desired_width_to_write = redis_exchange_data.desired_width_to_write;
          redis_exchange_data.desired_width_write_requested = false;
        }

        if(!has_state) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          continue;
        }

        redis_client.setCommandIs(GRIPPER_CURRENT_WIDTH_KEY, std::to_string(local_current_width));
        if(local_grasp_success != last_grasp_success_written) {
          redis_client.setCommandIs(GRIPPER_GRASP_SUCCESS_KEY, local_grasp_success);
          last_grasp_success_written = local_grasp_success;
        }
        if(desired_width_write_requested) {
          redis_client.setCommandIs(GRIPPER_DESIRED_WIDTH_KEY,
                                    std::to_string(local_desired_width_to_write));
        }

        redis_client.getCommandIs(GRIPPER_DESIRED_WIDTH_KEY, local_desired_width);
        redis_client.getCommandIs(GRIPPER_DESIRED_SPEED_KEY, local_desired_speed);
        redis_client.getCommandIs(GRIPPER_DESIRED_FORCE_KEY, local_desired_force);
        redis_client.getCommandIs(GRIPPER_MODE_KEY, local_mode);

        if(local_desired_width > local_max_width) {
          if(local_desired_width > local_max_width + GRIPPER_MAX_WIDTH_SLACK) {
            std::cout << "WARNING : Desired gripper width higher than max width. saturating to max width\n" << std::endl;
          }
          local_desired_width = local_max_width;
          redis_client.setCommandIs(GRIPPER_DESIRED_WIDTH_KEY, std::to_string(local_max_width));
        }
        if(local_desired_width < 0) {
          local_desired_width = 0.0;
          redis_client.setCommandIs(GRIPPER_DESIRED_WIDTH_KEY, std::to_string(0.0));
          std::cout << "WARNING : Desired gripper width lower than 0. saturating to min (0.0)\n" << std::endl;
        }
        if(local_desired_speed < 0) {
          local_desired_speed = 0.01;
          redis_client.setCommandIs(GRIPPER_DESIRED_SPEED_KEY, std::to_string(local_desired_speed));
          std::cout << "WARNING : Desired gripper speed lower than 0. saturating to min (0.01)\n" << std::endl;
        }
        if(local_desired_force < 0) {
          local_desired_force = 0.1;
          redis_client.setCommandIs(GRIPPER_DESIRED_FORCE_KEY, std::to_string(local_desired_force));
          std::cout << "WARNING : Desired gripper force lower than 0. saturating to min (0.1)\n" << std::endl;
        }

        {
          std::unique_lock<std::shared_mutex> lock(redis_exchange_mutex);
          redis_exchange_data.desired_width = local_desired_width;
          redis_exchange_data.desired_speed = local_desired_speed;
          redis_exchange_data.desired_force = local_desired_force;
          redis_exchange_data.mode = local_mode;
        }
      }
    } catch (...) {
      {
        std::lock_guard<std::mutex> lock(redis_thread_exception_mutex);
        redis_thread_exception = std::current_exception();
      }
      redis_thread_failed.store(true, std::memory_order_relaxed);
      redis_thread_running.store(false, std::memory_order_relaxed);
    }
  };

  std::thread redis_thread;

  auto stopRedisThread = [&]() {
    redis_thread_running.store(false, std::memory_order_relaxed);
    if(redis_thread.joinable()) {
      redis_thread.join();
    }
  };

  try {
    // connect to gripper
    franka::Gripper gripper(config.robot_ip);

    // setup stop thread 
    begin_move_flag.store(0);
    // std::thread stopGripperThread(stopGripper, std::ref(gripper));

    // create timer
    auto start_time = std::chrono::high_resolution_clock::now();

    // Publish "not ready" before the slow homing + startup init below. Homing
    // takes ~10 s, during which the init resets the command keys — any command
    // a client writes in that window would be silently wiped, leaving the
    // gripper open and unresponsive to the client's first grasp. Clients wait
    // for this to flip to "1" before issuing commands.
    redis_client.setCommandIs(GRIPPER_READY_KEY, "0");

    // Home the gripper so libfranka reports a valid max_width. Without homing,
    // readOnce().max_width can be 0, causing every open command to saturate.
    gripper.homing();

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
    gripper_desired_width_tmp = gripper_desired_width;
    gripper_desired_speed_tmp = gripper_desired_speed;
    gripper_desired_force_tmp = gripper_desired_force;
    gripper_mode_tmp = gripper_mode;
    gripper.move(gripper_desired_width, gripper_desired_speed);

    redis_client.setCommandIs(GRIPPER_DESIRED_WIDTH_KEY, std::to_string(gripper_desired_width));
    redis_client.setCommandIs(GRIPPER_DESIRED_SPEED_KEY, std::to_string(gripper_desired_speed));
    redis_client.setCommandIs(GRIPPER_DESIRED_FORCE_KEY, std::to_string(gripper_desired_force));
    redis_client.setCommandIs(GRIPPER_GRASP_SUCCESS_KEY, "0");

    // Homing + init done and the command keys are at their defaults; from here
    // client commands are honored. Flip the ready flag so waiting clients
    // proceed.
    redis_client.setCommandIs(GRIPPER_READY_KEY, "1");

    int failed_flag = 0;

    {
      std::unique_lock<std::shared_mutex> lock(redis_exchange_mutex);
      redis_exchange_data.current_width = gripper_state.width;
      redis_exchange_data.max_width = gripper_max_width;
      redis_exchange_data.desired_width = gripper_desired_width;
      redis_exchange_data.desired_speed = gripper_desired_speed;
      redis_exchange_data.desired_force = gripper_desired_force;
      redis_exchange_data.mode = gripper_mode;
      redis_exchange_data.grasp_success = "0";
      redis_exchange_data.has_state = true;
    }

    redis_thread_running.store(true, std::memory_order_relaxed);
    redis_thread = std::thread(redis_transfer);

    runloop = 1;
    while(runloop)
    {
      gripper_state = gripper.readOnce();

      if(redis_thread_failed.load(std::memory_order_relaxed)) {
        throw std::runtime_error("Gripper Redis transfer thread failed");
      }

      {
        std::unique_lock<std::shared_mutex> lock(redis_exchange_mutex, std::try_to_lock);
        if(lock.owns_lock()) {
          redis_exchange_data.current_width = gripper_state.width;
          redis_exchange_data.max_width = gripper_max_width;
          redis_exchange_data.has_state = true;
          gripper_desired_width_tmp = redis_exchange_data.desired_width;
          gripper_desired_speed_tmp = redis_exchange_data.desired_speed;
          gripper_desired_force_tmp = redis_exchange_data.desired_force;
          gripper_mode_tmp = redis_exchange_data.mode;
        }
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
          // Move the fingers to the target width and HOLD there. No retry /
          // back-off: libfranka reports move() as "failed" when the fingers
          // stop early (e.g. closing onto an object before reaching the
          // target width), but we want to keep holding that width — the old
          // behavior re-opened to current+1 cm and kept backing off every
          // loop, which released a held object on the lift.
          gripper.move(gripper_desired_width, gripper_desired_speed);
          failed_flag = 0;
        }
        else if(gripper_mode == "g")
        {
          // Mark the grasp as in flight so the Python side can tell a fresh
          // result from a stale one (grasp() below blocks until done).
          {
            std::unique_lock<std::shared_mutex> lock(redis_exchange_mutex);
            redis_exchange_data.grasp_success = GRIPPER_GRASP_PENDING;
          }
          // try {
          bool grasp_successful = gripper.grasp(0.0, gripper_desired_speed, gripper_desired_force, 1.0, 1.0);
            // begin_move_flag.store(1);
            // bool grasp_successful = gripper.grasp(gripper_desired_width, gripper_desired_speed, gripper_desired_force, 1.0, 1.0);
            // bool grasp_successful = gripper.grasp(0, gripper_desired_speed, gripper_desired_force, 0, 0);
          // Re-read the finger width after the blocking grasp. libfranka's
          // grasp() returns true even on an empty full-close given the wide
          // epsilon window above, so an object is only "held" if grasp()
          // succeeded AND the fingers ended up apart by more than
          // GRIPPER_GRASP_MIN_WIDTH.
          franka::GripperState post_grasp_state = gripper.readOnce();
          bool object_held =
              grasp_successful && (post_grasp_state.width > GRIPPER_GRASP_MIN_WIDTH);
          {
            std::unique_lock<std::shared_mutex> lock(redis_exchange_mutex);
            redis_exchange_data.current_width = post_grasp_state.width;
            redis_exchange_data.grasp_success = object_held ? "1" : "0";
            redis_exchange_data.has_state = true;
          }
          std::cout << "grasp status: " << grasp_successful
                    << " width: " << post_grasp_state.width
                    << " held: " << object_held << "\n";
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
            {
              std::unique_lock<std::shared_mutex> lock(redis_exchange_mutex);
              redis_exchange_data.desired_width = gripper_desired_width;
              redis_exchange_data.desired_width_to_write = gripper_desired_width;
              redis_exchange_data.desired_width_write_requested = true;
            }
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

    stopRedisThread();
    gripper.stop();


  } catch (const std::exception& ex) {
    // print exception
    stopRedisThread();
    std::cout << ex.what() << std::endl;
  }

  if (redis_thread_exception != nullptr) {
    try {
      std::rethrow_exception(redis_thread_exception);
    } catch (const std::exception& ex) {
      std::cout << "Gripper Redis transfer thread error: " << ex.what() << std::endl;
    }
  }



  return 0;
}
