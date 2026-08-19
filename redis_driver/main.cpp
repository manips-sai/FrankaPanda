/**
 * @file main.cpp
 * @brief Franka robot torque driver with joint safeties
*/
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include "SaiFrankaRedisClientLocal.h"
#include "SaiFrankaDriverConfig.h"
#include "tinyxml2.h"

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
std::string SAFETY_TORQUES_LOGGING_KEY;
std::string SENT_TORQUES_LOGGING_KEY;
std::string CONSTRAINED_NULLSPACE_KEY;
std::string T_ENDEFFECTOR_KEY;
std::string JOINT_TORQUES_DESIRED_KEY;
std::string TAU_EXT_HAT_FILTERED_KEY;

// user options 
// const bool USING_PANDA = false;  // user switch between Panda and FR3
// const bool USING_CONSERVATIVE_FR3 = false;  // true to use rectangular bounds, false to use configuration-based bounds 
// const bool VERBOSE = true;  // print out safety violations

// globals 
std::array<double, 7> joint_position_max_default;
std::array<double, 7> joint_position_min_default;
std::array<double, 7> joint_velocity_limits_default;
std::array<double, 7> joint_torques_limits_default;
std::array<double, 7> kv_safety;
std::array<double, 2> pos_zones;
std::array<double, 2> vel_zones;

// safety joint limits to trigger safety stop before hard limits 
std::array<double, 7> joint_position_max;
std::array<double, 7> joint_position_min;
std::array<double, 7> joint_velocity_limits;
std::array<double, 7> joint_velocity_upper_limits;
std::array<double, 7> joint_velocity_lower_limits;
std::array<double, 7> joint_torques_limits;

// safety monitoring 
const Eigen::Vector3d monitoring_point_ee_frame = Eigen::Vector3d(0.0, 0.0, 0.15);
const double safety_plane_z_coordinate = 0.28;
const double safety_cylinder_radius = 0.28;
const double safety_cylinder_height = 0.53;
bool safety_mode_flag = false;
bool safety_enabled = false;  
int safety_controller_count = 200;

// data 
Eigen::MatrixXd MassMatrix;
std::array<double, 7> tau_cmd_array{};
std::array<double, 7> q_array{};
std::array<double, 7> dq_array{};
std::array<double, 7> tau_sensed_array{};
std::array<double, 7> gravity_vector{};
std::array<double, 7> coriolis{};
std::array<double, 49> M_array{};  
std::vector<std::array<double, 7>> sensor_feedback;
std::vector<std::string> key_names;
// bool fDriverRunning = true;
// void sighandler(int sig)
// { fDriverRunning = false; }

unsigned long long counter = 0;

struct RedisSensorSnapshot {
    std::array<double, 49> mass_matrix{};
    std::array<double, 16> end_effector_transform{};
    std::array<double, 7> joint_positions{};
    std::array<double, 7> joint_velocities{};
    std::array<double, 7> joint_torques_sensed{};
    std::array<double, 7> robot_gravity{};
    std::array<double, 7> coriolis{};
    bool has_feedback = false;
};

struct RedisCommandSnapshot {
    std::array<double, 7> commanded_torques{};
    bool has_command = false;
};

template <std::size_t N>
void storeAtomicArray(std::array<std::atomic<double>, N>& destination,
                      const std::array<double, N>& source) {
    for (std::size_t i = 0; i < N; ++i) {
        destination[i].store(source[i], std::memory_order_relaxed);
    }
}

template <std::size_t N>
void loadAtomicArray(const std::array<std::atomic<double>, N>& source,
                     std::array<double, N>& destination) {
    for (std::size_t i = 0; i < N; ++i) {
        destination[i] = source[i].load(std::memory_order_relaxed);
    }
}

struct alignas(64) RedisSensorExchangeBuffer {
    std::atomic<std::uint64_t> sequence{0};
    std::atomic_bool has_feedback{false};
    std::array<std::atomic<double>, 49> mass_matrix{};
    std::array<std::atomic<double>, 16> end_effector_transform{};
    std::array<std::atomic<double>, 7> joint_positions{};
    std::array<std::atomic<double>, 7> joint_velocities{};
    std::array<std::atomic<double>, 7> joint_torques_sensed{};
    std::array<std::atomic<double>, 7> robot_gravity{};
    std::array<std::atomic<double>, 7> coriolis{};
};

struct alignas(64) RedisCommandExchangeBuffer {
    std::atomic<std::uint64_t> sequence{0};
    std::atomic_bool has_command{false};
    std::array<std::atomic<double>, 7> commanded_torques{};
};

void publishRedisSensorSnapshot(RedisSensorExchangeBuffer& buffer,
                                const RedisSensorSnapshot& snapshot) {
    const std::uint64_t sequence = buffer.sequence.load(std::memory_order_relaxed);
    buffer.sequence.store(sequence + 1, std::memory_order_release);
    storeAtomicArray(buffer.mass_matrix, snapshot.mass_matrix);
    storeAtomicArray(buffer.end_effector_transform, snapshot.end_effector_transform);
    storeAtomicArray(buffer.joint_positions, snapshot.joint_positions);
    storeAtomicArray(buffer.joint_velocities, snapshot.joint_velocities);
    storeAtomicArray(buffer.joint_torques_sensed, snapshot.joint_torques_sensed);
    storeAtomicArray(buffer.robot_gravity, snapshot.robot_gravity);
    storeAtomicArray(buffer.coriolis, snapshot.coriolis);
    buffer.has_feedback.store(snapshot.has_feedback, std::memory_order_relaxed);
    buffer.sequence.store(sequence + 2, std::memory_order_release);
}

bool tryReadRedisSensorSnapshot(const RedisSensorExchangeBuffer& buffer,
                                RedisSensorSnapshot& snapshot) {
    const std::uint64_t start_sequence = buffer.sequence.load(std::memory_order_acquire);
    if ((start_sequence & 1) != 0 ||
        !buffer.has_feedback.load(std::memory_order_relaxed)) {
        return false;
    }

    loadAtomicArray(buffer.mass_matrix, snapshot.mass_matrix);
    loadAtomicArray(buffer.end_effector_transform, snapshot.end_effector_transform);
    loadAtomicArray(buffer.joint_positions, snapshot.joint_positions);
    loadAtomicArray(buffer.joint_velocities, snapshot.joint_velocities);
    loadAtomicArray(buffer.joint_torques_sensed, snapshot.joint_torques_sensed);
    loadAtomicArray(buffer.robot_gravity, snapshot.robot_gravity);
    loadAtomicArray(buffer.coriolis, snapshot.coriolis);
    snapshot.has_feedback = true;

    const std::uint64_t end_sequence = buffer.sequence.load(std::memory_order_acquire);
    return start_sequence == end_sequence && (end_sequence & 1) == 0;
}

void publishRedisCommandSnapshot(RedisCommandExchangeBuffer& buffer,
                                 const RedisCommandSnapshot& snapshot) {
    const std::uint64_t sequence = buffer.sequence.load(std::memory_order_relaxed);
    buffer.sequence.store(sequence + 1, std::memory_order_release);
    storeAtomicArray(buffer.commanded_torques, snapshot.commanded_torques);
    buffer.has_command.store(snapshot.has_command, std::memory_order_relaxed);
    buffer.sequence.store(sequence + 2, std::memory_order_release);
}

bool tryReadRedisCommandSnapshot(const RedisCommandExchangeBuffer& buffer,
                                 RedisCommandSnapshot& snapshot) {
    const std::uint64_t start_sequence = buffer.sequence.load(std::memory_order_acquire);
    if ((start_sequence & 1) != 0 ||
        !buffer.has_command.load(std::memory_order_relaxed)) {
        return false;
    }

    loadAtomicArray(buffer.commanded_torques, snapshot.commanded_torques);
    snapshot.has_command = true;

    const std::uint64_t end_sequence = buffer.sequence.load(std::memory_order_acquire);
    return start_sequence == end_sequence && (end_sequence & 1) == 0;
}

enum Limit {
    SAFE = 0,
    MIN_SOFT,  // soft lower position limit
    MIN_HARD,  // hard lower position limit 
    MAX_SOFT,  // soft upper position limit 
    MAX_HARD,  // hard upper position limit 
    MIN_SOFT_VEL,  
    MIN_HARD_VEL,
    MAX_SOFT_VEL, 
    MAX_HARD_VEL
};

const std::vector<std::string> limit_state {"Safe", "Soft Min", "Hard Min", "Soft Max", "Hard Max", "Min Soft Vel", "Min Hard Vel", "Max Soft Vel", "Max Hard Vel"};

double getBlendingCoeff(const double& val, const double& low, const double& high) {
    return std::clamp((val - low) / (high - low), 0., 1.);
}

std::array<double, 7> getMaxJointVelocity(std::array<double, 7>& q) {
    std::array<double, 7> dq_max;
    dq_max[0] = std::min(joint_velocity_limits_default[0], std::max(0., -0.3 + sqrt(std::max(0., 12.0 * (2.75010 - q[0])))));
    dq_max[1] = std::min(joint_velocity_limits_default[1], std::max(0., -0.2 + sqrt(std::max(0., 5.17 * (1.79180 - q[1])))));
    dq_max[2] = std::min(joint_velocity_limits_default[2], std::max(0., -0.2 + sqrt(std::max(0., 7.00 * (2.90650 - q[2])))));
    dq_max[3] = std::min(joint_velocity_limits_default[3], std::max(0., -0.3 + sqrt(std::max(0., 8.00 * (-0.1458 - q[3])))));
    dq_max[4] = std::min(joint_velocity_limits_default[4], std::max(0., -0.35 + sqrt(std::max(0., 34.0 * (2.81010 - q[4])))));
    dq_max[5] = std::min(joint_velocity_limits_default[5], std::max(0., -0.35 + sqrt(std::max(0., 11.0 * (4.52050 - q[5])))));
    dq_max[6] = std::min(joint_velocity_limits_default[6], std::max(0., -0.35 + sqrt(std::max(0., 34.0 * (3.01960 - q[6])))));
    return dq_max;
}

std::array<double, 7> getMinJointVelocity(std::array<double, 7>& q) {
    std::array<double, 7> dq_min;
    dq_min[0] = std::max(-joint_velocity_limits_default[0], std::min(0., 0.3 - sqrt(std::max(0., 12.0 * (2.75010 + q[0])))));
    dq_min[1] = std::max(-joint_velocity_limits_default[1], std::min(0., 0.2 - sqrt(std::max(0., 5.17 * (1.79180 + q[1])))));
    dq_min[2] = std::max(-joint_velocity_limits_default[2], std::min(0., 0.2 - sqrt(std::max(0., 7.00 * (2.90650 + q[2])))));
    dq_min[3] = std::max(-joint_velocity_limits_default[3], std::min(0., 0.3 - sqrt(std::max(0., 8.00 * (3.04810 + q[3])))));
    dq_min[4] = std::max(-joint_velocity_limits_default[4], std::min(0., 0.35 - sqrt(std::max(0., 34.0 * (2.81010 + q[4])))));
    dq_min[5] = std::max(-joint_velocity_limits_default[5], std::min(0., 0.35 - sqrt(std::max(0., 11.0 * (-0.54092 + q[5])))));
    dq_min[6] = std::max(-joint_velocity_limits_default[6], std::min(0., 0.35 - sqrt(std::max(0., 34.0 * (3.01960 + q[6])))));
    return dq_min;
}

Sai::Franka::DriverConfig config;

int main (int argc, char** argv) {

	std::string config_file = "default_config.xml";

    if (argc > 1) {
        config_file = argv[1];
    }
	std::string config_file_path = std::string(CONFIG_FOLDER) + "/" + config_file;

	config = Sai::Franka::loadConfig(config_file_path);
	std::string redis_prefix = config.redis_prefix.empty() ? "" : config.redis_prefix + "::";

    JOINT_TORQUES_COMMANDED_KEY = redis_prefix + "commands::" + config.robot_name + "::control_torques";
    JOINT_ANGLES_KEY = redis_prefix + "sensors::" + config.robot_name + "::joint_positions";
    JOINT_VELOCITIES_KEY = redis_prefix + "sensors::" + config.robot_name + "::joint_velocities";
    JOINT_TORQUES_SENSED_KEY = redis_prefix + "sensors::" + config.robot_name + "::joint_torques";
    MASSMATRIX_KEY = redis_prefix + "sensors::" + config.robot_name + "::model::mass_matrix";
    CORIOLIS_KEY = redis_prefix + "sensors::" + config.robot_name + "::model::coriolis";
    ROBOT_GRAVITY_KEY = redis_prefix + "sensors::" + config.robot_name + "::model::robot_gravity";
    SAFETY_TORQUES_LOGGING_KEY = redis_prefix + "redis_driver::" + config.robot_name + "::safety_controller::safety_torques";
    SENT_TORQUES_LOGGING_KEY = redis_prefix + "redis_driver::" + config.robot_name + "::safety_controller::sent_torques";
    CONSTRAINED_NULLSPACE_KEY = redis_prefix + "redis_driver::" + config.robot_name + "::safety_controller::constraint_nullspace";
    T_ENDEFFECTOR_KEY = redis_prefix + "redis_driver::" + config.robot_name + "::T_end_effector";
    JOINT_TORQUES_DESIRED_KEY = redis_prefix + "sensors::" + config.robot_name + "::joint_torques_desired";
    TAU_EXT_HAT_FILTERED_KEY = redis_prefix + "sensors::" + config.robot_name + "::tau_ext_hat_filtered";

    // start redis client
    Sai::Franka::CDatabaseRedisClient* redis_client;
    Sai::Franka::HiredisServerInfo info;
    info.hostname_ = "127.0.0.1";
    info.port_ = 6379;
    info.timeout_ = { 1, 500000 }; // 1.5 seconds
    redis_client = new Sai::Franka::CDatabaseRedisClient();
    redis_client->serverIs(info);

    // // set up signal handler
    // signal(SIGABRT, &sighandler);
    // signal(SIGTERM, &sighandler);
    // signal(SIGINT, &sighandler);

    // debug_function(redis_client);

    for (int i = 0; i < 7; ++i) {
        tau_cmd_array[i] = 0;
    }

    redis_client->setDoubleArray(JOINT_TORQUES_COMMANDED_KEY, tau_cmd_array, 7);
    // safety to detect if controller is already running : wait 50 milliseconds
    usleep(50000);
    redis_client->getDoubleArray(JOINT_TORQUES_COMMANDED_KEY, tau_cmd_array, 7);
    for(int i = 0; i < 7; ++i) {
        if (tau_cmd_array[i] != 0) {
            std::cout << "Stop the controller before running the driver\n" << "\n";
            return -1;
        }
    }

    // prepare batch command
    key_names.push_back(JOINT_TORQUES_COMMANDED_KEY);
    key_names.push_back(MASSMATRIX_KEY);
    key_names.push_back(T_ENDEFFECTOR_KEY);
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

    if (config.robot_type == Sai::Franka::RobotType::FRANKA_PANDA) {
        std::cout << "Using Franka Panda specifications\n";
        // Panda specifications 
        joint_position_max_default = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
        joint_position_min_default = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
        joint_velocity_limits_default = {2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};
        joint_torques_limits_default = {87, 87, 87, 87, 12, 12, 12};

        // damping gains 
        // kv_safety = {20.0, 20.0, 20.0, 15.0, 10.0, 10.0, 5.0};
        kv_safety = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 5.0};

        // zone definitions
        pos_zones = {6., 9.};  // hard, soft
        // vel_zones = {5., 7.};  // hard, soft
        vel_zones = {6., 8.};  // hard, soft  (8, 6)
    } else if (config.robot_type == Sai::Franka::RobotType::FRANKA_RESEARCH_3) {
        std::cout << "Using FR3 specifications\n";
        // FR3 specifications
        joint_position_max_default = {2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159};
        joint_position_min_default = {-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159};
        // joint_velocity_limits_default = {2, 1, 1.5, 1.25, 3, 1.5, 3};  // FR3 conservative rectangle bounds 
        joint_velocity_limits_default = {3, 3, 3, 3, 3, 3, 3};  // FR3 conservative rectangle bounds 
        joint_torques_limits_default = {87, 87, 87, 87, 12, 12, 12};

        // damping gains 
        kv_safety = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};

        // zone definitions
        pos_zones = {6., 9.};  
        vel_zones = {6., 8.};
    } else {
		std::cout << "Unknown robot type\n";
		return -1;
	}

    // limit options
    bool _pos_limit_opt = true;
    bool _vel_limit_opt = false;

    // setup joint limit avoidance 
    std::vector<int> _pos_limit_flag(7, SAFE);
    std::vector<int> _vel_limit_flag(7, SAFE);
    Eigen::MatrixXd _J_s;  // constraint task jacobian for nullspace projection (n_limited x dof)   
    Eigen::MatrixXd _Lambda_s;  // op-space matrix
    Eigen::MatrixXd _Jbar_s;
    Eigen::MatrixXd _N_s = Eigen::MatrixXd::Identity(7, 7);  // nullspace matrix with the constraint task jacobian 
    Eigen::VectorXi _limited_joints(7);  // 1 or 0 depending on whether the joint is limited  
    Eigen::VectorXd _tau_limited = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd _torque_scaling_vector = Eigen::VectorXd::Ones(7);  // tau scaling based on the velocity signal 

    // override with new safety set (this set triggers software stop)
    double default_sf = 0.98;  // max violation safety factor 
    for (int i = 0; i < 7; ++i) {
        joint_position_max[i] = default_sf * joint_position_max_default[i];
        joint_position_min[i] = default_sf * joint_position_min_default[i];
        joint_velocity_limits[i] = default_sf * joint_velocity_limits_default[i];
        joint_torques_limits[i] = default_sf * joint_torques_limits_default[i];
    }

    // zone 1 and 2 definitions subject to tuning
    double soft_sf = 0.90;  // start of damping zone 
    double hard_sf = 0.95;  // start of feedback zone 
    double angle_tol = 1 * M_PI / 180;  // rad 
    double vel_tol = 0.1;  // rad/s (0.1 = 5 deg/s)
    double q_tol = 1e-1 * M_PI / 180;  

    Eigen::VectorXd soft_min_angles(7);
    Eigen::VectorXd soft_max_angles(7);
    Eigen::VectorXd hard_min_angles(7);
    Eigen::VectorXd hard_max_angles(7);
    Eigen::VectorXd soft_min_joint_velocity_limits(7);
    Eigen::VectorXd hard_min_joint_velocity_limits(7);
    Eigen::VectorXd soft_max_joint_velocity_limits(7);
    Eigen::VectorXd hard_max_joint_velocity_limits(7);

    for (int i = 0; i < 7; ++i) {
        soft_min_angles(i) = joint_position_min[i] + pos_zones[1] * angle_tol;
        hard_min_angles(i) = joint_position_min[i] + pos_zones[0] * angle_tol;
        soft_max_angles(i) = joint_position_max[i] - pos_zones[1] * angle_tol;
        hard_max_angles(i) = joint_position_max[i] - pos_zones[0] * angle_tol;
        soft_min_joint_velocity_limits(i) = - joint_velocity_limits[i] + vel_zones[1] * vel_tol;
        hard_min_joint_velocity_limits(i) = - joint_velocity_limits[i] + vel_zones[0] * vel_tol;
        soft_max_joint_velocity_limits(i) = joint_velocity_limits[i] - vel_zones[1] * vel_tol;
        hard_max_joint_velocity_limits(i) = joint_velocity_limits[i] - vel_zones[0] * vel_tol;

        // // specific joint offsets 
        // if (i == 6) {
        //     soft_min_joint_velocity_limits(i) += 4.5 * vel_tol;
        //     hard_min_joint_velocity_limits(i) += 4.5 * vel_tol;
        //     soft_max_joint_velocity_limits(i) -= 4.5 * vel_tol;
        //     hard_max_joint_velocity_limits(i) -= 4.5 * vel_tol;
        // }
    }

    // timing 
    std::clock_t start;
    double duration;

    std::array<RedisSensorExchangeBuffer, 2> redis_sensor_buffers;
    std::array<RedisCommandExchangeBuffer, 2> redis_command_buffers;
    std::atomic<int> published_sensor_buffer_index{0};
    std::atomic<int> published_command_buffer_index{0};
    std::atomic_bool redis_thread_running{false};
    std::atomic_bool redis_thread_failed{false};
    std::exception_ptr redis_thread_exception = nullptr;
    std::mutex redis_thread_exception_mutex;
    std::array<double, 7> latest_redis_tau_cmd_array = tau_cmd_array;

    RedisCommandSnapshot initial_command_snapshot;
    initial_command_snapshot.commanded_torques = tau_cmd_array;
    initial_command_snapshot.has_command = true;
    publishRedisCommandSnapshot(redis_command_buffers[0], initial_command_snapshot);
    publishRedisCommandSnapshot(redis_command_buffers[1], initial_command_snapshot);

    auto redis_transfer = [&]() {
        std::array<double, 7> local_tau_cmd_array{};
        std::vector<std::array<double, 7>> local_sensor_feedback(5);
        RedisSensorSnapshot sensor_snapshot;
        RedisCommandSnapshot command_snapshot;
        const auto redis_period = std::chrono::microseconds(1000);
        auto next_redis_tick = std::chrono::steady_clock::now();

        try {
            while (redis_thread_running.load(std::memory_order_relaxed)) {
                next_redis_tick += redis_period;
                const int sensor_buffer_index =
                    published_sensor_buffer_index.load(std::memory_order_acquire);
                bool has_feedback =
                    tryReadRedisSensorSnapshot(redis_sensor_buffers[sensor_buffer_index],
                                               sensor_snapshot);
                if (!has_feedback) {
                    has_feedback =
                        tryReadRedisSensorSnapshot(redis_sensor_buffers[1 - sensor_buffer_index],
                                                   sensor_snapshot);
                }

                if (!has_feedback) {
                    std::this_thread::sleep_until(next_redis_tick);
                    continue;
                }

                local_sensor_feedback[0] = sensor_snapshot.joint_positions;
                local_sensor_feedback[1] = sensor_snapshot.joint_velocities;
                local_sensor_feedback[2] = sensor_snapshot.joint_torques_sensed;
                local_sensor_feedback[3] = sensor_snapshot.robot_gravity;
                local_sensor_feedback[4] = sensor_snapshot.coriolis;

                Eigen::Map<const Eigen::Matrix<double, 7, 7>> local_MassMatrix(
                    sensor_snapshot.mass_matrix.data());
                Eigen::Map<const Eigen::Matrix<double, 4, 4>> local_T_ee(
                    sensor_snapshot.end_effector_transform.data());
                redis_client->setGetBatchCommands(key_names,
                                                  local_tau_cmd_array,
                                                  local_MassMatrix,
                                                  local_T_ee,
                                                  local_sensor_feedback);

                command_snapshot.commanded_torques = local_tau_cmd_array;
                command_snapshot.has_command = true;
                const int command_write_buffer_index =
                    1 - published_command_buffer_index.load(std::memory_order_relaxed);
                publishRedisCommandSnapshot(redis_command_buffers[command_write_buffer_index],
                                            command_snapshot);
                published_command_buffer_index.store(command_write_buffer_index,
                                                     std::memory_order_release);

                const auto now = std::chrono::steady_clock::now();
                if (next_redis_tick > now) {
                    std::this_thread::sleep_until(next_redis_tick);
                } else {
                    next_redis_tick = now;
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

    try {

    // connect to robot and gripper
    franka::Robot robot(config.robot_ip);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    // set collision behavior
    robot.setCollisionBehavior({{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}},
                                {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}},
                                {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}},
                                {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}});

    auto torque_control_callback = [&](const franka::RobotState& robot_state,
                                        franka::Duration period) -> franka::Torques 
    {
        // start = std::clock();
        sensor_feedback[0] = robot_state.q;
        sensor_feedback[1] = robot_state.dq;
        sensor_feedback[2] = robot_state.tau_J;
        sensor_feedback[3] = model.gravity(robot_state);
        sensor_feedback[4] = model.coriolis(robot_state);

        // // Publish desired torques and external torque estimate for impact diagnostics (at 100 Hz).
        // // tau_J_d: firmware expected joint torques.
        // // tau_ext_hat_filtered: model-based external torque estimate; ball impact shows up here.
        // // Throttled to every 10 ticks to avoid adding latency to the 1 kHz loop.
        // if (counter % 10 == 0) {
        //     redis_client->setDoubleArray(JOINT_TORQUES_DESIRED_KEY, robot_state.tau_J_d, 7);
        //     redis_client->setDoubleArray(TAU_EXT_HAT_FILTERED_KEY, robot_state.tau_ext_hat_filtered, 7);
        // }

        // compute joint velocity limits at configuration if using FR3 and not using conservative bounds 
        if (config.robot_type == Sai::Franka::RobotType::FRANKA_RESEARCH_3 && !config.use_conservative_bounds) {
            joint_velocity_lower_limits = getMinJointVelocity(sensor_feedback[0]);
            joint_velocity_upper_limits = getMaxJointVelocity(sensor_feedback[0]);
            for (int i = 0; i < 7; ++i) {
                soft_min_joint_velocity_limits(i) = joint_velocity_lower_limits[i] + pos_zones[1] * vel_tol;
                hard_min_joint_velocity_limits(i) = joint_velocity_lower_limits[i] + pos_zones[0] * vel_tol;
                soft_max_joint_velocity_limits(i) = joint_velocity_upper_limits[i] - pos_zones[1] * vel_tol;
                hard_max_joint_velocity_limits(i) = joint_velocity_upper_limits[i] - pos_zones[0] * vel_tol;
                
                // // specific value for last joint
                // if (i == 6) {
                //     soft_min_joint_velocity_limits(i) += 4.5 * vel_tol;
                //     hard_min_joint_velocity_limits(i) += 4.5 * vel_tol;
                //     soft_max_joint_velocity_limits(i) -= 4.5 * vel_tol;
                //     hard_max_joint_velocity_limits(i) -= 4.5 * vel_tol;
                // }
            }
        }

        // publish transformation matrices
        // Get end-effector pose relative to base frame
        // O_T_EE is a std::array<double, 16> in column-major format
        std::array<double, 16> pose_array = model.pose(
            franka::Frame::kFlange,
            robot_state);

        // redis_client->setEigenMatrixDerived(T_ENDEFFECTOR_KEY, T);

        M_array = model.mass(robot_state);

        if (redis_thread_failed.load(std::memory_order_relaxed)) {
            throw std::runtime_error("Redis transfer thread failed");
        }

        RedisCommandSnapshot command_snapshot;
        const int command_buffer_index =
            published_command_buffer_index.load(std::memory_order_acquire);
        bool has_command =
            tryReadRedisCommandSnapshot(redis_command_buffers[command_buffer_index],
                                        command_snapshot);
        if (!has_command) {
            has_command =
                tryReadRedisCommandSnapshot(redis_command_buffers[1 - command_buffer_index],
                                            command_snapshot);
        }
        if (has_command) {
            latest_redis_tau_cmd_array = command_snapshot.commanded_torques;
        }
        tau_cmd_array = latest_redis_tau_cmd_array;

        RedisSensorSnapshot sensor_snapshot;
        sensor_snapshot.mass_matrix = M_array;
        sensor_snapshot.end_effector_transform = pose_array;
        sensor_snapshot.joint_positions = sensor_feedback[0];
        sensor_snapshot.joint_velocities = sensor_feedback[1];
        sensor_snapshot.joint_torques_sensed = sensor_feedback[2];
        sensor_snapshot.robot_gravity = sensor_feedback[3];
        sensor_snapshot.coriolis = sensor_feedback[4];
        sensor_snapshot.has_feedback = true;
        const int sensor_write_buffer_index =
            1 - published_sensor_buffer_index.load(std::memory_order_relaxed);
        publishRedisSensorSnapshot(redis_sensor_buffers[sensor_write_buffer_index],
                                   sensor_snapshot);
        published_sensor_buffer_index.store(sensor_write_buffer_index,
                                            std::memory_order_release);

        Eigen::Map<const Eigen::Matrix<double, 7, 7>> MassMatrix(M_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> _tau(tau_cmd_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> _sensed_torques(sensor_feedback[2].data());  // sensed torques 
        Eigen::Map<Eigen::Matrix<double, 7, 1>> _coriolis(sensor_feedback[4].data());

        // reset containers
        _limited_joints.setZero();  // used to form the constraint jacobian 
        _N_s.setIdentity();

        // position limit saturation 
        if (_pos_limit_opt) {          
            for (int i = 0; i < 7; ++i) {
                double curr_q = robot_state.q[i];
                if (curr_q > soft_min_angles(i) && curr_q < soft_max_angles(i)) {
                    _pos_limit_flag[i] = SAFE;
                } else if (curr_q < hard_min_angles(i)) {
                    _pos_limit_flag[i] = MIN_HARD;
                    _limited_joints(i) = 1;
                } else if (curr_q < soft_min_angles(i)) {
                    _pos_limit_flag[i] = MIN_SOFT;
                    _limited_joints(i) = 1;
                } else if (curr_q > hard_max_angles(i)) {
                    _pos_limit_flag[i] = MAX_HARD;
                    _limited_joints(i) = 1;
                } else if (curr_q > soft_max_angles(i)) {
                    _pos_limit_flag[i] = MAX_SOFT;   
                    _limited_joints(i) = 1;       
                }
            }
        }

        // joint velocity limit saturation (lower priority than joint position limit; _tau_unit_limited will be overwritten)  
        if (_vel_limit_opt) {
            for (int i = 0; i < 7; ++i) {
                if (_pos_limit_flag[i] == SAFE) {
                    if (robot_state.dq[i] > soft_min_joint_velocity_limits[i] && robot_state.dq[i] < soft_max_joint_velocity_limits[i]) {
                        _vel_limit_flag[i] = SAFE;
                    } else if (robot_state.dq[i] > hard_max_joint_velocity_limits[i]) {
                        _vel_limit_flag[i] = MAX_HARD_VEL;
                        _limited_joints(i) = 1;   
                    } else if (robot_state.dq[i] > soft_max_joint_velocity_limits[i]) {
                        _vel_limit_flag[i] = MAX_SOFT_VEL;
                        _limited_joints(i) = 1;
                    } else if (robot_state.dq[i] < hard_min_joint_velocity_limits[i]) {
                        _vel_limit_flag[i] = MIN_HARD_VEL;
                        _limited_joints(i) = 1;
                    } else if (robot_state.dq[i] < soft_min_joint_velocity_limits[i]) {
                        _vel_limit_flag[i] = MIN_SOFT_VEL;
                        _limited_joints(i) = 1;
                    } 
                } else {
                    _vel_limit_flag[i] = SAFE;
                }
            } 
        }

        // safety verbose output
        if (config.verbose) {
            for (int i = 0; i < 7; ++i) {
                if (_pos_limit_flag[i] != SAFE) {
                    std::cout << counter << ": Joint " << i << " State: " << limit_state[_pos_limit_flag[i]] << "\n";
                    std::cout << "---\n";
                }
                if (_vel_limit_flag[i] != SAFE) {
                    std::cout << counter << ": Joint " << i << " State: " << limit_state[_vel_limit_flag[i]] << "\n";
                    std::cout << "---\n";
                }
            }
        }

        int n_limited_joints = _limited_joints.sum();
        _tau_limited = Eigen::VectorXd::Zero(7);
        _torque_scaling_vector = Eigen::VectorXd::Ones(7);

        if (n_limited_joints > 0) {
            for (int i = 0; i < 7; ++i) {
                if (_pos_limit_flag[i] == MIN_SOFT) {
                    // ramping kv damping proportional to violation difference up to max damping + command torques (goes from tau -> tau + max damping)
                    // double kv = kv_safety[i] * (robot_state.q[i] - soft_min_angles[i]) / (hard_min_angles[i] - soft_min_angles[i]);
                    double alpha = getBlendingCoeff(robot_state.q[i], soft_min_angles[i], hard_min_angles[i]);
                    _tau_limited(i) = _tau(i) - alpha * kv_safety[i] * robot_state.dq[i];

                } else if (_pos_limit_flag[i] == MAX_SOFT) {
                    // same as above, but for the upper soft limit 
                    // double kv = kv_safety[i] * (robot_state.q[i] - soft_max_angles[i]) / (hard_max_angles[i] - soft_max_angles[i]);
                    double alpha = getBlendingCoeff(robot_state.q[i], soft_max_angles[i], hard_max_angles[i]);
                    _tau_limited(i) = _tau(i) - alpha * kv_safety[i] * robot_state.dq[i];

                } else if (_pos_limit_flag[i] == MIN_HARD) {
                    // max damping + command torques blend with quadratic ramping (goes from tau + max damping -> holding tau + max damping with blending from tau to holding tau)
                    // double dist = (robot_state.q[i] - hard_min_angles[i]) / (joint_position_min[i] - hard_min_angles[i]);
                    double alpha = getBlendingCoeff(robot_state.q[i], hard_min_angles[i], joint_position_min[i]);
                    double tau_hold = joint_torques_limits_default[i];
                    _tau_limited(i) = (1 - std::pow(alpha, 2)) * _tau(i) + std::pow(alpha, 2) * tau_hold - kv_safety[i] * robot_state.dq[i];

                } else if (_pos_limit_flag[i] == MAX_HARD) {      
                    // same as above, but for the upper hard limit 
                    // double dist = (robot_state.q[i] - hard_max_angles[i]) / (joint_position_max[i] - hard_max_angles[i]);
                    double alpha = getBlendingCoeff(robot_state.q[i], hard_max_angles[i], joint_position_max[i]);
                    double tau_hold = - joint_torques_limits_default[i];
                    _tau_limited(i) = (1 - std::pow(alpha, 2)) * _tau(i) + std::pow(alpha, 2) * tau_hold - kv_safety[i] * robot_state.dq[i]; 

                } else if (_vel_limit_flag[i] == MIN_SOFT_VEL) {
                    // command torques blend with holding torques to soft joint velocity (goes from tau -> tau hold with blending)
                    double alpha = getBlendingCoeff(robot_state.dq[i], soft_min_joint_velocity_limits[i], hard_min_joint_velocity_limits[i]);
                    _tau_limited(i) = (1 - std::pow(alpha, 1)) * _tau(i);
                    // _torque_scaling_vector(i) = (1 - std::pow(alpha, 4));
                    // _torque_scaling_vector(i) = soft_min_joint_velocity_limits[i] / robot_state.dq[i];
                    // _tau(i) = 0;

                } else if (_vel_limit_flag[i] == MAX_SOFT_VEL) {
                    // same as above, but for the upper soft limit 
                    double alpha = getBlendingCoeff(robot_state.dq[i], soft_max_joint_velocity_limits[i], hard_max_joint_velocity_limits[i]);
                    _tau_limited(i) = (1 - std::pow(alpha, 1)) * _tau(i);
                    // _torque_scaling_vector(i) = (1 - std::pow(alpha, 4));
                    // _torque_scaling_vector(i) = soft_max_joint_velocity_limits[i] / robot_state.dq[i];
                    // _tau(i) = 0;

                } else if (_vel_limit_flag[i] == MIN_HARD_VEL) {
                    // full damping with soft joint velocity limits goal and a quadratic ramp to max torque 
                    double alpha = getBlendingCoeff(robot_state.dq[i], hard_min_joint_velocity_limits[i], - joint_velocity_limits_default[i]);
                    // _tau_vel_limited(i) = - kv_safety[i] * std::pow(alpha, 2) * (robot_state.dq[i] - 0 * hard_min_joint_velocity_limits[i]);
                    // _torque_scaling_vector(i) = soft_min_joint_velocity_limits[i] / robot_state.dq[i];
                    // _torque_scaling_vector(i) = 0;
                    // _tau_limited(i) = - kv_safety[i] * std::pow(alpha, 4) * (robot_state.dq[i] - 0 * hard_min_joint_velocity_limits[i]);
                    // _tau_limited(i) = - kv_safety[i] * std::pow(alpha, 4) * (robot_state.dq[i] - 0 * hard_min_joint_velocity_limits[i]);
                    // _tau_limited(i) = - kv_safety[i] * std::pow(alpha, 4) * (robot_state.dq[i] - 0 * hard_min_joint_velocity_limits[i]);
                    _tau_limited(i) = std::pow(alpha, 4) * joint_torques_limits_default[i] * 1e-2;
                    // _tau(i) = 0;  

                } else if (_vel_limit_flag[i] == MAX_HARD_VEL) {
                    // same as above, but for the upper hard limit 
                    double alpha = getBlendingCoeff(robot_state.dq[i], hard_max_joint_velocity_limits[i], joint_velocity_limits_default[i]);
                    // _tau_vel_limited(i) = - kv_safety[i] * std::pow(alpha, 2) * (robot_state.dq[i] - 0 * hard_max_joint_velocity_limits[i]);
                    // _torque_scaling_vector(i) = 0;
                    // _torque_scaling_vector(i) = soft_max_joint_velocity_limits[i] / robot_state.dq[i];
                    // _tau_limited(i) = - kv_safety[i] * std::pow(alpha, 4) * (robot_state.dq[i] - 0 * hard_max_joint_velocity_limits[i]);
                    _tau_limited(i) = - std::pow(alpha, 4) * joint_torques_limits_default[i] * 1e-2;
                    // _tau(i) = 0;  
                }
            }

            // compute revised torques 
            _J_s = Eigen::MatrixXd::Zero(n_limited_joints, 7);
            int cnt = 0;
            for (int i = 0; i < 7; ++i) {
                if (_limited_joints(i)) {
                    _J_s(cnt, i) = 1;
                    cnt++;
                }
            }
            Eigen::MatrixXd MassMatrixInverse = MassMatrix.llt().solve(Eigen::MatrixXd::Identity(7, 7));
            _Lambda_s = (_J_s * MassMatrixInverse * _J_s.transpose()).inverse();
            _Jbar_s = MassMatrixInverse * _J_s.transpose() * _Lambda_s;
            _N_s = Eigen::MatrixXd::Identity(7, 7) - _Jbar_s * _J_s;           
            _tau = _tau_limited + _N_s.transpose() * _tau;  

            for (int i = 0; i < 7; ++i) {
                tau_cmd_array[i] = _tau(i);
            }
        }

        // // safey keys
        // _N_s.setIdentity();
        // redis_client->setEigenMatrixDerived(SAFETY_TORQUES_LOGGING_KEY, _tau_limited);
        // redis_client->setEigenMatrixDerived(SENT_TORQUES_LOGGING_KEY, _tau);
        // redis_client->setEigenMatrixDerived(CONSTRAINED_NULLSPACE_KEY, _N_s);

        // safety checks
        // joint torques, velocity and positions
        for (int i = 0; i < 7; ++i) {
            // torque saturation
            if (tau_cmd_array[i] > joint_torques_limits[i]) {
                std::cout << "WARNING : Torque commanded on joint " << i << " too high (" << tau_cmd_array[i];
                std::cout << "), saturating to max value(" << joint_torques_limits[i] << ")" << std::endl;
                tau_cmd_array[i] = joint_torques_limits[i];
            }

            if (tau_cmd_array[i] < -joint_torques_limits[i]) {
                std::cout << "WARNING : Torque commanded on joint " << i << " too low (" << tau_cmd_array[i];
                std::cout << "), saturating to min value(" << -joint_torques_limits[i] << ")" << std::endl;
                tau_cmd_array[i] = -joint_torques_limits[i];
            }
            // // position limit
            // if(robot_state.q[i] > joint_position_max[i])
            // {
            //     safety_mode_flag = true;
            //     if(safety_controller_count == 200)
            //     {
            //     std::cout << "WARNING : Soft joint upper limit violated on joint " << i << ", engaging safety mode" << std::endl;
            //     }
            // }
            // if(robot_state.q[i] < joint_position_min[i])
            // {
            //     safety_mode_flag = true;
            //     if(safety_controller_count == 200)
            //     {
            //     std::cout << "WARNING : Soft joint lower limit violated on joint " << i << ", engaging safety mode" << std::endl;
            //     }
            // }
            // // velocity limit
            // if(abs(robot_state.dq[i]) > joint_velocity_limits[i])
            // {
            //     safety_mode_flag = true;
            //     if(safety_controller_count == 200)
            //     {
            //     std::cout << "WARNING : Soft velocity limit violated on joint " << i << ", engaging safety mode" << std::endl;
            //     }
            // }
        }

        if (safety_enabled) {
            // cartesian checks
            Eigen::Affine3d T_EE(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
            Eigen::Vector3d pos_monitoring_point = T_EE*monitoring_point_ee_frame;
            double radius_square = pos_monitoring_point(0)*pos_monitoring_point(0) + pos_monitoring_point(1)*pos_monitoring_point(1);
            double z_ee = pos_monitoring_point(2);

            // lower plane
            if (z_ee < safety_plane_z_coordinate) {
                safety_mode_flag = true;
                if (safety_controller_count == 200) {
                    std::cout << "WARNING : End effector too low, engaging safety mode" << std::endl;
                    std::cout << "position of monitoring point : " << pos_monitoring_point.transpose() << std::endl;
                }       
            }
            // cylinder
            if (z_ee < safety_cylinder_height && radius_square < safety_cylinder_radius*safety_cylinder_radius) {
                safety_mode_flag = true;
                if (safety_controller_count == 200) {
                    std::cout << "WARNING : End effector too close to center of workspace, engaging safety mode" << std::endl;
                    std::cout << "position of monitoring point : " << pos_monitoring_point.transpose() << std::endl;
                }       
            }
            // std::cout << pos_monitoring_point.transpose() << std::endl;
        }

        // safety mode
        if (safety_mode_flag) {
            for (int i = 0; i < 7; ++i) {
                tau_cmd_array[i] = -kv_safety[i] * robot_state.dq[i];
            }

            if (safety_controller_count == 0) {
                throw std::runtime_error("Stopping driver due to safety violation");
            }
            safety_controller_count--;
        }

        counter++;
        return tau_cmd_array;
    };

    // start real-time control loop
    // limit_rate=true enables libfranka's built-in torque rate clamping to
    // kMaxTorqueRate (1000 Nm/s per joint) before commands reach the firmware.
    // Without this, OpenSai's PD controller can produce |Δτ|/Δt >> 1000 Nm/s
    // at goal transitions (e.g. flick start from rest), firing
    // joint_motion_generator_acceleration_discontinuity.
    redis_thread_running.store(true, std::memory_order_relaxed);
    redis_thread = std::thread(redis_transfer);
    robot.control(torque_control_callback, true);
    // robot.control(torque_control_callback, false, franka::kMaxCutoffFrequency);
    redis_thread_running.store(false, std::memory_order_relaxed);
    if (redis_thread.joinable()) {
        redis_thread.join();
    }

    } catch (const std::exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
    std::cout << "counter : " << counter << std::endl;
    redis_thread_running.store(false, std::memory_order_relaxed);
    if (redis_thread.joinable()) {
        redis_thread.join();
    }
    }

    if (redis_thread_exception != nullptr) {
        try {
            std::rethrow_exception(redis_thread_exception);
        } catch (const std::exception& ex) {
            std::cout << "Redis transfer thread error: " << ex.what() << std::endl;
        }
    }

  return 0;
}
