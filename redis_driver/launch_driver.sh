sudo cpufreq-set -c 1 -g "performance"
taskset 0x2 ./build/sai_franka_robot_redis_driver default_config.xml
