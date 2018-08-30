sudo cpufreq-set -c 1 -g "performance"
taskset 0x2 ./build/franka_panda_redis_driver 172.16.0.10