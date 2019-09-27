sudo cpufreq-set -c 0 -g "performance"
taskset 0x1 ./build/franka_panda_redis_driver 172.16.0.10
