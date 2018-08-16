sudo cpufreq-set -c 0 -g "performance"
taskset 0x1 redis-server