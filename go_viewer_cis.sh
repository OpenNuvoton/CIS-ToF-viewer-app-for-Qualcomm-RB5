#!/bin/bash -
#
# Add "-e" Mean Exit Immediately If A Command Exits With A Non-Zero Status.
# Add "-v  Mean Prints Shell Input Lines As They Are Read.
# Add "-x  Mean Print Command xTraces Before Executing Command.
#

## Enable Thread Prio
#sysctl -w kernel.sched_rt_runtime_us=-1
#echo 4 > /sys/devices/system/cpu/cpu0/core_ctl/min_cpus
#echo 4 > /sys/devices/system/cpu/cpu4/core_ctl/min_cpus
#
#cat /sys/devices/system/cpu/cpu0/core_ctl/min_cpus
#cat /sys/devices/system/cpu/cpu4/core_ctl/min_cpus

## Run The Cpu In Turbo Mode
#echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
#echo performance > /sys/devices/system/cpu/cpu1/cpufreq/scaling_governor
#echo performance > /sys/devices/system/cpu/cpu2/cpufreq/scaling_governor
#echo performance > /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor
#echo performance > /sys/devices/system/cpu/cpu4/cpufreq/scaling_governor
#echo performance > /sys/devices/system/cpu/cpu5/cpufreq/scaling_governor
#echo performance > /sys/devices/system/cpu/cpu6/cpufreq/scaling_governor
#echo performance > /sys/devices/system/cpu/cpu7/cpufreq/scaling_governor

#read -p "Press enter to continue"

# A black full window (This is xWayland window, being launch separately) will be seen after comamnd "Xwayland & DISPLAY=:0".
# It is needed to use key sequence, win + tab, to return to the original terminal.
# arg1 = ranging mode number, if omit, default as mode 1.
export XDG_RUNTIME_DIR=/run/user/root
export GDK_BACKEND="x11"
Xwayland & DISPLAY=:0 LD_LIBRARY_PATH=./lib/ ./build/viewer $1

