#!/usr/bin/env sh

bat_level=$(cat /sys/bus/i2c/drivers/ads1015/3-0048/in4_input)
voltage=$(echo "scale=2;($bat_level+79.10)/503.1" | bc)
echo "$voltage volts"
