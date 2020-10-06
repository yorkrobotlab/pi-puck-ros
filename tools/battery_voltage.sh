#!/usr/bin/env sh

bat_level=$(cat /sys/bus/i2c/drivers/ads1015/11-0048/iio:device0/in_voltage0_raw)
voltage=$(echo "scale=2;($bat_level+39.55)/251.55" | bc)
echo "$voltage volts"
