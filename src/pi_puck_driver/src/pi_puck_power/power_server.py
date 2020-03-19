#!/usr/bin/env python
"""ROS Node to expose topics for motor speed and for motor steps of the Pi-puck."""

# ROS imports
import rospy
from sensor_msgs.msg import BatteryState

# Constants
NAN = float("nan")
BATTERIES = {
    "primary": {
        "path": "/sys/bus/i2c/drivers/ads1015/3-0048/in4_input",
        "design_capacity": 1.8,  # 1800 mAh,
        "min_voltage": 3.3,  #  battery protection will kick in if below 3.3 volts
        "max_voltage": 4.2,
        "power_supply_technology": BatteryState.POWER_SUPPLY_TECHNOLOGY_LION,
        "location": "Primary battery (e-puck underside slot)"
    },
    "aux": {
        "path": "/sys/bus/i2c/drivers/ads1015/3-0048/in5_input",
        "design_capacity": NAN,
        "min_voltage": 3.3,  # Value assumed, should be tuned to actual aux battery
        "max_voltage": 4.2,  # Value assumed, should be tuned to actual aux battery,
        "power_supply_technology": BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN,
        "location": "Auxiliary battery (Pi-puck JST connector)"
    }
}
HISTORY_MAX = 30  # 30 seconds at default rate
OVER_VOLTAGE_MARGIN = 1.03  # 3%
UNDER_VOLTAGE_MARGIN = 1.03  # 3%


def clamp(value, value_max=1, value_min=0):
    """Clamp between values."""
    return max(min(value, value_max), value_min)


class PiPuckBatteryServer(object):
    """ROS Node to the Pi-puck's battery ADC."""

    def __init__(self):
        """Initialise node."""
        rospy.init_node("power")

        self._rate = rospy.Rate(rospy.get_param("~rate", 1))
        self._publish_aux_battery = bool(rospy.get_param("~publish_aux_battery", False))

        self._battery_pub = rospy.Publisher('power/battery', BatteryState, queue_size=10)
        if self._publish_aux_battery:
            self._aux_battery_pub = rospy.Publisher('power/aux_battery',
                                                    BatteryState,
                                                    queue_size=10)

        self._battery_history = {}
        for battery in BATTERIES:
            self._battery_history[battery] = []

    @staticmethod
    def convert_adc_to_voltage(adc_value):
        """Convert ADC value to voltage.

        Accurate to 4 significant figures between 3.3 and 4.2 volts using linear fitting.
        """
        return (float(adc_value) + 79.10) / 503.1

    @staticmethod
    def get_battery_voltage(battery_path):
        """Get battery voltage given ADC device path."""
        with open(battery_path, "r") as battery_file_handle:
            battery_value = battery_file_handle.read()
        return PiPuckBatteryServer.convert_adc_to_voltage(int(battery_value))

    def get_battery_state(self, battery):
        """Get the current state of a battery."""
        battery_config = BATTERIES[battery]
        battery_state = BatteryState()

        battery_voltage = PiPuckBatteryServer.get_battery_voltage(battery_config["path"])

        self._battery_history[battery].insert(0, battery_voltage)
        self._battery_history[battery] = self._battery_history[battery][:HISTORY_MAX]

        split_point = len(self._battery_history[battery]) // 2
        head_half = self._battery_history[battery][:split_point]
        tail_half = self._battery_history[battery][split_point:]

        try:
            battery_delta = (sum(head_half) / len(head_half)) - (sum(tail_half) / len(tail_half))
        except ZeroDivisionError:
            battery_delta = 0.0

        battery_state.voltage = battery_voltage
        battery_state.present = True
        battery_state.design_capacity = battery_config["design_capacity"]
        battery_state.power_supply_technology = battery_config["power_supply_technology"]

        average_battery_voltage = sum(self._battery_history[battery]) / len(
            self._battery_history[battery])

        if average_battery_voltage >= battery_config["max_voltage"]:
            battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        elif battery_delta < 0:
            battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        elif battery_delta > 0:
            battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

        if average_battery_voltage >= battery_config["max_voltage"] * OVER_VOLTAGE_MARGIN:
            battery_state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE
        elif average_battery_voltage <= battery_config["min_voltage"]:
            # It is unclear whether this means "out of charge" or "will never charge again", we
            # assume here that it means "out of charge".
            battery_state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
        else:
            battery_state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

        battery_state.percentage = clamp(
            (average_battery_voltage - battery_config["min_voltage"]) /
            (battery_config["max_voltage"] - battery_config["min_voltage"]), 1.0, 0.0)

        battery_state.current = NAN
        battery_state.charge = NAN
        battery_state.capacity = NAN
        battery_state.location = battery_config["location"]

        return battery_state

    def publish_primary_battery(self):
        """Publish the primary battery status."""
        battery_message = self.get_battery_state("primary")
        self._battery_pub.publish(battery_message)

    def publish_aux_battery(self):
        """Publish the aux battery status."""
        battery_message = self.get_battery_state("aux")
        self._aux_battery_pub.publish(battery_message)

    def run(self):
        """ROS Node server."""
        while not rospy.is_shutdown():
            self.publish_primary_battery()
            if self._publish_aux_battery:
                self.publish_aux_battery()
            self._rate.sleep()


if __name__ == "__main__":
    PiPuckBatteryServer().run()
