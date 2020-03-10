#!/usr/bin/env python
"""ROS Node to expose topics for the short range IR sensors on the Pi-puck."""

import math

# ROS imports
import rospy
from sensor_msgs.msg import Range

# Standard imports
from smbus import SMBus

# Constants
I2C_CHANNEL = 4
EPUCK_I2C_ADDR = 0x1e

IR_CONTROL = 6
IR0_REFLECTED = 7
IR1_REFLECTED = 8
IR2_REFLECTED = 9
IR3_REFLECTED = 10
IR4_REFLECTED = 11
IR5_REFLECTED = 12
IR6_REFLECTED = 13
IR7_REFLECTED = 14

IRX_REFLECTED = (IR0_REFLECTED, IR1_REFLECTED, IR2_REFLECTED, IR3_REFLECTED, IR4_REFLECTED,
                 IR5_REFLECTED, IR6_REFLECTED, IR7_REFLECTED)

IR0_AMBIENT = 15
IR1_AMBIENT = 16
IR2_AMBIENT = 17
IR3_AMBIENT = 18
IR4_AMBIENT = 19
IR5_AMBIENT = 20
IR6_AMBIENT = 21
IR7_AMBIENT = 22

IRX_AMBIENT = (IR0_AMBIENT, IR1_AMBIENT, IR2_AMBIENT, IR3_AMBIENT, IR4_AMBIENT, IR5_AMBIENT,
               IR6_AMBIENT, IR7_AMBIENT)

IR_SENSOR_ANGLES = (0, 45, 90, 135, 180, 225, 270, 315)

IR_SENSOR_COUNT = 8

REFERENCE_FRAME_ID = "reflectance_sensor_{}"

BUS = SMBus(I2C_CHANNEL)


def calculate_distance(x):
    """Calculate an estimate of the range in metres."""
    try:
        result_in_mm = -(1250000.0 * math.log((20.0 * x) / 116727.0)) / 108221.0
        result = round(result_in_mm, 2) / 1000.0
    except ValueError:
        result = -float("inf")

    if result < 0.005:
        result = -float("inf")
    elif result > 0.05:
        result = float("inf")

    return result


def close_bus():
    """Close the I2C bus after the ROS Node is shutdown."""
    BUS.write_word_data(EPUCK_I2C_ADDR, IR_CONTROL, 0)  # Stop IR
    BUS.close()


def pi_puck_short_range_ir_server():
    """ROS Node server."""
    rospy.on_shutdown(close_bus)
    BUS.write_word_data(EPUCK_I2C_ADDR, IR_CONTROL, 1)  # Turn IR sensors on

    rospy.init_node("short_range_ir")

    ir_proximity_publishers = {}
    ir_proximity_frame_ids = {}

    tf_prefix_key = rospy.search_param("tf_prefix")
    if tf_prefix_key:
        tf_prefix = rospy.get_param(tf_prefix_key, None)
    else:
        tf_prefix = None
    if tf_prefix is not None and not tf_prefix.endswith("/"):
        tf_prefix += "/"

    reference_frame_id_template = REFERENCE_FRAME_ID
    if tf_prefix:
        reference_frame_id_template = tf_prefix + reference_frame_id_template

    for ir_sensor in range(IR_SENSOR_COUNT):
        ir_proximity_publishers[ir_sensor] = rospy.Publisher('short_range_ir/{}'.format(ir_sensor),
                                                             Range,
                                                             queue_size=10)
        ir_proximity_frame_ids[ir_sensor] = reference_frame_id_template.format(ir_sensor)

    rate = rospy.Rate(rospy.get_param('~rate', 5))

    while not rospy.is_shutdown():
        for ir_sensor in range(IR_SENSOR_COUNT):
            reflected_raw = int(BUS.read_word_data(EPUCK_I2C_ADDR, IRX_REFLECTED[ir_sensor]))
            ambient_raw = int(BUS.read_word_data(EPUCK_I2C_ADDR, IRX_AMBIENT[ir_sensor]))
            converted_distance_reading = calculate_distance(reflected_raw - ambient_raw)
            range_result = Range(radiation_type=Range.INFRARED,
                                 min_range=0,
                                 max_range=0.1,
                                 range=converted_distance_reading)
            range_result.header.frame_id = ir_proximity_frame_ids[ir_sensor]
            ir_proximity_publishers[ir_sensor].publish(range_result)
        rate.sleep()


if __name__ == "__main__":
    pi_puck_short_range_ir_server()
