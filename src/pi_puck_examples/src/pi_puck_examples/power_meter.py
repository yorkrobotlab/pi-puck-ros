#!/usr/bin/env python
"""Example ROS node to update the OLED with the current battery usage."""

# Python standard lib
from functools import partial

# ROS imports
import rospy
from sensor_msgs.msg import Image as ImageMessage, BatteryState

# Standard imports
from PIL import Image, ImageDraw, ImageFont

OLED_WIDTH = 128
OLED_HEIGHT = 32

FONT_PADDING = 2
FONT_SIZE = 12
FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 12)


def update_power_meter(data, publisher):
    """Update the power meter with battery reading."""
    battery_percent = float(data.percentage)

    image = Image.new("1", (OLED_WIDTH, OLED_HEIGHT))
    draw = ImageDraw.Draw(image)

    draw.rectangle([(0, OLED_HEIGHT // 2), (int(OLED_WIDTH * battery_percent), OLED_HEIGHT)],
                   fill=0xff)

    percent_text = str(int(battery_percent * 100))
    status_text = "Unknown"

    if data.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_FULL:
        status_text = "Charged"
    elif data.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
        status_text = "Charging"
        percent_text = "??"
    elif data.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING:
        status_text = "Discharging"

    draw.text((FONT_PADDING, FONT_PADDING),
              "Battery: " + percent_text + "%",
              font=FONT,
              fill=0xff)

    draw.text((FONT_PADDING, OLED_HEIGHT - FONT_PADDING - FONT_SIZE),
              status_text,
              font=FONT,
              fill=0x00)

    image_message = ImageMessage()
    image_message.data = image.convert("L").tobytes()
    image_message.width = image.width
    image_message.height = image.height
    image_message.step = image.width
    image_message.encoding = "mono8"

    publisher.publish(image_message)


def main():
    """Entry point function."""
    rospy.init_node("power_meter")

    image_publisher = rospy.Publisher("display/oled/image", ImageMessage, queue_size=10)
    rospy.Subscriber("power/battery", BatteryState,
                     partial(update_power_meter, publisher=image_publisher))

    rospy.spin()


if __name__ == "__main__":
    main()
