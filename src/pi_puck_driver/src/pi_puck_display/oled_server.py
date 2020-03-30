#!/usr/bin/env python
"""ROS Node to expose a topic for the ssd1306 driven OLED display (optionally) on the Pi-puck."""

# ROS imports
import rospy
from sensor_msgs.msg import Image as ImageMessage
from std_msgs.msg import String

# Standard imports
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import Image, ImageFont

OLED_I2C_PORT = 3
OLED_I2C_ADDRESS = 0x3C
OLED_WIDTH = 128
OLED_HEIGHT = 32
DEFAULT_FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"


class PiPuckOledServer(object):
    """ROS Node to an ssd1306 driven OLED display."""

    def __init__(self):
        """Initialise node."""
        rospy.init_node("oled")

        rospy.Subscriber("oled/text", String, self.string_callback)
        rospy.Subscriber("oled/image", ImageMessage, self.image_callback)

        self._persist = bool(rospy.get_param("~persist", True))
        font_path = str(rospy.get_param("~font_path", DEFAULT_FONT))
        font_size = int(rospy.get_param("~font_size", 14))
        self._font = ImageFont.truetype(font_path, font_size)

        self._device = None

    def string_callback(self, data):
        """Handle requests to display string on OLED."""
        with canvas(self._device) as draw:
            draw.rectangle((0, 0, OLED_WIDTH, OLED_HEIGHT), outline=0, fill=0x00)
            draw.text((0, 0), data.data, font=self._font, fill=0xFF)

    def image_callback(self, data):
        """Handle requests to display image on OLED."""
        image = PiPuckOledServer.convert_ros_image_to_pil(data)
        with canvas(self._device) as draw:
            draw.rectangle((0, 0, OLED_WIDTH, OLED_HEIGHT), outline=0, fill=0x00)
            draw.im.paste(image, (0, 0, OLED_WIDTH, OLED_HEIGHT))

    @staticmethod
    def convert_ros_image_to_pil(ros_image):
        """Convert a ROS image to PIL image."""
        if ros_image.encoding == "rgba8":
            image = Image.frombytes('RGBA', (ros_image.width, ros_image.height), ros_image.data,
                                    'raw')
        elif ros_image.encoding == "rgb8":
            image = Image.frombytes('RGB', (ros_image.width, ros_image.height), ros_image.data,
                                    'raw')
        elif ros_image.encoding == "mono8":
            image = Image.frombytes('L', (ros_image.width, ros_image.height), ros_image.data, 'raw')
        else:
            rospy.logerr("Unsupported encoding '{}'.".format(ros_image.encoding))

        if ros_image.width == OLED_WIDTH and ros_image.height == OLED_HEIGHT:
            return image

        return image.resize((OLED_WIDTH, OLED_HEIGHT))

    def close_device(self):
        """Close device and cleanup."""
        if self._device is not None:
            self._device.cleanup()
        self._device = None

    def run(self):
        """Run node (spin forever)."""
        serial = i2c(port=OLED_I2C_PORT, address=OLED_I2C_ADDRESS)

        self._device = ssd1306(serial, width=OLED_WIDTH, height=OLED_HEIGHT, rotate=0)
        self._device.persist = self._persist

        rospy.spin()


if __name__ == "__main__":
    PiPuckOledServer().run()
