#!/usr/bin/env python
"""Tool to launch ROS and setup environment variables automagically."""

import argparse
import os
import socket
import urllib2


def get_local_ip():
    """Get local ip address."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        sock.connect(('8.8.8.8', 1))
        ip = sock.getsockname()[0]
    except:
        ip = '127.0.0.1'
    finally:
        sock.close()
    return ip


def get_ros_master_ip():
    """Get the IP of the ROS master server."""
    local_ip = get_local_ip().split(".")
    ip_prefix = ".".join(local_ip[:-1])

    ip_format = "http://{}.{{}}:11311".format(ip_prefix)

    timeout = 0.0005

    while timeout < 0.01:
        for ip_suffix in map(str, range(256)):
            try:
                urllib2.urlopen(ip_format.format(ip_suffix), timeout=timeout)
            except urllib2.HTTPError as error:
                if error.code == 501:
                    return ip_prefix + "." + ip_suffix
            except urllib2.URLError:
                pass
            except socket.timeout:
                pass
        timeout *= 2

    return "127.0.0.1"


def main():
    """Entry point function."""
    arg_parser = argparse.ArgumentParser()

    arg_parser.add_argument("launch_file", type=str)
    arg_parser.add_argument("-m", "--ros-master", required=False, type=str)
    arg_parser.add_argument("-i", "--ros-ip", required=False, type=str)

    parsed_args = arg_parser.parse_args()

    if os.path.abspath(os.path.join(os.path.dirname(__file__),
                                    "devel")) not in os.environ["PKG_CONFIG_PATH"]:
        print("Please run `source devel/setup.bash first.`")
        return

    if parsed_args.ros_ip:
        ros_ip = parsed_args.ros_ip
    else:
        ros_ip = get_local_ip()

    if parsed_args.ros_master:
        ros_master = parsed_args.ros_master
    else:
        ros_master = get_ros_master_ip()

    print("Using ROS_IP of " + ros_ip + " and ROS master of " + ros_master)

    os.environ["ROS_IP"] = ros_ip
    os.environ["ROS_MASTER_URI"] = "http://{}:11311".format(ros_master)

    os.execlp("roslaunch", os.path.abspath(parsed_args.launch_file))


if __name__ == "__main__":
    main()
