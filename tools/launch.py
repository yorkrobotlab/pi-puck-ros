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
        ip_address = sock.getsockname()[0]
    except socket.error:
        ip_address = '127.0.0.1'
    except IndexError:
        ip_address = '127.0.0.1'
    finally:
        sock.close()
    return ip_address


def get_ros_master_ip():
    """Get the IP of the ROS master server."""
    local_ip = get_local_ip().split(".")
    ip_prefix = ".".join(local_ip[:-1])

    ip_format = "http://{}.{{}}:11311".format(ip_prefix)

    timeout = 0.0005

    while timeout < 0.1:
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
    arg_parser.add_argument("-r", "--robot-name", required=False, type=str)
    arg_parser.add_argument("-a", "--auto-robot-name", action="store_true")

    parsed_args = arg_parser.parse_args()

    if parsed_args.robot_name is not None and parsed_args.auto_robot_name:
        arg_parser.error("--robot-name and --auto-robot-name are mutually exclusive.")

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

    robot_name = None

    # Set robot name, possibly automagically
    if parsed_args.robot_name is not None:
        robot_name = parsed_args.robot_name
    elif parsed_args.auto_robot_name:
        robot_name = socket.gethostname().replace("-", "_")

    if robot_name is not None:
        print("Using robot name {}".format(robot_name))
        os.environ["ROBOT_NAME"] = robot_name

    # roslaunch assumes that the first argument is the command that launched it and ignores it (as
    # is standard), therefore we need to pass "roslaunch" again.
    os.execlp("roslaunch", "roslaunch", os.path.abspath(parsed_args.launch_file))


if __name__ == "__main__":
    main()
