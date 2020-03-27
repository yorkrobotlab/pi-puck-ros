#!/usr/bin/env python
"""Tool to launch ROS and setup environment variables automagically."""

import argparse
import os
import socket


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


def find_pi_pucks():
    """Get the IP of the ROS master server."""
    local_ip = get_local_ip().split(".")
    ip_prefix = ".".join(local_ip[:-1])

    for ip_suffix in map(str, range(256)):
        try:
            (hostname, _, __) = socket.gethostbyaddr(ip_prefix + "." + ip_suffix)
        except socket.herror:
            continue
        if "pi-puck" in hostname:
            yield hostname, ip_prefix + "." + ip_suffix


def main():
    """Entry point function."""
    argument_parser = argparse.ArgumentParser()

    argument_parser.add_argument("-s", "--ssh", action="store_true")

    parsed_args = argument_parser.parse_args()

    pi_pucks = list(find_pi_pucks())

    if pi_pucks:
        print("Pi-pucks:")
        for pi_puck_hostname, pi_puck_ip in pi_pucks:
            print("  - " + pi_puck_hostname + ", " + pi_puck_ip)

        if parsed_args.ssh:
            os.execlp("ssh", "ssh", "pi@" + pi_pucks[0][1])
    else:
        print("No Pi-pucks found.")


if __name__ == "__main__":
    main()
