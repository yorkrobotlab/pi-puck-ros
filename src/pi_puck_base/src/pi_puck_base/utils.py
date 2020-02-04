#!/usr/bin/env python
"""Generic tools for the Pi-puck."""
from socket import gethostname


def get_pi_puck_name():
    """Get the node name for the Pi-puck."""
    return gethostname().replace("-", "_")
