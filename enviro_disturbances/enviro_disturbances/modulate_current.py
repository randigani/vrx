#!/usr/bin/env python3

import logging
import math
import time
import threading
from gz.transport13 import Node
import argparse
from gz.msgs10.vector3d_pb2 import Vector3d, Empty

# publish to /ocean-current.
# Performs double modulation on the current.
# Takes target current vector, first oscillation period and offset,
# Second oscillation period, offset, and absoluteness (Only positive or can go negative)

