#!/usr/bin/env python3

import logging
import math
import time
import threading
from gz.transport13 import Node
import argparse
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.msgs10.boolean_pb2 import Boolean

# publish to gazebo topic /ocean_current.
# Takes target current vector, and the amount of time to reach the current.
# Linearly reaches the target current over time, then dies.

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

def main():
    node = Node()

    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('-v', '--velocity', type=float, required=True)
        parser.add_argument('-a', '--angle', type=float, required=True)
        parser.add_argument('--updraft', type=float, required=True)
        parser.add_argument('-t', '--dt', type=float, required=True)

        args = parser.parse_args()

        vel = args.velocity
        ang = args.angle
        updr = args.updraft
        dt = args.dt

    except:
        logging.critical("Failed to parse arguments!! Exiting")
        return

    logging.info("Starting current")

    _tgt = Vector3d()

    _tgt.x = vel * math.cos(math.radians(ang))
    _tgt.y = vel * math.sin(math.radians(ang))
    _tgt.z = updr

    _rcv = threading.Event()

    _org = Vector3d()

    def onetime_callback(msg):
        nonlocal _org
        _rcv.set()
        _org = msg
        
    node.subscribe(Vector3d, '/ocean_current', onetime_callback)

    if not _rcv.wait(timeout=0.5):
        logging.debug('Assuming no initial current...')
        _org.x = 0
        _org.y = 0
        _org.z = 0

    node.unsubscribe('/ocean_current')

    # terminate_others = node.advertise('/ocean_current/terminate_any_publishers', Boolean)
    # msg = Boolean()
    # msg.data = True
    # terminate_others.publish(msg)

    # time.sleep(0.5)
    # del terminate_others

    # def terminate_node(request):
    #     logging.warning("Termination requested from another node!")
    #     node.running = False
    #     raise KeyboardInterrupt

    # node.subscribe(
    #     Boolean, 
    #     '/ocean_current/terminate_any_publishers',
    #     terminate_node
    # )

    pub_current = node.advertise('/ocean_current', Vector3d)

    complete = False

    start = time.perf_counter()

    try:
        while not complete:
            now = time.perf_counter() - start

            if now >= dt:
                pub_current.publish(_tgt)
                complete = True
                break

            t = now / dt

            dx = (_tgt.x - _org.x) * t
            dy = (_tgt.y - _org.y) * t
            dz = (_tgt.z - _org.z) * t

            msg = Vector3d()

            msg.x = _org.x + dx
            msg.y = _org.y + dy
            msg.z = _org.z + dz

            pub_current.publish(msg)
            time.sleep(0.01)

    finally:
        logging.info("Stopping node")
