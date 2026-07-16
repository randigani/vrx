#!/usr/bin/env python3

import logging
import math
import time
import threading
from gz.transport13 import Node
import argparse
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.msgs10.boolean_pb2 import Boolean

# publish to /ocean-current.
# Takes target current vector, oscillation period and offset, and absolute or not. 
# Modulates current.

# Once the oscillating_current gets written there needs to be a service that terminates the node.

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

def main():
    node = Node()

    try:
        parser = argparse.ArgumentParser(exit_on_error=False)
        parser.add_argument('-v', '--top', type=float, required=True)
        parser.add_argument('-l', '--bottom', type=float, required=True)
        parser.add_argument('-a', '--angle', type=float, required=True)
        parser.add_argument('--updraft', type=float, required=False)
        parser.add_argument('-p', '--period', type=float, required=True)

        args = parser.parse_args()
        top = args.top
        bottom = args.bottom
        ang = args.angle
        updr = args.updraft
        period = args.period

    except:
        logging.critical("Failed to parse arguments!! Exiting")
        return

    logging.info("Starting current oscillation")

    if period <= 0:
        logging.critical('Period cannot be less than or equal to zero.')
        return
    elif period < 0.5:
        logging.warning(f'The period is set very short ({period}s)!')

    _rcv = threading.Event()

    _org = Vector3d()

    def onetime_callback(msg):
        nonlocal _org
        _rcv.set()
        _org = msg
        
    node.subscribe(Vector3d, '/ocean_current', onetime_callback)

    if not _rcv.wait(timeout=0.5):
        logging.debug('No other publishers here. Good to publish')
        _org.x = 0
        _org.y = 0
        _org.z = 0
    else:
        logging.error('Detected another publisher!! Please quit the other publisher. Exiting')
        return

    node.unsubscribe('/ocean_current')

    # terminate_others = node.advertise('/ocean_current/terminate_any_publishers', Boolean)
    # msg = Boolean()
    # msg.data = True
    # terminate_others.publish(msg)

    # time.sleep(0.5)
    # del terminate_others

    # def terminate_node(request):
        # node.running = False
        # logging.warning("Termination requested from another node!")
        # raise KeyboardInterrupt
# 
    # node.subscribe(
        # Boolean, 
        # '/ocean_current/terminate_any_publishers',
        # terminate_node
    # )

    pub_current = node.advertise('/ocean_current', Vector3d)

    offset = (top + bottom) / 2
    amplitude = (top - bottom) / 2

    start = time.perf_counter()

    counter = 0

    try:
        while True:
            now = time.perf_counter() - start
            osc = math.sin(2 * math.pi * now / period)
            vec = offset + amplitude * osc

            msg = Vector3d()

            msg.x = vec * math.cos(math.radians(ang))
            msg.y = vec * math.sin(math.radians(ang))
            msg.z = updr

            pub_current.publish(msg)

            if counter % 128 == 0:
                logging.info(f"\nOscillating current: \n{vec:.8f} m/s, {now:.8f} secs")
            
            counter += 1

            time.sleep(0.01)

    finally:
        logging.info("Stopping node!")
        msg = Vector3d()
        msg.x = 0
        msg.y = 0
        msg.z = 0

        pub_current.publish(msg)

            