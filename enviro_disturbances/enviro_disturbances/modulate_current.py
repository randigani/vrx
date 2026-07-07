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
# Takes target current vector, oscillation period and offset,
# Gain modulation multiple (n * period for oscillation)

def main():
    node = Node()
    
    try:
        parser = argparse.ArgumentParser(exit_on_error=False)
        parser.add_argument('-v', '--top', type=float, required=True)
        parser.add_argument('-l', '--bottom', type=float, required=True)
        parser.add_argument('-a', '--angle', type=float, required=True)
        parser.add_argument('-m', '--mult', type=float, required=True)
        parser.add_argument('--updraft', type=float, required=False)
        parser.add_argument('-p', '--period', type=float, required=True)

        args = parser.parse_args()
        top = args.top
        bottom = args.bottom
        ang = args.angle
        updr = args.updraft
        period = args.period
        m_period = period * args.mult  # Period for the amplitude modulation

    except:
        logging.critical("Failed to parse arguments!! Exiting")
        return
    
    logging.info("Starting current modulation")
    
    if period <= 0:
        logging.critical('Period cannot be less than or equal to zero.')
        return
    elif period < 0.5:
        logging.warning(f'The period is set very short ({period}s)!')
        
    if m_period <= 0:
        logging.critical('Modulation period cannot be less than or equal to zero.')
        return
    elif m_period < 1:
        logging.warning(f"The modulation period is set very short ({m_period}s)!")
        
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
    
    pub_current = node.advertise('/ocean_current', Vector3d)

    offset = (top + bottom) / 2
    amplitude = (top - bottom) / 2

    start = time.perf_counter()
    
    try:
        while True:
            now = time.perf_counter() - start
            osc = math.sin(2 * math.pi * now / period)
            m_osc = 0.5 + 0.5 * math.sin(2 * math.pi * now / m_period)
            vec = (offset + amplitude * osc) * m_osc

            msg = Vector3d()

            msg.x = vec * math.cos(math.radians(ang))
            msg.y = vec * math.sin(math.radians(ang))
            msg.z = updr

            pub_current.publish(msg)
            time.sleep(0.01)

    finally:
        logging.info("Stopping node")
        msg = Vector3d()
        msg.x = 0
        msg.y = 0
        msg.z = 0

        pub_current.publish(msg)
