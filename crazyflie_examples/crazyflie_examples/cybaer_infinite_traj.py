#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np


def executeTrajectory(timeHelper, cf, traj, rate=100, offset=np.zeros(3)):
    
    start_time = timeHelper.time()
    while not timeHelper.isShutdown():
        t = timeHelper.time() - start_time
        if t > traj.duration:
            break

        e = traj.eval(t)
        cf.cmdFullState(
            e.pos + np.array(cf.initialPosition) + offset,
            e.vel,
            e.acc,
            e.yaw,
            e.omega)

        timeHelper.sleepForRate(rate)


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]
    
    traj1 = Trajectory()
    traj1.loadcsv(Path(__file__).parent / 'data/figure8.csv')

    rate = 30.0
    Z = 1.0
    Nloops = 10
    offset=np.array([0, 0, Z])

    # # enable logging to SD card if present (assuming disabled before)
    # cf.setParam('usd.logging',1)
    # timeHelper.sleep(1.0)

    # takeoff and go to initial position
    print('takeoff')
    cf.takeoff(targetHeight=Z, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)
    pos = np.array(cf.initialPosition) + offset
    cf.goTo(pos, 0, 2.0)

    # fly figure8 until battery is low or Nloops times
    loop_counter = 0
    # status = cf.get_status()
    while loop_counter < Nloops: # and status['pm_state'] == 0:
        loop_counter += 1
        print(f'Trajectory repeat number {loop_counter} of maximum {Nloops} loops.')
        executeTrajectory(timeHelper, cf, traj1, rate, offset)
        # status = cf.get_status()
        # print(f'pm state : {status["pm_state"]}, battery left : {status["battery"]}')
    
    print('landing')
    cf.notifySetpointsStop()
    cf.land(targetHeight=0.03, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)

    # # disable logging to SD card if present (assuming enabled before)
    # cf.setParam('usd.logging',0)
    # timeHelper.sleep(1.0)


if __name__ == '__main__':
    main()
