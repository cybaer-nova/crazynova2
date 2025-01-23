#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    CFs = allcfs.crazyflies

    #traj1 = Trajectory()
    #traj1.loadcsv(Path(__file__).parent / 'data/figure8.csv')

    TRIALS = 1
    TIMESCALE = 1.0
    nCFs = len(CFs)
    #for i in range(TRIALS):
        #for cf in allcfs.crazyflies:
            #cf.uploadTrajectory(0, 0, traj1)

        #allcfs.takeoff(targetHeight=0.5, duration=2.0)
    for iCF in range(nCFs):
            CFs[iCF].takeoff(targetHeight=0.5+0.2*iCF, duration=1.0+0.5)
            timeHelper.sleep(2.0)
    timeHelper.sleep(3.0)
        
    for icf in range(nCFs): 
            print(allcfs.crazyflies[icf].initialPosition)
            pos = np.array(CFs[icf].initialPosition) + np.array([0, 0, 0.2*icf+1.6])
            print('comando para o drone icf', icf)
            print(pos)
            
            CFs[icf].goTo(pos, 0, 2.0)
            timeHelper.sleep(0.5)

    timeHelper.sleep(4.0)

    for icf in range(nCFs): 
            print(allcfs.crazyflies[icf].initialPosition)
            pos = np.array(CFs[icf].initialPosition) + np.array([0, 0, 0.2*icf+0.4])

            print('comando para o drone icf', icf)
            print(pos)
            
            CFs[icf].goTo(pos, 0, 2.0)
            timeHelper.sleep(0.5)

    timeHelper.sleep(4.0)

        #allcfs.startTrajectory(0, timescale=TIMESCALE)
        #timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
        #allcfs.startTrajectory(0, timescale=TIMESCALE, reverse=True)
        #timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)

    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)


if __name__ == '__main__':
    main()
