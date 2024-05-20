#!/usr/bin/env python

from crazyflie_py import Crazyswarm
import numpy as np

def main():

    Za = 1.2
    Zb = 1.0
    #dX = 0.6*0;
    #dY = -0.6*0;
    #dZ = -0.4*0;
    dX = 0.6;
    dY = -0.6*0;
    dZ = 0.6*0;
    dYaw = np.pi/2.0*0;
    dT = 2.0;
    nreps = 3;
    
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    
    print('Number of CFs: ',len(swarm.allcfs.crazyflies))
    cfa = swarm.allcfs.crazyflies[0]
    cfb = swarm.allcfs.crazyflies[1]

    # # enable logging to SD card if present (assuming disabled before)
    # cf.setParam('usd.logging',1)
    # timeHelper.sleep(1.0)

    # allcfs.takeoff(targetHeight=Zb, duration=1.0+Zb)
    cfa.takeoff(targetHeight=Za, duration=1.0+Za)
    timeHelper.sleep(1.0)
    cfb.takeoff(targetHeight=Zb, duration=1.0+Zb)
    timeHelper.sleep(2+Zb)
    
    # (absolute) x-axis position control
    posa_ini = cfa.initialPosition
    posa = np.array(posa_ini) + np.array([0, 0, Za])
    cfa.goTo(posa, 0, dT)
    timeHelper.sleep(dT)
    posb_ini = cfb.initialPosition
    posb = np.array(posb_ini) + np.array([0, 0, Zb])
    cfb.goTo(posb, 0, dT)
    timeHelper.sleep(dT)

    # timeHelper.sleep(10)
    
    for irep in range(nreps):
        posa = np.array(posa_ini) + np.array([dX, dY, Za+dZ])
        cfa.goTo(posa, dYaw, dT)
        timeHelper.sleep(dT)
        posb = np.array(posb_ini) + np.array([dX, dY, Zb+dZ])
        cfb.goTo(posb, dYaw, dT)
        timeHelper.sleep(dT)
        timeHelper.sleep(3.0)
        posa = np.array(posa_ini) + np.array([0, 0, Za])
        cfa.goTo(posa, 0, dT)
        timeHelper.sleep(dT)
        posb = np.array(posb_ini) + np.array([0, 0, Zb])
        cfb.goTo(posb, 0, dT)
        timeHelper.sleep(dT)
        timeHelper.sleep(3.0)
    
    # allcfs.land(targetHeight=0.02, duration=1.0+dT)
    cfa.land(targetHeight=0.02, duration=1.0+Za)
    timeHelper.sleep(dT)
    cfb.land(targetHeight=0.02, duration=1.0+Zb)
    timeHelper.sleep(dT)

    # # disable logging to SD card if present (assuming enabled before)
    # cf.setParam('usd.logging',0)
    # timeHelper.sleep(1.0)


if __name__ == '__main__':
    main()
