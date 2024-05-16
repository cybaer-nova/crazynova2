#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
from crazyflie_interfaces.msg import LogDataGeneric

import numpy as np
# import rowan

status = {}
state = {}
state_desired = {}

def status_simple_topic_callback(msg):
    """
    Call back for topic /cfXXX/status_simple.
    vars: ["pm.vbatMV","supervisor.info"]
    """
    global status
    status = {  'id': msg.header.frame_id,
                'timestamp_sec': msg.header.stamp.sec,
                'timestamp_nsec': msg.header.stamp.nanosec,
                'supervisor': msg.values[1],
                'battery': msg.values[0]}
    
    # print(f'status_simple_topic_callback was called {status}')

def pose_simple_topic_callback(msg):
    """
    Call back for topic /cfXXX/pose_simple.
    vars: ["stateEstimateZ.x","stateEstimateZ.y","stateEstimateZ.z","stateEstimate.qx","stateEstimate.qy","stateEstimate.qz","stateEstimate.qw"]
    """
    pos = np.array([msg.values[0], msg.values[1], msg.values[2]])/1000.0 # conversion from mm to m
    lbd = np.array([msg.values[3], msg.values[4], msg.values[5]])*np.pi/180.0 # conversion from deg/s to rad/s
    # quat = np.array([msg.values[3], msg.values[4], msg.values[5], msg.values[6]])
    # lbd = rowan.to_euler(quat)

    global state
    state = {   'id': msg.header.frame_id,
                'timestamp_sec': msg.header.stamp.sec,
                'timestamp_nsec': msg.header.stamp.nanosec,
                'pos': pos,
                'lbd': lbd }
                # 'quat': quat,
    
    # print(f'pose_simple_topic_callback was called {state}')

def twist_simple_topic_callback(msg):
    """
    Call back for topic /cfXXX/twist_simple.
    vars: ["stateEstimateZ.vx", "stateEstimateZ.vy", "stateEstimateZ.vz","stateEstimateZ.rateRoll","stateEstimateZ.ratePitch","stateEstimateZ.rateYaw"]
    """
    vel = np.array([msg.values[0], msg.values[1], msg.values[2]])/1000.0 # conversion from mm/ to m/s
    om = np.array([msg.values[3], msg.values[4], msg.values[5]])*np.pi/180.0 # conversion from deg/s to rad/s

    global state
    state = {   'id': msg.header.frame_id,
                'timestamp_sec': msg.header.stamp.sec,
                'timestamp_nsec': msg.header.stamp.nanosec,
                'vel': vel,
                'om': om }
    
    # print(f'twist_simple_topic_callback was called {state}')


def executeControl(timeHelper, cf, rate=20):
    
    start_time = timeHelper.time()
    
    global status
    global state
    global state_desired
    
    print(f'controller has state: {state}')

    # # define here controller
    # ctr_pos = {}
    # ctr_vel = {}
    # ctr_acc = {}
    # ctr_yaw = {}
    # ctr_omega = {}

    # cf.cmdFullState(
    #     ctr_pos,
    #     ctr_vel,
    #     ctr_acc,
    #     ctr_yaw,
    #     ctr_omega)
                
    timeHelper.sleepForRate(rate)

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]
    node = cf.node

    node.statusSubscriber = node.create_subscription(
        LogDataGeneric, f'{cf.prefix}/status_simple', status_simple_topic_callback, 10)
    node.get_logger().info(f'subscribed topic: {cf.prefix}/status_simple')
    
    node.poseSubscriber = node.create_subscription(
        LogDataGeneric, f'{cf.prefix}/pose_simple', pose_simple_topic_callback, 10)
    node.get_logger().info(f'subscribed topic: {cf.prefix}/pose_simple')
    
    node.twistSubscriber = node.create_subscription(
        LogDataGeneric, f'{cf.prefix}/twist_simple', twist_simple_topic_callback, 10)
    node.get_logger().info(f'subscribed topic: {cf.prefix}/twist_simple')
    
    rate = 20.0
    Nloops = 2
    Z = 1.0
    offset=np.array([0, 0, Z])

    # # # enable logging to SD card if present (assuming disabled before)
    # # cf.setParam('usd.logging',1)
    # # timeHelper.sleep(1.0)

    # takeoff and go to initial position
    print('takeoff')
    cf.takeoff(targetHeight=Z, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)
    pos = np.array(cf.initialPosition) + offset
    cf.goTo(pos, 0, 2.0)
    timeHelper.sleep(Z+2.0)
    
    # fly traj until battery is low or Nloops times
    executeControl(timeHelper, cf, rate)
    
    print('landing')
    cf.notifySetpointsStop()
    pos = np.array(cf.initialPosition) + offset
    cf.goTo(pos, 0, 2.0)
    timeHelper.sleep(Z+2.0)
    cf.land(targetHeight=0.03, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)

    # # disable logging to SD card if present (assuming enabled before)
    # cf.setParam('usd.logging',0)
    # timeHelper.sleep(1.0)


if __name__ == '__main__':
    main()
