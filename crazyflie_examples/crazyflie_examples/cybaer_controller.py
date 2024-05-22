#!/usr/bin/env python

import traceback
# from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
from crazyflie_interfaces.msg import LogDataGeneric

import numpy as np
# import rowan

status = {  'id': 0,
            'timestamp_sec': 0,
            'timestamp_nsec': 0,
            'supervisor': 0,
            'battery': 0}
# state = {   'id': 0,
#             'timestamp_sec': 0,
#             'timestamp_nsec': 0,
#             'pos': np.zeros(3),
#             'lbd': np.zeros(3),
#             'vel': np.zeros(3),
#             'om': np.zeros(3) }
pose = {   'id': 0,
            'timestamp_sec': 0,
            'timestamp_nsec': 0,
            'pos': np.zeros(3),
            'lbd': np.zeros(3) }
twist = {   'id': 0,
            'timestamp_sec': 0,
            'timestamp_nsec': 0,
            'vel': np.zeros(3),
            'om': np.zeros(3) }

def status_simple_topic_callback(msg):
    """
    Call back for topic /cfXXX/status_simple.
    vars: ["pm.vbatMV","supervisor.info"]
    """
    global status
    status = {  'id': msg.header.frame_id,
                'timestamp_sec': msg.header.stamp.sec,
                'timestamp_nsec': msg.header.stamp.nanosec,
                'supervisor': int(msg.values[1]),
                'battery': msg.values[0]*1e-3}
    
    # print(f'status_simple_topic_callback was called {status}')

def pose_simple_topic_callback(msg):
    """
    Call back for topic /cfXXX/pose_simple.
    vars: ["stateEstimateZ.x","stateEstimateZ.y","stateEstimateZ.z","stateEstimate.qx","stateEstimate.qy","stateEstimate.qz","stateEstimate.qw"]
    """
    position = np.array([msg.values[0], msg.values[1], msg.values[2]])/1000.0 # conversion from mm to m
    lbd = np.array([msg.values[3], msg.values[4], msg.values[5]])*np.pi/180.0 # conversion from deg/s to rad/s
    # quat = np.array([msg.values[3], msg.values[4], msg.values[5], msg.values[6]])
    # lbd = rowan.to_euler(quat)

    global pose
    pose = {   'id': msg.header.frame_id,
                'timestamp_sec': msg.header.stamp.sec,
                'timestamp_nsec': msg.header.stamp.nanosec,
                'pos': position,
                'lbd': lbd }
                # 'quat': quat,
    
    # print(f'pose_simple_topic_callback was called {pose}')

def twist_simple_topic_callback(msg):
    """
    Call back for topic /cfXXX/twist_simple.
    vars: ["stateEstimateZ.vx", "stateEstimateZ.vy", "stateEstimateZ.vz","stateEstimateZ.rateRoll","stateEstimateZ.ratePitch","stateEstimateZ.rateYaw"]
    """
    vel = np.array([msg.values[0], msg.values[1], msg.values[2]])/1000.0 # conversion from mm/ to m/s
    om = np.array([msg.values[3], msg.values[4], msg.values[5]])*np.pi/180.0 # conversion from deg/s to rad/s

    global twist
    twist = {   'id': msg.header.frame_id,
                'timestamp_sec': msg.header.stamp.sec,
                'timestamp_nsec': msg.header.stamp.nanosec,
                'vel': vel,
                'om': om }
    
    # print(f'twist_simple_topic_callback was called {twist}')

def getReference(t,pos_start):
     
    state_desired = {   'pos': np.zeros(3),
                        'lbd': np.zeros(3),
                        'vel': np.zeros(3),
                        'om': np.zeros(3) }

    if t < 10.0:
        state_desired["pos"] = pos_start + np.array([0.3,0,0])
    elif t < 20.0:
        state_desired["pos"] = pos_start + np.array([-0.3,0,0])
    else:
        state_desired["pos"] = pos_start
    
    return state_desired


def getState():
    
    global status
    global pose
    global twist

    # todo: decode supervisor info https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/#supervisorinfo
    # Bit 0 = Can be armed - the system can be armed and will accept an arming command 
    # Bit 1 = is armed - the system is armed 
    # Bit 2 = auto arm - the system is configured to automatically arm 
    # Bit 3 = can fly - the Crazyflie is ready to fly 
    # Bit 4 = is flying - the Crazyflie is flying. 
    # Bit 5 = is tumbled - the Crazyflie is up side down. 
    # Bit 6 = is locked - the Crazyflie is in the locked state and must be restarted.

    state = {   'pos': pose["pos"],
                'lbd': pose["lbd"],
                'vel': twist["vel"],
                'om': twist["om"],
                'batt': status["battery"],
                'supervisor': status["supervisor"] }

    return state

def doControl(state,state_desired):
    
    # # position controller
    # ctr_pos = state_desired["pos"]
    # ctr_vel = np.array([0.0,0.0,0.0])
    # ctr_acc = np.array([0.0,0.0,0.0])
    # ctr_yaw = 0
    # ctr_omega = np.array([0.0,0.0,0.0])

    # # velocity controller in xy
    # k_pos = 1.5   # 2 is OK but oscillatory, 5 is too high!
    # pos = pose["pos"]
    # pos_d = state_desired["pos"]
    # e_pos = pos - pos_d
    # ctr_pos = np.array([np.nan,np.nan,pos_d[2]])
    # ctr_vel = -k_pos*e_pos
    # ctr_vel[2] = 0
    # ctr_acc = np.array([0.0,0.0,0.0])
    # ctr_yaw = 0
    # ctr_omega = np.array([0.0,0.0,0.0])

    # # velocity controller
    # k_pos = 1.5   # 2 is OK but oscillatory, 5 is too high!
    # pos = pose["pos"]
    # pos_d = state_desired["pos"]
    # e_pos = pos - pos_d
    # ctr_pos = np.array([np.nan,np.nan,np.nan])
    # ctr_vel = -k_pos*e_pos
    # ctr_acc = np.array([0.0,0.0,0.0])
    # ctr_yaw = 0
    # ctr_omega = np.array([0.0,0.0,0.0])

    # acceleration controller in xy
    k_p = 2   # 2 is OK but oscillatory, 5 is too high!
    k_d = 0.1   # 2 is OK but oscillatory, 5 is too high!
    pos = state["pos"]
    pos_d = state_desired["pos"]
    e_pos = pos - pos_d
    vel = state["vel"]
    vel_d = state_desired["vel"]
    e_vel = vel - vel_d

    ctr_pos = np.array([np.nan,np.nan,pos_d[2]])
    ctr_vel = np.array([np.nan,np.nan,0.0])
    ctr_acc = -k_p*e_pos - k_d*e_vel
    ctr_acc[2] = 0.0
    ctr_yaw = 0
    ctr_omega = np.array([0.0,0.0,0.0])

    ctr = { 'pos': ctr_pos,
            'vel': ctr_vel,
            'acc': ctr_acc,
            'yaw': ctr_yaw,
            'omega': ctr_omega }

    return ctr

def executeControl(timeHelper, cf, rate=20):
    
    global status
    global pose
    global twist

    start_time = timeHelper.time()
    pos_start = pose["pos"]
    
    print('Executing controller script...')
    print('Battery = ',status["battery"],'Supervisor: ',format(status["supervisor"], '08b'))

    while not timeHelper.isShutdown():
            t_total = timeHelper.time() - start_time
            if t_total > 30.0:
                break

            state_desired = getReference(t_total,pos_start)
            state = getState()
            # if state["supervisor"]
            ctr = doControl(state,state_desired)

            cf.cmdFullState(
                ctr["pos"],
                ctr["vel"],
                ctr["acc"],
                ctr["yaw"],
                ctr["omega"])
            
            timeHelper.sleepForRate(rate)

    print('Finished controller script.')

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
    Z = 1.0
    offset=np.array([0, 0, Z])

    # # # enable logging to SD card if present (assuming disabled before)
    # # cf.setParam('usd.logging',1)
    # # timeHelper.sleep(1.0)

    global pose
    global twist
    global status

    print('Preparing for takeoff...')
    timeHelper.sleep(5.0)
    try:
        test_pos = pose["pos"]
        test_lbd = pose["lbd"]
        test_vel = twist["vel"]
        test_om = twist["om"]
        test_batt = status["battery"]
        test_superv = status["supervisor"]
        print('System checks all OK. Battery = ',test_batt,'Supervisor: ',format(test_superv, '08b'))
    except Exception as e:
        print('Aborting flight: exception occurred.')
        traceback.print_exc()
        return

    # takeoff and go to initial position
    print('Takeoff')
    cf.takeoff(targetHeight=Z, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)
    pos = np.array(cf.initialPosition) + offset
    cf.goTo(pos, 0, 2.0)
    timeHelper.sleep(Z+2.0)
    
    try:
        executeControl(timeHelper, cf, rate)
    except Exception as e:
        print('Aborting controller: exception occurred.')
        traceback.print_exc()

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
