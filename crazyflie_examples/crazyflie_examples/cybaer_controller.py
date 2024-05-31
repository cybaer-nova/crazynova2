#!/usr/bin/env python

import traceback
# from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
from crazyflie_interfaces.msg import LogDataGeneric

import numpy as np
from scipy import constants as spc
import csv
import os
from datetime import datetime
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


# fields = ['ttag_ros','time','px','py','pz','vx','vy','vz','phi','theta','psi','omx','omy','omz','pxd','pyd','pzd','vxd','vyd','vzd','u_px','u_py','u_pz','u_vx','u_vy','u_vz','u_ax','u_ay','u_az','u_psi','batt','supervisor']

def getLogDict(state,state_desired,ctrl):

    dict_log = dict(state)
    dict_log.update(state_desired)
    dict_log.update(ctrl)

    return dict_log


def status_simple_topic_callback(msg):
    """
    Call back for topic /cfXXX/status_simple.
    vars: ["pm.vbatMV","supervisor.info"]
    """
    global status
    status = {  'ttag': msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
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
    pose = {'ttag': msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
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
    twist = {   'ttag': msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
                'vel': vel,
                'om': om }
    
    # print(f'twist_simple_topic_callback was called {twist}')

def getReference(t,pos_start):
     
    state_desired = {   'ttag': t,
                        'pos_d': np.zeros(3),
                        'vel_d': np.zeros(3),
                        'acc_d': np.zeros(3),
                        'lbd_d': np.zeros(3),
                        'om_d': np.zeros(3) }

    dX = 0.7

    if t < 10.0:
        state_desired["pos_d"] = pos_start + np.array([dX,0,0])
    elif t < 20.0:
        state_desired["pos_d"] = pos_start + np.array([-dX,0,0])
    else:
        state_desired["pos_d"] = pos_start
    
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

    state = {   'ttag': pose["pos"],
                'pos': pose["pos"],
                'lbd': pose["lbd"],
                'vel': twist["vel"],
                'om': twist["om"],
                'batt': status["battery"],
                'supervisor': status["supervisor"] }

    return state

def doControl(state,state_desired):

    # get desired state
    lbd_d = state_desired["lbd_d"]
    yaw_d = lbd_d[2]
    pos_d = state_desired["pos_d"]
    vel_d = state_desired["vel_d"]
    acc_d = state_desired["acc_d"]

    # get state
    pos = state["pos"]
    vel = state["vel"]
    lbd = state["lbd"]
    yaw = lbd[2]

    # Controller and vehicle parameters
    m = 0.032 # [kg] crazyflie has arround 32g of mass 
    g = np.array([0,0,spc.g])
    k_p = np.array([1,1,5])   # ?? (20 Hz)
    k_d = np.array([0.1,0.1,2])   # ??
    k_yaw = 1 # ??
    
    # define control errors (assuming zero yaw)
    e_pos = pos - pos_d
    e_vel = vel - vel_d
    e_yaw = yaw - yaw_d*0 # force zero desired yaw

    # comput control action (in acc and convert to [T zB])
    ctr_acc = -k_p*e_pos - k_d*e_vel + g + acc_d
    ctr_T = np.linalg.norm(ctr_acc)*m
    if abs(ctr_T) < 1e-5:
        ctr_zB = np.zeros(3)
    else:
        ctr_zB = m*ctr_acc/ctr_T
    
    # get roll and pitch from zB 
    # for zero yaw, zB = [cos(phi)*sin(theta); -sin(phi) ; cos(phi)*cos(theta)])
    # for non-zero yaw build desired rotation matriz and extract desired Euler angles
    ctr_phi = np.arcsin(ctr_zB[1]);
    ctr_theta = np.arctan(ctr_zB[0]/ctr_zB[2]);
    ctr_omegaz = -k_yaw*e_yaw;

    print('[Crazyflie control] ctr_T = ',ctr_T,'ctr_zB = ',ctr_zB)

    max_T_newtons = m*spc.g*1.0;
    max_T_units = 60000;
    ctr = { 'e_pos': e_pos,
            'e_vel': e_vel,
            'e_yaw': e_yaw,
            'u_acc': ctr_acc,
            'u_T': ctr_T*max_T_units/max_T_newtons,
            'u_phi': ctr_phi*180.0/np.pi,
            'u_theta': ctr_theta*180.0/np.pi,
            'u_omz': ctr_omegaz*180.0/np.pi,}

    return ctr

def executeControl(timeHelper,cf,rate,logwriter):
    
    start_time = timeHelper.time()
    state = getState()
    pos_start = state["pos"]
    exit_code = 0
    
    print('[Crazyflie control] Executing controller script...')
    print('[Crazyflie control] Battery = ',state["batt"],'Supervisor: ',format(state["supervisor"], '08b'))

    while not timeHelper.isShutdown():
            t_total = timeHelper.time() - start_time
            if t_total > 30.0:
                break

            state_desired = getReference(t_total,pos_start)
            state = getState()

            # TODO if state["supervisor"] OK ...
            ctr = doControl(state,state_desired)
            # else break and exit signaling something went wrong.
            
            # limit actuation            
            np.clip(ctr["u_phi"],-10,10)
            np.clip(ctr["u_theta"],-10,10)
            np.clip(ctr["u_omz"],-50,50)
            np.clip(ctr["u_T"],0.0,2^16)

            print('[Crazyflie control] ctr = ',ctr)

            cf.cmdVelLegacy(
                ctr["u_phi"],
                ctr["u_theta"],
                ctr["u_omz"],
                ctr["u_T"])
            # cf.cmdVelLegacy(
            #     0.0,
            #     0.0,
            #     0.0,
            #     ctr["u_T"])
            
            
            # get log dict
            log = getLogDict(state,state_desired,ctr)
            logwriter.writerow(log)

            timeHelper.sleepForRate(rate)

    print('[Crazyflie control] Finished controller script. (exit_code = ',exit_code,')')

    return exit_code

def main():
    # get CF drone object:
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    # add subscribers to CF object:
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

    # define HL control parameters
    rate = 20.0
    Z = 1.0
    dT = 2.0
    offset=np.array([0, 0, Z])
    do_internal_log = 0

    # enable logging to SD card if present (assuming disabled before)
    if do_internal_log:
        cf.setParam('usd.logging',1)
        timeHelper.sleep(1.0)

    global pose
    global twist
    global status

    print('[Crazyflie control] Preparing for takeoff in 3 s...')
    timeHelper.sleep(3.0)
    try:
        state = getState()
        state_desired = getReference(0,np.array([0,0,0]))
        ctr = doControl(state,state_desired)
        log = getLogDict(state,state_desired,ctr)
        print('[Crazyflie control] System checks all OK. Battery = ',state['batt'],'Supervisor: ',format(state['supervisor'], '08b'))
    except Exception as e:
        print('[Crazyflie control] Aborting flight: exception occurred.')
        traceback.print_exc()
        return
    
    # setting up custom logging to CSV file
    outputcsv = datetime.today().strftime('%Y%m%dT%H%M%S')+'_CtrlLog.csv'
    csvfile = open(outputcsv,'w', newline='')
    print("[Crazyflie control] Writing log data to: ",outputcsv)
    fields = list(log.keys())
    logwriter = csv.DictWriter(csvfile, fieldnames=fields)
    logwriter.writeheader()
    state = getState()
    state_desired = getReference(0,np.array([0,0,0]))
    ctr = doControl(state,state_desired)
    log = getLogDict(state,state_desired,ctr)
    logwriter.writerow(log)
    print("[Crazyflie control] log data fields: ",fields)

    # takeoff and go to initial position
    print('[Crazyflie control] Takeoff')
    cf.takeoff(targetHeight=Z, duration=Z+dT)
    timeHelper.sleep(Z+dT)
    pos = np.array(cf.initialPosition) + offset
    cf.goTo(pos, 0, dT)
    timeHelper.sleep(dT)

    state = getState()
    state_desired = getReference(0,np.array([0,0,0]))
    ctr = doControl(state,state_desired)
    log = getLogDict(state,state_desired,ctr)
    logwriter.writerow(log)
    
    try:
        exit_code = executeControl(timeHelper,cf,rate,logwriter)
    except Exception as e:
        print('[Crazyflie control] Aborting controller: exception occurred.')
        traceback.print_exc()

    print('[Crazyflie control] RTL')
    cf.notifySetpointsStop()
    pos = np.array(cf.initialPosition) + offset
    cf.goTo(pos, 0, dT)
    timeHelper.sleep(dT)
    
    state = getState()
    state_desired = getReference(0,np.array([0,0,0]))
    ctr = doControl(state,state_desired)
    log = getLogDict(state,state_desired,ctr)
    logwriter.writerow(log)

    print('[Crazyflie control] Landing')
    cf.land(targetHeight=0.03, duration=Z+dT)
    timeHelper.sleep(Z+dT)
    
    state = getState()
    state_desired = getReference(0,np.array([0,0,0]))
    ctr = doControl(state,state_desired)
    log = getLogDict(state,state_desired,ctr)
    logwriter.writerow(log)
    csvfile.close()
    
    # disable logging to SD card if present (assuming enabled before)
    if do_internal_log:
        cf.setParam('usd.logging',0)
        timeHelper.sleep(1.0)


if __name__ == '__main__':
    main()
