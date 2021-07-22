#!/usr/bin/env python

import signal
import sys

import rospy
import time
import json

# websocket
import threading
# import asyncio
from simple_websocket_server import WebSocketServer, WebSocket

from std_msgs.msg import String
from robotis_controller_msgs.msg import SyncWriteItem
from robotis_controller_msgs.msg import StatusMsg
from op3_walking_module_msgs.msg import WalkingParam
from sensor_msgs.msg import Imu
from op3_walking_module_msgs.srv import GetWalkingParam

pubSWI = rospy.Publisher('/robotis/sync_write_item', SyncWriteItem, queue_size=10)
pubBT = rospy.Publisher('/robotis/open_cr/button', String, queue_size=10)
pubEnaMod = rospy.Publisher('/robotis/enable_ctrl_module', String, queue_size=10)
pubWalkCmd = rospy.Publisher('/robotis/walking/command', String, queue_size=10)
pubSetParams = rospy.Publisher('/robotis/walking/set_params', WalkingParam, queue_size=10)

currentWalkParams = None # ? params
walkParams = None

server = None

robotIsOn = True
walking_module_enabled = False

class Vector2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Vector2yaw:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
    
    @staticmethod
    def multiply(a, b):
        temp = Vector2yaw(0,0,0)
        temp.x = a.x * b.x
        temp.y = a.y * b.y
        temp.yaw = a.yaw * b.yaw
        return temp

    @staticmethod
    def add(a, b):
        temp = Vector2yaw(0,0,0)
        temp.x = a.x + b.x
        temp.y = a.y + b.y
        temp.yaw = a.yaw + b.yaw
        return temp
    
    def set(self, target):
        self.x = target.x
        self.y = target.y
        self.yaw = target.yaw

    def stepToTarget(self, target, step):
        if(self.x < target.x): 
            self.x += step.x
            if self.x > target.x: self.x = target.x
        elif(self.x > target.x): 
            self.x -= step.x
            if self.x < target.x: self.x = target.x

        if(self.y < target.y): 
            self.y += step.y
            if self.y > target.y: self.y = target.y
        elif(self.y > target.y): 
            self.y -= step.y
            if self.y < target.y: self.y = target.y

        if(self.yaw < target.yaw): 
            self.yaw += step.yaw
            if self.yaw > target.yaw: self.yaw = target.yaw
        elif(self.yaw > target.yaw): 
            self.yaw -= step.yaw
            if self.yaw < target.yaw: self.yaw = target.yaw


CONTROL_MODE_HEADLESS = 0
CONTROL_MODE_YAWMODE = 1

class Walking:
    def __init__(self):
        global CONTROL_MODE_YAWMODE
        global CONTROL_MODE_HEADLESS
        self.control = None # held controller socket id
        self.turn_mode = CONTROL_MODE_YAWMODE
        self.max_speed = 40
        self.stationary_offset = Vector2yaw(0.0,0.0,0.0)
        self.feed_rate = 10
        self.step = Vector2yaw(1,1,1)
        self.vectorMultiplier = Vector2yaw(1,1,1)
        self.vectorCurrent = Vector2yaw(0,0,0)
        self.vectorTarget = Vector2yaw(0,0,0) # normalized

    def setTarget(self, newTarget): # normalized input
        self.vectorTarget = Vector2yaw.add(Vector2yaw.multiply(newTarget * self.vectorMultiplier),self.stationary_offset)

    def stepToTargetVel(self):
        self.vectorCurrent.stepToTarget(self.target, self.step)

    def sendWithWalkParams(self):
        if(self.control == None): return

        global walkParams
        global pubSetParams
        walkParams.x_move_amplitude = self.vectorCurrent.x
        walkParams.y_move_amplitude = self.vectorCurrent.y
        walkParams.angle_move_amplitude = self.vectorCurrent.yaw
        
        pubSetParams.publish(walkParams)
        send_message(-1, "update_walking", self.getWalkingCurrent())
    
    def setWalkingOffset(self):
        self.stationary_offset.set(self.vectorCurrent)

    def setWalkingConf(self, confDict):
        if confDict[0] == 'max_speed': self.max_speed = confDict[1]
        elif confDict[0] == 'stationary_offset':
            confOffset = confDict[1]
            self.stationary_offset.x = confOffset[0]
            self.stationary_offset.y = confOffset[1]
            self.stationary_offset.yaw = confOffset[2]
        elif confDict[0] == 'feed_rate': self.feed_rate = confDict[1]
        elif confDict[0] == 'step':
            confStep = confDict[1]
            if(confStep[0] == "xy"):
                self.step.x = confStep[1]
                self.step.y = confStep[1]
            elif(confStep[0] == "yaw"):
                self.step.yaw = confStep[1]
        elif confDict[0] == 'multipler':
            confStep = confDict[1]
            if(confStep[0] == "xy"):
                self.step.x = confStep[1]
                self.step.y = confStep[1]
            elif(confStep[0] == "yaw"):
                self.step.yaw = confStep[1]
        elif confDict[0] == 'turn_mode': self.turn_mode = confDict[1]
    def getWalkingConf(self):
        offset = self.stationary_offset
        multiplier = self.vectorMultiplier
        step = self.step
        confDict = {
            'control': self.control,
            'max_speed': self.max_speed,
            'turn_mode': self.turn_mode,
            'stationary_offset': [offset.x, offset.y, offset.yaw],
            'feed_rate': self.feed_rate,
            'step':[step.x, step.y, step.yaw],
            'multiplier': [multiplier.x, multiplier.y, multiplier.yaw]
        }
        return confDict
    def getWalkingCurrent(self):
        current = self.vectorCurrent
        array = [current.x, current.y, current.yaw]
        return array

walking = Walking()



joints = [
    "head_pan",
    "head_tilt",
    "l_ank_pitch",
    "l_ank_roll",
    "l_el",
    "l_hip_pitch",
    "l_hip_roll",
    "l_hip_yaw",
    "l_knee",
    "l_sho_pitch",
    "l_sho_roll",
    "r_ank_pitch",
    "r_ank_roll",
    "r_el",
    "r_hip_pitch",
    "r_hip_roll",
    "r_hip_yaw",
    "r_knee",
    "r_sho_pitch",
    "r_sho_roll"
]

def enableWalk():
    pubEnaMod.publish("walking_module")

def setDxlTorque():
    global robotIsOn

    isTorqueOn = False

    if robotIsOn == False:
        return
    robotIsOn = False

    syncwrite_msg = SyncWriteItem()
    syncwrite_msg.item_name = "torque_enable"
    for joint_name in joints:
        syncwrite_msg.joint_name.append(joint_name)
        syncwrite_msg.value.append(isTorqueOn) 

    pubSWI.publish(syncwrite_msg)

def startRobot():
    global robotIsOn
    if robotIsOn:
        return
    robotIsOn = True
    pubBT.publish("user_long")

def setWalkCmd(walkCmd):
    if walkCmd == "start" or walkCmd == "stop" or walkCmd == "balance on" or walkCmd == "balance off" or walkCmd == "save":
        pubWalkCmd.publish(walkCmd)

def setWalkParams(param):
    global walkParam

    paramName = param[0]
    paramValue = param[1]
    print(param)
    print(paramName)

    if paramName == u"init_x_offset" : walkParams.init_x_offset = paramValue
    elif paramName == u"init_y_offset" : walkParams.init_y_offset = paramValue
    elif paramName == u"init_z_offset" : walkParams.init_z_offset = paramValue
    elif paramName == u"init_roll_offset" : walkParams.init_roll_offset = paramValue
    elif paramName == u"init_pitch_offset" : walkParams.init_pitch_offset = paramValue
    elif paramName == u"init_yaw_offset" : walkParams.init_yaw_offset = paramValue
    elif paramName == u"period_time" : walkParams.period_time = paramValue
    elif paramName == u"dsp_ratio" : walkParams.dsp_ratio = paramValue
    elif paramName == u"step_fb_ratio" : walkParams.step_fb_ratio = paramValue
    elif paramName == u"x_move_amplitude" : walkParams.x_move_amplitude = paramValue
    elif paramName == u"y_move_amplitude" : walkParams.y_move_amplitude = paramValue
    elif paramName == u"z_move_amplitude" : walkParams.z_move_amplitude = paramValue
    elif paramName == u"angle_move_amplitude" : walkParams.angle_move_amplitude = paramValue
    elif paramName == u"move_aim_on" : walkParams.move_aim_on = paramValue
    elif paramName == u"balance_enable" : walkParams.balance_enable = paramValue
    elif paramName == u"balance_hip_roll_gain" : walkParams.balance_hip_roll_gain = paramValue
    elif paramName == u"balance_knee_gain" : walkParams.balance_knee_gain = paramValue
    elif paramName == u"balance_ankle_roll_gain" : walkParams.balance_ankle_roll_gain = paramValue
    elif paramName == u"balance_ankle_pitch_gain" : walkParams.balance_ankle_pitch_gain = paramValue
    elif paramName == u"y_swap_amplitude" : walkParams.y_swap_amplitude = paramValue
    elif paramName == u"z_swap_amplitude" : walkParams.z_swap_amplitude = paramValue
    elif paramName == u"arm_swing_gain" : walkParams.arm_swing_gain = paramValue
    elif paramName == u"pelvis_offset" : walkParams.pelvis_offset = paramValue
    elif paramName == u"hip_pitch_offset" : walkParams.hip_pitch_offset = paramValue
    elif paramName == u"p_gain" : walkParams.p_gain = paramValue
    elif paramName == u"i_gain" : walkParams.i_gain = paramValue
    elif paramName == u"d_gain" : walkParams.d_gain = paramValue

    pubSetParams.publish(walkParams)

def getWalkParams():
    global currentWalkParams
    global walkParams
    rospy.wait_for_service('/robotis/walking/get_params')
    try:
        getParams = rospy.ServiceProxy('/robotis/walking/get_params', GetWalkingParam)
        resp = getParams()
        params = resp.parameters
        walkParams = params
        paramsDict = {
                "init_x_offset" : params.init_x_offset,             
                "init_y_offset" : params.init_y_offset,
                "init_z_offset" : params.init_z_offset,
                "init_roll_offset" : params.init_roll_offset,
                "init_pitch_offset" : params.init_pitch_offset,
                "init_yaw_offset" : params.init_yaw_offset,
                "period_time" : params.period_time,
                "dsp_ratio" : params.dsp_ratio,
                "step_fb_ratio" : params.step_fb_ratio,
                "x_move_amplitude" : params.x_move_amplitude,
                "y_move_amplitude" : params.y_move_amplitude,
                "z_move_amplitude" : params.z_move_amplitude,
                "angle_move_amplitude" : params.angle_move_amplitude,
                "move_aim_on" : params.move_aim_on,
                "balance_enable" : params.balance_enable,
                "balance_hip_roll_gain" : params.balance_hip_roll_gain,
                "balance_knee_gain" : params.balance_knee_gain,
                "balance_ankle_roll_gain" : params.balance_ankle_roll_gain,
                "balance_ankle_pitch_gain" : params.balance_ankle_pitch_gain,
                "y_swap_amplitude" : params.y_swap_amplitude,
                "z_swap_amplitude" : params.z_swap_amplitude,
                "arm_swing_gain" : params.arm_swing_gain,
                "pelvis_offset" : params.pelvis_offset,
                "hip_pitch_offset" : params.hip_pitch_offset,
                "p_gain" : params.p_gain,
                "i_gain" : params.i_gain,
                "d_gain" : params.d_gain        
        }
        currentWalkParams = paramsDict
        return paramsDict
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

clients = {}


def send_message(id, cmd, params):
    resp = {
        "cmd" : cmd,
        "params" : params
    }
    respJson = json.dumps(resp)
    if(id >= 0):
        clients[id].send_message(unicode(respJson, "utf-8"))
    else:
        for client in clients.values():
            client.send_message(unicode(respJson, "utf-8"))

def init_gyro():
    init_gyro_msg = SyncWriteItem()
    init_gyro_msg.item_name = "imu_control"
    init_gyro_msg.joint_name.append("open-cr")
    init_gyro_msg.value.append(8)
    pubSWI.publish(init_gyro_msg)

imu = Imu()

def handleImu(imu_msg_):
    global imu
    imu = imu_msg_

def onFinishInitPose():
    enableWalk()

def handleStatusMsg(statusMsg):

    if(statusMsg.status_msg == "Finish Init Pose"):
        enableWalk()
        send_message(-1, "torque_control", True)

    statusDict = {
        'type':statusMsg.type,
        'module_name':statusMsg.module_name,
        'status_msg':statusMsg.status_msg
    }
    send_message(-1, 'update_status', statusDict)

class WS(WebSocket):
    def handle(self):
        data = json.loads(self.data)

        cmd = data['cmd']

        if cmd == 'torque_on':
            startRobot()
        elif cmd == 'torque_off':
            setDxlTorque()
            send_message(-1, "torque_control", False)
        elif cmd == 'start_walk':
            setWalkCmd("start")
            send_message(-1, "walk_control", True)
        elif cmd == 'stop_walk':
            setWalkCmd("stop")
            send_message(-1, "walk_control", False)
        elif cmd == 'save_walk_params':
            setWalkCmd("save")
            send_message(-1, "walk_params_saved", True)
        elif cmd == 'get_walk_params':
            send_message(-1, "update_walk_params", getWalkParams())
        elif cmd == 'set_walk_params':
            setWalkParams(data['params'])
            send_message(-1, "controller_msg", 'Walk params changed')
        elif cmd == 'set_walking':
            if(self.address[1] == walking.control):
                vectorDict = data['params']
                vector = Vector2yaw(vectorDict["x"], vectorDict["y"], vectorDict["yaw"])
                walking.setTarget(vector)
        elif cmd == 'set_walking_offset':
            walking.setWalkingOffset()
            send_message(-1, "controller_msg", 'Walking offset changed')
        elif cmd == 'set_walking_conf':
            walking.setWalkingConf(data['params'])
            send_message(-1, "controller_msg", 'Walking configuration changed')
        elif cmd == 'set_control_walking':
            if data['params'] == 1:
                walking.control = self.address[1]
                send_message(-1, "control_override", self.address)
            else:
                walking.control = None
                send_message(-1, "control_override", -1)
        elif cmd == 'get_walking_conf':
            send_message(-1, "update_walking_conf", walking.getWalkingConf())
        elif cmd == 'get_walking':
            send_message(self.address[1], "update_walking", walking.getWalkingConf())
        elif cmd == 'gyro_init':
            init_gyro()
            send_message(-1, "controller_msg", "Init gyro success")

    def connected(self):
        print(self.address, 'connected')
        clients.update({self.address[1]: self})
        send_message(self.address[1], "device_connected", self.address)
        send_message(self.address[1], "torque_control", robotIsOn)

    def handle_close(self):
        clients.pop(self.address)
        print(self.address, 'closed')
        send_message(-1, "device_disconnected", self.address)


def forever_ws(num):
    global server

    server = WebSocketServer('', 8077, WS)
    print("Websocket is running...")
    server.serve_forever()


t1 = threading.Thread(target=forever_ws, args=(10,))

def main():
    t1.start()
    rospy.init_node('main', anonymous=True)

    #rospy.Subscriber("/robotis/open_cr/imu", Imu, handleImu)
    rospy.Subscriber("/robotis/status", StatusMsg, handleStatusMsg)
    enableWalk()
    print("program runnning")
    rate = rospy.Rate(walking.feed_rate) # 10hz
    while not rospy.is_shutdown():
        walking.sendWithWalkParams()
        rate.sleep()


def close_sig_handler(signal, frame):
    global server

    server.close()
    sys.exit()

signal.signal(signal.SIGINT, close_sig_handler)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        t1.join()
        exit()
