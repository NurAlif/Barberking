#!/usr/bin/env python3

import os
import argparse
import math

import signal
import sys

import rospy
import time
import json

# websocket
import threading
# import asyncio
from simple_websocket_server import WebSocketServer, WebSocket
from simple_pid import PID

from std_msgs.msg import String
from robotis_controller_msgs.msg import SyncWriteItem
from robotis_controller_msgs.msg import StatusMsg
from op3_walking_module_msgs.msg import WalkingParam
from sensor_msgs.msg import Imu
from op3_walking_module_msgs.srv import GetWalkingParam

import cv2
import pycuda.autoinit

from utils.yolo_classes import get_cls_dict
from utils.camera import add_camera_args, Camera
from utils.display import show_fps
from utils.visualization import BBoxVisualization
from utils.mjpeg import MjpegServer
from utils.yolo_with_plugins import TrtYOLO

###############################################################################

# YOLO

class Tracking:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.m = 100.0

track_ball = Tracking()

control_angle = 0
control_speed = 0
control_mode = 0
control_angle_time = 0
control_speed_time = 0
control_timeout = 100

out_scale = 0.2
servo_scale = 100
motor_speed = 290 # /500
turn_diff = 300 # /500
turn_max_t = 10
turn_cooldown = 3
move_max_t = 10
move_cooldown = 3

offset_in_x = 0.13
offset_in_y = 0.15

servo_x = 1500
servo_y = 1500


pid_x = PID(2, 0.5, 0.05, setpoint=0)
pid_y = PID(2, 0.5, 0.05, setpoint=0)

pid_x.output_limits = (-1.3, 1.3)
pid_y.output_limits = (-1.3, 1.3)


def loop_and_detect(cam, trt_yolo, conf_th, vis, mjpeg_server):

    fps = 0.0
    tic = time.time()

    # control vars
    res_x = 640.0
    res_y = 480.0

    lost_count = 0
    max_lost = 120

    turn_time = 0
    move_time = 0

    global track_ball
    global pid_x
    global pid_y

    global control_angle
    global control_speed
    global control_mode
    global control_angle_time
    global control_speed_time
    global control_timeout

    global out_scale
    global servo_scale
    global motor_speed # /500
    global turn_diff # /500
    global turn_max_t
    global turn_cooldown
    global move_max_t
    global move_cooldown
    
    global offset_in_x
    global offset_in_y

    global servo_x
    global servo_y

    # MAIN LOOP
    while True:
        img = cam.read()
        if img is None:
            break
        boxes, confs, clss = trt_yolo.detect(img, conf_th)

        json = vis.get_json(boxes, confs, clss)
        for client in clients:
            client.send_message(json)

        img = vis.draw_bboxes(img, boxes, confs, clss)
        img = show_fps(img, fps)
        mjpeg_server.send_img(img)
        toc = time.time()
        curr_fps = 1.0 / (toc - tic)
        # calculate an exponentially decaying average of fps number
        fps = curr_fps if fps == 0.0 else (fps*0.95 + curr_fps*0.05)


        # ============      Control       ============
        closest = Tracking()
        closest_c = 0.0
        d_closest = 100.0

        found = False

        for i_box, i_class, conf_c in zip(boxes, clss, confs):
            if i_class == 0:
                tracked = Tracking()
                size_x = (i_box[2] - i_box[0]) /res_x
                size_y = (i_box[3] - i_box[1]) /res_y
                tracked.x = ((size_x / 2.0 + i_box[0]) / res_x) - 0.5 + offset_in_x
                tracked.y = ((size_y / 2.0 + i_box[1]) / res_y) - 0.5 + offset_in_y
                tracked.m = math.sqrt((size_x*size_x) + (size_y*size_y))

                d_x = tracked.x - track_ball.x
                d_y = tracked.y - track_ball.y

                dist = math.sqrt((d_x*d_x) + (d_y*d_y))

                if dist < d_closest:
                    d_closest = dist
                    closest_c = conf_c
                    closest = tracked
                    found = True

        if found:
            print(" ")
            print("inference time : ", (toc - tic))
#            print("confidence : ", closest_c)
            track_ball = closest
            lost_count = 0
        else:
            lost_count += 1

            if lost_count > max_lost:
                lost_count = 0
                track_ball.x = 0.0
                track_ball.y = 0.0
        tic = toc

        # PID

        out_x = pid_x(track_ball.x) * out_scale
        out_y = pid_y(track_ball.y) * out_scale

        servo_x += out_x * servo_scale
        servo_y += out_y * servo_scale

        servo_x = max(min(servo_x, 2000), 1000)
        servo_y = max(min(servo_y, 2000), 1000)

        # motor turn
        motor_l = 1500
        motor_r = 1500

        if control_mode > 0:

            if found:
                turn = 0
                if servo_x < 1300:
                    turn = -1
                    motor_l -= turn_diff
                elif servo_x > 1700:
                    motor_l += turn_diff
                    turn = 1

                if turn != 0:
                    turn_time += 1
                    if turn_time > turn_max_t:
                        turn_time = -turn_cooldown
                        motor_l = 1500

                # motor move
                motor_go = False
                if servo_y > 1000:
                    motor_l += motor_speed
                    motor_r += motor_speed
                    motor_go = True

                if motor_go:
                    move_time += 1
                    if move_time > move_max_t:
                        move_time = -move_cooldown
                        motor_r = 1500

        else:
            if control_angle < 0:
                motor_l += turn_diff
            elif control_angle > 0:
                motor_l -= turn_diff

            if control_speed < 0:
                motor_l -= motor_speed
                motor_r -= motor_speed
            elif control_speed > 0:
                motor_l += motor_speed
                motor_r += motor_speed


        motor_l = max(min(motor_l, 2000), 1000)
        motor_r = max(min(motor_r, 2000), 1000)

                
        res_s = 'C' + str(int(servo_x)) + 'X' + str(int(servo_y)) + 'Y' + str(int(motor_l)) + 'N' + str(int(motor_r)) + 'M'
        # print(res_s)
        res = bytes(res_s, 'utf-8')

        #ser.write(res)

def inference():
    args = parse_args()
    if args.category_num <= 0:
        raise SystemExit('ERROR: bad category_num (%d)!' % args.category_num)
    if not os.path.isfile('yolo/%s.trt' % args.model):
        raise SystemExit('ERROR: file (yolo/%s.trt) not found!' % args.model)

    cam = Camera(args)
    if not cam.isOpened():
        raise SystemExit('ERROR: failed to open camera!')

    cls_dict = get_cls_dict(args.category_num)
    vis = BBoxVisualization(cls_dict)
    trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box)

    mjpeg_server = MjpegServer(port=args.mjpeg_port)
    print('MJPEG server started...')
    try:
        loop_and_detect(cam, trt_yolo, conf_th=0.3, vis=vis,
                        mjpeg_server=mjpeg_server)
    except Exception as e:
        print(e)
    finally:
        mjpeg_server.shutdown()
        cam.release()

def parse_args():
    desc = 'MJPEG version of trt_yolo'
    parser = argparse.ArgumentParser(description=desc)
    parser = add_camera_args(parser)
    category_num = 80
    model = "yolov3-tiny_last"
    letter_box = False
    mjpeg_port = 8090
    args = {
        
    }
    return args

    
###############################################################################

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
    def __init__(self, x=0.0, y=0.000, yaw=0.000):
        self.x = x
        self.y = y
        self.yaw = yaw
    
    @staticmethod
    def multiply(a, b):
        return Vector2yaw(a.x * b.x, a.y * b.y, a.yaw * b.yaw)

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
        selfx = self.x
        selfy = self.y
        selfyaw = self.yaw
        targetx = target.x
        targety = target.y
        targetyaw = target.yaw
        if(selfx < targetx): 
            self.x += step.x
            if selfx > targetx: self.x = targetx
        elif(selfx > targetx): 
            self.x -= step.x
            if selfx < targetx: self.x = targetx

        if(selfy < targety): 
            self.y += step.y
            if selfy > targety: self.y = targety
        elif(selfy > targety): 
            self.y -= step.y
            if selfy < targety: self.y = targety

        if(selfyaw < targetyaw): 
            self.yaw += step.yaw
            if selfyaw > targetyaw: self.yaw = targetyaw
        elif(selfyaw > targetyaw): 
            self.yaw -= step.yaw
            if selfyaw < targetyaw: self.yaw = targetyaw


CONTROL_MODE_HEADLESS = 0
CONTROL_MODE_YAWMODE = 1

class Walking:
    def __init__(self):
        global CONTROL_MODE_YAWMODE
        global CONTROL_MODE_HEADLESS
        self.control = None # held controller socket id
        self.turn_mode = CONTROL_MODE_YAWMODE
        self.max_speed = 40
        self.stationary_offset = Vector2yaw()
        self.feed_rate = 10
        self.step = Vector2yaw(0.001,0.001,0.001)
        self.vectorMultiplier = Vector2yaw(0.02, 0.02, 0.02)
        self.vectorCurrent = Vector2yaw()
        self.vectorTarget = Vector2yaw() # normalized

    def setTarget(self, newTarget): # normalized input
        self.vectorTarget = Vector2yaw.add(Vector2yaw.multiply(newTarget, self.vectorMultiplier), self.stationary_offset)

    def stepToTargetVel(self):
        self.vectorCurrent.stepToTarget(self.vectorTarget, self.step)
        # self.vectorCurrent.set(self.vectorTarget)

    def sendWithWalkParams(self):
        if(self.control == None): return

        global walkParams
        global pubSetParams
        walkParams.x_move_amplitude = self.vectorCurrent.y
        if(self.turn_mode == CONTROL_MODE_HEADLESS):
            walkParams.y_move_amplitude = self.vectorCurrent.x
            walkParams.angle_move_amplitude = 0.0
        else:
            walkParams.angle_move_amplitude = self.vectorCurrent.yaw
            walkParams.y_move_amplitude = 0.0
        
        pubSetParams.publish(walkParams)
        send_message(-1, "update_walking", self.getWalkingCurrent())
    
    def setWalkingOffset(self):
        self.stationary_offset.set(self.vectorCurrent)

    def setWalkingConf(self, confDict):
        confName = confDict[0]
        confValue = confDict[1]
        if confName == 'max_speed': self.max_speed = confValue
        elif confName == 'stationary_offset':
            self.stationary_offset.x = confValue[0]
            self.stationary_offset.y = confValue[1]
            self.stationary_offset.yaw = confValue[2]
        elif confName == 'feed_rate': self.feed_rate = confValue
        elif confName == 'step':
            if(confValue[0] == "xy"):
                self.step.x = confValue[1]
                self.step.y = confValue[1]
            elif(confValue[0] == "yaw"):
                self.step.yaw = confValue[1]
        elif confName == 'multipler':
            if(confValue[0] == "xy"):
                self.step.x = confValue[1]
                self.step.y = confValue[1]
            elif(confValue[0] == "yaw"):
                self.step.yaw = confValue[1]
        elif confName == 'turn_mode': self.turn_mode = confValue
        elif confName == 'offset':
            if confValue[0] == "x": self.stationary_offset.x = confValue[1]
            elif confValue[0] == "y": self.stationary_offset.y = confValue[1]
            elif confValue[0] == "yaw": self.stationary_offset.yaw = confValue[1]

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
        if(self.control == None):
            confDict['control'] = -1
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
    # "l_el",
    "l_hip_pitch",
    "l_hip_roll",
    "l_hip_yaw",
    "l_knee",
    "l_sho_pitch",
    "l_sho_roll",
    "r_ank_pitch",
    "r_ank_roll",
    # "r_el",
    "r_hip_pitch",
    "r_hip_roll",
    "r_hip_yaw",
    "r_knee",
    "r_sho_pitch",
    "r_sho_roll"
]

def enableWalk():
    pubEnaMod.publish("walking_module")

def setDxlTorque(): # list comprehension
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

def initGyro():
    syncwrite_msg = SyncWriteItem()
    syncwrite_msg.item_name = "imu_control"
    syncwrite_msg.joint_name.append("open-cr")
    syncwrite_msg.value.append(8)

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
        clients[id].send_message(respJson)
    else:
        for client in clients.values():
            client.send_message(respJson)

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
    print(statusMsg.status_msg)
    if(statusMsg.status_msg == "Walking Enabled"):
        init_gyro()
        print("init gyro...")

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
        elif cmd == "set_walking":
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
        elif cmd == "set_control_walking":
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
        clientID = self.address[1]
        clients.update({clientID: self})
        send_message(clientID, "device_connected", self.address)
        send_message(clientID, "torque_control", robotIsOn)

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


    print("program runnning")
    rate = rospy.Rate(walking.feed_rate) # 10hz

    time.sleep(20)
    getWalkParams()
    # startRobot()
    
    while not rospy.is_shutdown():
        walking.stepToTargetVel()
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
