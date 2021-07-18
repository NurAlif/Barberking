#!/usr/bin/env python

import signal
import sys

import rospy
import time
import json

# websocket
import threading
from simple_websocket_server import WebSocketServer, WebSocket

from std_msgs.msg import String
from robotis_controller_msgs.msg import SyncWriteItem
from op3_walking_module_msgs.msg import WalkingParam
from op3_walking_module_msgs.srv import GetWalkingParam

pubSWI = rospy.Publisher('/robotis/sync_write_item', SyncWriteItem, queue_size=10)
pubBT = rospy.Publisher('/robotis/open_cr/button', String, queue_size=10)
pubEnaMod = rospy.Publisher('/robotis/enable_ctrl_module', String, queue_size=10)
pubWalkCmd = rospy.Publisher('/robotis/walking/command', String, queue_size=10)

server = None

robotIsOn = True

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

def setDxlTorque(isTorqueOn):
    global robotIsOn

    if (robotIsOn == False and isTorqueOn == 0):
        return
    else:
        robotIsOn = False

    if (robotIsOn == True and isTorqueOn == 1):
        return
    else:
        robotIsOn = True

    syncwrite_msg = SyncWriteItem()
    syncwrite_msg.item_name = "torque_enable"
    for joint_name in joints:
        syncwrite_msg.joint_name.append(joint_name)
        syncwrite_msg.value.append(isTorqueOn) 

    pubSWI.publish(syncwrite_msg)

def startRobot():
    # global robotIsOn
    # if robotIsOn == True:
    #     return
    
    # time.sleep(1)
    # robotIsOn = True
    pubBT.publish("user_long")

def enableWalk():
    pubEnaMod.publish("walking_module")

def setWalkCmd(walkCmd):
    if walkCmd == "start" or walkCmd == "stop" or walkCmd == "balance on" or walkCmd == "balance off" or walkCmd == "save":
        pubWalkCmd.publish(walkCmd)

def getWalkParams():
    rospy.wait_for_service('/robotis/walking/get_params')
    try:
        getParams = rospy.ServiceProxy('/robotis/walking/get_params', GetWalkingParam)
        resp = getParams()
        return resp.parameters
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

clients = []

class WS(WebSocket):
    def handle(self):
        print(self.address, 'receive data: ', self.data)

        data = json.loads(self.data)

        cmd = data['cmd']

        if cmd == 'torque_on':
            startRobot()
        elif cmd == 'torque_off':
            setDxlTorque(0)
        elif cmd == 'ena_walk':
            enableWalk()
        elif cmd == 'start_walk':
            setWalkCmd("start")
        elif cmd == 'stop_walk':
            setWalkCmd("stop")
        elif cmd == 'get_walk_params':
            params = getWalkParams()

            dict

            print("return param type: ")
            print(json.dumps())


        #for client in clients:
        #    client.send_message(self.address[0] + u' - ' + self.data)

    def connected(self):
        print(self.address, 'connected')
        for client in clients:
            client.send_message(self.address[0] + u' - connected')
        clients.append(self)


    def handle_close(self):
        clients.remove(self)
        print(self.address, 'closed')
        for client in clients:
            client.send_message(self.address[0] + u' - disconnected')

def forever_ws(num):
    global server

    server = WebSocketServer('', 8077, WS)
    print("Websocket is running...")
    server.serve_forever()


t1 = threading.Thread(target=forever_ws, args=(10,))

def main():
    t1.start()
    rospy.init_node('main', anonymous=True)
    print("program runnning")
    rospy.spin()


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
