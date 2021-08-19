#!/usr/bin/env python3

import os
import time
import argparse

import cv2
import pycuda.autoinit

import math

from utils.yolo_classes import get_cls_dict
from utils.camera import add_camera_args, Camera, camera_args
from utils.display import show_fps
from utils.visualization import BBoxVisualization
from utils.mjpeg import MjpegServer
from utils.yolo_with_plugins import TrtYOLO


# control vars
res_x = 640.0
res_y = 480.0

max_lost = 120
goto_zero = 0.01    
stop_zone = 0.20

offset_in_x = 0.13
offset_in_y = 0.15

lost_count = 0


# YOLO PARAMS

category_num = 80
model = "yolov3-tiny_last"
letter_box = False
mjpeg_port = 8090

cam = None
trt_yolo = None
conf_th = 0.3 
vis = None
mjpeg_server = None 


class Tracking:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.m = 100.0

    def set(self, tar):
        self.x = tar.x
        self.y = tar.y
        self.m = tar.m

def parse_args():
    desc = 'MJPEG version of trt_yolo'
    parser = argparse.ArgumentParser(description=desc)
    parser = add_camera_args(parser)
    parser.add_argument(
        '-c', '--category_num', type=int, default=80,
        help='number of object categories [80]')
    parser.add_argument(
        '-m', '--model', type=str, required=True,
        help=('[yolov3-tiny|yolov3|yolov3-spp|yolov4-tiny|yolov4|'
              'yolov4-csp|yolov4x-mish]-[{dimension}], where '
              '{dimension} could be either a single number (e.g. '
              '288, 416, 608) or 2 numbers, WxH (e.g. 416x256)'))
    parser.add_argument(
        '-l', '--letter_box', action='store_true',
        help='inference with letterboxed image [False]')
    parser.add_argument(
        '-p', '--mjpeg_port', type=int, default=8090,
        help='MJPEG server port [8090]')
    args = parser.parse_args()
    return args


def detect(track_ball):

    global tic
    global lost_count

    img = cam.read()
    if img is None:
        return None
    boxes, confs, clss = trt_yolo.detect(img, conf_th)

    json = vis.get_json(boxes, confs, clss)

    img = vis.draw_bboxes(img, boxes, confs, clss)

    mjpeg_server.send_img(img)

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
        track_ball.set(closest)
        lost_count = 0
    else:
        # lost_count += 1

        # if track_ball.x < -stop_zone:
        #     track_ball.x += goto_zero
        # if track_ball.x > stop_zone:
        #     track_ball.x -= goto_zero
        # if track_ball.y < -stop_zone:
        #     track_ball.y += goto_zero
        # if track_ball.y > stop_zone:
        #     track_ball.y -= goto_zero    

        # if lost_count > max_lost:
        #     lost_count = 0
            track_ball.x = 0.0
            track_ball.y = 0.0


def startInference():

    global cam
    global trt_yolo
    global vis
    global mjpeg_server

    if category_num <= 0:
        raise SystemExit('ERROR: bad category_num (%d)!' % category_num)
    if not os.path.isfile('/home/nvidia/project/tensorrt_demos/yolo/%s.trt' % model):
        raise SystemExit('ERROR: file (yolo/%s.trt) not found!' % model)

    cam_args = camera_args()
    cam = Camera(cam_args)
    if not cam.isOpened():
        raise SystemExit('ERROR: failed to open camera!')

    cls_dict = get_cls_dict(category_num)
    vis = BBoxVisualization(cls_dict)
    trt_yolo = TrtYOLO(model, category_num, letter_box)

    mjpeg_server = MjpegServer(port=mjpeg_port)
    print('MJPEG server started...')

def inferenceLoop(track_ball):
    while(True):
        detect(track_ball)


def shutdown():
    global mjpeg_server
    global cam
    mjpeg_server.shutdown()
    cam.release()