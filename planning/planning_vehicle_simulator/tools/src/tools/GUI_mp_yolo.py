from __future__ import division
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QMessageBox, QFileDialog
from GUI.displayGUI import *
from PyQt5 import QtCore, QtGui
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.Qt import QThread, QTextCursor
import PyQt5
import time
# import pandas as pd
import numpy as np
import pickle
import threading
import serial
from GUI.WGS84 import CoordConverter
from GUI.CircleQueue2 import MyQueue
import GUI.WGS84 as wg
import csv
import multiprocessing as mp
import copy
from pynput import keyboard
import binascii
from multiprocessing import Process, Queue, Manager,Value, Lock
from multiprocessing.managers import BaseManager
import multiprocessing
import os
#import psutil

#目标检测


from models import *
from utils.utils import *
from utils.datasets import *
from my_utils.miss_det_comp import *

import os
import sys
import time
import datetime
import argparse

import torch
from torch.utils.data import DataLoader
from torchvision import datasets
from torch.autograd import Variable
import torch.nn.functional as F
from my_utils.server import *
# from utils.my_utils import *

import cv2

parser = argparse.ArgumentParser()
parser.add_argument('--image_folder', type=str, default='data/samples', help='path to dataset')
parser.add_argument('--video_path', type=str, default='/home/gogo/ll/Test/2019-08-13-210638.webm', help='path to dataset')
parser.add_argument('--config_path', type=str, default='config/yolov3-pedes-voc.cfg', help='path to model config file')
parser.add_argument('--weights_path', type=str, default='checkpoints/yolo_v5/yolov3_ckpt_27.pth', help='path to weights file')
# parser.add_argument('--weights_path', type=str, default='checkpoints/yolov3_ckpt608_36.pth', help='path to weights file')
# parser.add_argument('--weights_path', type=str, default='weights/yolov3.weights', help='path to weights file')
parser.add_argument('--class_path', type=str, default='data/class_dict.names', help='path to class label file')
parser.add_argument('--conf_thres', type=float, default=0.7, help='object confidence threshold')
parser.add_argument('--nms_thres', type=float, default=0.4, help='iou thresshold for non-maximum suppression')
parser.add_argument('--batch_size', type=int, default=1, help='size of the batches')
parser.add_argument('--n_cpu', type=int, default=0, help='number of cpu threads to use during batch generation')
parser.add_argument('--img_size', type=int, default=416, help='size of each image dimension')
parser.add_argument('--use_cuda', type=bool, default=True, help='whether to use cuda if available')

#print(opt)

os.makedirs('output', exist_ok=True)
def prep_image(image,size):

    begin2 = time.time()

    orig_img = image.copy()
    orig_im_size = orig_img.shape[1], orig_img.shape[0]

    input_img = cv2.resize(image, (size,size)) / 255
    #print("Resize img time: {:4.4f}".format(time.time()-begin2))
    begin3 = time.time()

    # Channels-first
    input_img = np.transpose(input_img, (2, 0, 1))
    # As pytorch tensor
    input_img = torch.from_numpy(input_img).float().unsqueeze(0)
    #print("transform img time: {:4.4f}".format(time.time()-begin3)) 
    begin3 = time.time()

    return input_img, orig_img, orig_im_size

def write(x, y, img,classes):
    c1 = tuple(x[0:2].int())
    c2 = tuple(x[2:4].int())
    cls = int(x[-1])
    cls_conf = float(x[-2])
    label = "{}:{:.4f}".format(classes[cls],cls_conf)
    dist = "dist:{:1.2f}m".format(float(y))
    # 
    color = (0,0,255)
    cv2.rectangle(img, c1, c2,color, 2)
    t_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_PLAIN, 1 , 1)[0]
    c1 = c1[0] , c1[1] 
    c2 = c1[0] + t_size[0] + 3 , c1[1] + (t_size[1]  + 5)*2

    cv2.rectangle(img, c1, c2,color, -1)
    cv2.putText(img, label, (c1[0], c1[1] + t_size[1] + 4), cv2.FONT_HERSHEY_PLAIN, 1, [225,255,255], 1)
    cv2.putText(img, dist, (c1[0], c2[1] + t_size[1] + 4), cv2.FONT_HERSHEY_PLAIN, 1, [255,255,255], 1)
    return img

def compute_dist(boxes):
    boxes_h = boxes[:,3] - boxes[:,1] 
    dist = np.ones((boxes.size(0),1))
    for box_index in range(boxes_h.size(0)):
        if boxes_h[box_index] >=528:
            alpha = 5.2
        elif boxes_h[box_index] >=279 and boxes_h[box_index] <528:
            alpha = 5.1
        elif boxes_h[box_index] >=0 and boxes_h[box_index] <279:
            alpha = 5.0
        else :
            alpha = 0
            print(" boxes_h is negative")
        dist[box_index] = 16800 / (boxes_h[box_index] * alpha)
    return dist


def yolo_run(controlInfo):
    opt = parser.parse_args()
    confidence = opt.conf_thres
    nms_thres = opt.nms_thres
    start = 0

    classes = load_classes(opt.class_path)
    print('classes',type(classes))

    print ('\nPerforming object detection:')
    prev_time = time.time()

    # Set up model
    print('Loading network........')
    model = Darknet(opt.config_path, img_size=opt.img_size)
    if opt.weights_path:
        if opt.weights_path.endswith(".pth"):
            model.load_state_dict(torch.load(opt.weights_path))
        else:
            model.load_darknet_weights(opt.weights_path)

    cuda = torch.cuda.is_available() and opt.use_cuda
    Tensor = torch.cuda.FloatTensor if cuda else torch.FloatTensor
    if cuda:
        model.cuda()
    # Set in evaluation mode
    model.eval() 
    print('Network successfully loading')

    img_size = opt.img_size

    video_path = opt.video_path

    # cap = cv2.VideoCapture(video_path)
    openCamera=False
    while not openCamera:
        try:
            cap = cv2.VideoCapture(0)
            cap.set(3,1280)
            cap.set(4,720)
            cap.set(5, 8)
            if cap.isOpened:
                print('success open capture')
                openCamera=True
        except:
            time.sleep(1)
            print('try to open the capture')

    #assert cap.isOpened(), 'Cannot capture source'
    
    frames = 0
    mgj = 0
    last_output = None
    #
    apex_danger_list = np.array([[[175, 623], [1034, 591], [1175, 720],[90, 720]]])
    apex_warnning_list = np.array([[[363,404],[822,386], [1034, 591],[175, 623]]])
    apex_ok_list = np.array([[[425,330],[760,322], [822, 386],[363, 404]]])
    apex_list = np.concatenate((apex_danger_list,apex_warnning_list,apex_ok_list),axis=0)

    frames_obstacle_sign = [False,False,False]
    stop_value = 0
    server_init_mem()

    while cap.isOpened():
        #print('FPS is: ', cap.get(5))
        start = time.time()
        frame, img = cap.read()
        # img_list = os.listdir(r'D:\dataset\AutoDrive\distance_img')
        mgj += 1
        #print("Read img time: {:4.2f}".format(time.time()-start))
        # cv2.imshow('frame',img)
        # print(img.shape)
        # key = cv2.waitKey(1)
        # if key & 0xFF == ord('q'):
        #     break

        start2 = time.time()
        if frame : 
            input_img, orig_img,orig_im_size = prep_image(img,img_size)

            #print("Prep img time: {:4.2f}".format(time.time()-start2))
            start3 = time.time()

            orig_img_dim = torch.FloatTensor(orig_im_size)
            input_img = Variable(input_img.type(Tensor))

            with torch.no_grad():
                output = model(input_img)
                output = non_max_suppression(output,confidence,nms_thres)[0]

            #print("Output detection result time: {:4.2f}".format(time.time()-start3))
            start4 = time.time()

            if output is None:
                frames += 1
                stop_value = 0
                #print("This is the {} frame ,FPS of the video is {:5.2f}".format( frames,1 / (time.time() - start)))
                # 画出区域，并写上当前信号
                orig_img = draw_area(orig_img,apex_list,stop_value)
                #server_send(str(0))
                controlInfo.changeTargetDetectionSign(stop_value)
                cv2.imshow("frame", orig_img)
                #print('imshow time {:4.2f}'.format(time.time()-start4))
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q'):
                    break
                continue
            else:
                #  
                output = person_iou_bike(output)
        
                # 检测扰动修正
                # output -->[x1,y1,x2,y2,p,p_c,c] n*7 
                # output_d -->[x,y,w,h,0,0,1,1,1] n*12
                
                output[:,:4] = xyxy2xywh(output[:,:4])
                output_d = miss_det_compensation(output,last_output)
                last_output = output_d.clone()

                output_d[:,:4] = xywh2xyxy(output_d[:,:4]) 
                output_new = output_d[:,:7]

                output_new = output_new.cpu()
                im_dim = orig_img_dim #.repeat(output_new.size(0),1)

                # s_x,s_y
                scaling_factor_x = img_size / im_dim[0] 
                scaling_factor_y = img_size / im_dim[1] 

                
                #----no_padding -1920*1080 ------------
                output_new[:,[0,2]] /= scaling_factor_x
                output_new[:,[1,3]] /= scaling_factor_y

                for i in range(output_new.shape[0]):
                    output_new[i, [0,2]] = torch.clamp(output_new[i, [0,2]], 0.0, im_dim[0])
                    output_new[i, [1,3]] = torch.clamp(output_new[i, [1,3]], 0.0, im_dim[1])
                
                
                # Draw the distance of the object
                dist = compute_dist(output_new)

                list(map(lambda x,y: write(x, y, orig_img,classes), output_new,dist))
                
                # 修正完后，做后续的判断操作
                # 画出给定行驶安全区域
                # 判断物体是否处于区域内
                # 根据情况，发出停障信号
                # 图像输出

                # judge_object 函数
                frame_obstacle_sign = judge_object(output_new,apex_list)
                # 决策函数
                stop_value = decision_make(frame_obstacle_sign,frames_obstacle_sign)
                #print("begin send!\n")
                #server_send(str(stop_value))
                controlInfo.changeTargetDetectionSign(stop_value)
                #print("complete send!\n")
                orig_img = draw_area(orig_img,apex_list,stop_value)

                #print("dra wthe result time: {:4.2f}".format(time.time()-start4))
                start5 = time.time()
                
                # cv2.imwrite(f'output2/{frames:06d}.jpg',orig_img)
                cv2.imshow("frame", orig_img)
                #print("This is the {} frame ,FPS:{:5.2f},Time:{:4.2f}".format( frames,1 / (time.time() - start),time.time() - start))

                key = cv2.waitKey(1)
                if key & 0xFF == ord('q'):
                    break
                frames += 1
                
        





class serialInfo():
    def __init__(self):
        self.flag = 0
        self.data_length = 0
        self.velocity = 0
        self.turn_angle = 0
        self.send_turn_angle=0
        self.brake = 0
        self.gears = 0
        self.heartbeat = 0


class plannerParameter(object):
    def __init__(self):
        self.stanleyInfo = {'stanley_k': 4.0, 'offset_angle': 0, 'steering_ratio': 17}
        self.turnAngle = 0
        self.stopSign = 0
        self.velocityLevel = 10
        self.controlModel = 0
        self.serialSendFlag = True
        self.serialInfoSend = serialInfo()
        self.serialInfoSend_keyboard = serialInfo()
        self.serialInfoSend_trajectory = serialInfo()
        self.existSerialData_trajectory = False
        self.existSerialData_keyboard = False
        self.existSerialData = False
        self.serialData = []
        self.serialStr = '0'
        self.targetDetectionSign = '0'
        self.test=Value('i',10)
        self.test2=2
        self.beginTracking=False

    def getDataSend(self):
        sendData = 0
        str_data = None
        
        #print(type(self.controlModel),'////////////',self.existSerialData_boardkey)
        if self.controlModel == 1 and self.existSerialData_keyboard:
            self.serialInfoSend = self.serialInfoSend_keyboard
            self.existSerialData = True
  
        elif self.controlModel == 2 and self.existSerialData_trajectory:
            self.serialInfoSend = self.serialInfoSend_trajectory
            self.existSerialData = True
            
        elif self.controlModel == 0:
            self.existSerialData = False

        self.serialDeal()
        if len(self.serialData):

            sendData = 0xAAAF01
            sum = 0xAA + 0xAF + 0x01
            for bit in self.serialData:
                sendData = sendData << 8
                sendData = sendData | bit
                sum += bit
            sum = sum & 0xFF
            sendData = sendData << 8
            sendData = sendData | sum
            self.serialData.clear()
            self.serialSendFlag = True
            str_data = hex(sendData)
            if str_data != None and self.beginTracking:
                self.serialStr = str_data
                hex_data = bytes.fromhex(str_data[2:])
                return hex_data
            else:
                return False
        else:
            return False
                #ser.write(hex_data)

    def serialDeal(self):

        if self.serialSendFlag and self.existSerialData:
            self.serialInfoSend.send_turn_angle=self.serialInfoSend.turn_angle+ self.stanleyInfo['offset_angle']
            self.serialData.append(self.serialInfoSend.flag)
            self.serialData.append(self.serialInfoSend.data_length)
            self.serialData.append(self.serialInfoSend.gears)
            self.serialData.append(self.serialInfoSend.velocity)
            self.serialData.append(self.serialInfoSend.send_turn_angle & 0x00FF)
            self.serialData.append(self.serialInfoSend.send_turn_angle >> 8)
            self.serialData.append(self.serialInfoSend.brake)
            self.serialData.append(self.serialInfoSend.heartbeat & 0x00FF)
            self.serialData.append(self.serialInfoSend.heartbeat >> 8)

            self.serialSendFlag = False

    def setSerialSend_trajectory(self,data,flage):
        self.serialInfoSend_trajectory=data
        self.existSerialData_trajectory=flage

    def setSerialSend_boardkey(self,data,flage):
        self.serialInfoSend_keyboard=data
        self.existSerialData_keyboard=flage

    def setBeginTracking(self,flage):
        self.beginTracking=flage

    def setStanleyInfo(self,data):
        self.stanleyInfo=data

    def getStanleyInfo(self):
        return self.stanleyInfo

    def getSerialInfoSend(self):
        return self.serialInfoSend

    def getSerialDisplay(self):
        return {'velocityLevel':self.velocityLevel,'targetDetectionSign':self.targetDetectionSign,'serialStr':self.serialStr}

    def getVelocityLevel(self):
        return self.velocityLevel

    def changeControlData(self,str,val=None):
        if str=='velocityLevel_add' and self.velocityLevel<20:
            self.velocityLevel+=1
            print(self.velocityLevel)
        elif str=='velocityLevel_sub' and self.velocityLevel>0:
            self.velocityLevel-=1
        elif str=='controlModel':
            self.controlModel=val
        elif str=='stanley_k' or str=='offset_angle' or str =='steering_ratio':
            self.stanleyInfo[str]=val
            # for i in self.stanleyInfo:
            #     print(self.stanleyInfo[i],'-----')

    def getTargetDetectionSign(self):
        return self.targetDetectionSign

    def changeTargetDetectionSign(self,value):
        if value==0:
            self.targetDetectionSign='安全'
        elif value==1:
            self.targetDetectionSign='危急'


class messageDeal():
    def __init__(self):
        self.messageList = []

    def inMessage(self, msg):
        self.messageList.append(msg)

    def replaceMessage(self,msgList):
        for data in msgList:
            self.messageList.append(data)
        for data in self.messageList:
            msgList.remove(data)

    def clearMessage(self):
        self.messageList.clear()

    def messageupdate(self, myWin):
        for msg in self.messageList:
            myWin.message.moveCursor(QTextCursor.End)
            myWin.message.append(msg)
        self.messageList.clear()

class trajectoryData():
    def __init__(self):
        self.rplist = []
        self.timestamp = 0
        self.pitch = 0
        self.velocity = 0
        self.heading = 0
        self.rtk_status = 0

class gps_data():
    def __init__(self):
        self.lat = 0
        self.lon = 0
        self.x = 0
        self.y = 0
        self.heading = 0  # 航向角
        self.speed = 0
        self.status = 0  # 导航状态
        self.roll = 0  # 横滚角
        self.roadtype = 0  # 道路属性
        self.lanetype = 0  # 当前车道|总车道(4|4)
        self.pitch = 0  # 俯仰角
        self.velocity = 0
        self.staenum = 0  # 卫星个数
        self.gpstime = 0  # gps时间
        self.isvalid = 0  # 有效位
        self.timestamp = 0  # 时间戳

        self.status_timer = 0
        self.status_delay1 = 10  # 从状态好到状态不好的延迟
        self.status_delay2 = 3  # 从状态不好到状态好的延迟
        self.rtk_status_dic = {"good": 4, "normal": 3, "good-bad": 2, "bad": 1, "bad-good": 5, "normal-bad": 6}
        self.rtk_status = 4  # 记录上一次的RTK状态
        self.gps_dic = {4: 'good', 3: 'normal', 1: 'bad'}
        self.gps_status='none'
        self.rtk_dic = {"int": 4, "float": 3, "bad": 1}
        self.rp_status = 4

    def getRTKStatus(self):
        if self.status == self.rtk_dic["int"]:  # 状态很好
            self.rtk_status = self.rtk_status_dic["good"]
        elif self.status == self.rtk_dic["float"]:  # 状态转一般
            self.rtk_status = self.rtk_status_dic["normal"]
        elif self.status == self.rtk_dic["bad"]:  # 状态变坏
            if self.rtk_status == self.rtk_status_dic["good"]:
                self.rtk_status = self.rtk_status_dic["good-bad"]
                self.status_timer = time.time()  # 记录时间点
            elif self.rtk_status == self.rtk_status_dic["normal"]:
                self.rtk_status = self.rtk_status_dic["normal-bad"]
                self.status_timer = time.time()
            elif self.rtk_status == self.rtk_status_dic["good-bad"]:
                if time.time() - self.status_timer > self.status_delay1:
                    self.rtk_status = self.rtk_status_dic["bad"]
            elif self.rtk_status == self.rtk_status_dic["normal-bad"]:
                if time.time() - self.status_timer > self.status_delay1:
                    self.rtk_status = self.rtk_status_dic["bad"]
        if self.rtk_status == self.rtk_status_dic["bad"]:
            self.rp_status = 1
        elif self.rtk_status == self.rtk_status_dic["good"]:
            self.rp_status = 4
        elif self.rtk_status == self.rtk_status_dic["normal"]:
            self.rp_status = 3
        elif self.rtk_status == self.rtk_status_dic["good-bad"]:
            self.rp_status = 4
        elif self.rtk_status == self.rtk_status_dic["normal-bad"]:
            self.rp_status = 3


    def getUTM(self):
        self.x, self.y = CoordConverter.LatLonToUTMXY(self.lat, self.lon)
        self.gps_status=self.gps_dic[self.rtk_status]

    def dealGPS(self,GPFPD_split_list):
        try:
            self.gpstime = int(float(GPFPD_split_list[2]))
            self.heading = float(GPFPD_split_list[3])
            self.pitch = float(GPFPD_split_list[4])
            self.roll = float(GPFPD_split_list[5])
            self.lat = float(GPFPD_split_list[6])
            self.lon = float(GPFPD_split_list[7])
            ve = float(GPFPD_split_list[9])
            vs = float(GPFPD_split_list[10])
            NSV1 = int(GPFPD_split_list[13])
            NSV2 = int(GPFPD_split_list[14])
            status_str_list = GPFPD_split_list[15].split('*')
        except:
            return False

        self.roadtype = 1
        self.lanetype = 1

        self.velocity = np.sqrt(ve ** 2 + vs ** 2)

        status = status_str_list[0]
        if status == '0B' or status == '4B':
            self.status = 'good'
        elif status == '05' or status == '45':
            self.status = 'normal'
        else:
            self.status = 'bad'

        self.satenum = NSV2

        self.isvalid = 1
        # TODO
        self.timestamp = int(time.time() * 1000)
        # gpsData.getRTKStatus()
        self.getUTM()

class rp_data():
    def __init__(self):
        self.num = 0
        self.x = 0
        self.y = 0
        self.heading = 0
        self.distance = 0
        self.curvature = 0
        self.speed_mode = 0
        self.obs_strategy = 0
        self.follow_strategy = 0
        self.special_mode = 0
        self.obs_search_strategy = 0
        self.cross_option = 0
        self.reserved_option = 0


class MyWindow(QMainWindow, Ui_MainWindow, QWidget):
    def __init__(self, parent=None):
        super(MyWindow, self).__init__(parent)
        self.setupUi(self)
        self.retranslateUi(self)
        self.bg = None
        self.controlInfo=None
        self.allFlage=None

    def inVariable(self,controlInfo,allFlage):
        self.controlInfo=controlInfo
        self.allFlage=allFlage

    def initUI(self, q_trajectoryFileName, messageText, myWin, foundTrajectoryFlage, controlInfo,messagePrintText,GPSDataStr):
        print('input initUI')
        self.setWindowTitle('automatic drive')
        self.setWindowIcon(QIcon('car2.ico'))
        self.improGUI()
        self.savePlannerInfoButton.clicked.connect(lambda :self.saveControlInfo(messagePrintText))
        self.openTrajectoryButton.clicked.connect(lambda: self.openTrajectory(q_trajectoryFileName, messagePrintText))
        self.openPlannerInfoButton.clicked.connect(lambda :self.openControlInfo(messagePrintText))
        self.speedUpButton.clicked.connect(self.changeVelocity)
        self.speedDownButton.clicked.connect(self.changeVelocity)
        self.plannerStanleyK_text.valueChanged.connect(self.changeControlInfo_StanleyK)
        self.plannerOffsetAngle_text.valueChanged.connect(self.changeControlInfo_OffsetAngle)
        self.plannerSteeringRatio_text.valueChanged.connect(self.changeControlInfo_SteeringRatio)
        self.traBeginFollowTrajectoryButton.clicked.connect(self.beginOrPause)
        self.traPauseFollowTrajectoryButton.clicked.connect(self.beginOrPause)
        self.bg.buttonClicked.connect(lambda :self.switchControlMoudel(messagePrintText))
        self.t_update = update()
        self.t_update.inVaribale(messageText, myWin, foundTrajectoryFlage, controlInfo,messagePrintText,GPSDataStr)
        self.t_update.start()
        # self.t_gpsReceive=gpsReceive()
        # self.t_gpsReceive.start()
        # self.t_plannerThread=plannerThread()
        # self.t_plannerThread.start()
        #self.t_serialThread = serialThread()
        #self.t_serialThread.start()
        # self.t_keyboardPlannerThread=keyboardPlannerThread()
        # self.t_keyboardPlannerThread.start()
        # self.t_keyboardThread=keyboardThread()
        # self.t_keyboardThread.start()
        # self.t_foundStartThread=foundStartThread()
        # self.t_foundStartThread.start()

        self.show()

        print('output initUI')

    def beginOrPause(self):
        sender = self.sender()
        if sender.text() == '开始':
            self.controlInfo.setBeginTracking(True)
        elif sender.text() == '暂停':
            self.controlInfo.setBeginTracking(False)

    def switchControlMoudel(self,messagePrintText):
        sender = self.sender()
        if sender == self.bg:
            self.controlInfo.changeControlData('controlModel',self.bg.checkedId())
            if self.bg.checkedId() == 1:
                messagePrintText.append('Control Mode : Keyboard')
            elif self.bg.checkedId() == 2:
                messagePrintText.append('Control Mode : Trajectory')

    def improGUI(self):
        self.bg = QtWidgets.QButtonGroup(self)
        self.bg.addButton(self.plannerMoudelSelect_keyboard, 1)
        self.bg.addButton(self.plannerMoudelSelect_trajectory, 2)
        self.traBeginFollowTrajectoryButton.setCheckable(True)
        self.traPauseFollowTrajectoryButton.setCheckable(True)
        self.traBeginFollowTrajectoryButton.setAutoExclusive(True)
        self.traPauseFollowTrajectoryButton.setAutoExclusive(True)
        scrollbar = self.message.verticalScrollBar()
        scrollbar.setSliderPosition(scrollbar.maximum())

    def closeEvent(self, event):
        reply = QMessageBox.question(self, '消息', '是否退出？', QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.allFlage['mainQuit']=True
            quitTime=time.time()
            while not allFlage['subprogramAlreadyQuit']:
                time.sleep(0.1)
                if time.time()-quitTime>2:
                    break
            event.accept()

        else:
            event.ignore()

    def openTrajectory(self, q_trajectoryFileName, messagePrintText):
        trajectoryFileName = QFileDialog.getOpenFileName(self, '打开文件', '/', '*.csv')
        messagePrintText.append('File path : ' + trajectoryFileName[0])
        q_trajectoryFileName.put(trajectoryFileName[0], timeout=1)
        try:
            trajectory_test = csv.reader(trajectoryFileName[0])
            messagePrintText.append('Trajectory loaded successfully')
        except:
            QMessageBox.critical(None, '错误', '打开失败！')
            messagePrintText.append('Trajectory loading failed')

    def openControlInfo(self,messagePrintText):
        openFileName = QFileDialog.getOpenFileName(self, '打开配置', '/', '*.pki')
        try:
            openInfo = open(openFileName[0], 'rb')
            self.controlInfo.setStanleyInfo(pickle.load(openInfo))
            openInfo.close()
            stanleyInfo=self.controlInfo.getStanleyInfo()
            self.plannerStanleyK_text.setProperty("value", stanleyInfo['stanley_k'])
            self.plannerSteeringRatio_text.setProperty("value", stanleyInfo['steering_ratio'])
            self.plannerOffsetAngle_text.setProperty("value", stanleyInfo['offset_angle'])
            messagePrintText.append('Load configuration successfully')
        except:
            QMessageBox.critical(None, '错误', '打开失败！')
            messagePrintText.append('Load configuration failed')

    def saveControlInfo(self,messagePrintText):
        saveFileName = QFileDialog.getSaveFileName(self, '保存配置', 'controlInfo.pki')
        try:
            saveInfo = open(saveFileName[0], 'wb')
            pickle.dump(self.controlInfo.getStanleyInfo(), saveInfo)
            saveInfo.close()
            messagePrintText.append('Save configuration successfully')
        except:
            QMessageBox.critical(None, '错误', '保存失败！')
            messagePrintText.append('Save configuration failed')

    def changeVelocity(self):
        sender = self.sender()
        if sender.text() == "加速" :
            self.controlInfo.changeControlData('velocityLevel_add')
        elif sender.text() == "减速" :
            self.controlInfo.changeControlData('velocityLevel_sub')

    def changeControlInfo_StanleyK(self, value):
        val = round(value, 1)
        self.controlInfo.changeControlData('stanley_k',val)
        print(val)

    def changeControlInfo_OffsetAngle(self, value):
        val = round(value, 1)
        self.controlInfo.changeControlData('offset_angle', val)
        print(val)

    def changeControlInfo_SteeringRatio(self, value):
        val = round(value, 1)
        self.controlInfo.changeControlData('steering_ratio', val)
        print(val)



####################################################串口数据接收
# 创建串口对象

class gpsReceive():
    def __init__(self):
        pass

    def run(self, messagePrintText,GPSDataStr):
        gpsData = gps_data()
        openSerial = False
        serialMsg = ''
        serialMsg_l = ''
        # plist = list(serial.tools.list_ports.comports())
        # if plist:  # 如果串口列表不为
        #   ser = se.Serial(se_name, 115200, timeout=1, parity=se.PARITY_NONE)  # 打开串口,指定设备名称，波特率，超时，奇偶校验
        while not openSerial:
            try:
                ser = serial.Serial("/dev/ttyUSB1", 115200, timeout=1,
                                    parity=serial.PARITY_NONE)  # 打开串口,指定设备名称，波特率，超时，奇偶校验
                openSerial = True
                serialMsg = 'Successfully open the GPS serial port'
            except:
                serialMsg = 'Failed to open GPS serial port'
            if serialMsg != serialMsg_l:
                messagePrintText.append(serialMsg)
                serialMsg_l = serialMsg
            time.sleep(0.5)
            print('try to open gps port')

        messagePrintText.append('Thread:Start receiving GPS data')

        while True and openSerial:
            # data=str(ser.read(99))[2:]
            GPFPD_split_list = ['none']
            try:
                data = str(ser.readline())[2:]
                GPFPD_split_list = data.split(',')
                # if data_list[0]=='$GPFPD':
                #     print('ooooo')
                #print(data)
            except:
                pass
            #ser.flushInput()
            # print(len(GPFPD_split_list))
            # print('==================================',GPFPD_split_list[0])
            if GPFPD_split_list[0] == "$GPFPD":
                if len(GPFPD_split_list) == 16:  # 收到完整GPFPD帧
                    # print('************************')
                    #print(data)
                    if len(GPSDataStr):
                        GPSDataStr[0]=GPFPD_split_list.copy()
                    else:
                        GPSDataStr.append(GPFPD_split_list.copy())
                    
                    # globalsFlags.gpsData=True


###########################################Navigation

# 发送轨迹程序，接收收到的起始点信息，在全局轨迹中截取要发送到轨迹并转化为局部坐标发送

class foundStart():
    def __init__(self):
        self.SEARCH_WIDTH = 20
        self.END_ID = 0
        self.global_rp_list_utm = []
        self.current_id = 0
        self.trajectoryFileName = None

    def readMap(self):
        # global globalsFlags
        #
        # if globalsFlags.openTrajectoryFileNameFlag:
        csv_file = open(self.trajectoryFileName, 'r')  # 打开地图文件
        reader = csv.reader(csv_file)
        i = 0
        # 遍历地图文件左右行，提取路点信息存入地图对象global_rp_list_utm中
        for item in reader:
            if i:
                rpData = rp_data()
                rpData.num = int(item[0])
                rpData.x, rpData.y = CoordConverter.LatLonToUTMXY(float(item[1]), float(item[2]))
                rpData.heading = float(item[3])
                rpData.distance = float(item[4])
                rpData.curvaturev = float(item[5])
                rpData.speed_mode = int(item[6])
                rpData.obs_strategy = int(item[7])
                rpData.follow_strategy = int(item[8])
                rpData.special_mode = int(item[9])
                rpData.obs_search_strategy = int(item[10])
                rpData.cross_option = int(item[11])
                rpData.reserved_option = int(item[13])
                self.global_rp_list_utm.append(rpData)
            i += 1

        self.END_ID = i - 2

    def pub_trajectory(self, q_trajectory, start_id, rp1):
        trajectory = trajectoryData()  # 要发送的轨迹
        trajectory.rplist = []
        x0, y0, theta0 = rp1.x, rp1.y, rp1.heading  # 当前车体坐标作为坐标原点
        trajectory.pitch = rp1.pitch  # 俯仰角
        trajectory.velocity = rp1.velocity  # 速率
        trajectory.heading = rp1.heading  # 航向角
        trajectory.rtk_status = rp1.status  # 导航状态
        end_id = start_id + 400
        if end_id > self.END_ID:
            end_id = self.END_ID
        # 计算距离
        last_distance = CoordConverter.get_distance_UTM(x0, y0, self.global_rp_list_utm[start_id].x,
                                                        self.global_rp_list_utm[start_id].y)
        for i in range(start_id, end_id):  # 搜索符合条件发送的点，近处密集，远处稀疏
            rp = copy.copy(self.global_rp_list_utm[i])
            if i == start_id:  # 如果是第一个点，直接计算距离
                rp.distance = last_distance
                rp.x, rp.y, rp.heading = CoordConverter.get_relative_coord(x0, y0, theta0, rp.x, rp.y, rp.heading)
                trajectory.rplist.append(rp)  # 添加到发送点列
            else:  # 如果不是第一个点，则累加距离，每个点的距离表示到车体原点的距离
                rp.distance = rp.distance + last_distance  # 计算车体的距离
                last_distance = rp.distance  # 累加距离
                if rp.distance < 15:  # 如果距离车小于15米, 则发送每一个点
                    rp.x, rp.y, rp.heading = CoordConverter.get_relative_coord(x0, y0, theta0, rp.x, rp.y, rp.heading)
                    if rp.heading < -90 or rp.heading > 90:  # 如果转角过大，则截止，不发送回勾点
                        break
                    else:  # 转角不大
                        trajectory.rplist.append(rp)
                elif rp.distance < 80 and i % 5 == 0:  # 如果距离超过15米，且不超过80m，则每5个点取一个发送
                    rp.x, rp.y, rp.heading = CoordConverter.get_relative_coord(x0, y0, theta0, rp.x, rp.y,
                                                                               rp.heading)
                    if rp.heading < -90 or rp.heading > 90:  # 如果转角过大，则截止，不发送回勾点
                        break
                    else:  # 转角不大
                        trajectory.rplist.append(rp)
                else:  # 超过80m
                    break
        trajectory.timestamp = int(time.time() * 1000)  # 加上时间戳
        #print(len(trajectory.rplist),'+++++++')
        try:
            q_trajectory.put(trajectory)
        except:
            pass

    def run(self,q_trajectory, q_trajectoryFileName, messagePrintText, foundTrajectoryFlage,GPSDataStr):
        existMap = False
        current_id = 0
        messagePrintText.append('Thread:Start looking for a starting point')
        gpsData=gps_data()
        while True:
            # TODO while not rospy.is_shutdown():
            if len(GPSDataStr):
                gpsData.dealGPS(GPSDataStr[0])
            try:
                # print(self.trajectoryFileName)
                self.trajectoryFileName = q_trajectoryFileName.get_nowait()
                print(self.trajectoryFileName)
                timeMap = time.time()
                messagePrintText.append('Start loading the map')
                print('Start loading the map')
                self.readMap()  # 读取地图数据
                messagePrintText.append('Map loaded')
                print('Map loaded')
                messagePrintText.append('Time:' + str(int(time.time() - timeMap)) + 's')
                print('Time:' + str(int(time.time() - timeMap)) + 's')
                print(len(self.global_rp_list_utm))
                existMap = True
            except:
                pass
            if existMap:
                x0, y0, theta0 = gpsData.x, gpsData.y, gpsData.heading
                found_flag = False
                start_id = current_id - 5  # 小于5个点开始搜索
                if start_id < 0:
                    start_id = 0
                while not found_flag:
                    x1, y1, theta1 = self.global_rp_list_utm[start_id].x, self.global_rp_list_utm[start_id].y, \
                                     self.global_rp_list_utm[start_id].heading  # 得到全局UTM坐标
                    x_local, y_local, azimuth_local = CoordConverter.get_relative_coord(x0, y0, theta0, x1, y1,
                                                                                        theta1)  # 得到当前所搜索点相对车体的坐标
                    # print('x1:',x1,'y1:',y1,'theta1:',theta1,'x0:',x0,'y0:',y0,'theta0:',theta0)

                    if 20 > x_local > 0 and abs(y_local) < self.SEARCH_WIDTH and abs(
                            azimuth_local) < 50:  # 当前点在正前方,x偏移20，y偏移正负20，范围偏移正负50
                        current_id = start_id  # 更新current_id,下次从current_id开始搜索
                        found_flag = True  # 找到起始点
                    else:
                        start_id += 1  # 不符合要求则继续搜索下一个点
                        if start_id >= self.END_ID:  # 如果已经搜索到最后
                            current_id = 0  # 复位current_id,跳出循环,下次从头搜索
                            start_id = 0
                    foundTrajectoryFlage[0] = found_flag
                if found_flag:
                    self.pub_trajectory(q_trajectory, current_id, gpsData)

###########################################planner

class plannerControl():
    def __init__(self):
        self.targetDetectionSign = ''
        self.serialSend = serialInfo()

    def stanley_method(self, theta_e, efa, v, k, offset_angle, steering_ratio):
        if v < 3.5:  # 如果速度小于3.5m/s
            v = 3.5
        return int((theta_e + np.arctan(efa / (k * v)) * 180 / np.pi) * steering_ratio)

    def Stop_call(self, stop):
        print(type(stop.data))
        print(stop.data)
        try:
            value = stop.data.split(' ')[-1]
            self.targetDetectionSign = value
        except:
            pass

    def decisionMaking(self):
        if self.targetDetectionSign == '危险':  # 减速
            self.serialSend.velocity = int(self.serialSend.velocity / 4)

        elif self.targetDetectionSign == '危急':  # 急刹
            self.serialSend.velocity = 0x00
            self.serialSend.brake = 0x02
            self.serialSend.gears = 0x00

        elif self.targetDetectionSign == '警告':  # 降速
            self.serialSend.velocity = int(self.serialSend.velocity / 2)

    def run(self, q_trajectory, controlInfo, messagePrintText):
        # d30_serial_pub = rospy.Publisher('d30_serial', serial_send, queue_size=5)
        # TODO rate = rospy.Rate(10)
        #print(controlInfo.getTest(),'---')
        turnangle = 0
        set_speed = 0
        heartbeat = 0x0000
        messagePrintText.append('Thread:Start the tracking process')
        trajectory = q_trajectory.get()

        while True:
            # TODO while not rospy.is_shutdown():
            try:
                trajectory = q_trajectory.get_nowait()
                #print(len(trajectory.rplist),'*********')
            except:
                pass
            #print(len(trajectory.rplist))
            time_e = int(time.time() * 1000) - trajectory.timestamp
            #print(time_e)
            if time_e > 1000:  # 如果GPS数据超时1s，认为数据过期，清除gps数据点列
                trajectory.rplist = []
                #print('clear')
            if len(trajectory.rplist):  # 如果存在导航轨迹
                temp_rplist = copy.deepcopy(trajectory.rplist)  # 获取当前轨迹
                if temp_rplist[-1].distance > 3:  # 如果估计长度大于3m
                    headpoint_index = 0
                    distance = 0
                    aim_distance = 0
                    if trajectory.velocity > 3:
                        aim_distance = 3
                    else:
                        aim_distance = 2.5
                    while distance < aim_distance:  # 相邻点之间间隔2.5m
                        headpoint_index += 1
                        distance = temp_rplist[headpoint_index].distance
                    efa, theta_e = -temp_rplist[headpoint_index].y, temp_rplist[headpoint_index].heading
                    stanleyInfo=controlInfo.getStanleyInfo()
                    turnangle = self.stanley_method(theta_e, efa, trajectory.velocity,
                                                    stanleyInfo['stanley_k'],
                                                    stanleyInfo['offset_angle'],
                                                    stanleyInfo['steering_ratio'])
                    if turnangle > 400:
                        turnangle = 400
                    elif turnangle < -400:
                        turnangle = -400
                    set_speed = controlInfo.getVelocityLevel()
            else:  # 如果没有导航轨迹
                turnangle = 0
                set_speed = 0

            self.serialSend.flag = 0xFF
            self.serialSend.data_length = 0x07
            self.serialSend.velocity = set_speed
            self.serialSend.turn_angle = turnangle + 500
            if (set_speed == 0):
                self.serialSend.brake = 0x02
                self.serialSend.gears = 0x00
            else:
                self.serialSend.gears = 0x01
                self.serialSend.brake = 0x00
            self.serialSend.heartbeat = heartbeat
            self.serialSend.all = 0x00
            self.serialSend.heartbeat += 1
            if self.serialSend.heartbeat > 0xFFFF:
                self.serialSend.heartbeat = 0x0000
            self.targetDetectionSign=controlInfo.getTargetDetectionSign
            self.decisionMaking()
            #print(self.targetDetectionSign,time.time())
            #controlInfo.changeTargetDetectionSign(self.targetDetectionSign)
            #print(self.serialSend.velocity)
            controlInfo.setSerialSend_trajectory(self.serialSend,True)
            # controlInfo.serialInfoSend_trajectory = self.serialSend
            # controlInfo.existSerialData_trajectory = True

            time.sleep(0.01)
            # TODO rate.sleep()


############################################键盘控制

class keyboardControl():
    def __init__(self):
        self.serialSend = serialInfo()

    def run(self, keysPressed, keysReleased, controlInfo,messagePrintText):
        messagePrintText.append('Thread:Start the keyboard control process')
        heartbeat = 0x0000
        while True:
            # TODO while not rospy.is_shutdown():
            if 'w' in keysPressed or 's' in keysPressed:
                velocity = controlInfo.getVelocityLevel()
            else:
                velocity = 0

            if 'w' in keysPressed and 's' not in keysPressed and 'stop' not in keysPressed:
                gears = 0x01
            elif 's' in keysPressed and 'w' not in keysPressed and 'stop' not in keysPressed:
                gears = 0x02
            else:
                gears = 0x00

            if 'a' in keysPressed and 'd' not in keysPressed:
                turn_angle = -150
            elif 'd' in keysPressed and 'a' not in keysPressed:
                turn_angle = 150
            else:
                turn_angle = 0

            if 'stop' in keysPressed:
                gears = 0x00
                brake = 0x02
            else:
                brake = 0x00

            self.serialSend.velocity = velocity
            self.serialSend.gears = gears
            self.serialSend.turn_angle = turn_angle + 500
            self.serialSend.brake = brake
            self.serialSend.flag = 0xFF
            self.serialSend.data_length = 0x07
            self.serialSend.heartbeat = heartbeat
            heartbeat += 1
            if heartbeat >= 0xFFFF:
                heartbeat = 0x0000
            # controlInfo.serialInfoSend_keyboard = self.serialSend
            # controlInfo.existSerialData_keyboard = True
            # print(controlInfo.existSerialData_keyboard, '***')
            controlInfo.setSerialSend_boardkey(self.serialSend,True)
            time.sleep(0.01)


###########################################键盘信息读取
class keyboardMonitor():
    def __init__(self):
        self.keysPressed=None
        self.keysReleased=None

    def on_press(self, key):
        global messageText
        try:
            if key.char not in self.keysPressed:
                # messageText.inMessage('{0} pressed'.format(key.char))
                try:
                    self.keysPressed.append(key.char)
                    self.keysReleased.remove(key.char)
                except:
                    pass

        except AttributeError:
            # messageText.inMessage('special key {0} pressed'.format(key))
            pass

        if key == keyboard.Key.space and 'stop' not in self.keysPressed:
            # messageText.inMessage('{0} pressed'.format(key))
            try:
                self.keysPressed.append('stop')
                self.keysReleased.remove('stop')
            except:
                pass

    def on_release(self, key):
        global messageText
        try:
            if key.char not in self.keysReleased:
                # messageText.inMessage('{0} released'.format(key))
                try:
                    self.keysReleased.append(key.char)
                    self.keysPressed.remove(key.char)
                except:
                    pass

        except AttributeError:
            # messageText.inMessage('special key {0} release'.format(key))
            pass

        if key == keyboard.Key.space and 'stop' not in self.keysReleased:
            # messageText.inMessage('{0} released'.format(key))
            try:
                self.keysReleased.append('stop')
                self.keysPressed.remove('stop')
            except:
                pass

        if key == keyboard.Key.esc:
            return False

    def run(self,keysPressed,keysReleased,messagePrintText):
        global messageText
        self.keysPressed=keysPressed
        self.keysReleased=keysReleased
        messagePrintText.append('Thread:Start the keyboard monitoring process')
        while True:
            # TODO while not rospy.is_shutdown():
            with keyboard.Listener(on_press=self.on_press, on_release=self.on_release, suppress=False) as listener:
                listener.join()


###########################################串口发送

class serialSend():
    def __init__(self):
        self.planner_model = []

        self.data_keyboard = []
        self.data_keyboard_flag = True

        self.data_planner = []
        self.data_planner_flag = True

    def run(self,controlInfo,messagePrintText):
        serialText = ''
        serialText_l = ''
        openSerial = False
        while not openSerial:
            try:
                ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
                # ser = serial.Serial("com4", 115200, timeout=1)
                serialText = 'Open serial port successfully'
                openSerial = True
            except:
                serialText = 'Open serial port failed'
            if serialText != serialText_l:
                messagePrintText.append(serialText)
                serialText_l = serialText
            time.sleep(0.5)
        messagePrintText.append('Thread:Start serial port sending process')
        while True:
            # TODO while not rospy.is_shutdown():
            hex_data=controlInfo.getDataSend()
            if hex_data:
                ser.write(hex_data)



###########################################面板更新

class update(QThread):
    def __init__(self):
        super().__init__()
        self.messageText = None
        self.myWin = None
        self.foundTrajectoryFlage = None
        self.controlInfoList = None
        self.messagePrintText=None
        self.GPSDataStr=None
        self.yoloSign=None

    def inVaribale(self, messageText, myWin, foundTrajectoryFlage, controlInfo,messagePrintText,GPSDataStr):
        self.messageText = messageText
        self.myWin = myWin
        self.foundTrajectoryFlage = foundTrajectoryFlage
        self.controlInfo = controlInfo
        self.messagePrintText=messagePrintText
        self.GPSDataStr=GPSDataStr

    def run(self):
        gpsData = gps_data()
        print('input update',os.getpid())
        while True:
            try:
                gpsData.dealGPS(self.GPSDataStr[0])
            except:
                pass
            self.myWin.GPS_heading_text.setText(str(gpsData.heading))
            self.myWin.GPS_lat_text.setText(str(gpsData.lat))
            self.myWin.GPS_lon_text.setText(str(gpsData.lon))
            self.myWin.GPS_status_text.setText(str(gpsData.status))
            self.myWin.GPS_sateNum_text.setText(str(gpsData.staenum))
            self.myWin.GPS_velocity_text.setText(str(gpsData.velocity))
            self.myWin.GPS_time_text.setText(str(gpsData.gpstime))
            self.myWin.traRTKStatus_text.setText(str(gpsData.rtk_status))

            try:
                if self.foundTrajectoryFlage[0]:
                    self.myWin.traFollowStatus_text.setText('found')
                else:
                    self.myWin.traFollowStatus_text.setText('not found')
                serialInfoSend=self.controlInfo.getSerialInfoSend()
                dictDisplaySerial=self.controlInfo.getSerialDisplay()
                
            except:
                pass

            self.myWin.carStatusVelocity_text.setText(str(serialInfoSend.velocity))
            self.myWin.carStatusTurnAngle_text.setText(str(serialInfoSend.send_turn_angle - 500))
            self.myWin.carStatusGears_text.setText(str(serialInfoSend.gears))
            self.myWin.carStatusBrake_text.setText(str(serialInfoSend.brake))
            self.myWin.carStatusVelocityLevel_text.setText(str(dictDisplaySerial['velocityLevel']))
            self.myWin.carStatusTargetDetection_text.setText(dictDisplaySerial['targetDetectionSign'])
            self.myWin.carStatusSerialData_text.setText(dictDisplaySerial['serialStr'])

            try:
                self.messageText.replaceMessage(self.messagePrintText)
                self.messageText.messageupdate(self.myWin)
            except:
                pass

def gpsReceive_process(  messagePrintText,GPSDataStr):
    print('input gpsReceive_process:',os.getpid())
    mp_gpsReceive = gpsReceive()
    mp_gpsReceive.run(  messagePrintText,GPSDataStr)

def foundStart_process( q_trajectory, q_trajectoryFileName, messagePrintText, foundTrajectoryFlage,GPSDataStr):
    print('input foundStart',os.getpid())
    mp_foundStart = foundStart()
    mp_foundStart.run( q_trajectory, q_trajectoryFileName, messagePrintText, foundTrajectoryFlage,GPSDataStr)

def plannerControl_process(q_trajectory, controlInfo, messagePrintText):
    print('input plannerControl',os.getpid())
    mp_plannerControl = plannerControl()
    mp_plannerControl.run(q_trajectory, controlInfo, messagePrintText)

def keyboardControl_process( keysPressed, keysReleased, controlInfo,messagePrintText):
    print('input keyboardControl',os.getpid())
    mp_keyboardControl = keyboardControl()
    mp_keyboardControl.run(keysPressed, keysReleased, controlInfo,messagePrintText)

def keyboardMoniter_process(keysPressed, keysReleased,messagePrintText):
    print('input keyboard',os.getpid())
    mp_keyboardMoniter=keyboardMonitor()
    mp_keyboardMoniter.run(keysPressed, keysReleased,messagePrintText)

def serialSend_process(controlInfo,messagePrintText):
    print('input serialSend',os.getpid())
    mp_serialSend=serialSend()
    mp_serialSend.run(controlInfo,messagePrintText)
    
def yolo_process(controlInfo,allFlage):
    print('input yolo',os.getpid())
    if allFlage['startYolo']:
        yolo_run(controlInfo)

def main_process(q_trajectory,q_trajectoryFileName,controlInfo,GPSDataStr,foundTrajectoryFlage,messagePrintText,allFlage):
    print('main PID:',os.getpid())
    mgrKeysPressed=Manager()
    keysPressed = mgrKeysPressed.list()
    mgrKeysReleased = Manager()
    keysReleased = mgrKeysReleased.list()

    proc_gpsReceive = Process(target=gpsReceive_process,args=( messagePrintText,GPSDataStr),daemon=True)
    proc_foundStart = Process(target=foundStart_process,args=(q_trajectory, q_trajectoryFileName, messagePrintText, foundTrajectoryFlage,GPSDataStr),daemon=True)
    proc_plannerControl = Process(target=plannerControl_process, args=(q_trajectory, controlInfo, messagePrintText),daemon=True)
    proc_keyboardControl = Process(target=keyboardControl_process,args=(keysPressed, keysReleased, controlInfo,messagePrintText),daemon=True)
    proc_keyboardMoniter=Process(target=keyboardMoniter_process,args=(keysPressed, keysReleased,messagePrintText),daemon=True)
    proc_serialSend=Process(target=serialSend_process,args=(controlInfo,messagePrintText),daemon=True)
    proc_yolo=Process(target=yolo_process,args=(controlInfo,allFlage),daemon=True)

    proc_gpsReceive.start()
    proc_foundStart.start()
    proc_plannerControl.start()
    proc_keyboardControl.start()
    proc_keyboardMoniter.start()
    proc_serialSend.start()
    proc_yolo.start()
    allQuitFlage=False
    while True:
        if allFlage['mainQuit']:
            allQuitFlage=True

        if not (proc_gpsReceive.is_alive() and proc_serialSend.is_alive() and proc_plannerControl.is_alive() and proc_keyboardMoniter.is_alive() and proc_keyboardControl.is_alive() and proc_foundStart.is_alive() and proc_yolo.is_alive()):
            allQuitFlage=True
            print('ALL SUBPROGRAM QUIT')
            messagePrintText.append('ALL SUBPROGRAM QUIT!!!!!!!')

        if allQuitFlage:
            proc_foundStart.terminate()
            proc_gpsReceive.terminate()
            proc_keyboardControl.terminate()
            proc_keyboardMoniter.terminate()
            proc_plannerControl.terminate()
            proc_serialSend.terminate()
            proc_yolo.terminate()
            print('quit------------')
            allFlage['subprogramAlreadyQuit']=True
            break
        time.sleep(0.5)

class MyManager(BaseManager):
    pass

def myManager():
    m = MyManager()
    m.start()
    return m

if __name__ == '__main__':

    print('Main PID:',os.getpid())
    MainPID=os.getpid()
    MyManager.register('plannerParameter', plannerParameter)
    mgrPlanner = myManager()
    controlInfo = mgrPlanner.plannerParameter()
    trajectory = trajectoryData()
    messageText = messageDeal()
    manager=Manager()
    q_trajectoryFileName=manager.Queue(maxsize=5)
    q_trajectory = manager.Queue(maxsize=1)
    GPSDataStr = manager.list()
    foundTrajectoryFlage = manager.list()
    foundTrajectoryFlage.append(False)
    messagePrintText = manager.list()
    allFlage=manager.dict()
    allFlage['mainQuit']=False
    allFlage['subprogramAlreadyQuit']=False
    allFlage['startYolo']=True
    # quitFlage = manager.list()
    # quitFlage.append(False)
    # quitFlage.append(False)
    #yoloSign=manager.Value('i',0)

    proc_main=Process(target=main_process,args=(q_trajectory,q_trajectoryFileName,controlInfo,GPSDataStr,foundTrajectoryFlage,messagePrintText,allFlage))
    proc_main.start()

    app = QApplication(sys.argv)
    myWin = MyWindow()
    myWin.inVariable(controlInfo,allFlage)
    myWin.initUI(q_trajectoryFileName, messageText, myWin, foundTrajectoryFlage, controlInfo,messagePrintText, GPSDataStr)
    app.exec_()
    if proc_main.is_alive():
        proc_main.terminate()
    manager.shutdown()
    sys.exit(0)






