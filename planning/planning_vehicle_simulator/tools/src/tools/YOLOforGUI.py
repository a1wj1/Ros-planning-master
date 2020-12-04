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
parser.add_argument('--video_path', type=str, default='/home/gogo/ll/Test/2019-08-13-210638.webm',
                    help='path to dataset')
parser.add_argument('--config_path', type=str, default='config/yolov3-pedes-voc.cfg', help='path to model config file')
parser.add_argument('--weights_path', type=str, default='checkpoints/yolov3_ckpt416_43.pth',
                    help='path to weights file')
# parser.add_argument('--weights_path', type=str, default='checkpoints/yolov3_ckpt608_36.pth', help='path to weights file')
# parser.add_argument('--weights_path', type=str, default='weights/yolov3.weights', help='path to weights file')
parser.add_argument('--class_path', type=str, default='data/class_dict.names', help='path to class label file')
parser.add_argument('--conf_thres', type=float, default=0.7, help='object confidence threshold')
parser.add_argument('--nms_thres', type=float, default=0.4, help='iou thresshold for non-maximum suppression')
parser.add_argument('--batch_size', type=int, default=1, help='size of the batches')
parser.add_argument('--n_cpu', type=int, default=0, help='number of cpu threads to use during batch generation')
parser.add_argument('--img_size', type=int, default=416, help='size of each image dimension')
parser.add_argument('--use_cuda', type=bool, default=True, help='whether to use cuda if available')

# print(opt)

os.makedirs('output', exist_ok=True)


def prep_image(image, size):
    begin2 = time.time()

    orig_img = image.copy()
    orig_im_size = orig_img.shape[1], orig_img.shape[0]

    input_img = cv2.resize(image, (size, size)) / 255
    # print("Resize img time: {:4.4f}".format(time.time()-begin2))
    begin3 = time.time()

    # Channels-first
    input_img = np.transpose(input_img, (2, 0, 1))
    # As pytorch tensor
    input_img = torch.from_numpy(input_img).float().unsqueeze(0)
    # print("transform img time: {:4.4f}".format(time.time()-begin3))
    begin3 = time.time()

    return input_img, orig_img, orig_im_size


def write(x, y, img, classes):
    c1 = tuple(x[0:2].int())
    c2 = tuple(x[2:4].int())
    cls = int(x[-1])
    cls_conf = float(x[-2])
    label = "{}:{:.4f}".format(classes[cls], cls_conf)
    dist = "dist:{:1.2f}m".format(float(y))
    #
    color = (0, 0, 255)
    cv2.rectangle(img, c1, c2, color, 2)
    t_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_PLAIN, 1, 1)[0]
    c1 = c1[0], c1[1]
    c2 = c1[0] + t_size[0] + 3, c1[1] + (t_size[1] + 5) * 2

    cv2.rectangle(img, c1, c2, color, -1)
    cv2.putText(img, label, (c1[0], c1[1] + t_size[1] + 4), cv2.FONT_HERSHEY_PLAIN, 1, [225, 255, 255], 1)
    cv2.putText(img, dist, (c1[0], c2[1] + t_size[1] + 4), cv2.FONT_HERSHEY_PLAIN, 1, [255, 255, 255], 1)
    return img


def compute_dist(boxes):
    boxes_h = boxes[:, 3] - boxes[:, 1]
    dist = np.ones((boxes.size(0), 1))
    for box_index in range(boxes_h.size(0)):
        if boxes_h[box_index] >= 528:
            alpha = 5.2
        elif boxes_h[box_index] >= 279 and boxes_h[box_index] < 528:
            alpha = 5.1
        elif boxes_h[box_index] >= 0 and boxes_h[box_index] < 279:
            alpha = 5.0
        else:
            alpha = 0
            print(" boxes_h is negative")
        dist[box_index] = 16800 / (boxes_h[box_index] * alpha)
    return dist


def yolo_run(controlInfo,globalData):
    opt = parser.parse_args()
    confidence = opt.conf_thres
    nms_thres = opt.nms_thres
    start = 0

    classes = load_classes(opt.class_path)
    print('classes', type(classes))

    print('\nPerforming object detection:')
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
    openCamera = False
    while not openCamera:
        try:
            cap = cv2.VideoCapture(0)
            cap.set(3, 1280)
            cap.set(4, 720)
            cap.set(5, 8)
            if cap.isOpened:
                print('success open capture')
                openCamera = True
        except:
            time.sleep(1)
            print('try to open the capture')

    # assert cap.isOpened(), 'Cannot capture source'

    frames = 0
    mgj = 0
    last_output = None
    #
    apex_danger_list = np.array([[[175, 623], [1034, 591], [1175, 720], [90, 720]]])
    apex_warnning_list = np.array([[[363, 404], [822, 386], [1034, 591], [175, 623]]])
    apex_ok_list = np.array([[[425, 330], [760, 322], [822, 386], [363, 404]]])
    apex_list = np.concatenate((apex_danger_list, apex_warnning_list, apex_ok_list), axis=0)

    frames_obstacle_sign = [False, False, False]
    stop_value = 0
    server_init_mem()

    while cap.isOpened():
        # print('FPS is: ', cap.get(5))
        start = time.time()
        frame, img = cap.read()
        # img_list = os.listdir(r'D:\dataset\AutoDrive\distance_img')
        mgj += 1
        # print("Read img time: {:4.2f}".format(time.time()-start))
        # cv2.imshow('frame',img)
        # print(img.shape)
        # key = cv2.waitKey(1)
        # if key & 0xFF == ord('q'):
        #     break

        start2 = time.time()
        if frame:
            input_img, orig_img, orig_im_size = prep_image(img, img_size)

            # print("Prep img time: {:4.2f}".format(time.time()-start2))
            start3 = time.time()

            orig_img_dim = torch.FloatTensor(orig_im_size)
            input_img = Variable(input_img.type(Tensor))

            with torch.no_grad():
                output = model(input_img)
                output = non_max_suppression(output, confidence, nms_thres)[0]

            # print("Output detection result time: {:4.2f}".format(time.time()-start3))
            start4 = time.time()

            if output is None:
                frames += 1
                stop_value = 0
                # print("This is the {} frame ,FPS of the video is {:5.2f}".format( frames,1 / (time.time() - start)))
                # 画出区域，并写上当前信号
                orig_img = draw_area(orig_img, apex_list, stop_value)
                # server_send(str(0))
                controlInfo.changeTargetDetectionSign(stop_value)
                #cv2.imshow("frame", orig_img)
                dis_image = cv2.resize(orig_img, (701, 391))
                displayQPixMap = cv2.cvtColor(dis_image, cv2.COLOR_BGR2RGB)
                globalData.operateDisplayTargetDetection('change', displayQPixMap)
                # print('imshow time {:4.2f}'.format(time.time()-start4))
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

                output[:, :4] = xyxy2xywh(output[:, :4])
                output_d = miss_det_compensation(output, last_output)
                last_output = output_d.clone()

                output_d[:, :4] = xywh2xyxy(output_d[:, :4])
                output_new = output_d[:, :7]

                output_new = output_new.cpu()
                im_dim = orig_img_dim  # .repeat(output_new.size(0),1)

                # s_x,s_y
                scaling_factor_x = img_size / im_dim[0]
                scaling_factor_y = img_size / im_dim[1]

                # ----no_padding -1920*1080 ------------
                output_new[:, [0, 2]] /= scaling_factor_x
                output_new[:, [1, 3]] /= scaling_factor_y

                for i in range(output_new.shape[0]):
                    output_new[i, [0, 2]] = torch.clamp(output_new[i, [0, 2]], 0.0, im_dim[0])
                    output_new[i, [1, 3]] = torch.clamp(output_new[i, [1, 3]], 0.0, im_dim[1])

                # Draw the distance of the object
                dist = compute_dist(output_new)

                list(map(lambda x, y: write(x, y, orig_img, classes), output_new, dist))

                # 修正完后，做后续的判断操作
                # 画出给定行驶安全区域
                # 判断物体是否处于区域内
                # 根据情况，发出停障信号
                # 图像输出

                # judge_object 函数
                frame_obstacle_sign = judge_object(output_new, apex_list)
                # 决策函数
                stop_value = decision_make(frame_obstacle_sign, frames_obstacle_sign)
                # print("begin send!\n")
                # server_send(str(stop_value))
                controlInfo.changeTargetDetectionSign(stop_value)
                # print("complete send!\n")
                orig_img = draw_area(orig_img, apex_list, stop_value)

                # print("dra wthe result time: {:4.2f}".format(time.time()-start4))
                start5 = time.time()

                # cv2.imwrite(f'output2/{frames:06d}.jpg',orig_img)
                #cv2.imshow("frame", orig_img)
                dis_image = cv2.resize(orig_img, (701, 391))
                displayQPixMap = cv2.cvtColor(dis_image, cv2.COLOR_BGR2RGB)
                globalData.operateDisplayTargetDetection('change', displayQPixMap)
                # print("This is the {} frame ,FPS:{:5.2f},Time:{:4.2f}".format( frames,1 / (time.time() - start),time.time() - start))

                key = cv2.waitKey(1)
                if key & 0xFF == ord('q'):
                    break
                frames += 1