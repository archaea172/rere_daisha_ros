import os
import cv2
import numpy as np

from scipy.special import softmax

from hobot_dnn import pyeasy_dnn as dnn

class Ultralytics_YOLO_Detect_Bayese_YUV420SP():
    def __init__(self, model_path, classes_num, nms_thres, score_thres, reg, strides):
        #init
        self.REG = reg
        self.CLASSES_NUM = classes_num
        self.SCORE_THRESHOLD = score_thres
        self.NMS_THRESHOLD = nms_thres
        self.CONF_THRES_RAW = -np.log(1/self.SCORE_THRESHOLD - 1)
        self.input_H, self.input_W = self.quantize_model[0].inputs[0].properties.shape[2:4]
        self.strides = strides