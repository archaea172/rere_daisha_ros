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
        
        # DFL calculates the expected coefficients, which only needs to be generated once.
        self.weights_static = np.array([i for i in range(reg)]).astype(np.float32)[np.newaxis, np.newaxis, :]

        self.grids = []
        for stride in self.strides:
            assert self.input_H % stride == 0, f"{stride=}, {self.input_H=}: input_H % stride != 0"
            assert self.input_W % stride == 0, f"{stride=}, {self.input_W=}: input_W % stride != 0"
            grid_H, grid_W = self.input_H // stride, self.input_W // stride
            self.grids.append(np.stack([np.tile(np.linspace(0.5, grid_H-0.5, grid_H), reps=grid_H), 
                            np.repeat(np.arange(0.5, grid_W+0.5, 1), grid_W)], axis=0).transpose(1,0))
