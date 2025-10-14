import os
import cv2
import numpy as np

from scipy.special import softmax

from hobot_dnn import pyeasy_dnn as dnn

class Ultralytics_YOLO_Detect_Bayese_YUV420SP():
    def __init__(self, model_path, classes_num, nms_thres, score_thres, reg, strides):
        self.quantize_model = dnn.load(model_path)
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
        
        self.coco_names = ['blue_ball', 'red_ball', 'yellow_ball']

    def preprocess_yuv420sp(self, img):
        RESIZE_TYPE = 0
        LETTERBOX_TYPE = 1
        PREPROCESS_TYPE = LETTERBOX_TYPE

        self.img_h, self.img_w = img.shape[0:2]
        if PREPROCESS_TYPE == RESIZE_TYPE:
            # 利用resize的方式进行前处理, 准备nv12的输入数据
            input_tensor = cv2.resize(img, (self.input_W, self.input_H), interpolation=cv2.INTER_NEAREST) # 利用resize重新开辟内存节约一次
            input_tensor = self.bgr2nv12(input_tensor)
            self.y_scale = 1.0 * self.input_H / self.img_h
            self.x_scale = 1.0 * self.input_W / self.img_w
            self.y_shift = 0
            self.x_shift = 0
        elif PREPROCESS_TYPE == LETTERBOX_TYPE:
            # 利用 letter box 的方式进行前处理, 准备nv12的输入数据
            self.x_scale = min(1.0 * self.input_H / self.img_h, 1.0 * self.input_W / self.img_w)
            self.y_scale = self.x_scale
            
            if self.x_scale <= 0 or self.y_scale <= 0:
                raise ValueError("Invalid scale factor.")
            
            new_w = int(self.img_w * self.x_scale)
            self.x_shift = (self.input_W - new_w) // 2
            x_other = self.input_W - new_w - self.x_shift
            
            new_h = int(self.img_h * self.y_scale)
            self.y_shift = (self.input_H - new_h) // 2
            y_other = self.input_H - new_h - self.y_shift
            
            input_tensor = cv2.resize(img, (new_w, new_h))
            input_tensor = cv2.copyMakeBorder(input_tensor, self.y_shift, y_other, self.x_shift, x_other, cv2.BORDER_CONSTANT, value=[127, 127, 127])
            input_tensor = self.bgr2nv12(input_tensor)
        else:
            exit(-1)

        return input_tensor
    
    def bgr2nv12(self, bgr_img):
        height, width = bgr_img.shape[0], bgr_img.shape[1]
        area = height * width
        yuv420p = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
        y = yuv420p[:area]
        uv_planar = yuv420p[area:].reshape((2, area // 4))
        uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,))
        nv12 = np.zeros_like(yuv420p)
        nv12[:height * width] = y
        nv12[height * width:] = uv_packed
        return nv12
    
    def forward(self, input_tensor):
        quantize_outputs = self.quantize_model[0].forward(input_tensor)
        return quantize_outputs

    def c2numpy(self, outputs):
        outputs = [dnnTensor.buffer for dnnTensor in outputs]
        return outputs
    
    def postProcess(self, outputs):
        
        # 1. モデルからの単一出力を取得し、形状を整える
        # original shape: (1, 7, 8400, 1) -> transposed shape: (8400, 7)
        output_data = outputs[0].squeeze().transpose()

        # 2. 信頼度の低いバウンディングボックスをフィルタリング
        # 4列目以降がクラススコアなので、行ごとの最大値を取得
        scores = np.max(output_data[:, 4:], axis=1)
        # スコアが閾値を超えているものだけを残す
        mask = scores > self.SCORE_THRESHOLD
        filtered_data = output_data[mask]
        
        if not filtered_data.any():
            return []

        # 3. 残ったデータからボックス、スコア、クラスIDを抽出
        filtered_boxes = filtered_data[:, :4] # cx, cy, w, h
        filtered_scores = filtered_data[:, 4:]
        
        # 各ボックスの最終的な信頼度とクラスIDを取得
        confidences = np.max(filtered_scores, axis=1)
        class_ids = np.argmax(filtered_scores, axis=1)

        # 4. バウンディングボックスの形式を (cx, cy, w, h) から (x, y, w, h) に変換
        # NMSBoxesは (x, y, w, h) 形式を要求するため
        x = filtered_boxes[:, 0] - filtered_boxes[:, 2] / 2
        y = filtered_boxes[:, 1] - filtered_boxes[:, 3] / 2
        w = filtered_boxes[:, 2]
        h = filtered_boxes[:, 3]
        cv_boxes = np.column_stack((x, y, w, h))

        # 5. Non-Maximum Suppression (NMS) を実行して重複ボックスを削除
        indices = cv2.dnn.NMSBoxes(cv_boxes.tolist(), confidences.tolist(), self.SCORE_THRESHOLD, self.NMS_THRESHOLD)

        if len(indices) == 0:
            return []

        # 6. 最終結果をフォーマットする
        results = []
        for i in indices.flatten():
            # (cx, cy, w, h) から (x1, y1, x2, y2) を計算
            cx, cy, width, height = filtered_boxes[i]
            x1 = cx - width / 2
            y1 = cy - height / 2
            x2 = cx + width / 2
            y2 = cy + height / 2
            
            # 元の画像座標系に変換
            x1_final = int((x1 - self.x_shift) / self.x_scale)
            y1_final = int((y1 - self.y_shift) / self.y_scale)
            x2_final = int((x2 - self.x_shift) / self.x_scale)
            y2_final = int((y2 - self.y_shift) / self.y_scale)
            
            # 座標が画像範囲内に収まるようにクリッピング
            x1_final = max(0, min(x1_final, self.img_w))
            y1_final = max(0, min(y1_final, self.img_h))
            x2_final = max(0, min(x2_final, self.img_w))
            y2_final = max(0, min(y2_final, self.img_h))

            # クラスIDと信頼度を取得
            class_id = class_ids[i]
            confidence = confidences[i]
            
            results.append((class_id, confidence, x1_final, y1_final, x2_final, y2_final))

        return results
