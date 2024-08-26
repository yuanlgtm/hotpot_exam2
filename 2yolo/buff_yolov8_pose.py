# -*- coding: utf-8 -*-

import cv2
import numpy as np
from openvino.runtime import Core



# 相机内参矩阵、畸变系数、3D参考点坐标
camera_matrix = np.array([
    [2075.15666, 0.0, 646.02307],
    [0.0, 2073.92438, 479.8963],
    [0.0, 0.0, 1.0]
], dtype=np.float32)
distortion_coefficients = np.array([-0.051148, 0.231678, 0.000775, 0.002697, 0.0], dtype=np.float32)
buff_3d_points = np.array([
    [0, 0.1700, 0.1750],
    [0, -0.1700, 0.1750],
    [0, -0.1850, -0.1650],
    [0, 0, -0.7150],
    [0, 0.1850, -0.1650]
], dtype=np.float32)

def xywh2xyxy(x):
    y = np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left point x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left point y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y

    return y

# 检测框左上点和右下点的坐标转为xywh
def xyxy2xywh(a):
    b = np.copy(a)
    b[:, 0] = (a[:, 2] + a[:, 0]) / 2  # centre x
    b[:, 1] = (a[:, 3] + a[:, 1]) / 2  # centre y
    b[:, 2] = a[:, 2] - a[:, 0]  # w
    b[:, 3] = a[:, 3] - a[:, 1]  # h

    return b

# 自适应缩放图片大小
def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), scaleup=True):
    img_shape = (1920, 1080)
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    ratio = min(new_shape[0] / img_shape[0], new_shape[1] / img_shape[1])
    if not scaleup:
        ratio = min(ratio, 1.0)

    new_unpad = (int(round(img_shape[1] * ratio)), int(round(img_shape[0] * ratio)))
    dw = (new_shape[1] - new_unpad[0]) / 2.0
    dh = (new_shape[0] - new_unpad[1]) / 2.0

    if img_shape[::-1] != new_unpad:
        resized_img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    else:
        resized_img = img

    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw + 0.1))
    resized_img = cv2.copyMakeBorder(resized_img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)

    return resized_img


# 归一化，转换为深度学习框架需要的格式，并添加batch维度， (1, channels, height, width)
def preprocess(src_img):
    output_img = src_img / 255.0
    output_img = np.transpose(output_img, (2, 0, 1))
    data = np.expand_dims(output_img, axis=0)

    return data

# 判断分类:红色灯光下的中心R标、未被击打扇叶、已被击打扇叶;蓝色灯光下的中心R标、未被击打扇叶、已被击打扇叶
def find_max_index(arr):
    if len(arr) == 0:
        raise ValueError("数组不能为空")

    max_value = arr[0]
    max_index = 0
    for i in range(1, len(arr)):
        if arr[i] > max_value:
            max_value = arr[i]
            max_index = i

    return max_index

# 非极大值抑制处理
def nms_bbox(boxes_points, iou_thresh):
    x_1 = boxes_points[:, 0]
    y_1 = boxes_points[:, 1]
    x_2 = boxes_points[:, 2]
    y_2 = boxes_points[:, 3]
    scores = boxes_points[:, 4]
    areas = (x_2 - x_1 + 1) * (y_2 - y_1 + 1)
    # 按照置信度排序
    order = scores.argsort()[::-1]
    # print(order)
    target_point_idx = []

    while order.size > 0:
        max_thres_idx = order[0]
        target_point_idx.append(max_thres_idx)
        xx1 = np.maximum(x_1[max_thres_idx], x_1[order[1:]])
        yy1 = np.maximum(y_1[max_thres_idx], y_1[order[1:]])
        xx2 = np.minimum(x_2[max_thres_idx], x_2[order[1:]])
        yy2 = np.minimum(y_2[max_thres_idx], y_2[order[1:]])
        # 计算置信度高的bbox和其他剩下bbox之间交叉区域的面积
        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)
        inter_area = w * h
        ovr = inter_area / (areas[max_thres_idx] + areas[order[1:]] - inter_area)
        min_bbox = np.where(ovr <= iou_thresh)[0]
        order = order[min_bbox + 1]

    output = []
    for i in target_point_idx:
        output.append(boxes_points[i].tolist())

    return np.array(output)

# 进行一个边界截断，以免溢出,并且将检测框的坐标（左上角x，左上角y，宽度，高度）--->>>（左上角x，左上角y，右下角x，右下角y）
def clip_boxes(boxes, shape):
    top_left_x = boxes[:, 0].clip(0, shape[1])
    top_left_y = boxes[:, 1].clip(0, shape[0])
    bottom_right_x = (boxes[:, 0] + boxes[:, 2]).clip(0, shape[1])
    bottom_right_y = (boxes[:, 1] + boxes[:, 3]).clip(0, shape[0])
    boxes[:, 0] = top_left_x  # 左上
    boxes[:, 1] = top_left_y
    boxes[:, 2] = bottom_right_x  # 右下
    boxes[:, 3] = bottom_right_y


# 将检测框(x y w h)从img1_shape(预测图) 缩放到 img0_shape(原图)
def scale_boxes(img1_shape, boxes, img0_shape):
    gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])  # gain  = old / new
    pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2  # wh padding
    # boxes[:, 0] -= pad[0]
    # boxes[:, 1] -= pad[1]
    # boxes[:, :4] /= gain  # 检测框坐标点还原到原图上
    # num_kpts = boxes.shape[1] // 3  # 56 // 3 = 18
    # for kid in range(2, num_kpts + 1):
    #     boxes[:, kid * 3 - 1] = (boxes[:, kid * 3 - 1] - pad[0]) / gain
    #     boxes[:, kid * 3] = (boxes[:, kid * 3] - pad[1]) / gain
    boxes[:, 6:] /= gain  # 关键点坐标还原到原图上
    clip_boxes(boxes, img0_shape)

    return boxes

# 绘制装甲板
def draw_buff_points(img, keypoints, radius=5, color=(0, 0, 255)):
    for point in keypoints:
        cv2.circle(img, (int(point[6]), int(point[7])), radius, color, -1)
        cv2.circle(img, (int(point[8]), int(point[9])), radius, color, -1)
        cv2.circle(img, (int(point[10]), int(point[11])), radius, color, -1)
        cv2.circle(img, (int(point[12]), int(point[13])), radius, color, -1)
        cv2.line(img, (int(point[6]), int(point[7])), (int(point[8]), int(point[9])), color, 2)
        cv2.line(img, (int(point[8]), int(point[9])), (int(point[10]), int(point[11])), color, 2)
        cv2.line(img, (int(point[10]), int(point[11])), (int(point[12]), int(point[13])), color, 2)
        cv2.line(img, (int(point[12]), int(point[13])), (int(point[6]), int(point[7])), color, 2)
        cv2.putText(img, str(point[4]), (int(point[6]), int(point[7])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        cv2.putText(img, str(point[5]), (int(point[8]), int(point[9])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

# 绘制检测框
def draw_detection_box(img, keypoints, color=(0, 255, 0), thickness=2):
    for point in keypoints:
        x, y, w, h = point[:4]/255.0
        cv2.rectangle(img, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), color, thickness)

# npn
def npn(bbox_init):
    for point in bbox_init:
        x1, y1, x2, y2 = point[6], point[7], point[8], point[9]
        x3, y3, x4, y4 = point[10], point[11], point[12], point[13]
        buff_2d_points = np.array([
            [x1, y1],
            [x2, y2],
            [x3, y3],
            [(x1 + x2 + x3 + x4)/4, (y1 + y2 + y3 + y4)/4],
            [x4, y4]
        ], dtype=np.float32)

        success, rotation_vector, translation_vector = cv2.solvePnP(buff_3d_points, buff_2d_points, camera_matrix, distortion_coefficients)

        print("旋转向量:\n", rotation_vector)
        print("平移向量:\n", translation_vector)


if __name__ == '__main__':
    # 读取ONNX模型和视频
    ie = Core()
    model = ie.read_model(model="best_csf.onnx")
    compiled_model = ie.compile_model(model=model, device_name="CPU")
    input_layer = compiled_model.input(0)

    cap = cv2.VideoCapture('test_video.mp4')
    # 30,1920,1080
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    output_video = cv2.VideoWriter('output_video.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps,
                                   (width, height))


    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # 预处理帧
        input_image = letterbox(frame, new_shape=640, color=(114, 114, 114), scaleup=True)
        input_image = preprocess(input_image)
        # input_image = cv2.resize(frame, (640, 640))
        # input_image = input_image.astype(np.float32)
        # input_image = input_image.transpose(2, 0, 1)  # HWC to CHW
        # input_image = np.expand_dims(input_image, axis=0)
        # print(input_image.shape)
        # print(type(input_image))

        # 推理
        results = compiled_model([input_image])[compiled_model.output(0)]
        # 18*8400
        outputs_init = results[0]
        # 8400*18
        outputs = outputs_init.T
        # print(outputs.shape)

        # 解析检测框和置信度
        boxes = []
        classes_idx = []
        confidences = []
        keypoints = []
        boxes_points = []
        for output in outputs:
            # print(output.shape)
            x, y, w, h = output[0:4]
            scores = output[4:10]
            class_idx = find_max_index(scores)
            confidence = np.max(scores)
            if confidence > 0.6:
                x, y, w, h = output[0:4]
                scores = output[4:10]
                class_idx = find_max_index(scores)
                confidence = np.max(scores)
                boxes.append([x -w/2, y - h/2, x + w/2, y + h/2])
                classes_idx.append(class_idx)
                confidences.append(confidence)
                keypoints.append(output[10:])
                boxes_points.append([x, y, x + w, y + h, confidence, class_idx] + output[10:].tolist())
        # 转成二维数组，除confidences为一维数组
        boxes = np.array(boxes)
        classes = np.array(classes_idx)
        confidences = np.array(confidences)
        keypoints = np.array(keypoints)
        boxes_points = np.array(boxes_points)
        # print(boxes_points.shape)

        if len(boxes_points) != 0:
            # 非极大值抑制
            bbox = nms_bbox(boxes_points, iou_thresh=0.3)
            # print(bbox.shape)
            # 还原坐标
            bbox_init = scale_boxes((640, 640), bbox, (1920, 1080))
            # 绘制装甲板点
            # draw_buff_points(frame, bbox_init)

            # npn
            # npn(bbox_init)

            # 绘制检测框
            draw_detection_box(frame, bbox_init)
            # draw_detection_box(frame, bbox)

        # 显示结果
        # cv2.imshow('frame', frame)

        # 输出视频
        output_video.write(frame)


    cap.release()
    output_video.release()

