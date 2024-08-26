# -*- coding: utf-8 -*-
from openvino.runtime import Core
import numpy as np
import cv2
# from ultralytics.yolo.utils import ROOT, yaml_load
# from ultralytics.yolo.utils.checks import check_yaml


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
    [0, 0.1850, -0.1650]
], dtype=np.float32)

CLASSES = {
    0: 'blue_not_hit',
    1: 'blue_hitted',
    2: 'blue_r',
    3: 'red_not_hit',
    4: 'red_hitted',
    5: 'red_r',

}

MODEL_NAME = "yolov8n-pose"
colors = ((255, 0, 0), (255, 0, 255), (170, 0, 255), (255, 0, 85), (255, 0, 170), (85, 255, 0),
          (255, 170, 0), (0, 255, 0), (255, 255, 0), (0, 255, 85), (170, 255, 0), (0, 85, 255),
          (0, 255, 170), (0, 0, 255), (0, 255, 255), (85, 0, 255), (0, 170, 255))

def draw_bounding_box(img, class_id, confidence, x, y, x_plus_w, y_plus_h, point):
    if point is not None and len(point) > 0:
        x1, y1, x2, y2 = point[0], point[1], point[2], point[3]
        x3, y3, x4, y4 = point[4], point[5], point[6], point[7]
        # print(type(x1))
        buff_2d_points = np.array([
            [x1, y1],
            [x2, y2],
            [x3, y3],
            [x4, y4]
        ], dtype=np.float32)

        success, rotation_vector, translation_vector = cv2.solvePnP(buff_3d_points, buff_2d_points, camera_matrix, distortion_coefficients)
        if success:
            # 将旋转向量和平移向量转换为字符串
            rotation_vector_str = np.array2string(rotation_vector.flatten(), precision=2, separator=', ')
            translation_vector_str = np.array2string(translation_vector.flatten(), precision=2, separator=', ')

    if class_id in CLASSES:
        label = f'{CLASSES[class_id]} ({confidence:.2f})({rotation_vector_str})({translation_vector_str})'
        color = colors[class_id]
        cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)
        cv2.putText(img, label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    else:
        print(f"Warning: Class ID {class_id} not found in CLASSES dictionary.")





core = Core()
model = core.read_model(model="best_csf.onnx")
net = core.compile_model(model = model, device_name="GPU")
output_node = net.outputs[0]
ir = net.create_infer_request()
cap = cv2.VideoCapture("test_video.mp4")

fps = int(cap.get(cv2.CAP_PROP_FPS))
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
output_video = cv2.VideoWriter('output_video.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps,
                               (width, height))

while True:
    ret, frame = cap.read()
    if not ret:
          break
    [height, width, _] = frame.shape
    length = max((height, width))
    image = np.zeros((length, length, 3), np.uint8)
    image[0:height, 0:width] = frame
    scale = length / 640

    blob = cv2.dnn.blobFromImage(image, scalefactor=1 / 255, size=(640, 640), swapRB=True)
    # 基于OpenVINO实现推理计算
    outputs = ir.infer(blob)[output_node]
    outputs = np.array([cv2.transpose(outputs[0])])
    rows = outputs.shape[1]
    # print(outputs.shape)
    # print(rows)
    boxes = []
    classes_scores = []
    index_idx = []
    preds_kpts = []
    for i in range(rows):
        # 置信度处理
        scores_2 = []
        for j in range(5):
            score = outputs[0][i][4+j]
            scores_2.append(score)
        max_index = np.argmax(scores_2)
        classes_score = scores_2[max_index]

        # 关键点处理
        key_points = outputs[0][i][10:]
        if classes_score >= 0.9:
            box = [
                outputs[0][i][0] - (0.5 * outputs[0][i][2]), outputs[0][i][1] - (0.5 * outputs[0][i][3]),
                outputs[0][i][2], outputs[0][i][3]]
            boxes.append(box)
            index_idx.append(max_index)
            classes_scores.append(classes_score)
            preds_kpts.append(key_points)

    result_boxes = cv2.dnn.NMSBoxes(boxes, classes_scores, 0.9, 0.1, 0.5)

    detections = []
    # # 处理r标
    # for i in range(len(result_boxes)):
    #     index = result_boxes[i]
    #     class_id = index_idx[index]
    #     if class_id == 2 or class_id == 5:
    #         r_x = box[0] + (box[2] / 2)
    #         r_y = box[1] + (box[3] / 2)
    # 处理边框
    for i in range(len(result_boxes)):
        index = result_boxes[i]
        box = boxes[index]
        class_id = index_idx[index]
        pred_kpts = preds_kpts[index]

        # print(type(pred_kpts))
        # print(pred_kpts.shape)
        # print(pred_kpts)
        # 包含八个元素的一维ndarray

        # 绘制边框
        draw_bounding_box(frame, index_idx[index], classes_scores[index], round(box[0] * scale), round(box[1] * scale),
                          round((box[0] + box[2]) * scale), round((box[1] + box[3]) * scale), pred_kpts)
        # print(index_idx[index], classes_scores[index])

    output_video.write(frame)

cap.release()
cv2.destroyAllWindows()