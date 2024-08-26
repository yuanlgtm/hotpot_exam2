import cv2
import onnxruntime as ort
import numpy as np
import math

MODEL_PATH = "yolov8n-pose.onnx"    # 检测模型文件
KPT_THRESHOLD = 0.5                 # 关键点筛选阈值
CLASS_THRESHOLD = 0.7               # 分类筛选阈值
NMS_THRESHOLD = 0.6
STEPS = 3

# 调色板，17个关键点按顺序对应的颜色
palette = np.array([[255, 128, 0], [255, 153, 51], [255, 178, 102],
                    [230, 230, 0], [255, 153, 255], [153, 204, 255],
                    [255, 102, 255], [255, 51, 255], [102, 178, 255],
                    [51, 153, 255], [255, 153, 153], [255, 102, 102],
                    [255, 51, 51], [153, 255, 153], [102, 255, 102],
                    [51, 255, 51], [0, 255, 0], [0, 0, 255], [255, 0, 0],
                    [255, 255, 255]])
# 17个关键点连接顺序
skeleton = [[16, 14], [14, 12], [17, 15], [15, 13], [12, 13], [6, 12],
            [7, 13], [6, 7], [6, 8], [7, 9], [8, 10], [9, 11], [2, 3],
            [1, 2], [1, 3], [2, 4], [3, 5], [4, 6], [5, 7]]

# 骨架颜色
pose_limb_color = palette[[9, 9, 9, 9, 7, 7, 7, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16, 16, 16]]
# 关键点颜色
pose_kpt_color = palette[[16, 16, 16, 16, 16, 0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 9, 9]]

# 神经网络输出的检测框xywh转为检测框左上点和右下点的坐标
# 为什么要选择所有行？
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
# 填充边界为什么是114？中等明暗度的灰色
# img_shape: 一个包含两个整数的元组，通常表示图像的高度和宽度，即 (height, width)
# img_shape[::-1]反转元组，使得(height, width)变为(width, height)
# resize使用interpolation=cv2.INTER_LINEAR表示线性插值
def letterbox(src_img, new_shape=(640, 640), color=(114, 114, 114), scaleup=True):
    img_shape = src_img.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    ratio = min(new_shape[0] / img_shape[0], new_shape[1] / img_shape[1])
    if not scaleup:
        ratio = min(ratio, 1.0)

    new_unpad = (int(round(img_shape[1] * ratio)), int(round(img_shape[0] * ratio)))
    dw = (new_shape[1] - new_unpad[0]) / 2.0
    dh = (new_shape[0] - new_unpad[1]) / 2.0

    if img_shape[::-1] != new_unpad:
        resized_img = cv2.resize(src_img, new_unpad, interpolation=cv2.INTER_LINEAR)

    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw + 0.1))
    resized_img = cv2.copyMakeBorder(src_img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)

    return resized_img

# 对获取到的结果进行预处理
# 归一化，转换为深度学习框架需要的格式，并添加batch维度， (1, channels, height, width)
def preprocess(src_img):
    output_img = src_img / 255.0
    output_img = np.transpose(output_img, (2, 0, 1))
    data = np.expand_dims(output_img, axis=0)

    return data

# 将检测框(x y w h)从img1_shape(预测图) 缩放到 img0_shape(原图)
"""
    :param img1_shape: 缩放后的图像尺度
    :param boxes:  预测的box信息
    :param img0_shape: 原始图像尺度
"""
def scale_boxes(img1_shape, boxes, img0_shape):
    gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])  # gain  = old / new
    pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2  # wh padding
    boxes[:, 0] -= pad[0]
    boxes[:, 1] -= pad[1]
    boxes[:, :4] /= gain  # 检测框坐标点还原到原图上
    num_kpts = boxes.shape[1] // 3  # 56 // 3 = 18
    for kid in range(2, num_kpts + 1):
        boxes[:, kid * 3 - 1] = (boxes[:, kid * 3 - 1] - pad[0]) / gain
        boxes[:, kid * 3] = (boxes[:, kid * 3] - pad[1]) / gain
    # boxes[:, 5:] /= gain  # 关键点坐标还原到原图上
    clip_boxes(boxes, img0_shape)

    return boxes


def clip_boxes(boxes, shape):
    # 进行一个边界截断，以免溢出
    # 并且将检测框的坐标（左上角x，左上角y，宽度，高度）--->>>（左上角x，左上角y，右下角x，右下角y）
    top_left_x = boxes[:, 0].clip(0, shape[1])
    top_left_y = boxes[:, 1].clip(0, shape[0])
    bottom_right_x = (boxes[:, 0] + boxes[:, 2]).clip(0, shape[1])
    bottom_right_y = (boxes[:, 1] + boxes[:, 3]).clip(0, shape[0])
    boxes[:, 0] = top_left_x  # 左上
    boxes[:, 1] = top_left_y
    boxes[:, 2] = bottom_right_x  # 右下
    boxes[:, 3] = bottom_right_y

# 非极大值抑制处理
def nms_bbox(box_points, iou_thresh):
    x_1 = box_points[:, 0]
    y_1 = box_points[:, 1]
    x_2 = box_points[:, 2]
    y_2 = box_points[:, 3]
    scores = box_points[:, 4]
    areas = (x_2 - x_1 + 1) * (y_2 - y_1 + 1)
    # 按照置信度排序
    order = scores.argsort()[::-1]
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
        output.append(box_points[i].tolist())

    return np.array(output)

# 绘制关键点及连线
"""
    :function: 在图像上打上人体关键点
    :param img: 输入图像
    :param kpts: 关键点坐标信息
    :param step: 每个关键点含有的信息步长
"""
def plot_kpts(img, kpts, step=STEPS):
    num_kpts = len(kpts) // step
    for i in range(num_kpts):
        r, g, b = pose_kpt_color[i]
        x_coord, y_coord, score = kpts[step * i], kpts[step * i + 1], kpts[step * i + 2]
        if score > KPT_THRESHOLD:
            cv2.circle(img, (int(x_coord), int(y_coord)), 2, (int(r), int(g), int(b)), -1)
            cv2.putText(img, str(i), (int(x_coord) + 2, int(y_coord)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (int(r), int(g), int(b)), 1)

    for sk_id, sk in enumerate(skeleton):
        r, g, b = pose_limb_color[sk_id]
        pos1 = (int(kpts[(sk[0] - 1) * step]), int(kpts[(sk[0] - 1) * step + 1]))
        pos2 = (int(kpts[(sk[1] - 1) * step]), int(kpts[(sk[1] - 1) * step + 1]))
        conf1 = kpts[(sk[0] - 1) * step + 2]
        conf2 = kpts[(sk[1] - 1) * step + 2]
        if conf1 > KPT_THRESHOLD and conf2 > KPT_THRESHOLD:
            cv2.line(img, pos1, pos2, (int(r), int(g), int(b)), 2)


"""
    :function: 判定摔倒
    :param kpts: 关键点坐标信息
    :param step: 每个关键点含有的信息步长
"""
def judge_fall_down(img, kpts, step=STEPS):
    left_shoulder_idx = 5
    right_shoulder_idx = 6
    left_hip_idx = 11
    right_hip_idx = 12
    is_point_visible_ = False
    if(kpts[left_shoulder_idx * step + 2] > 0.80 and kpts[right_shoulder_idx * step + 2] > 0.80 and
            kpts[left_hip_idx * step + 2] > 0.80 and kpts[right_hip_idx * step + 2] > 0.80):
        is_point_visible_ = True
    # 得到肩部两个关键点连线的中点
    if is_point_visible_:
        shoulder_mid_point = [(kpts[left_shoulder_idx * step + 0] + kpts[right_shoulder_idx * step + 0]) / 2.0,
                              (kpts[left_shoulder_idx * step + 1] + kpts[right_shoulder_idx * step + 1]) / 2.0]
        # 得到胯部两个关键点连线的中点点
        hip_mid_point = [(kpts[left_hip_idx * step + 0] + kpts[right_hip_idx * step + 0]) / 2.0,
                         (kpts[left_hip_idx * step + 1] + kpts[right_hip_idx * step + 1]) / 2.0]
        # cv2.circle(img, (int((kpts[left_shoulder_idx * step + 0] + kpts[right_shoulder_idx * step + 0]) / 2.0),
        #                  int((kpts[left_shoulder_idx * step + 1] + kpts[right_shoulder_idx * step + 1]) / 2.0)), 2,
        #             (255, 0, 0), -1)
        # cv2.circle(img, (int((kpts[left_hip_idx * step + 0] + kpts[right_hip_idx * step + 0]) / 2.0),
        #                  int((kpts[left_hip_idx * step + 1] + kpts[right_hip_idx * step + 1]) / 2.0)), 2,
        #            (255, 0, 0), -1)
        # cv2.line(img, (int((kpts[left_shoulder_idx * step + 0] + kpts[right_shoulder_idx * step + 0]) / 2.0),
        #                 int((kpts[left_shoulder_idx * step + 1] + kpts[right_shoulder_idx * step + 1]) / 2.0)),
        #             (int((kpts[left_hip_idx * step + 0] + kpts[right_hip_idx * step + 0]) / 2.0),
        #                 int((kpts[left_hip_idx * step + 1] + kpts[right_hip_idx * step + 1]) / 2.0)), (255, 0, 0), 1)
        # 计算上面两个中点之间的连线，与水平线之间的夹角
        angle = math.atan2(hip_mid_point[1] - shoulder_mid_point[1],
                           hip_mid_point[0] - shoulder_mid_point[0])
        angle = angle * 180.0 / math.pi
    else:
        angle = -1e10
    return angle, is_point_visible_


# 关键点检测器类
class KptDetector:
    def __init__(self):
        super(KptDetector, self).__init__()
        # 导入模型，开启OnnxRuntime推理
        self.session = ort.InferenceSession(MODEL_PATH, providers=ort.get_available_providers())
        self.input_name = self.session.get_inputs()[0].name
        self.label_name = self.session.get_outputs()[0].name

    # 推理函数
    def inference(self, src_img):
        img = letterbox(src_img)
        data = preprocess(img)
        pred = self.session.run(
            [self.label_name], {self.input_name: data.astype(np.float32)})[0]
        # print(pred.shape)
        pred = pred[0]
        # print(pred.shape)
        pred = cv2.transpose(pred, (1, 0))
        pred = pred[pred[:, 4] > CLASS_THRESHOLD]
        status = 0
        if len(pred) == 0:
            status = 0

        else:
            bboxes = xywh2xyxy(pred)
            bboxes = nms_bbox(bboxes, NMS_THRESHOLD)
            bboxes = np.array(bboxes)
            bboxes = xyxy2xywh(bboxes)
            bboxes = scale_boxes(img.shape, bboxes, src_img.shape)

            for bbox in bboxes:
                det_bbox, det_scores, kpts = bbox[0:4], bbox[4], bbox[5:]
                angle, is_point_visible = judge_fall_down(src_img, kpts)
                if -1e3 < angle < 45.0 or angle > 135.0 and is_point_visible:
                    status = 2
                    cv2.putText(src_img, "Fall Down!", (20, 20), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 0, 255), 1)
                else:
                    status = 1
                    cv2.putText(src_img, "Normal", (20, 20), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 0, 255), 1)
                plot_kpts(src_img, kpts)

        return status, src_img


# 下面为测试程序

if __name__ == "__main__":
    detector = KptDetector()
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        status, resultImg = detector.inference(frame)
        cv2.imshow("test", resultImg)
        if cv2.waitKey(10) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
