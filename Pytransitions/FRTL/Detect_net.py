#!/usr/bin/env python3
import sys, cv2, os, torch, json, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import onnxruntime
from torchvision import transforms
from PIL import Image as PILImage


# Mesma ordem usada no treinamento
CLASS_NAMES = (
    "BACKGROUND",  # índice 0
    "bases",       # índice 1
    "small_base",  # índice 2
    "large_base"   # índice 3
)

THRESHOLD = 0.4

# =============================
# Funções auxiliares de NMS/IoU
# =============================
def area_of(left_top, right_bottom) -> torch.Tensor:
    hw = torch.clamp(right_bottom - left_top, min=0.0)
    return hw[..., 0] * hw[..., 1]

def iou_of(boxes0, boxes1, eps=1e-5):
    overlap_left_top = torch.max(boxes0[..., :2], boxes1[..., :2])
    overlap_right_bottom = torch.min(boxes0[..., 2:], boxes1[..., 2:])
    overlap_area = area_of(overlap_left_top, overlap_right_bottom)
    area0 = area_of(boxes0[..., :2], boxes0[..., 2:])
    area1 = area_of(boxes1[..., :2], boxes1[..., 2:])
    return overlap_area / (area0 + area1 - overlap_area + eps)

def hard_nms(box_scores, iou_threshold, top_k=5, candidate_size=200):
    scores = box_scores[:, -1]
    boxes = box_scores[:, :-1]
    picked = []
    _, indexes = scores.sort(descending=True)
    indexes = indexes[:candidate_size]
    while len(indexes) > 0:
        current = indexes[0]
        picked.append(current.item())
        if 0 < top_k == len(picked) or len(indexes) == 1:
            break
        current_box = boxes[current, :]
        indexes = indexes[1:]
        rest_boxes = boxes[indexes, :]
        iou = iou_of(rest_boxes, current_box.unsqueeze(0))
        indexes = indexes[iou <= iou_threshold]
    return box_scores[picked, :]

# =============================
# Função de predição
# =============================
def predict(session, image, width, height, threshold=0.7):
    # Lista de classes — mesma ordem usada no treinamento
    CLASS_NAMES = (
        "BACKGROUND",  # índice 0
        "bases",       # índice 1
        "small_base",  # índice 2
        "large_base"   # índice 3
    )

    img = image.copy()
    img_resized = cv2.resize(image, (300, 300))
    pil_img = PILImage.fromarray(cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB))

    preprocess = transforms.Compose([
        transforms.Resize((300, 300)),
        transforms.ToTensor()
    ])
    img_norm = preprocess(pil_img).unsqueeze(0)
    data = np.array(img_norm).astype('float32')

    input_name = session.get_inputs()[0].name
    output_scores = session.get_outputs()[0].name
    output_boxes = session.get_outputs()[1].name

    scores, boxes = session.run([output_scores, output_boxes], {input_name: data})
    scores, boxes = torch.tensor(scores), torch.tensor(boxes)

    labels = []
    picked_box_probs = []

    for class_index in range(1, scores.shape[2]):
        probs = scores[0, :, class_index]
        mask = probs > threshold
        probs = probs[mask]
        if len(probs) == 0:
            continue
        subset_boxes = boxes[0, mask, :]
        box_probs = torch.cat([subset_boxes, probs.reshape(-1, 1)], dim=1)
        box_probs = hard_nms(box_probs, 0.45, -1, 200)
        picked_box_probs.append(box_probs)
        labels.extend([class_index] * box_probs.size(0))

    detections = []
    if picked_box_probs:
        picked_box_probs = torch.cat(picked_box_probs)

        # Correção de escala
        picked_box_probs[:, 0] *= width   # xmin
        picked_box_probs[:, 1] *= height  # ymin
        picked_box_probs[:, 2] *= width   # xmax
        picked_box_probs[:, 3] *= height  # ymax

        for i, box in enumerate(picked_box_probs):
            box_int = [int(x) for x in box[:4]]
            class_id = labels[i]
            class_name = CLASS_NAMES[class_id] if class_id < len(CLASS_NAMES) else str(class_id)
            score = float(box[-1])

            # Calcula o centro da bounding box
            cx = int((box_int[0] + box_int[2]) / 2)
            cy = int((box_int[1] + box_int[3]) / 2)

            # Adiciona ao JSON de saída
            detections.append({
                "label": class_name,
                "box": box_int,
                "score": score,
                "center": {"x": cx, "y": cy}
            })

            # Desenha bounding box
            cv2.rectangle(img, (box_int[0], box_int[1]), (box_int[2], box_int[3]), (255, 255, 0), 3)
            
            # Desenha o centro (um círculo)
            cv2.circle(img, (cx, cy), radius=6, color=(0, 0, 255), thickness=-1)

            # Mostra classe + confiança (%)
            label_text = f"{class_name} {score*100:.1f}%"
            cv2.putText(img, label_text, (box_int[0] + 10, box_int[1] + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 2)


    return img, detections



# =============================
# Nó ROS 2 com CUDA
# =============================
class SSDNode(Node):
    def __init__(self):
        super().__init__('ssd_detector')
        self.bridge = CvBridge()

        model_path = "Pytransitions/FRTL/mobilenet/models/FRTL/mb1-ssd.onnx"
        if not os.path.exists(model_path):
            self.get_logger().error(f"Modelo não encontrado: {model_path}")
            sys.exit(1)

        # Escolhe o provider: CUDA se disponível, senão CPU
        available_providers = onnxruntime.get_available_providers()
        if 'CUDAExecutionProvider' in available_providers:
            self.get_logger().info("Usando CUDAExecutionProvider (GPU).")
            providers = ['CUDAExecutionProvider']
        else:
            self.get_logger().warn("CUDAExecutionProvider não disponível. Usando CPUExecutionProvider.")
            providers = ['CPUExecutionProvider']

        self.session = onnxruntime.InferenceSession(model_path, providers=providers)

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        # Publishers
        self.publisher_image = self.create_publisher(Image, '/ssd/image_annotated', 10)
        self.publisher_detections = self.create_publisher(String, '/ssd/detections', 10)

        self.get_logger().info("Nó SSD iniciado. Publicando em /ssd/image_annotated e /ssd/detections")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        start = time.time()
        annotated_img, detections = predict(self.session, frame, frame.shape[1], frame.shape[0], THRESHOLD)
        end = time.time()

        # Publica imagem anotada
        img_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
        self.publisher_image.publish(img_msg)

        # Publica JSON de detecções
        detection_msg = String()
        detection_msg.data = json.dumps({"detections": detections})
        self.publisher_detections.publish(detection_msg)

        self.get_logger().info(f"Inferência: {end - start:.3f} s, {len(detections)} objetos detectados")
        time.sleep(0.1)

# =============================
# Main
# =============================
def main(args=None):
    rclpy.init(args=args)
    node = SSDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
