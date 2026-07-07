import cv2
import numpy as np
from pathlib import Path

from pinkylib import Camera

SCRIPT_DIR = Path(__file__).resolve().parent
CFG_PATH = SCRIPT_DIR / "yolov3.cfg"
WEIGHTS_PATH = SCRIPT_DIR / "yolov3.weights"
NAMES_PATH = SCRIPT_DIR / "coco.names"
WINDOW_NAME = "Pinky YOLO Detection"

CONFIDENCE_THRESHOLD = 0.5
NMS_THRESHOLD = 0.4
INPUT_SIZE = (416, 416)


def load_class_names(names_path: Path) -> list[str]:
    with names_path.open(encoding="utf-8") as file:
        return [line.strip() for line in file if line.strip()]


def load_yolo_model(cfg_path: Path, weights_path: Path):
    net = cv2.dnn.readNetFromDarknet(str(cfg_path), str(weights_path))
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    layer_names = net.getLayerNames()
    unconnected_layers = net.getUnconnectedOutLayers()
    if len(unconnected_layers.shape) == 2:
        output_layers = [layer_names[index - 1] for index in unconnected_layers.flatten()]
    else:
        output_layers = [layer_names[index - 1] for index in unconnected_layers]

    return net, output_layers


def detect_objects(frame, net, output_layers, class_names):
    height, width = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(
        frame, 1 / 255.0, INPUT_SIZE, swapRB=True, crop=False
    )
    net.setInput(blob)
    detections = net.forward(output_layers)

    boxes = []
    confidences = []
    class_ids = []

    for output in detections:
        for detection in output:
            scores = detection[5:]
            class_id = int(np.argmax(scores))
            confidence = float(scores[class_id])
            if confidence < CONFIDENCE_THRESHOLD:
                continue

            center_x = int(detection[0] * width)
            center_y = int(detection[1] * height)
            box_width = int(detection[2] * width)
            box_height = int(detection[3] * height)
            left = int(center_x - box_width / 2)
            top = int(center_y - box_height / 2)

            boxes.append([left, top, box_width, box_height])
            confidences.append(confidence)
            class_ids.append(class_id)

    indices = cv2.dnn.NMSBoxes(boxes, confidences, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)
    result = frame.copy()

    if len(indices) > 0:
        for index in np.array(indices).flatten():
            left, top, box_width, box_height = boxes[index]
            label = f"{class_names[class_ids[index]]}: {confidences[index]:.2f}"
            cv2.rectangle(
                result,
                (left, top),
                (left + box_width, top + box_height),
                (0, 255, 0),
                2,
            )
            cv2.putText(
                result,
                label,
                (left, max(top - 10, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

    return result


def is_window_closed(window_name: str) -> bool:
    try:
        return cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1
    except cv2.error:
        return True


def main():
    class_names = load_class_names(NAMES_PATH)
    net, output_layers = load_yolo_model(CFG_PATH, WEIGHTS_PATH)

    cam = Camera()
    cam.start()

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    try:
        while True:
            frame = cam.get_frame()
            if frame is None:
                if is_window_closed(WINDOW_NAME):
                    break
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            detected_frame = detect_objects(frame, net, output_layers, class_names)
            cv2.imshow(WINDOW_NAME, detected_frame)

            if is_window_closed(WINDOW_NAME):
                break
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        if hasattr(cam, "stop"):
            cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
