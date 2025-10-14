import numpy as np
import cv2
from openvino import Core,properties

class Yolov4TrashDetector:
    def __init__(self):
        self.CONF_THRESH = 0.35
        self.NMS_THRESH = 0.5
        self.CLASSES = [
            'Aluminium foil', 'Bottle cap', 'Bottle', 'Broken glass', 'Can', 'Carton',
            'Cigarette', 'Cup', 'Lid', 'Other litter', 'Other plastic', 'Paper',
            'Plastic bag - wrapper', 'Plastic container', 'Pop tab', 'Straw',
            'Styrofoam piece', 'Unlabeled litter'
        ]
        self.ie = Core()
        self.model_path = "best.xml"
        self.weights_path = "best.bin"
        self.compiled_model = self.ie.compile_model(
            self.model_path,
            "CPU",
            config={properties.inference_num_threads(): 4}
        )
        self.input_layer = self.compiled_model.input(0)
        self.output_layer = self.compiled_model.output(0)

    def __format_to_square(self,img, size=640):
        h, w = img.shape[:2]
        scale = min(size / w, size / h)

        new_w, new_h = int(w * scale), int(h * scale)
        resized = cv2.resize(img, (new_w, new_h))
        canvas = np.zeros((size, size, 3), dtype=np.float32)
        pad_x, pad_y = (size - new_w) // 2, (size - new_h) // 2
        canvas[pad_y:pad_y+new_h, pad_x:pad_x+new_w] = resized
        # NCNN/OpenVINO expect [N,C,H,W]

        #canvas_uint8 = (canvas * 255.0).astype(np.uint8)
        #cv2.imwrite("canvas.jpg", canvas_uint8)
        #print(canvas)

        blob = np.expand_dims(np.transpose(canvas, (2,0,1)), axis=0)
        return blob.astype(np.float32), scale, pad_x, pad_y

    def __process_yolo_output(self,out, scale, pad_x, pad_y, orig_shape, score_threshold=0.45, nms_threshold=0.5):
        """
        out: [num_classes+4, num_detections] like YOLOv11 NCNN output
        Returns: list of [x, y, w, h, score, class_id]
        """ 
        boxes, confidences, class_ids = [], [], []
        out = np.squeeze(out, axis=0)  
        num_dets = out.shape[1]
        for i in range(num_dets):
            cx, cy, w, h = out[0:4, i]
            #print(cx, cy, w, h)
            class_scores = out[4:, i]
            cls_id = int(np.argmax(class_scores))
            conf = float(class_scores[cls_id])
            if conf < score_threshold:
                continue
            left   = int((cx - 0.5*w - pad_x) / scale)
            top    = int((cy - 0.5*h - pad_y) / scale)
            width  = int(w / scale)
            height = int(h / scale)

            boxes.append([left, top, width, height])
            confidences.append(conf)
            class_ids.append(cls_id)

        # Apply NMS
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.CONF_THRESH, self.NMS_THRESH)
        results = []
        for i in indices:
            x, y, w, h = boxes[i]
            results.append([x, y, w, h, confidences[i], class_ids[i]])
        return results

    def __draw_detections(self,img, detections):
        for (x, y, w, h, score, cls_id) in detections:
            label = f"{self.CLASSES[cls_id]} {score:.2f}"
            color = (0, 255, 0)
            cv2.rectangle(img, (x, y), (x+w, y+h), color, 2)
            cv2.putText(img, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return img

    def get_Results(self,img):
        img = np.array(img,dtype=np.float32) / 255.0
        input_blob, scale, pad_x, pad_y = self.__format_to_square(img)

        result = self.compiled_model([input_blob])[self.output_layer]

        predictions = self.__process_yolo_output(result, scale, pad_x, pad_y, img.shape)

        return predictions

    def visualize_detections(self,img):
        predictions = self.get_Results(img)

        drawn_img = self.__draw_detections(img,predictions)

        return drawn_img

    