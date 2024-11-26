#!/usr/bin/env python3
"""Example module for Hailo Detection + ByteTrack + Supervision."""


import time
from picamera2 import MappedArray, Picamera2, Preview
import supervision as sv
import numpy as np
import cv2
from typing import Dict, List
from picamera2.devices import Hailo
from libcamera import controls
from threading import Lock

import os
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH")


class DetectionEngine:
    def __init__(self, model_path='/home/pi/Adeept_DarkPaw/own_code/models/yolov8m.hef',
                 labels='/home/pi/Adeept_DarkPaw/own_code/models/coco.txt', score_thresh=0.5, max_detections=3):
        self.lock = Lock()
        self.results = None
        self.color_palette = sv.ColorPalette.DEFAULT
        self.font_scale = 0.5
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_line_type = cv2.LINE_AA
        self.tracker = sv.ByteTrack()
        self.last_exec_time = time.time_ns()/1e6
        self.model = Hailo(hef_path=model_path)
        with open(labels, "r", encoding="utf-8") as f:
            self.class_names: List[str] = f.read().splitlines()
        self.threshold = score_thresh
        self.max_detections = max_detections
        self.model_h, self.model_w, _ = self.model.get_input_shape()
        print(self.model.get_input_shape())
        self.video_w, self.video_h = 800, 600
        self.camera = Picamera2()
        self.camera.set_controls({"AwbMode": controls.AwbModeEnum.Indoor})
        self.camera_config = self.camera.create_preview_configuration(main={'size': (self.video_w, self.video_h),
                                                                            'format': 'XRGB8888'},
                                                                      raw={'format': 'SGRBG10'},
                                                                      controls={'FrameRate': 30})
        self.camera.preview_configuration.align()
        self.camera.configure(self.camera_config)

    def preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        """Preprocess the frame to match the model's input size."""
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        if self.model_h != self.video_h or self.model_w != self.video_w:
            return cv2.resize(frame, (self.model_w, self.model_h), interpolation=cv2.INTER_AREA)
        return frame

    def extract_detections(self, hailo_output: List[np.ndarray]) -> Dict[str, np.ndarray]:
        """Extract detections from the HailoRT-postprocess output."""
        xyxy: List[np.ndarray] = []
        confidence: List[float] = []
        class_id: List[int] = []
        num_detections: int = 0

        for i, detections in enumerate(hailo_output):
            if len(detections) == 0:
                continue
            for detection in detections:
                y0, x0, y1, x1 = detection[:4]
                score = detection[4]

                if score < self.threshold:
                    continue

                # Convert bbox to xyxy absolute pixel values
                bbox = np.array([int(x0 * self.video_w),
                                int(y0 * self.video_h),
                                int(x1 * self.video_w),
                                int(y1 * self.video_h)])
                xyxy.append(bbox)
                confidence.append(score)
                class_id.append(i)
                num_detections += 1

        # get best max_detections values
        if len(confidence) > 1:
            conf_items = np.argsort(np.array(confidence))[::-1]
            cor_xyxy = []
            cor_confidence = []
            cor_class_id = []
            num_detections = self.max_detections
            for index in conf_items[:self.max_detections]:
                cor_xyxy.append(xyxy[index])
                cor_confidence.append(confidence[index])
                cor_class_id.append(class_id[index])
                
        else:
            cor_xyxy = xyxy
            cor_confidence = confidence
            cor_class_id = class_id

        return {
            "xyxy": np.array(cor_xyxy),
            "confidence": np.array(cor_confidence),
            "class_id": np.array(cor_class_id),
            "num_detections": num_detections,
        }

    def run_tracker_algorithm(
        self,
        detections: Dict[str, np.ndarray],
    ) -> sv.Detections:
        """Postprocess the detections by annotating the frame with bounding boxes and labels."""
        if detections["xyxy"].shape[0] > 0:
            sv_detections = sv.Detections(
                xyxy=detections["xyxy"],
                confidence=detections["confidence"],
                class_id=detections["class_id"],
            )
        else:
            sv_detections = sv.Detections.empty()

        # Update detections with tracking information
        sv_detections = self.tracker.update_with_detections(sv_detections)
        return sv_detections

    def run_inference(self):
        while True:
            full_frame = self.camera.capture_array('main')
            eval_frame = self.preprocess_frame(full_frame)
            results = self.model.run(eval_frame)
            if len(results) == 1:
                results = results[0]
            detections = self.extract_detections(results)
            sv_detections = self.run_tracker_algorithm(detections)
            with self.lock:
                self.results = sv_detections

    def get_results(self):
        with self.lock:
            return_value = self.results
        return return_value

    def postprocess_frames(self, request):
        # ToDo: Make annotations more beautiful: colour code different items!
        sv_detections = self.get_results()
        with MappedArray(request, "main") as m:
            if sv_detections:
                for class_id, tracker_id, confidence, bbox in zip(sv_detections.class_id, sv_detections.tracker_id,
                                                                  sv_detections.confidence, sv_detections.xyxy):
                    x0, y0, x1, y1 = bbox

                    label = f"#{tracker_id} {self.class_names[class_id]} {(confidence * 100):.1f} %"
                    color = self.color_palette.by_idx(tracker_id)
                    # draw bounding box
                    cv2.rectangle(m.array, (int(x0), int(y0)),
                                  (int(x1), int(y1)),
                                  color.as_bgr(), 2)
                    # get the width and height of the text box
                    (text_width, text_height) = cv2.getTextSize(label, self.font,
                                                                fontScale=self.font_scale,
                                                                thickness=self.font_line_type)[0]
                    # make the coords of the box with a small padding of two pixels
                    cv2.rectangle(m.array, (int(x0)-1, int(y0)),
                                  (int(x0) + text_width, int(y0) - text_height - 4), color.as_bgr(), cv2.FILLED)

                    cv2.putText(img=m.array,
                                text=label,
                                org=(int(x0) + 2, int(y0) - 6),
                                fontFace=self.font,
                                fontScale=self.font_scale,
                                color=(255, 255, 255), thickness=1,
                                lineType=self.font_line_type)
            exec_time = time.time_ns() / 1e6
            fps = 1000 / (exec_time - self.last_exec_time)
            self.last_exec_time = exec_time
            cv2.putText(img=m.array,
                        text='FPS = {:04.1f}'.format(fps),
                        org=(self.video_w - 120, 20),
                        fontFace=self.font,
                        fontScale=self.font_scale,
                        color=(255, 255, 255),
                        thickness=1,
                        lineType=self.font_line_type)


def main() -> None:
    """Main function to run the video processing."""
    # results_queue = Queue()

    detector = DetectionEngine(model_path='/home/pi/Adeept_DarkPaw/own_code/models/yolov10b.hef',
                               score_thresh=0.65,
                               max_detections=3)
    detector.camera.start_preview(Preview.QTGL, x=0, y=0, width=detector.video_w, height=detector.video_h)
    detector.camera.start()
    time.sleep(1)

    detector.camera.pre_callback = detector.postprocess_frames
    while True:
        detector.run_inference()


if __name__ == "__main__":
    main()
