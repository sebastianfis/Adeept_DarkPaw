#!/usr/bin/env python3
"""Example module for Hailo Detection + ByteTrack + Supervision."""


import time
from picamera2 import MappedArray, Picamera2, Preview
import supervision as sv
import numpy as np
import cv2
from queue import Queue
from typing import Dict, List
import threading
from picamera2.devices import Hailo
from libcamera import controls
import os

os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH")


class DetectionEngine:
    def __init__(self, model_path='/home/pi/Adeept_DarkPaw/own_code/models/yolov8m.hef',
                 labels='/home/pi/Adeept_DarkPaw/own_code/models/coco.txt', score_thresh=0.5, max_detections=3):

        self.results = None
        self.box_annotator = sv.RoundBoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()
        self.tracker = sv.ByteTrack()
        self.last_exec_time = time.time_ns()/1e6
        self.model = Hailo(hef_path=model_path)
        with open(labels, "r", encoding="utf-8") as f:
            self.class_names: List[str] = f.read().splitlines()
        self.threshold = score_thresh
        self.max_detections = max_detections
        self.model_h, self.model_w, _ = self.model.get_input_shape()
        print(self.model.get_input_shape())
        self.video_w, self.video_h = 1280, 960
        self.camera = Picamera2()
        self.camera.set_controls({"AwbMode": controls.AwbModeEnum.Indoor})
        self.camera_config = self.camera.create_preview_configuration(main={'size': (self.video_w, self.video_h),
                                                                            'format': 'XRGB8888'},
                                                                      lores={'size': (self.model_w, self.model_h),
                                                                             'format': 'RGB888'},
                                                                      raw={'format': 'SGRBG10'},
                                                                      controls={'FrameRate': 30})
        self.camera.configure(self.camera_config)
        # self.camera.start()
        # time.sleep(1)

    def preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        """Preprocess the frame to match the model's input size."""
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
                bbox, score = detection[:4], detection[4]

                if score < self.threshold:
                    continue

                # Convert bbox to xyxy absolute pixel values
                bbox[0], bbox[1], bbox[2], bbox[3] = (
                    int(bbox[1] * self.video_w),
                    int(bbox[0] * self.video_h),
                    int(bbox[3] * self.video_w),
                    int(bbox[2] * self.video_h),
                )

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

        # # Generate tracked labels for annotated objects
        # labels: List[str] = [
        #     f"#{tracker_id} {self.class_names[class_id]} {(confidence * 100):.1f} %"
        #     for class_id, tracker_id, confidence in zip(sv_detections.class_id, s
        #     v_detections.tracker_id, sv_detections.confidence)
        # ]
        #
        # # Annotate objects with bounding boxes
        # frame = self.box_annotator.annotate(
        #     scene=frame, detections=sv_detections
        # )
        # # Annotate objects with labels
        # frame = self.label_annotator.annotate(
        #     scene=frame, detections=sv_detections, labels=labels
        # )
        # exec_time = time.time_ns()/1e6
        # fps = 1000/(exec_time-self.last_exec_time)
        # self.last_exec_time = exec_time
        # cv2.putText(img=frame,
        #             text='FPS = {:04.1f}'.format(fps),
        #             org=(frame.shape[1] - 120, 20),
        #             fontFace=cv2.FONT_HERSHEY_SIMPLEX,
        #             fontScale=0.5,
        #             color=(255, 255, 255),
        #             thickness=1,
        #             lineType=cv2.LINE_AA)
        # return annotated_labeled_frame

    def run_inference(self):
        while True:
            # full_frame = self.camera.capture_array('main')
            eval_frame = self.camera.capture_array('lores')
            results = self.model.run(eval_frame)
            if len(results) == 1:
                results = results[0]
            detections = self.extract_detections(results)
            sv_detections = self.run_tracker_algorithm(detections)
            self.results = sv_detections

    def postprocess_frames(self, request):
        sv_detections = self.results
        print("detection!")
        if sv_detections:
            with MappedArray(request, "main") as m:
                # Generate tracked labels for annotated objects
                # labels: List[str] = [
                #     f"#{tracker_id} {self.class_names[class_id]} {(confidence * 100):.1f} %"
                #     for class_id, tracker_id, confidence in zip(sv_detections.class_id,
                #                                                 sv_detections.tracker_id,
                #                                                 sv_detections.confidence)]
                # Annotate objects with bounding boxes
                # m.array = self.box_annotator.annotate(
                #     scene=m.array, detections=sv_detections
                # )
                # # Annotate objects with labels
                # m.array = self.label_annotator.annotate(
                #     scene=m.array, detections=sv_detections, labels=labels
                # )
                for class_id, tracker_id, confidence, bbox in zip(sv_detections.class_id, sv_detections.tracker_id,
                                                                  sv_detections.confidence, sv_detections.xyxy):
                # for class_id, confidence, bbox in zip(sv_detections["class_id"], sv_detections["confidence"], sv_detections["xyxy"]):
                    x0, y0, x1, y1 = bbox
                    label = f"#{tracker_id} {self.class_names[class_id]} {(confidence * 100):.1f} %"
                    # label = f"#{self.class_names[class_id]} {(confidence * 100):.1f} %"
                    cv2.rectangle(m.array, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0, 0), 2)
                    cv2.putText(m.array, label, (int(x0) + 5, int(y0) + 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0, 0), 1, cv2.LINE_AA)
                exec_time = time.time_ns() / 1e6
                fps = 1000 / (exec_time - self.last_exec_time)
                self.last_exec_time = exec_time
                cv2.putText(img=m.array,
                            text='FPS = {:04.1f}'.format(fps),
                            org=(self.video_w - 120, 20),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.5,
                            color=(255, 255, 255),
                            thickness=1,
                            lineType=cv2.LINE_AA)


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
    # eval_thread = threading.Thread(target=detector.run_inference)
    # eval_thread.Daemon = True
    # eval_thread.start()

    # while True:
    #     if not results_queue.empty():
    #         result_frame = results_queue.get()
    #         cv2.imshow("Object detection", result_frame)
    #         cv2.waitKey(1)


if __name__ == "__main__":
    main()
