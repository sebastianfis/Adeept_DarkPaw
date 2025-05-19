#!/usr/bin/env python3
"""Example module for Hailo Detection + ByteTrack + Supervision."""

import time
from picamera2 import MappedArray, Picamera2, Preview
import supervision as sv
import numpy as np
import cv2
import os
from typing import Dict, List
from multiprocessing import Process, SimpleQueue, Value
from picamera2.devices import Hailo
from libcamera import controls
from threading import Lock, Event
import logging
import signal

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH")
from flask import Flask, Response
import cv2


class DetectionEngine:
    def __init__(self, model_path='/home/pi/Adeept_DarkPaw/own_code/models/yolov8m.hef',
                 labels='/home/pi/Adeept_DarkPaw/own_code/models/coco.txt',
                 score_thresh=0.5,
                 max_detections=3,
                 video_w=800,
                 video_h=600,
                 master_fps=30):
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
        self.video_w, self.video_h = video_w, video_h
        self.master_fps = master_fps
        self.fps = master_fps
        self.detection_counter = 0
        self.detect_flag = True
        self.running = True

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

    def run_inference(self, frame):
        if self.detect_flag:
            eval_frame = self.preprocess_frame(frame)
            results = self.model.run(eval_frame)
            if len(results) == 1:
                results = results[0]
            detections = self.extract_detections(results)
            sv_detections = self.run_tracker_algorithm(detections)
            exec_time = time.time_ns() / 1e6
            fps = 0.9 * self.fps + 0.1 * 1000 / (exec_time - self.last_exec_time)
            with self.lock:
                self.results = sv_detections
                self.fps = fps
                self.last_exec_time = exec_time
            self.detect_flag = False
        factor = round(self.master_fps/self.fps)
        self.detection_counter += 1
        if self.detection_counter >= factor:
            self.detect_flag = True



    def stop(self):
        self.running = False

    def get_results(self, as_dict=False):
        with self.lock:
            return_value = self.results
        if return_value is None:
            return None
        if as_dict:
            return_dict = {}
            for class_id, tracker_id, confidence, bbox in zip(return_value.class_id, return_value.tracker_id,
                                                              return_value.confidence, return_value.xyxy):
                return_dict[tracker_id] = {'class': class_id,
                                           'conf': confidence,
                                           'bbox': bbox}
            return return_dict
        else:
            return return_value

    def postprocess_frames(self, input_frame):
        sv_detections = self.get_results()
        frame = input_frame
        if sv_detections:
            for class_id, tracker_id, confidence, bbox in zip(sv_detections.class_id, sv_detections.tracker_id,
                                                              sv_detections.confidence, sv_detections.xyxy):
                x0, y0, x1, y1 = bbox

                label = f"#{tracker_id} {self.class_names[class_id]} {(confidence * 100):.1f} %"
                # logging.info(label)
                color = self.color_palette.by_idx(tracker_id)
                # draw bounding box
                frame = cv2.rectangle(frame, (int(x0), int(y0)),
                                      (int(x1), int(y1)),
                                      color.as_bgr(), 2)
                # get the width and height of the text box
                (text_width, text_height) = cv2.getTextSize(label, self.font,
                                                            fontScale=self.font_scale,
                                                            thickness=self.font_line_type)[0]
                # make the coords of the box with a small padding of two pixels
                frame = cv2.rectangle(frame, (int(x0)-1, int(y0)),
                                      (int(x0) + text_width, int(y0) - text_height - 4),
                                      color.as_bgr(), cv2.FILLED)

                frame = cv2.putText(img=frame,
                                    text=label,
                                    org=(int(x0) + 2, int(y0) - 6),
                                    fontFace=self.font,
                                    fontScale=self.font_scale,
                                    color=(255, 255, 255), thickness=1,
                                    lineType=self.font_line_type)

        with self.lock:
            fps = self.fps
        frame = cv2.putText(img=frame,
                            text='FPS = {:04.1f}'.format(fps),
                            org=(self.video_w - 120, 20),
                            fontFace=self.font,
                            fontScale=self.font_scale,
                            color=(255, 255, 255),
                            thickness=1,
                            lineType=self.font_line_type)
        return frame


def detection_worker(frames_to_detect_queue: SimpleQueue,
                     frames_to_detect_size_counter: Value,
                     frames_with_detection_queue: SimpleQueue,
                     frames_with_detection_size_counter: Value,
                     detection_queue: SimpleQueue,
                     detection_size_counter: Value,
                     control_event: Event,
                     video_w=800,
                     video_h=600,
                     master_fps=30
                     ):
    detector = DetectionEngine(model_path='/home/pi/Adeept_DarkPaw/own_code/models/yolov11m.hef',
                               score_thresh=0.65,
                               max_detections=3,
                               video_w=video_w,
                               video_h=video_h,
                               master_fps=master_fps)
    while True:
        if control_event.is_set():
            break
        if not frames_to_detect_queue.empty():
            frame = frames_to_detect_queue.get()
            with frames_to_detect_size_counter.get_lock():
                frames_to_detect_size_counter.value -= 1
            detector.run_inference(frame)
            frame_with_detections = detector.postprocess_frames(frame)
            while frames_with_detection_size_counter.value > 5:
                logger.info('too many frames in q. dropping frame')
                frames_with_detection_queue.get()  # Drop the oldest frame to prevent queue backup
                with frames_with_detection_size_counter.get_lock():
                    frames_with_detection_size_counter.value -= 1
            frames_with_detection_queue.put(frame_with_detections)
            with frames_with_detection_size_counter.get_lock():
                frames_with_detection_size_counter.value += 1
        detections = detector.get_results(as_dict=True)
        if detection_size_counter.value > 2:
            detection_queue.get()  # Drop the oldest detections to prevent queue backup
            with detection_size_counter.get_lock():
                detection_size_counter.value -= 1
        detection_queue.put(detections)
        with detection_size_counter.get_lock():
            detection_size_counter.value += 1


def main() -> None:
    """Main function to run the video processing."""
    picam2 = Picamera2()  # Global camera instance
    video_w, video_h = 800, 600
    picam2.set_controls({"AwbMode": controls.AwbModeEnum.Indoor})
    camera_config = picam2.create_video_configuration(main={'size': (video_w, video_h),
                                                            'format': 'XRGB8888'},
                                                            controls={'FrameRate': 30})
    picam2.preview_configuration.align()
    picam2.configure(camera_config)
    outgoing_frame_queue = SimpleQueue()  # Queue(maxsize=5) # Keep it small to avoid latency
    outgoing_frame_size_counter = Value('i', 0)
    incoming_frame_queue = SimpleQueue()  # Keep it small to avoid latency
    incoming_frame_size_counter = Value('i', 0)
    detection_queue = SimpleQueue()  # Queue(maxsize=2)
    detection_size_counter = Value('i', 0)
    detection_stopped = Event()
    detection_stopped.clear()

    def feed_frame(request):
        frame = request.make_array("main")
        while outgoing_frame_size_counter.value > 5:
            outgoing_frame_queue.get()  # Drop the oldest frame to prevent queue backup
            with outgoing_frame_size_counter.get_lock():
                outgoing_frame_size_counter.value -= 1
        outgoing_frame_queue.put(frame)
        with outgoing_frame_size_counter.get_lock():
            outgoing_frame_size_counter.value += 1

    def cleanup(*args):
        print("Cleaning up and releasing camera...")
        try:
            picam2.stop()
            picam2.close()
        except Exception as e:
            print(f"Error while stopping camera: {e}")
            detection_stopped.set()
            detector_process.terminate()
            detector_process.join()

            cv2.destroyAllWindows()

    # Register signal handlers (Ctrl+C, SIGTERM)
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    try:
        picam2.pre_callback = feed_frame
        picam2.start()

        detector_process = Process(target=detection_worker, args=(outgoing_frame_queue,
                                                                  outgoing_frame_size_counter,
                                                                  incoming_frame_queue,
                                                                  incoming_frame_size_counter,
                                                                  detection_queue,
                                                                  detection_size_counter,
                                                                  detection_stopped
                                                                  ),
                                   kwargs={'video_w': video_w, 'video_h': video_h})
        detector_process.start()
        time.sleep(1)

        app = Flask(__name__)

        def generate_frames():
            while True:
                if not incoming_frame_queue.empty():
                    frame = incoming_frame_queue.get()
                    with incoming_frame_size_counter.get_lock():
                        incoming_frame_size_counter.value -= 1

                    _, buffer = cv2.imencode('.jpg', frame)
                    frame_bytes = buffer.tobytes()

                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

        @app.route('/video_feed')
        def video_feed():
            return Response(generate_frames(),
                            mimetype='multipart/x-mixed-replace; boundary=frame')

        app.run(host='0.0.0.0', port=4664)

    except Exception as e:
        print(f"Exception occurred: {e}")
        detection_stopped.set()
    finally:
        cleanup()


if __name__ == "__main__":
    main()
