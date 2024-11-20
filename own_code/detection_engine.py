#!/usr/bin/env python3
"""Example module for Hailo Detection + ByteTrack + Supervision."""

import argparse
import base64
# import gevent
import time
from picamera2 import MappedArray, Picamera2, Preview
import supervision as sv
import numpy as np
import cv2
import queue
import sys
import os, random
from typing import Dict, List, Tuple
import threading
from hailo_utils import HailoAsyncInference


# def initialize_arg_parser() -> argparse.ArgumentParser:
#     """Initialize argument parser for the script."""
#     parser = argparse.ArgumentParser(
#         description="Detection Example - Tracker with ByteTrack and Supervision"
#     )
#     parser.add_argument(
#         "-n", "--net", help="Path for the HEF model.", default="yolov5m_wo_spp_60p.hef"
#     )
#     parser.add_argument(
#         "-i", "--input_video", default="input_video.mp4", help="Path to the input video."
#     )
#     parser.add_argument(
#         "-o", "--output_video", default="output_video.mp4", help="Path to the output video."
#     )
#     parser.add_argument(
#         "-l", "--labels", default="coco.txt", help="Path to a text file containing labels."
#     )
#     parser.add_argument(
#         "-s", "--score_thresh", type=float, default=0.5, help="Score threshold - between 0 and 1."
#     )
#     return parser


class DetectionEngine:
    def __init__(self, model_path='/models/yolov8m.hef', labels='/models/coco.txt', score_thresh=0.5):
        self.input_queue = queue.Queue()
        self.output_queue = queue.Queue()
        self.box_annotator = sv.RoundBoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()
        self.tracker = sv.ByteTrack()
        self.last_exec_time = time.time_ns()/1e6
        self.hailo_inference = HailoAsyncInference(
            hef_path=model_path,
            input_queue=self.input_queue,
            output_queue=self.output_queue,
        )
        with open(labels, "r", encoding="utf-8") as f:
            self. class_names: List[str] = f.read().splitlines()

        self.thresh = score_thresh
        self.model_h, self.model_w, _ = self.hailo_inference.get_input_shape()
        self.video_w, self.video_h = 1280, 960
        self.camera = Picamera2()
        self.camera_config = self.camera.create_preview_configuration(main={'size': (self.video_w, self.video_h), 'format': 'XRGB8888'},
                                                     lores={'size': (self.model_w, self.model_h), 'format': 'RGB888'},
                                                     controls={'FrameRate': 30})
        self.camera.configure(self.camera_config)
        self.camera.start()
        time.sleep(1)

    def preprocess_frame(self, frame: np.ndarray, video_h: int, video_w: int) -> np.ndarray:
        """Preprocess the frame to match the model's input size."""
        if self.model_h != video_h or self.model_w != video_w:
            return cv2.resize(frame, (self.model_w, self.model_h))
        return frame

    def extract_detections(self, hailo_output: List[np.ndarray], h: int, w: int, threshold: float = 0.5) -> Dict[str, np.ndarray]:
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

                if score < threshold:
                    continue

                # Convert bbox to xyxy absolute pixel values
                bbox[0], bbox[1], bbox[2], bbox[3] = (
                    bbox[1] * w,
                    bbox[0] * h,
                    bbox[3] * w,
                    bbox[2] * h,
                )

                xyxy.append(bbox)
                confidence.append(score)
                class_id.append(i)
                num_detections += 1

        return {
            "xyxy": np.array(xyxy),
            "confidence": np.array(confidence),
            "class_id": np.array(class_id),
            "num_detections": num_detections,
        }

    def postprocess_detections(
        self,
        frame: np.ndarray,
        detections: Dict[str, np.ndarray],
        class_names: List[str],
    ) -> np.ndarray:
        """Postprocess the detections by annotating the frame with bounding boxes and labels."""
        sv_detections = sv.Detections(
            xyxy=detections["xyxy"],
            confidence=detections["confidence"],
            class_id=detections["class_id"],
        )

        # Update detections with tracking information
        sv_detections = self.tracker.update_with_detections(sv_detections)

        # Generate tracked labels for annotated objects
        labels: List[str] = [
            f"#{tracker_id} {class_names[class_id]}"
            for class_id, tracker_id in zip(sv_detections.class_id, sv_detections.tracker_id)
        ]

        # Annotate objects with bounding boxes
        annotated_frame: np.ndarray = self.box_annotator.annotate(
            scene=frame.copy(), detections=sv_detections
        )
        # Annotate objects with labels
        annotated_labeled_frame: np.ndarray = self.label_annotator.annotate(
            scene=annotated_frame, detections=sv_detections, labels=labels
        )
        return annotated_labeled_frame

    def capture_frames(self):
        full_frame = self.camera.get_frame("main")
        eval_frame = self.camera.switch_mode_and_capture_array(self.camera_config, "lores")
        """Capture frames from the default camera and emit them to clients."""
        return full_frame, eval_frame

    # TODO: work on evaluation function!
    def run_inference(self, result_queue):



        while True:
            # try:
            #     # ret, frame = cap.read()
            #     # Encode the frame as JPEG
            #     flag, buffer = cv2.imencode('.jpg', frame)
            # except:
            folder_dir = "E:/Wallpapers"
            im_name = random.choice(os.listdir(folder_dir))  # change dir name to whatever
            if not '.jpg' in im_name:
                continue
            flag, buffer = cv2.imencode('.jpg',
                                        cv2.resize(cv2.imread(os.path.join(folder_dir, im_name)), (800, 600)))
            # print("Error: Failed to capture frame.")
            # break

            if not flag:
                continue
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            socketio.emit('frame', jpg_as_text)
            now_time = time.time_ns() / 1e6
            FPS = 1000/(now_time - self.last_exec_time)
            print(FPS)
            self.last_exec_time = now_time
            # socketio.sleep(1e-3)

            # Emit the encoded frame to all connected clients


        # cap.release()

def main() -> None:
    """Main function to run the video processing."""
    # Parse command-line arguments
    args = initialize_arg_parser().parse_args()


    # Initialize components for video processing
    #frame_generator = sv.get_video_frames_generator(source_path=args.input_video)
    #video_info = sv.VideoInfo.from_video_path(video_path=args.input_video)
    #video_w, video_h = video_info.resolution_wh

    # start, end = sv.Point(x=0, y=1080), sv.Point(x=3840, y=1080)
    # line_zone = sv.LineZone(start=start, end=end)

    # Load class names from the labels file


    # Start the asynchronous inference in a separate thread
    inference_thread: threading.Thread = threading.Thread(target=hailo_inference.run)
    inference_thread.start()

    # Initialize video sink for output
    with sv.VideoSink(target_path=args.output_video, video_info=video_info) as sink:
        # Process each frame in the video
        for frame in tqdm(frame_generator, total=video_info.total_frames):
            # Preprocess the frame
            preprocessed_frame: np.ndarray = preprocess_frame(
                frame, model_h, model_w, video_h, video_w
            )

            # Put the frame into the input queue for inference
            input_queue.put([preprocessed_frame])

            # Get the inference result from the output queue
            results: List[np.ndarray]
            _, results = output_queue.get()

            # Deals with the expanded results from hailort versions < 4.19.0
            if len(results) == 1:
                results = results[0]

            # Extract detections from the inference results
            detections: Dict[str, np.ndarray] = extract_detections(
                results, video_h, video_w, args.score_thresh
            )

            # Postprocess the detections and annotate the frame
            annotated_labeled_frame: np.ndarray = postprocess_detections(
                frame, detections, class_names, tracker, box_annotator, label_annotator
            )

            # Write annotated frame to output video
            sink.write_frame(frame=annotated_labeled_frame)

    # Signal the inference thread to stop and wait for it to finish
    input_queue.put(None)
    inference_thread.join()


if __name__ == "__main__":
    main()
