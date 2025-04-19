#!/usr/bin/env python3
"""Example module for Hailo Detection + ByteTrack + Supervision."""


# import time
# from picamera2 import MappedArray, Picamera2, Preview
import supervision as sv
import numpy as np
import cv2
import os
import ctypes
from typing import Dict, List
# from picamera2.devices import Hailo
# from libcamera import controls
from threading import Lock
import logging
import argparse
import gi
import setproctitle
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import hailo

from hailo_apps_infra.hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
    get_default_parser,
    detect_hailo_arch,
)

from hailo_apps_infra.gstreamer_helper_pipelines import(
    QUEUE,
    SOURCE_PIPELINE,
    INFERENCE_PIPELINE,
    INFERENCE_PIPELINE_WRAPPER,
    TRACKER_PIPELINE,
    USER_CALLBACK_PIPELINE,
    DISPLAY_PIPELINE,
)
from hailo_apps_infra.gstreamer_app import GStreamerApp


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH")


# class DetectionEngine:
#     def __init__(self, model_path='/home/pi/Adeept_DarkPaw/own_code/models/yolov8m.hef',
#                  labels='/home/pi/Adeept_DarkPaw/own_code/models/coco.txt', score_thresh=0.5, max_detections=3):
#         self.lock = Lock()
#         self.results = None
#         self.color_palette = sv.ColorPalette.DEFAULT
#         self.font_scale = 0.5
#         self.font = cv2.FONT_HERSHEY_SIMPLEX
#         self.font_line_type = cv2.LINE_AA
#         self.tracker = sv.ByteTrack()
#         self.last_exec_time = time.time_ns()/1e6
#         self.model = Hailo(hef_path=model_path)
#         with open(labels, "r", encoding="utf-8") as f:
#             self.class_names: List[str] = f.read().splitlines()
#         self.threshold = score_thresh
#         self.max_detections = max_detections
#         self.model_h, self.model_w, _ = self.model.get_input_shape()
#         print(self.model.get_input_shape())
#         self.video_w, self.video_h = 800, 600
#         self.fps = 0
#         self.camera = Picamera2()
#         self.camera.set_controls({"AwbMode": controls.AwbModeEnum.Indoor})
#         self.camera_config = self.camera.create_video_configuration(main={'size': (self.video_w, self.video_h),
#                                                                           'format': 'XRGB8888'},
#                                                                     raw={'format': 'SGRBG10'},
#                                                                     controls={'FrameRate': 30})
#         self.camera.preview_configuration.align()
#         self.camera.configure(self.camera_config)
#         self.camera.pre_callback = self.postprocess_frames
#
    # def preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
    #     """Preprocess the frame to match the model's input size."""
    #     frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    #     if self.model_h != self.video_h or self.model_w != self.video_w:
    #         return cv2.resize(frame, (self.model_w, self.model_h), interpolation=cv2.INTER_AREA)
    #     return frame
    #
    # def extract_detections(self, hailo_output: List[np.ndarray]) -> Dict[str, np.ndarray]:
    #     """Extract detections from the HailoRT-postprocess output."""
    #     xyxy: List[np.ndarray] = []
    #     confidence: List[float] = []
    #     class_id: List[int] = []
    #     num_detections: int = 0
    #
    #     for i, detections in enumerate(hailo_output):
    #         if len(detections) == 0:
    #             continue
    #         for detection in detections:
    #             y0, x0, y1, x1 = detection[:4]
    #             score = detection[4]
    #
    #             if score < self.threshold:
    #                 continue
    #
    #             # Convert bbox to xyxy absolute pixel values
    #             bbox = np.array([int(x0 * self.video_w),
    #                             int(y0 * self.video_h),
    #                             int(x1 * self.video_w),
    #                             int(y1 * self.video_h)])
    #             xyxy.append(bbox)
    #             confidence.append(score)
    #             class_id.append(i)
    #             num_detections += 1
    #
    #     # get best max_detections values
    #     if len(confidence) > 1:
    #         conf_items = np.argsort(np.array(confidence))[::-1]
    #         cor_xyxy = []
    #         cor_confidence = []
    #         cor_class_id = []
    #         num_detections = self.max_detections
    #         for index in conf_items[:self.max_detections]:
    #             cor_xyxy.append(xyxy[index])
    #             cor_confidence.append(confidence[index])
    #             cor_class_id.append(class_id[index])
    #
    #     else:
    #         cor_xyxy = xyxy
    #         cor_confidence = confidence
    #         cor_class_id = class_id
    #
    #     return {
    #         "xyxy": np.array(cor_xyxy),
    #         "confidence": np.array(cor_confidence),
    #         "class_id": np.array(cor_class_id),
    #         "num_detections": num_detections,
    #     }
    #
    # def run_tracker_algorithm(
    #     self,
    #     detections: Dict[str, np.ndarray],
    # ) -> sv.Detections:
    #     """Postprocess the detections by annotating the frame with bounding boxes and labels."""
    #     if detections["xyxy"].shape[0] > 0:
    #         sv_detections = sv.Detections(
    #             xyxy=detections["xyxy"],
    #             confidence=detections["confidence"],
    #             class_id=detections["class_id"],
    #         )
    #     else:
    #         sv_detections = sv.Detections.empty()
    #
    #     # Update detections with tracking information
    #     sv_detections = self.tracker.update_with_detections(sv_detections)
    #     return sv_detections
    #
    # def run_inference(self):
    #     full_frame = self.camera.capture_array('main')
    #     eval_frame = self.preprocess_frame(full_frame)
    #     results = self.model.run(eval_frame)
    #     if len(results) == 1:
    #         results = results[0]
    #     detections = self.extract_detections(results)
    #     sv_detections = self.run_tracker_algorithm(detections)
    #     exec_time = time.time_ns() / 1e6
    #     fps = 0.9 * self.fps + 0.1 * 1000 / (exec_time - self.last_exec_time)
    #     with self.lock:
    #         self.results = sv_detections
    #         self.fps = fps
    #         self.last_exec_time = exec_time
    #
    # def get_results(self, as_dict=False):
    #     with self.lock:
    #         return_value = self.results
    #     if as_dict:
    #         return_dict = {}
    #         for class_id, tracker_id, confidence, bbox in zip(return_value.class_id, return_value.tracker_id,
    #                                                           return_value.confidence, return_value.xyxy):
    #             return_dict[tracker_id] = {'class': class_id,
    #                                        'conf': confidence,
    #                                        'bbox': bbox}
    #         return return_dict
    #     else:
    #         return return_value
    #
    # def postprocess_frames(self, request):
    #     sv_detections = self.get_results()
    #     with MappedArray(request, "main") as m:
    #         if sv_detections:
    #             for class_id, tracker_id, confidence, bbox in zip(sv_detections.class_id, sv_detections.tracker_id,
    #                                                               sv_detections.confidence, sv_detections.xyxy):
    #                 x0, y0, x1, y1 = bbox
    #
    #                 label = f"#{tracker_id} {self.class_names[class_id]} {(confidence * 100):.1f} %"
    #                 # logging.info(label)
    #                 color = self.color_palette.by_idx(tracker_id)
    #                 # draw bounding box
    #                 cv2.rectangle(m.array, (int(x0), int(y0)),
    #                               (int(x1), int(y1)),
    #                               color.as_bgr(), 2)
    #                 # get the width and height of the text box
    #                 (text_width, text_height) = cv2.getTextSize(label, self.font,
    #                                                             fontScale=self.font_scale,
    #                                                             thickness=self.font_line_type)[0]
    #                 # make the coords of the box with a small padding of two pixels
    #                 cv2.rectangle(m.array, (int(x0)-1, int(y0)),
    #                               (int(x0) + text_width, int(y0) - text_height - 4), color.as_bgr(), cv2.FILLED)
    #
    #                 cv2.putText(img=m.array,
    #                             text=label,
    #                             org=(int(x0) + 2, int(y0) - 6),
    #                             fontFace=self.font,
    #                             fontScale=self.font_scale,
    #                             color=(255, 255, 255), thickness=1,
    #                             lineType=self.font_line_type)
    #
    #         with self.lock:
    #             fps = self.fps
    #         cv2.putText(img=m.array,
    #                     text='FPS = {:04.1f}'.format(fps),
    #                     org=(self.video_w - 120, 20),
    #                     fontFace=self.font,
    #                     fontScale=self.font_scale,
    #                     color=(255, 255, 255),
    #                     thickness=1,
    #                     lineType=self.font_line_type)


class DetectionEngine_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.lock = Lock()
        self.results = None
        self.max_detections = 3
        self.color_palette = sv.ColorPalette.DEFAULT
        self.font_scale = 0.5
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_line_type = cv2.LINE_AA

    # def new_function(self):  # New function example
    #     return "The meaning of life is: "

    def get_results(self, as_dict=False):
        with self.lock:
            return_value = self.results
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


def app_callback(pad, info, user_data):
    # Get the GstBuffer from the probe info
    buffer = info.get_buffer()
    # Check if the buffer is valid
    if buffer is None:
        return Gst.PadProbeReturn.OK

    # Using the user_data to count the number of frames
    user_data.increment()
    string_to_print = f"Frame count: {user_data.get_count()}\n"

    # Get the caps from the pad
    form, width, height = get_caps_from_pad(pad)

    # If the user_data.use_frame is set to True, we can get the video frame from the buffer
    frame = None
    if form is not None and width is not None and height is not None:
        # Get video frame
        frame = get_numpy_from_buffer(buffer, form, width, height)

    # Get the detections from the buffer
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    # Parse the detections
    detection_count = 0
    results = {}
    for detection in detections:
        label = detection.get_label()
        bbox = detection.get_bbox()
        confidence = detection.get_confidence()
        # Get track ID
        track_id = 0
        track = detection.get_objects_typed(hailo.HAILO_UNIQUE_ID)
        if len(track) == 1:
            track_id = track[0].get_id()
        results[track_id] = {'class': detection.get_class_id(),
                             'conf': confidence,
                             'bbox': bbox}
        string_to_print += f"Detection: ID: {track_id} Label: {label} Confidence: {confidence:.2f}\n"

        # FIXME: Postprocessing works in general, but drawing the bounding boxes in this callback does not!
        # if user_data.use_frame:
        # Note: using imshow will not work here, as the callback function is not running in the main thread
        color = user_data.color_palette.by_idx(track_id)
        x0, y0, x1, y1 = bbox.xmin()*width, bbox.ymin()*height, bbox.xmax()*width, bbox.ymax()*height
        cv2.rectangle(frame, (int(x0), int(y0)),
                      (int(x1), int(y1)),
                      color.as_bgr(), 2)
        # get the width and height of the text box
        (text_width, text_height) = cv2.getTextSize(label, user_data.font,
                                                    fontScale=user_data.font_scale,
                                                    thickness=user_data.font_line_type)[0]
        # make the coords of the box with a small padding of two pixels
        cv2.rectangle(frame, (int(x0)-1, int(y0)),
                      (int(x0) + text_width, int(y0) - text_height - 4), color.as_bgr(), cv2.FILLED)

        cv2.putText(img=frame,
                    text=label,
                    org=(int(x0) + 2, int(y0) - 6),
                    fontFace=user_data.font,
                    fontScale=user_data.font_scale,
                    color=(255, 255, 255), thickness=1,
                    lineType=user_data.font_line_type)
        detection_count += 1
        if detection_count > user_data.max_detections:
            break

    # if user_data.use_frame:
    # Convert the frame to BGR
    # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    # user_data.set_frame(frame)

    with user_data.lock:
        user_data.results = results
    logging.info(string_to_print)

    # Write the modified frame back into the GStreamer buffer
    # Buffer Kopieren
    # writable_buffer = buffer.copy()
    # writable_buffer = gst_buffer_make_writable(writable_buffer)
    # Neuen Buffer inhalt mappen und mit neuem Frame beschreiben
    # success, map_info = writable_buffer.map(Gst.MapFlags.WRITE)
    # if success:
    #     try:
    #         # Konvertiere zu beschreibbarem Speicher (nur nötig, wenn buffer=... Probleme macht)
    #         writable_mem = bytearray(map_info.data)
    #         arr = np.ndarray((height, width, 3), dtype=np.uint8, buffer=writable_mem)
    #         modified_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    #         np.copyto(arr, modified_frame)
    #
    #         # Wenn du willst, kannst du writable_mem zurück in map_info.data schreiben
    #         # Nur nötig, wenn du mit der Original-Speicheradresse weiterarbeiten willst
    #         map_info.data = writable_mem
    #     finally:
    #         writable_buffer.unmap(map_info)
    def modify_buffer(buf):
        size = buf.get_size()
        new_buffer = Gst.Buffer.new_allocate(None, size, None)

        # Originaldaten kopieren
        success, original_map = buf.map(Gst.MapFlags.READ)
        success2, new_map = new_buffer.map(Gst.MapFlags.WRITE)

        writable_mem = bytearray(original_map.data)
        arr = np.ndarray((height, width, 3), dtype=np.uint8, buffer=writable_mem)
        modified_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        np.copyto(arr, modified_frame)
        ptr = ctypes.cast(new_map.data, ctypes.POINTER(ctypes.c_uint8))
        for i in range(new_map.size):
            ptr[i] = writable_mem[i]

        buffer.unmap(original_map)
        new_buffer.unmap(new_map)
        return new_buffer
    #
    # Buffer Probe Info mit dem geänderten Buffer erstellen!
    def pad_probe_callback(pad, info):
        buffer = info.get_buffer()
        if not buffer:
            return Gst.PadProbeReturn.OK

        new_buffer = modify_buffer(buffer)

        # Jetzt ersetzen wir den Buffer im Probe-Info Objekt
        info.set_data('buffer', new_buffer)
        return Gst.PadProbeReturn.OK

    pad.add_probe(Gst.PadProbeType.BUFFER, pad_probe_callback)

    # new_info = Gst.PadProbeInfo.new_buffer(new_buffer)
    # # Ursprünglichen Buffer durch neuen ersetzen!
    # if new_info.type & Gst.PadProbeType.BUFFER:
    #     Gst.PadProbeReturn.OK_Drop
    # else:
    #     Gst.PadProbeReturn.OK
    # success, map_info = buffer.map(Gst.MapFlags.WRITE)
    # if not success:
    #     raise RuntimeError("Failed to map buffer for writing")
    #
    # try:
    #     # Convert the frame to RGB (GStreamer format) and copy it back to the buffer
    #     modified_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    #     np.copyto(np.ndarray(shape=(height, width, 3), dtype=np.uint8, buffer=map_info.data), modified_frame)
    # finally:
    #     buffer.unmap(map_info)

    return Gst.PadProbeReturn.OK


def gst_buffer_make_writable(buffer: Gst.Buffer) -> Gst.Buffer:
    writable_buffer = Gst.Buffer.new()
    writable_buffer.copy_into(
        buffer,
        Gst.BufferCopyFlags.FLAGS
        | Gst.BufferCopyFlags.TIMESTAMPS
        | Gst.BufferCopyFlags.META
        | Gst.BufferCopyFlags.MEMORY,  # copies memory as reference
        0,
        18446744073709551615,  # fancy -1
    )
    return writable_buffer


def STREAM_PIPELINE(show_fps='true', name='hailo_display'):
    """
    Creates a GStreamer pipeline string for streaming the video.
    # It includes the hailooverlay plugin to draw bounding boxes and labels on the video.

    Args:
        show_fps (str, optional): Whether to show the FPS on the video sink. Should be 'true' or 'false'. Defaults to 'false'.
        name (str, optional): The prefix name for the pipeline elements. Defaults to 'hailo_display'.

    Returns:
        str: A string representing the GStreamer pipeline for displaying the video.
    """
    # Construct the stream pipeline string
    stream_pipeline = (
        # f'{OVERLAY_PIPELINE(name=f"{name}_overlay")} ! ' overlay is done in custom app callback!
        # f'{QUEUE(name=f"{name}_videoconvert_q")} ! '
        f'videoconvert name={name}_videoconvert n-threads=2 qos=false ! '
        f'{QUEUE(name=f"{name}_q")} ! '
        f'videoconvert ! openh264enc deadline=1 ! rtpvp8pay ! webrtcbin name=sendrecv'  # ChatGPT Antwort. Als encoder schlägt CGT vp8enc statt openh264enc vor
        # f'videoconvert ! openh264enc ! mpegtsmux ! rtpmp2tpay ! udpsink host=0.0.0.0 port=4665 text-overlay={show_fps} signal-fps-measurements=true ' # aus dem web
        # f'fpsdisplaysink name={name} video-sink={video_sink} sync={sync} text-overlay={show_fps} signal-fps-measurements=true ' # original display pipeline
    )

    return stream_pipeline


# -----------------------------------------------------------------------------------------------
# User Gstreamer Application
# -----------------------------------------------------------------------------------------------

# This class inherits from the hailo_rpi_common.GStreamerApp class
class GStreamerDetectionApp(GStreamerApp):
    def __init__(self, app_callback, user_data, parser=None, Use_local_display=False):
        if parser is None:
            parser = get_default_parser()
        parser.add_argument(
            "--labels-json",
            default=None,
            help="Path to costume labels JSON file",
        )
        # Call the parent class constructor
        super().__init__(parser, user_data)
        # Additional initialization code can be added here
        # Set Hailo parameters these parameters should be set based on the model used
        self.batch_size = self.options_menu.batch_size
        self.local_display = Use_local_display

        # Determine the architecture if not specified
        if self.options_menu.arch is None:
            detected_arch = detect_hailo_arch()
            if detected_arch is None:
                raise ValueError("Could not auto-detect Hailo architecture. Please specify --arch manually.")
            self.arch = detected_arch
            print(f"Auto-detected Hailo architecture: {self.arch}")
        else:
            self.arch = self.options_menu.arch

        self.hef_path = self.options_menu.hef_path

        # Set the post-processing shared object file
        self.post_process_so = os.path.join(self.current_path, '../resources/libyolo_hailortpp_postprocess.so')
        self.post_function_name = "filter_letterbox"
        # User-defined label JSON file
        self.labels_json = self.options_menu.labels_json

        self.app_callback = app_callback

        self.thresholds_str = (
            f"nms-score-threshold={self.options_menu.score_threshold} "
            f"nms-iou-threshold={self.options_menu.iou_threshold} "
            f"output-format-type=HAILO_FORMAT_TYPE_FLOAT32"
        )

        # Set the process title
        setproctitle.setproctitle("Hailo Detection App")

        self.create_pipeline()
        identity = self.pipeline.get_by_name("identity_callback")
        if identity:
            identity_pad = identity.get_static_pad("src")
            identity_pad.add_probe(Gst.PadProbeType.BUFFER, app_callback, user_data)

    def get_pipeline_string(self):
        source_pipeline = SOURCE_PIPELINE(self.video_source, self.video_width, self.video_height)
        detection_pipeline = INFERENCE_PIPELINE(
            hef_path=self.hef_path,
            post_process_so=self.post_process_so,
            post_function_name=self.post_function_name,
            batch_size=self.batch_size,
            config_json=self.labels_json,
            additional_params=self.thresholds_str)
        detection_pipeline_wrapper = INFERENCE_PIPELINE_WRAPPER(detection_pipeline)
        tracker_pipeline = TRACKER_PIPELINE(class_id=-1)
        user_callback_pipeline = USER_CALLBACK_PIPELINE()
        if self.local_display:
            display_pipeline = DISPLAY_PIPELINE(video_sink=self.video_sink, sync=self.sync, show_fps=self.show_fps)
        else:
            display_pipeline = STREAM_PIPELINE()

        pipeline_string = (
            f'{source_pipeline} ! '
            f'{detection_pipeline_wrapper} ! '
            f'{tracker_pipeline} ! '
            f'{user_callback_pipeline} ! '
            f'{display_pipeline}'
        )
        print(pipeline_string)
        return pipeline_string

    def get_pipeline_reference(self):
        return self.pipeline


def main() -> None:
    """Main function to run the video processing."""

    # detector = DetectionEngine(model_path='/home/pi/Adeept_DarkPaw/own_code/models/yolov10b.hef',
    #                           score_thresh=0.65,
    #                           max_detections=3)
    # detector.camera.start_preview(Preview.QTGL, x=0, y=0, width=detector.video_w, height=detector.video_h)
    # detector.camera.start()
    # time.sleep(1)

    # detector.camera.pre_callback = detector.postprocess_frames
    # while True:
    #     detector.run_inference()
    # Create an instance of the user app callback class
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", "-i", type=str, default="rpi")
    parser.add_argument("--use-frame", "-u", action="store_true", default=True,
                        help="Use frame from the callback function")
    parser.add_argument("--show-fps", "-f", action="store_true", default=True, help="Print FPS on sink")
    parser.add_argument("--hef-path", default= '/home/pi/Adeept_DarkPaw/own_code/hailo-rpi5-examples/resources/yolov11s_h8l.hef',
                        # default='/home/pi/Adeept_DarkPaw/own_code/models/yolov10b.hef',
                        help="Path to HEF file")
    parser.add_argument("--score_threshold",  type=float, default=0.7)
    parser.add_argument("--iou_threshold",  type=float, default=0.65)
    parser.add_argument("--batch_size", type=int, default=2)
    parser.add_argument("--disable-sync", default=True, action="store_true",
                        help="Disables display sink sync, will run as fast as possible. Relevant when using file source.")
    parser.add_argument(
        "--disable-callback", default=False,  action="store_true",
        help="Disables the user's custom callback function in the pipeline. Use this option to run the pipeline without invoking the callback logic."
    )
    parser.add_argument("--dump-dot", default=False, action="store_true", help="Dump the pipeline graph to a dot file pipeline.dot")
    parser.add_argument(
        "--arch",
        default=None,
        choices=['hailo8', 'hailo8l'],
        help="Specify the Hailo architecture (hailo8 or hailo8l). Default is None , app will run check.",
    )
    user_data = DetectionEngine_class()
    app = GStreamerDetectionApp(app_callback, user_data, parser, Use_local_display=True)
    app.run()


if __name__ == "__main__":
    main()
