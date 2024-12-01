import logging
from AppServer import setup_webserver
from detection_engine import DetectionEngine
from queue import Queue
from threading import Thread, Event
from AdditionalEquipment import LED, DistSensor
from MotionControl import MotionController
import signal
import RPi.GPIO as GPIO

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

GPIO.setmode(GPIO.BCM)

# TODO: Add remote control and behaviour

class DefaultModeNetwork:
    def __init__(self):
        self.command_queue = Queue()
        self.dist_sensor = DistSensor()
        self.dist_sensor.enable_cont_meaurement()
        self.motion_controller = MotionController()
        self.mode = 'remote_controlled'

        # start up lighting
        self.led_instance = LED()
        self.lights_thread = Thread(target=self.led_instance.run_lights)
        self.lights_thread.start()

        # start up distance measurement
        self.dist_measure_thread = Thread(target=self.dist_sensor.measure_cont)
        self.dist_measure_thread.start()

        # start up detection engine incl. camera
        self.detector = DetectionEngine(model_path='/home/pi/Adeept_DarkPaw/own_code/models/yolov10b.hef',
                                        score_thresh=0.70,
                                        max_detections=3)
        self.keyboard_trigger = Event()

        # start up web interface
        self.stream, self.streamserver, self.webserver = setup_webserver(self.command_queue, self.detector.camera)

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, signal, frame):
        logging.info('Signal detected. Stopping threads.')
        self.keyboard_trigger.set()

    def run(self):
        self.led_instance.light_setter('all_good', breath=True)
        while not self.keyboard_trigger.is_set():
            self.detector.run_inference()
            abstand = self.dist_sensor.read_last_measurement()
            logging.info("Gemessene Entfernung = %.1f cm" % abstand)
            if not self.command_queue.empty() and self.mode == 'remote_controlled':
                command_str = self.command_queue.get()
                self.motion_controller.execute_command(command_str)

        # until some keyboard event is detected
        self.shutdown()

    def shutdown(self):
        # for triggering the shutdown procedure when a signal is detected
        # trigger shutdown procedure
        self.webserver.shutdown()
        self.stream.shutdown()
        self.detector.camera.stop()
        self.dist_sensor.disable_cont_meaurement()
        self.led_instance.shutdown()


        # and finalize shutting them down
        self.webserver.join()
        self.streamserver.join()
        self.dist_measure_thread.join()
        self.lights_thread.join()
        GPIO.cleanup()
        logging.info("Stopped all threads")


class Behaviour:
    def __init__(self, name: str):
        pass


if __name__ == '__main__':
    dmn = DefaultModeNetwork()
    dmn.run()



