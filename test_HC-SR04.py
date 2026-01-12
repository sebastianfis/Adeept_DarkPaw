import gpiod
import time
from collections import deque

CHIP = "gpiochip0"
TRIG = 23
ECHO = 24
TIMEOUT = 1  # seconds (int, safest)
raw_history = deque(maxlen=5)  # median window
ema_value = None
ema_alpha = 0.3  # 0.2–0.4 is good

chip = gpiod.Chip(CHIP)

trig = chip.get_line(TRIG)
echo = chip.get_line(ECHO)

trig.request(consumer="hc_sr04", type=gpiod.LINE_REQ_DIR_OUT)
trig.set_value(0)

echo.request(consumer="hc_sr04", type=gpiod.LINE_REQ_EV_BOTH_EDGES)

print("Starting HC-SR04 test (Ctrl+C to stop)")

try:
    while True:
        while echo.event_wait(0):
            echo.event_read()  # flush old events

        # Trigger pulse
        trig.set_value(1)
        time.sleep(15e-6)
        trig.set_value(0)

        # Wait for rising edge
        if not echo.event_wait(TIMEOUT):
            print("Timeout waiting for rising edge")
            time.sleep(0.5)
            continue

        ev1 = echo.event_read()
        if ev1.type != gpiod.LineEvent.RISING_EDGE:
            print("Unexpected event:", ev1.type)
            continue

       # t_start = time.perf_counter()

        # Wait for falling edge
        if not echo.event_wait(TIMEOUT):
            print("Timeout waiting for falling edge")
            time.sleep(0.5)
            continue

        ev2 = echo.event_read()
        if ev2.type != gpiod.LineEvent.FALLING_EDGE:
            print("Unexpected event:", ev2.type)
            continue

        # t_end = time.perf_counter()
        pulse_start = ev1.sec + ev1.nsec * 1e-9
        pulse_end = ev2.sec + ev2.nsec * 1e-9
        duration = pulse_end - pulse_start

        # HC-SR04 physical limits
        if duration < 150e-6 or duration > 25e-3:
            continue

        distance = duration * 34300 / 2

        if 2 <= distance <= 400:
            # ---- Median filter ----
            raw_history.append(distance)
            if len(raw_history) < 3:
                smooth_distance = None  # wait until window filled

            median_dist = sorted(raw_history)[len(raw_history) // 2]

            # ---- EMA smoothing ----
            if ema_value is None:
                ema_value = median_dist
            else:
                ema_value = (
                        ema_alpha * median_dist +
                        (1 - ema_alpha) * ema_value
                )

            smooth_distance = ema_value
        else:
            smooth_distance = None  # Value out of range

        print(f"Distance: {smooth_distance:.1f} cm")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    trig.release()
    echo.release()
    chip.close()
