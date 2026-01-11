import gpiod
import time

CHIP = "gpiochip0"
TRIG = 23
ECHO = 24
TIMEOUT = 1  # seconds (int, safest)

chip = gpiod.Chip(CHIP)

trig = chip.get_line(TRIG)
echo = chip.get_line(ECHO)

trig.request(consumer="hc_sr04", type=gpiod.LINE_REQ_DIR_OUT)
trig.set_value(0)

echo.request(consumer="hc_sr04", type=gpiod.LINE_REQ_EV_BOTH_EDGES)

print("Starting HC-SR04 test (Ctrl+C to stop)")

try:
    while True:
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

        pulse_ns = ev2.timestamp - ev1.timestamp
        pulse_s = pulse_ns * 1e-9
        distance = (pulse_s * 34300) / 2

        print(f"Distance: {distance:.1f} cm")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    trig.release()
    echo.release()
    chip.close()
