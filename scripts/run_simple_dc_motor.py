#!/usr/bin/env python
import argparse
import keyboard
import RPi.GPIO as GPIO
import time
from typing import Callable, Dict


class PWMController(object):
    def __init__(self, m1: int, m2: int, m3: int = -1, m4: int = -1):
        """Create a PWM controller on the GPIOs.

        :param m1:
        :param m2:
        :param m3:
        :param m4:
        """
        self._m1: int = m1
        self._m2: int = m2
        self._m3: int = m3
        self._m4: int = m4

        self._pwm = 50

        PIN = 18
        D1 = 12
        D2 = 26

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(PIN, GPIO.IN, GPIO.PUD_UP)

        GPIO.setup(args.M1, GPIO.OUT)
        GPIO.setup(args.M2, GPIO.OUT)
        GPIO.setup(args.M3, GPIO.OUT)
        GPIO.setup(args.M4, GPIO.OUT)

        self._motor1 = self._create_motor(D1)
        self._motor2 = self._create_motor(D2)

    def _create_motor(self, pin: int):
        GPIO.setup(pin, GPIO.OUT)
        motor = GPIO.PWM(pin, 500)
        motor.start(50)
        return motor

    def _set_motor(self, a1: int, a2: int, b1: int, b2: int):
        GPIO.output(self._m1, a1)
        GPIO.output(self._m2, a2)
        GPIO.output(self._m3, b1)
        GPIO.output(self._m4, b2)

    def forward(self):
        self._set_motor(1, 0, 1, 0)

    def stop(self):
        self._set_motor(0, 0, 0, 0)

    def reverse(self):
        self._set_motor(0, 1, 0, 1)

    def left(self):
        self._set_motor(1, 0, 0, 0)

    def right(self):
        self._set_motor(0, 0, 1, 0)

    def cleanup(self):
        GPIO.cleanup()

    def increase_speed(self):
        if self._pwm + 10 < 101:
            self._pwm += 10
            self._motor1.ChangeDutyCycle(self._pwm)
            self._motor2.ChangeDutyCycle(self._pwm)

    def decrease_speed(self):
        if self._pwm - 10 > -1:
            self._pwm -= 10
            self._motor1.ChangeDutyCycle(self._pwm)
            self._motor2.ChangeDutyCycle(self._pwm)


if __name__ == "__main__":

    arg_parser = argparse.ArgumentParser(
        description="Program to test DC motors using GPIOs"
    )
    arg_parser.add_argument("-M1", help="PWM signal for M1", type=int, default=6)
    arg_parser.add_argument("-M2", help="PWM signal for M2", type=int, default=13)
    arg_parser.add_argument("-M3", help="PWM signal for M3", type=int, default=20)
    arg_parser.add_argument("-M4", help="PWM signal for M4", type=int, default=21)

    args, _ = arg_parser.parse_known_args()

    controller = PWMController(args.m1, args.m2, args.m3, args.m4)

    mappings: Dict[str, Callable[[], None]] = {
        "down": controller.reverse,
        "up": controller.forward,
        "left": controller.left,
        "right": controller.right,
        "space": controller.stop,
    }

    def on_keyboard_pressed(e):
        pressed_key = keyboard.normalize_name(e.name)
        try:
            mappings[pressed_key]()
        except KeyError:
            pass

    keyboard.on_press(on_keyboard_pressed)

    print("Starting")
    controller.stop()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Finishing program")
    finally:
        controller.cleanup()
        print("Cleaned up")
