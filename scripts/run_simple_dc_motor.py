#!/usr/bin/env python
import argparse
import enum
import keyboard
import RPi.GPIO as GPIO
import time
from typing import Callable, Dict, Optional, List


class PWMController(object):
    class Direction(enum.IntEnum):
        STOP = 0
        FORWARD = 1
        BACKWARD = 2

    def __init__(
        self, pwm_pin: int, direction_pin1: Optional[int], direction_pin2: Optional[int]
    ) -> None:
        """Create a PWMController with direction

        Args:
            pwm_pin: pin of the PWM channel.
            direction_pin1: first bit of the 2  bit channel to indicate direction.
            direction_pin2: second bit of the 2 bit channel to indicate direction.
        """
        self._direction_pin1: int = direction_pin1
        self._direction_pin2: int = direction_pin2

        self._duty_cycle = 0

        GPIO.setup(self._direction_pin1, GPIO.OUT)
        GPIO.setup(self._direction_pin2, GPIO.OUT)

        GPIO.setup(pwm_pin, GPIO.OUT)
        self._motor = GPIO.PWM(pwm_pin, 500)  # default frequency
        self._motor.start(self._duty_cycle)  # default duty cycle

    def set_frequency(self, freq: float) -> None:
        """Set frequency in Hz"""
        self._motor.ChangeFrequency(freq)

    def _set_direction(self, direction: Direction) -> None:
        if direction.value > 3:
            raise RuntimeError("Can't set this direction")
        GPIO.output(self._direction_pin1, (direction.value >> 0) & 1)
        GPIO.output(self._direction_pin2, (direction.value >> 0) & 1)

    def stop(self) -> None:
        self._motor.ChangeDutyCycle(0)
        self._set_direction(self.Direction.STOP)

    def forward(self) -> None:
        self._set_direction(self.Direction.FORWARD)

    def reverse(self) -> None:
        self._set_direction(self.Direction.BACKWARD)

    def increase_speed(self) -> None:
        if self._duty_cycle + 10 <= 100:
            self._duty_cycle += 10
            self._motor.ChangeDutyCycle(self._duty_cycle)

    def decrease_speed(self) -> None:
        if self._duty_cycle - 10 >= 0:
            self._duty_cycle -= 10
            self._motor.ChangeDutyCycle(self._duty_cycle)


class MotorBuilder(object):
    def __init__(self, on_register_controller: Callable[[PWMController], None]) -> None:
        self.__on_register_controller: Callable[
            [PWMController], None
        ] = on_register_controller

    def create_motor(self, direction_pin1: int, direction_pin2: int, pwm_pin: int):
        controller = PWMController(direction_pin1, direction_pin2, pwm_pin)
        self.__on_register_controller(controller)
        return controller


class GPIOCommunication(object):
    """ Wrapper for communication with the GPIOs"""

    def __init__(self) -> None:
        self._registered_controllers: List[PWMController] = list()

    def _register_controller(self, controller: PWMController) -> None:
        self._registered_controllers.append(controller)

    def connect(self, pin: int) -> "GPIOCommunication":
        GPIO.setup(pin, GPIO.IN, GPIO.PUD_UP)
        return self

    def __enter__(self) -> MotorBuilder:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        return MotorBuilder(self._register_controller)

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        for pwm_controller in self._registered_controllers:
            pwm_controller.stop()
        GPIO.cleanup()


if __name__ == "__main__":

    arg_parser = argparse.ArgumentParser(
        description="Program to test DC motors using GPIOs"
    )
    arg_parser.add_argument(
        "-m1", help="Direction signal 1 for M1", type=int, default=20
    )
    arg_parser.add_argument(
        "-m2", help="Direction signal 2 for M1", type=int, default=21
    )
    arg_parser.add_argument("-pwma", help="PWM signal for M1", type=int, default=26)

    arg_parser.add_argument(
        "-m3", help="Direction signal 1 for M2", type=int, default=6
    )
    arg_parser.add_argument(
        "-m4", help="Direction signal 2 for M2", type=int, default=13
    )
    arg_parser.add_argument("-pwmb", help="PWM signal for M2", type=int, default=12)

    args, _ = arg_parser.parse_known_args()

    communication = GPIOCommunication()

    with communication.connect(18) as motor_builder:
        m1 = motor_builder.create_motor(args.pwma, args.m1, args.m2)
        m2 = motor_builder.create_motor(args.pwmb, args.m3, args.m4)

        mappings: Dict[str, Callable[[], None]] = {
            "down": m1.reverse,
            "up": m1.forward,
            "left": m2.reverse,
            "right": m2.forward,
        }

        def on_keyboard_pressed(e):
            pressed_key = keyboard.normalize_name(e.name)
            try:
                mappings[pressed_key]()
            except KeyError:
                pass

        keyboard.on_press(on_keyboard_pressed)

        print("Starting")

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Finishing program")
        finally:
            print("Cleaned up")
