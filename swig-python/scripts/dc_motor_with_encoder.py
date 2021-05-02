#!/usr/bin/env python
import time

from motor_controllers.bcm2835 import (
    BCM2835Interface,
    BCM2835BinaryChannelConfiguration,
    BCM2835PWMChannelConfiguration,
    BinarySignal_BINARY_HIGH,
    BinarySignal_BINARY_LOW,
    ChannelMode_OUTPUT,
    ChannelMode_EVENT_DETECT,
    EventDetectType_EVENT_BOTH_EDGES,
)
from motor_controllers.encoder import (
    Direction_BACKWARD,
    Direction_INVALID,
    Encoder,
)


if __name__ == "__main__":
    communication = BCM2835Interface()

    pwmConf = BCM2835PWMChannelConfiguration()
    pwmConf.pinNumber = 26
    pwmConf.channel = 0
    pwmConf.range = 1024

    pwm = communication.configureChannel(pwmConf)
    pwm.setPWMFrequency(11718.75)

    m1Conf = BCM2835BinaryChannelConfiguration()
    m1Conf.pinNumber = 20
    m1Conf.channelMode = ChannelMode_OUTPUT
    m1 = communication.configureChannel(m1Conf)

    m2Conf = BCM2835BinaryChannelConfiguration()
    m2Conf.pinNumber = 21
    m2Conf.channelMode = ChannelMode_OUTPUT
    m2 = communication.configureChannel(m2Conf)

    # Encoder
    p2Conf = BCM2835BinaryChannelConfiguration()
    p2Conf.pinNumber = 27
    p2Conf.channelMode = ChannelMode_EVENT_DETECT
    p2Conf.eventDetectValue = EventDetectType_EVENT_BOTH_EDGES
    p2 = communication.configureChannel(p2Conf)

    p3Conf = BCM2835BinaryChannelConfiguration()
    p3Conf.pinNumber = 22
    p3Conf.channelMode = ChannelMode_EVENT_DETECT
    p3Conf.eventDetectValue = EventDetectType_EVENT_BOTH_EDGES
    p3 = communication.configureChannel(p3Conf)

    # Create a quadrature encoder of resolution 13
    # encoder = Encoder(p2, p3, 13) TODO workaround the unique_ptr

    communication.start()
    encoder.start(50)

    m1.set(BinarySignal_BINARY_HIGH)
    m2.set(BinarySignal_BINARY_LOW)
    pwm.setDutyCycle(0.5)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Finishing program")
    finally:
        print("Cleaned up")
