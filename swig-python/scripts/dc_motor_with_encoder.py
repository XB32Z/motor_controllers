#!/usr/bin/env python
import time

from motor_controllers.bcm2835 import (
    BCM2835Interface,
    BCM2835BinaryChannelConfiguration,
    BCM2835PWMChannelConfiguration,
    ChannelMode_OUTPUT,
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
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Finishing program")
    finally:
        print("Cleaned up")
