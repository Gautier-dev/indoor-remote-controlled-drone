#!/usr/bin/env python

# Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt 

FREQUENCY_PWM = 500
MAX_PWM = 2000
MIN_PWM = 1000
output_pins = {
    'JETSON_XAVIER': 18,
    'JETSON_NANO': 33,
    'JETSON_NX': 33,
    'CLARA_AGX_XAVIER': 18,
    'JETSON_TX2_NX': 32,
}
output_pin = output_pins.get(GPIO.model, None)
if output_pin is None:
    raise Exception('PWM not supported on this board')

def pwm_to_dutycycle(duration_value):
    return duration_value/1000000*FREQUENCY_PWM*100

# Pin Setup:
GPIO.setmode(GPIO.BOARD)
# set pin as an output pin with optional initial state of HIGH
GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
p = GPIO.PWM(output_pin, FREQUENCY_PWM)
print("output pwm pin {} set, type {}".format(output_pin, type(p)))
PwmValue = 1500

p.start(pwm_to_dutycycle(1500))

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("gimbal")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    global PwmValue
    global p
    payload = msg.payload.decode('UTF-8')
    if payload == "up":
        PwmValue += 50
        print("up")
    if payload == "down":
        PwmValue -= 50
        print("down")
    p.ChangeDutyCycle(pwm_to_dutycycle(PwmValue))
    print("Pwm Changed to {}, dutycycle {} ".format(PwmValue, (pwm_to_dutycycle(PwmValue))))

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

try:
    client.connect("192.168.0.185", 1883, 60)
except:
    print("Connection problems")


try:
    client.loop_forever()
finally:
    p.stop()
    GPIO.cleanup()

    
