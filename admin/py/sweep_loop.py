import time
import math
import random
from dynamixel_sdk import *

DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 57600
PROTOCOL = 2.0
ADDR_TORQUE = 64
ADDR_GOAL = 116
ADDR_PROFILE_VEL = 112
ADDR_PROFILE_ACC = 108

PAN_ID = 1
TILT_ID = 2
PAN_CENTRE = 1295
TILT_CENTRE = 845
PAN_RANGE = 250
TILT_RANGE = 200

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

for sid in (PAN_ID, TILT_ID):
    packetHandler.write1ByteTxRx(portHandler, sid, ADDR_TORQUE, 1)
    packetHandler.write4ByteTxRx(portHandler, sid, ADDR_PROFILE_VEL, 0)
    packetHandler.write4ByteTxRx(portHandler, sid, ADDR_PROFILE_ACC, 30)

pan = PAN_CENTRE
tilt = TILT_CENTRE
packetHandler.write4ByteTxRx(portHandler, PAN_ID, ADDR_GOAL, pan)
packetHandler.write4ByteTxRx(portHandler, TILT_ID, ADDR_GOAL, tilt)
time.sleep(1)

try:
    while True:
        # Pick random target
        target_pan  = PAN_CENTRE  + random.uniform(-PAN_RANGE,  PAN_RANGE)
        target_tilt = TILT_CENTRE + random.uniform(-TILT_RANGE, TILT_RANGE)

        # Sweep to it over 2 seconds — exactly like the sweep test
        duration = 2.0
        rate = 50
        steps = int(duration * rate)

        start_pan  = pan
        start_tilt = tilt

        for i in range(steps):
            progress = i / steps
            pan  = start_pan  + (target_pan  - start_pan)  * progress
            tilt = start_tilt + (target_tilt - start_tilt) * progress
            packetHandler.write4ByteTxRx(portHandler, PAN_ID,  ADDR_GOAL, int(pan))
            packetHandler.write4ByteTxRx(portHandler, TILT_ID, ADDR_GOAL, int(tilt))
            time.sleep(1.0 / rate)

        pan = target_pan
        tilt = target_tilt

except KeyboardInterrupt:
    pass
finally:
    packetHandler.write1ByteTxRx(portHandler, PAN_ID, ADDR_TORQUE, 0)
    packetHandler.write1ByteTxRx(portHandler, TILT_ID, ADDR_TORQUE, 0)
    portHandler.closePort()
    print("\nDone")