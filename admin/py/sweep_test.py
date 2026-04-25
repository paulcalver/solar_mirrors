import time
from dynamixel_sdk import *

DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 57600
PROTOCOL = 2.0
ADDR_TORQUE = 64
ADDR_GOAL = 116
ADDR_PROFILE_VEL = 112
ADDR_PROFILE_ACC = 108
PAN_ID = 1

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

packetHandler.write1ByteTxRx(portHandler, PAN_ID, ADDR_TORQUE, 1)
packetHandler.write4ByteTxRx(portHandler, PAN_ID, ADDR_PROFILE_VEL, 0)
packetHandler.write4ByteTxRx(portHandler, PAN_ID, ADDR_PROFILE_ACC, 30)

# Sweep from 1000 to 1300 over 5 seconds at 50Hz
start = 1000
end = 1300
duration = 1.7
rate = 50
steps = int(duration * rate)

for i in range(steps):
    pos = start + (end - start) * (i / steps)
    packetHandler.write4ByteTxRx(portHandler, PAN_ID, ADDR_GOAL, int(pos))
    time.sleep(1.0 / rate)

time.sleep(1)
packetHandler.write1ByteTxRx(portHandler, PAN_ID, ADDR_TORQUE, 0)
portHandler.closePort()