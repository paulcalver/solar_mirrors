from dynamixel_sdk import *

DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 57600
PROTOCOL = 2.0
ADDR_TORQUE = 64
PAN_ID = 1
TILT_ID = 2

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

packetHandler.write1ByteTxRx(portHandler, PAN_ID, ADDR_TORQUE, 0)
packetHandler.write1ByteTxRx(portHandler, TILT_ID, ADDR_TORQUE, 0)

portHandler.closePort()
print("Torque disabled on both servos")