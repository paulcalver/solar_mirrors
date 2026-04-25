import time
from dynamixel_sdk import *

# ── Configuration ──────────────────────────────────────────
DEVICENAME   = '/dev/tty.usbserial-FTB8HRQO'  # your U2D2 port
BAUDRATE     = 57600
PROTOCOL     = 2.0

# XL330 control table addresses
ADDR_TORQUE  = 64
ADDR_GOAL    = 116
ADDR_POS     = 132

# Servo IDs
PAN_ID       = 1
TILT_ID      = 2

# Position constants
CENTRE = 1024   # 90 degrees -- true centre
LEFT   = 512    # 45 degrees
RIGHT  = 1536   # 135 degrees
UP     = 612    # tilt up
DOWN   = 1350   # tilt down

# ── Connect ─────────────────────────────────────────────────
portHandler   = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL)

if not portHandler.openPort():
    print("Failed to open port -- check U2D2 is connected")
    quit()

if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to set baud rate")
    quit()

print("Connected successfully\n")

# ── Helper functions ─────────────────────────────────────────
def enable_torque(servo_id):
    packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE, 1)

def disable_torque(servo_id):
    packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE, 0)

def move_to(servo_id, position, label=""):
    packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_GOAL, position)
    if label:
        print(f"  Servo {servo_id} → {label} ({position})")
    time.sleep(1.5)

def read_position(servo_id):
    pos, _, _ = packetHandler.read4ByteTxRx(portHandler, servo_id, ADDR_POS)
    degrees = round((pos / 4096) * 360, 1)
    print(f"  Servo {servo_id} current position: {pos} steps = {degrees} degrees")

# ── Tests ────────────────────────────────────────────────────
print("=== Enabling torque on both servos ===")
enable_torque(PAN_ID)
enable_torque(TILT_ID)
time.sleep(0.5)

print("\n=== Centring both servos ===")
move_to(PAN_ID,  CENTRE, "centre")
move_to(TILT_ID, CENTRE, "centre")

print("\n=== Reading positions ===")
read_position(PAN_ID)
read_position(TILT_ID)

print("\n=== Testing PAN (Servo 1) ===")
move_to(PAN_ID, LEFT,   "left")
move_to(PAN_ID, CENTRE, "centre")
move_to(PAN_ID, RIGHT,  "right")
move_to(PAN_ID, CENTRE, "centre")

print("\n=== Testing TILT (Servo 2) ===")
move_to(TILT_ID, UP,     "up")
move_to(TILT_ID, CENTRE, "centre")
move_to(TILT_ID, DOWN,   "down")
move_to(TILT_ID, CENTRE, "centre")

print("\n=== All tests complete -- disabling torque ===")
disable_torque(PAN_ID)
disable_torque(TILT_ID)

portHandler.closePort()
print("Done -- both servos back at centre, torque off")