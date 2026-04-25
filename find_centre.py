import time
import sys
import tty
import termios
import select
from dynamixel_sdk import *

# ── Dynamixel config ───────────────────────────────────────
DEVICENAME      = '/dev/ttyUSB0'
BAUDRATE        = 57600
PROTOCOL        = 2.0

ADDR_TORQUE     = 64
ADDR_GOAL       = 116
ADDR_POS_D_GAIN = 80
ADDR_POS_I_GAIN = 82
ADDR_POS_P_GAIN = 84

PAN_ID          = 1
TILT_ID         = 2

# ── Light level config (same mapping as mirror_behaviour.py) ──
VOLTAGE_MIN = 0.1
VOLTAGE_MAX = 2.8

# ── Initialise ADS1115 ─────────────────────────────────────
ads_channel = None
try:
    import board
    import busio
    from adafruit_ads1x15.ads1115 import ADS1115
    from adafruit_ads1x15.analog_in import AnalogIn

    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS1115(i2c)
    ads.data_rate = 8    # slowest rate = internal averaging smooths PWM
    ads_channel = AnalogIn(ads, 0)
    print("ADS1115 connected")
except Exception as e:
    print(f"ADS1115 not available: {e}")
    print("Continuing without light readings")

def read_light():
    """Return (voltage, light_level) or (None, None) on failure."""
    if ads_channel is None:
        return None, None
    try:
        v = ads_channel.voltage
        if v <= VOLTAGE_MIN:
            level = 0.0
        elif v >= VOLTAGE_MAX:
            level = 1.0
        else:
            level = (v - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN)
        return v, level
    except OSError:
        return None, None

# ── Connect to servos ──────────────────────────────────────
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

def set_pid_gains(sid, p=1500, i=0, d=500):
    """Set position PID gains. Higher P = more responsive to small errors."""
    packetHandler.write2ByteTxRx(portHandler, sid, ADDR_POS_P_GAIN, p)
    packetHandler.write2ByteTxRx(portHandler, sid, ADDR_POS_I_GAIN, i)
    packetHandler.write2ByteTxRx(portHandler, sid, ADDR_POS_D_GAIN, d)

# Enable torque and set PID gains BEFORE any movement commands
packetHandler.write1ByteTxRx(portHandler, PAN_ID,  ADDR_TORQUE, 1)
packetHandler.write1ByteTxRx(portHandler, TILT_ID, ADDR_TORQUE, 1)
set_pid_gains(PAN_ID,  p=1500, i=0, d=500)
set_pid_gains(TILT_ID, p=1500, i=0, d=500)

pan  = 1142
tilt = 845

print("\nControls:")
print("  a/d = pan left/right  (coarse, 20 counts)")
print("  w/s = tilt up/down    (coarse, 20 counts)")
print("  A/D = pan left/right  (fine, 1 count)")
print("  W/S = tilt up/down    (fine, 1 count)")
print("  q   = quit and print final position")
print()

fd = sys.stdin.fileno()
old = termios.tcgetattr(fd)
tty.setcbreak(fd)

try:
    while True:
        packetHandler.write4ByteTxRx(portHandler, PAN_ID,  ADDR_GOAL, pan)
        packetHandler.write4ByteTxRx(portHandler, TILT_ID, ADDR_GOAL, tilt)

        v, level = read_light()
        if v is not None:
            # Build a simple ASCII bar for light level
            bar_width = 20
            filled = int(level * bar_width)
            bar = '█' * filled + '·' * (bar_width - filled)
            print(f"\rpan={pan:<5}  tilt={tilt:<5}  "
                  f"V={v:.3f}V  light={level:.2f}  [{bar}]    ",
                  end='', flush=True)
        else:
            print(f"\rpan={pan:<5}  tilt={tilt:<5}    ",
                  end='', flush=True)

        # Non-blocking read: check for key, timeout to refresh readout
        if select.select([sys.stdin], [], [], 0.1)[0]:
            k = sys.stdin.read(1)
            if   k == 'a': pan  -= 20
            elif k == 'd': pan  += 20
            elif k == 'w': tilt += 20
            elif k == 's': tilt -= 20
            elif k == 'A': pan  -= 1    # single-count fine mode
            elif k == 'D': pan  += 1
            elif k == 'W': tilt += 1
            elif k == 'S': tilt -= 1
            elif k == 'q': break
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    packetHandler.write1ByteTxRx(portHandler, PAN_ID,  ADDR_TORQUE, 0)
    packetHandler.write1ByteTxRx(portHandler, TILT_ID, ADDR_TORQUE, 0)
    portHandler.closePort()
    print(f"\n\nFinal position: PAN_CENTRE = {pan}, TILT_CENTRE = {tilt}")