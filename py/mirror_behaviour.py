import time
import math
import sys
import tty
import termios
import select
import argparse
from collections import deque
from dynamixel_sdk import *

# ── Arguments ──────────────────────────────────────────────
parser = argparse.ArgumentParser(description='Mirror behaviour controller')
parser.add_argument('--simulate', action='store_true',
                    help='Use keypress simulation instead of ADS1115 (for testing indoors)')
args = parser.parse_args()

# ── Configuration ──────────────────────────────────────────
DEVICENAME   = '/dev/ttyUSB0'
BAUDRATE     = 57600
PROTOCOL     = 2.0

ADDR_TORQUE  = 64
ADDR_GOAL    = 116
ADDR_POS     = 132

PAN_ID       = 1
TILT_ID      = 2

# ── Position limits ────────────────────────────────────────
PAN_CENTRE   = 1300
TILT_CENTRE  = 850
PAN_RANGE    = 200
TILT_RANGE   = 100

# ── Light level configuration ──────────────────────────────
SEEK_THRESHOLD    = 0.3
EXPLORE_THRESHOLD = 0.2

# Voltage-to-light-level mapping (based on divider output at A0)
# 0.0V at A0 = darkness, 3.0V at A0 = full panel output (~12V before divider)
VOLTAGE_MIN    = 0.1   # below this we call it 'dark' (noise floor)
VOLTAGE_MAX    = 2.8   # above this we call it 'full light'
SAMPLE_WINDOW  = 8     # number of readings to average

light_level = 0.0
voltage_history = deque(maxlen=SAMPLE_WINDOW)

# ── Initialise ADS1115 (only if not simulating) ────────────
ads_channel = None
if not args.simulate:
    try:
        import board
        import busio
        from adafruit_ads1x15.ads1115 import ADS1115
        from adafruit_ads1x15.analog_in import AnalogIn

        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS1115(i2c)
        ads_channel = AnalogIn(ads, 0)
        print("ADS1115 initialised on A0")
    except Exception as e:
        print(f"Failed to initialise ADS1115: {e}")
        print("Falling back to simulation mode")
        args.simulate = True

# ── Connect to servos ──────────────────────────────────────
portHandler   = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL)

if not portHandler.openPort():
    print("Failed to open port")
    quit()
portHandler.setBaudRate(BAUDRATE)
print("Connected to servos\n")

# ── Helper functions ───────────────────────────────────────
def enable_torque(sid):
    packetHandler.write1ByteTxRx(portHandler, sid, ADDR_TORQUE, 1)

def disable_torque(sid):
    packetHandler.write1ByteTxRx(portHandler, sid, ADDR_TORQUE, 0)

def move_to(sid, position):
    position = max(0, min(4095, int(position)))
    packetHandler.write4ByteTxRx(portHandler, sid, ADDR_GOAL, position)

def get_spiral_position(step, pan_centre, tilt_centre):
    """Generate expanding spiral search pattern"""
    steps_per_ring = 16
    ring  = step // steps_per_ring
    angle = (step % steps_per_ring) * (2 * math.pi / steps_per_ring)

    radius_pan  = ring * 25
    radius_tilt = ring * 15

    pan  = pan_centre  + int(radius_pan  * math.sin(angle))
    tilt = tilt_centre + int(radius_tilt * math.cos(angle))

    return pan, tilt

def read_voltage():
    """Read voltage from A0. Returns None on read failure."""
    try:
        return ads_channel.voltage
    except OSError:
        return None

def update_light_level_from_adc():
    """Sample the ADC, update rolling average, return light level 0.0–1.0."""
    global light_level

    v = read_voltage()
    if v is not None:
        voltage_history.append(v)

    if not voltage_history:
        return light_level  # no data yet, keep previous

    avg_voltage = sum(voltage_history) / len(voltage_history)

    # Map to 0.0–1.0 range with clipping
    if avg_voltage <= VOLTAGE_MIN:
        light_level = 0.0
    elif avg_voltage >= VOLTAGE_MAX:
        light_level = 1.0
    else:
        light_level = (avg_voltage - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN)

    return light_level

def check_keypress():
    """Check for keypress input (used in simulation mode or for quit)."""
    global light_level
    if select.select([sys.stdin], [], [], 0)[0]:
        key = sys.stdin.read(1)
        if args.simulate:
            if key == 'h':
                light_level = 1.0
                print(f"\nLight: HIGH ({light_level})")
            elif key == 'l':
                light_level = 0.4
                print(f"\nLight: LOW ({light_level})")
            elif key == '0':
                light_level = 0.0
                print(f"\nLight: NONE ({light_level})")
        if key == 'q':
            return 'quit'
    return None

# ── Enable torque ──────────────────────────────────────────
enable_torque(PAN_ID)
enable_torque(TILT_ID)
move_to(PAN_ID,  PAN_CENTRE)
move_to(TILT_ID, TILT_CENTRE)
time.sleep(1)

if args.simulate:
    print("SIMULATION MODE")
    print("Controls: h = high light  l = low light  0 = no light  q = quit")
else:
    print("LIVE MODE (reading from ADS1115)")
    print("Controls: q = quit")
print("─" * 50)

# ── Main loop ──────────────────────────────────────────────
mode      = 'seek'
seek_step = 0
t         = 0

try:
    while True:
        # ── Update light level ─────────────────────────────
        if not args.simulate:
            update_light_level_from_adc()

        # ── Check for keypress (always, for 'q' at minimum) ─
        if check_keypress() == 'quit':
            break

        # ── Seek mode ──────────────────────────────────────
        if mode == 'seek':
            pan, tilt = get_spiral_position(seek_step, PAN_CENTRE, TILT_CENTRE)

            # Clamp to safe range
            pan  = max(512,  min(1536, pan))
            tilt = max(768,  min(1280, tilt))

            move_to(PAN_ID,  pan)
            move_to(TILT_ID, tilt)
            print(f"\rSEEKING  light={light_level:.2f}  "
                  f"pan={pan}  tilt={tilt}    ", end='')

            time.sleep(0.6)
            seek_step += 1

            # Reset after 5 rings
            if seek_step > 16 * 5:
                seek_step = 0

            if light_level >= SEEK_THRESHOLD:
                print(f"\n** LOCKED at light={light_level:.2f} **")
                mode = 'explore'
                t    = 0

        # ── Explore mode ───────────────────────────────────
        elif mode == 'explore':
            # Amplitude scales gently with light (0.6 to 1.0 of full range)
            # so the mirror always explores meaningfully once locked
            amp_scale = 0.6 + 0.4 * light_level
            amplitude = PAN_RANGE  * amp_scale
            tilt_amp  = TILT_RANGE * amp_scale

            # Slow, meditative frequency — one pan cycle every 40–80 seconds
            speed = 0.05 + light_level * 0.1

            # Golden ratio inverse (0.618) prevents pan/tilt cycles from
            # syncing, giving organic-feeling paths that take a long time
            # to visually repeat
            pan  = PAN_CENTRE  + amplitude * math.sin(speed * t)
            tilt = TILT_CENTRE + tilt_amp  * math.sin(speed * t * 0.618)

            move_to(PAN_ID,  pan)
            move_to(TILT_ID, tilt)
            print(f"\rEXPLORING  light={light_level:.2f}  "
                  f"pan={int(pan)}  tilt={int(tilt)}    ", end='')

            t += 0.1
            time.sleep(0.1)

            if light_level < EXPLORE_THRESHOLD:
                print(f"\n** LOST LOCK at light={light_level:.2f} **")
                mode      = 'seek'
                seek_step = 0

finally:
    print("\nShutting down")
    disable_torque(PAN_ID)
    disable_torque(TILT_ID)
    portHandler.closePort()