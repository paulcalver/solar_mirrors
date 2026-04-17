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

ADDR_TORQUE      = 64
ADDR_GOAL        = 116
ADDR_POS         = 132
ADDR_PROFILE_VEL = 112
ADDR_POS_P_GAIN  = 84
ADDR_POS_I_GAIN  = 82
ADDR_POS_D_GAIN  = 80

PAN_ID       = 1
TILT_ID      = 2

# ── Position limits ────────────────────────────────────────
PAN_CENTRE   = 1295
TILT_CENTRE  = 825

# Explore mode — small circles around a locked centre
EXPLORE_PAN_RANGE    = 15
EXPLORE_TILT_RANGE   = 22

# Seek mode — wandering noise-based drift
SEEK_PAN_RANGE       = 125   # max drift from centre in pan
SEEK_TILT_RANGE      = 75    # max drift from centre in tilt
SEEK_EXPANSION_TIME  = 120.0 # seconds for search to reach full range
SEEK_START_FRACTION  = 0.3   # starts at 30% of max range

# ── Motion profile ──────────────────────────────────────────
SEEK_VELOCITY    = 15    # slow, smooth motion in seek mode
EXPLORE_VELOCITY = 4     # slow glide between waypoints

# ── Light level configuration ──────────────────────────────
SEEK_THRESHOLD    = 0.30
EXPLORE_THRESHOLD = 0.15

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
        ads.data_rate = 8    # slowest rate = internal averaging smooths PWM
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
def set_pid_gains(sid, p=1500, i=0, d=500):
    """Set position PID gains. Higher P = more responsive to small errors."""
    packetHandler.write2ByteTxRx(portHandler, sid, ADDR_POS_P_GAIN, p)
    packetHandler.write2ByteTxRx(portHandler, sid, ADDR_POS_I_GAIN, i)
    packetHandler.write2ByteTxRx(portHandler, sid, ADDR_POS_D_GAIN, d)

def set_velocity(sid, velocity):
    """Set profile velocity. Lower = slower/smoother, 0 = max speed."""
    packetHandler.write4ByteTxRx(portHandler, sid, ADDR_PROFILE_VEL, velocity)

def enable_torque(sid):
    packetHandler.write1ByteTxRx(portHandler, sid, ADDR_TORQUE, 1)

def disable_torque(sid):
    packetHandler.write1ByteTxRx(portHandler, sid, ADDR_TORQUE, 0)

def move_to(sid, position):
    position = max(0, min(4095, int(position)))
    packetHandler.write4ByteTxRx(portHandler, sid, ADDR_GOAL, position)

def smooth_noise(t, offset=0.0):
    """Smooth pseudo-random drift using incommensurate sine frequencies.
    Returns a value roughly in [-1, 1] that varies continuously over time.
    Uses three incommensurate frequencies so the pattern never exactly repeats."""
    return (math.sin(t * 0.13 + offset) * 0.5 +
            math.sin(t * 0.07 + offset * 1.7) * 0.3 +
            math.sin(t * 0.03 + offset * 2.3) * 0.2)

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

# ── Enable torque and configure servos ────────────────────
enable_torque(PAN_ID)
enable_torque(TILT_ID)
set_pid_gains(PAN_ID,  p=1500, i=0, d=500)
set_pid_gains(TILT_ID, p=1500, i=0, d=500)
set_velocity(PAN_ID,  SEEK_VELOCITY)
set_velocity(TILT_ID, SEEK_VELOCITY)
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
mode       = 'seek'
seek_time  = 0.0   # elapsed time in seek mode (for expansion ramp)
t          = 0     # explore mode trajectory time

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
            # Search radius slowly expands from SEEK_START_FRACTION to 1.0
            # over SEEK_EXPANSION_TIME seconds, giving the mirror a sense
            # of gradually widening its attention.
            expansion = min(1.0, seek_time / SEEK_EXPANSION_TIME)
            radius_scale = SEEK_START_FRACTION + (1.0 - SEEK_START_FRACTION) * expansion

            pan_amp  = SEEK_PAN_RANGE  * radius_scale
            tilt_amp = SEEK_TILT_RANGE * radius_scale

            # Pan and tilt use independent noise offsets so motion is uncorrelated
            pan  = PAN_CENTRE  + pan_amp  * smooth_noise(seek_time, offset=0.0)
            tilt = TILT_CENTRE + tilt_amp * smooth_noise(seek_time, offset=100.0)

            # Safety clamps
            pan  = max(512, min(1536, pan))
            tilt = max(768, min(1280, tilt))

            move_to(PAN_ID,  pan)
            move_to(TILT_ID, tilt)
            print(f"\rSEEKING  light={light_level:.2f}  "
                  f"radius={radius_scale:.2f}  "
                  f"pan={int(pan)}  tilt={int(tilt)}    ", end='')

            time.sleep(0.1)
            seek_time += 0.1

            if light_level >= SEEK_THRESHOLD:
                print(f"\n** LOCKED at light={light_level:.2f} **")
                set_velocity(PAN_ID,  EXPLORE_VELOCITY)
                set_velocity(TILT_ID, EXPLORE_VELOCITY)
                mode = 'explore'
                t    = 0

        # ── Explore mode ───────────────────────────────────
        elif mode == 'explore':
            # Use the servo's own velocity profile to create smooth motion
            # between waypoints. We command a new waypoint every ~1 second
            # and let the servo glide there using its internal controller.

            amp_scale = 0.6 + 0.4 * light_level
            pan_amp   = EXPLORE_PAN_RANGE  * amp_scale
            tilt_amp  = EXPLORE_TILT_RANGE * amp_scale

            # Slow circle — one full loop every ~60 seconds
            angular_speed = 0.10 + light_level * 0.05

            # Ramp-in to prevent jump at entry
            RAMP_DURATION = 3.0
            ramp = min(1.0, t / RAMP_DURATION)
            ramp = ramp * ramp * (3 - 2 * ramp)  # smoothstep

            drift_speed = angular_speed * 0.13
            drift_pan   = pan_amp  * 0.3 * math.sin(drift_speed * t) * ramp
            drift_tilt  = tilt_amp * 0.3 * math.cos(drift_speed * t * 0.618) * ramp

            pan  = PAN_CENTRE  + drift_pan  + pan_amp  * 0.7 * math.cos(angular_speed * t) * ramp
            tilt = TILT_CENTRE + drift_tilt + tilt_amp * 0.7 * math.sin(angular_speed * t) * ramp

            move_to(PAN_ID,  pan)
            move_to(TILT_ID, tilt)
            print(f"\rEXPLORING  light={light_level:.2f}  ramp={ramp:.2f}  "
                  f"pan={int(pan)}  tilt={int(tilt)}    ", end='')

            t += 1.0           # big time step — waypoint every second
            time.sleep(1.0)    # wait for servo to glide there

            if light_level < EXPLORE_THRESHOLD:
                print(f"\n** LOST LOCK at light={light_level:.2f} **")
                set_velocity(PAN_ID,  SEEK_VELOCITY)
                set_velocity(TILT_ID, SEEK_VELOCITY)
                mode      = 'seek'
                seek_time = 0.0   # reset expansion so search starts tight again

finally:
    print("\nShutting down")
    disable_torque(PAN_ID)
    disable_torque(TILT_ID)
    portHandler.closePort()