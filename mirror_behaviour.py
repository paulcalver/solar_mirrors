import time
import math
import sys
import tty
import termios
import select
import random
import argparse
from collections import deque
from dynamixel_sdk import *

# ── Arguments ──────────────────────────────────────────────
parser = argparse.ArgumentParser(description='Mirror behaviour controller')
parser.add_argument('--simulate', action='store_true',
                    help='Use keypress simulation instead of ADS1115 (for testing)')
args = parser.parse_args()

# ── Configuration ──────────────────────────────────────────
DEVICENAME   = '/dev/ttyUSB0'
BAUDRATE     = 57600
PROTOCOL     = 2.0

ADDR_TORQUE      = 64
ADDR_GOAL        = 116
ADDR_PROFILE_VEL = 112
ADDR_PROFILE_ACC = 108
ADDR_POS_P_GAIN  = 84
ADDR_POS_I_GAIN  = 82
ADDR_POS_D_GAIN  = 80

PAN_ID       = 1
TILT_ID      = 2

# ── Position limits ────────────────────────────────────────
PAN_CENTRE   = 1224
TILT_CENTRE  = 784

# Seek mode — wide random sweeps
SEEK_PAN_RANGE       = 100
SEEK_TILT_RANGE      = 70

# Explore mode — small wandering within lock zone
EXPLORE_PAN_RANGE    = 10
EXPLORE_TILT_RANGE   = 10

# ── Motion tuning ──────────────────────────────────────────
SWEEP_SPEED          = 125    # counts per second during a sweep
                              # (sweep test confirmed 175 is smooth floor,
                              #  125 gives comfortable margin above)

SEEK_MIN_DURATION    = 1.5    # minimum time for a seek sweep, even if target is close
SEEK_MAX_DURATION    = 5.0    # cap for very long sweeps

EXPLORE_MIN_DURATION = 2.0    # explore sweeps are slower
EXPLORE_MAX_DURATION = 4.0
EXPLORE_PAUSE_MIN    = 3.0    # thinking pauses between explore moves
EXPLORE_PAUSE_MAX    = 6.0

# ── Servo control ──────────────────────────────────────────
PROFILE_ACCELERATION = 30
UPDATE_RATE_HZ       = 50
UPDATE_PERIOD        = 1.0 / UPDATE_RATE_HZ

# ── Light level configuration ──────────────────────────────
SEEK_THRESHOLD    = 0.80
EXPLORE_THRESHOLD = 0.30

VOLTAGE_MIN       = 0.1
VOLTAGE_MAX       = 2.8
SAMPLE_WINDOW     = 8

light_level     = 0.0
voltage_history = deque(maxlen=SAMPLE_WINDOW)

# ── ADC timing (outdoor / sunlight defaults) ──────────────
ADC_DATA_RATE     = 128
ADC_READ_INTERVAL = 0.05
last_adc_read     = 0.0

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
        ads.data_rate = ADC_DATA_RATE
        ads_channel = AnalogIn(ads, 0)
        print(f"ADS1115 initialised on A0 at {ADC_DATA_RATE}Hz")
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

# ── Servo helpers ──────────────────────────────────────────
def set_pid_gains(sid, p=1500, i=0, d=500):
    packetHandler.write2ByteTxRx(portHandler, sid, ADDR_POS_P_GAIN, p)
    packetHandler.write2ByteTxRx(portHandler, sid, ADDR_POS_I_GAIN, i)
    packetHandler.write2ByteTxRx(portHandler, sid, ADDR_POS_D_GAIN, d)

def set_velocity(sid, velocity):
    packetHandler.write4ByteTxRx(portHandler, sid, ADDR_PROFILE_VEL, velocity)

def set_acceleration(sid, acceleration):
    packetHandler.write4ByteTxRx(portHandler, sid, ADDR_PROFILE_ACC, acceleration)

def enable_torque(sid):
    packetHandler.write1ByteTxRx(portHandler, sid, ADDR_TORQUE, 1)

def disable_torque(sid):
    packetHandler.write1ByteTxRx(portHandler, sid, ADDR_TORQUE, 0)

def move_to(sid, position):
    position = max(0, min(4095, int(position)))
    packetHandler.write4ByteTxRx(portHandler, sid, ADDR_GOAL, position)

# ── ADC helpers ────────────────────────────────────────────
def read_voltage():
    try:
        return ads_channel.voltage
    except OSError:
        return None

def update_light_level_from_adc():
    global light_level, last_adc_read
    v = read_voltage()
    if v is not None:
        voltage_history.append(v)
    if not voltage_history:
        return light_level
    avg_voltage = sum(voltage_history) / len(voltage_history)
    if avg_voltage <= VOLTAGE_MIN:
        light_level = 0.0
    elif avg_voltage >= VOLTAGE_MAX:
        light_level = 1.0
    else:
        light_level = (avg_voltage - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN)
    last_adc_read = time.time()
    return light_level

def check_keypress():
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

# ── THE MOTION PRIMITIVE ──────────────────────────────────
# This is the smooth sweep that the test script proved works.
# Everything else in the script is just choosing where to sweep to.
def sweep_to(target_pan, target_tilt, duration, label="MOVING"):
    """Smoothly sweep both servos to (target_pan, target_tilt) over `duration` seconds.
    Returns 'quit' if user pressed q, 'locked' if light threshold crossed,
    'lost' if dropped below explore threshold, or None if sweep completed normally.
    Reads ADC and checks keypress during sweep."""
    global pan, tilt

    start_pan  = pan
    start_tilt = tilt
    steps = max(1, int(duration * UPDATE_RATE_HZ))

    for i in range(steps):
        progress = (i + 1) / steps
        p = start_pan  + (target_pan  - start_pan)  * progress
        t = start_tilt + (target_tilt - start_tilt) * progress
        move_to(PAN_ID,  p)
        move_to(TILT_ID, t)
        pan  = p
        tilt = t

        # Update ADC if enough time has passed
        now = time.time()
        if not args.simulate and now - last_adc_read >= ADC_READ_INTERVAL:
            update_light_level_from_adc()

        # Check for quit
        if check_keypress() == 'quit':
            return 'quit'

        # Check for mode transitions mid-sweep
        if mode == 'seek' and light_level >= SEEK_THRESHOLD:
            return 'locked'
        if mode == 'explore' and light_level < EXPLORE_THRESHOLD:
            return 'lost'

        # Print progress occasionally
        if i % 10 == 0:
            print(f"\r{label}  light={light_level:.2f}  "
                  f"pan={int(pan)}  tilt={int(tilt)}    ",
                  end='', flush=True)

        time.sleep(UPDATE_PERIOD)

    return None

def hold_for(duration, label="HOLDING"):
    """Hold position for duration, still reading ADC and checking keypress.
    Same return values as sweep_to."""
    end_time = time.time() + duration
    while time.time() < end_time:
        now = time.time()
        if not args.simulate and now - last_adc_read >= ADC_READ_INTERVAL:
            update_light_level_from_adc()
        if check_keypress() == 'quit':
            return 'quit'
        if mode == 'seek' and light_level >= SEEK_THRESHOLD:
            return 'locked'
        if mode == 'explore' and light_level < EXPLORE_THRESHOLD:
            return 'lost'
        print(f"\r{label}  light={light_level:.2f}  "
              f"pan={int(pan)}  tilt={int(tilt)}    ",
              end='', flush=True)
        time.sleep(0.1)
    return None

def sweep_duration(from_pan, from_tilt, to_pan, to_tilt, min_dur, max_dur):
    """Calculate duration so sweep speed stays near SWEEP_SPEED."""
    dist = math.hypot(to_pan - from_pan, to_tilt - from_tilt)
    return max(min_dur, min(max_dur, dist / SWEEP_SPEED))

# ── Enable torque and configure servos ────────────────────
enable_torque(PAN_ID)
enable_torque(TILT_ID)
set_pid_gains(PAN_ID,  p=1500, i=0, d=500)
set_pid_gains(TILT_ID, p=1500, i=0, d=500)
set_velocity(PAN_ID,  0)    # zero = no velocity cap, let PID track positions
set_velocity(TILT_ID, 0)
set_acceleration(PAN_ID,  PROFILE_ACCELERATION)
set_acceleration(TILT_ID, PROFILE_ACCELERATION)

pan  = PAN_CENTRE
tilt = TILT_CENTRE
move_to(PAN_ID,  pan)
move_to(TILT_ID, tilt)
time.sleep(1)

if args.simulate:
    print("SIMULATION MODE")
    print("Controls: h = high light  l = low light  0 = no light  q = quit")
else:
    print("LIVE MODE (reading from ADS1115)")
    print("Controls: q = quit")
print("─" * 50)

# ── Main loop ──────────────────────────────────────────────
mode = 'seek'
quit_requested = False

try:
    while not quit_requested:

        if mode == 'seek':
            # Pick a random target anywhere in the search area
            target_pan  = PAN_CENTRE  + random.uniform(-SEEK_PAN_RANGE,  SEEK_PAN_RANGE)
            target_tilt = TILT_CENTRE + random.uniform(-SEEK_TILT_RANGE, SEEK_TILT_RANGE)

            # Safety clamps
            target_pan  = max(512, min(1536, target_pan))
            target_tilt = max(768, min(1280, target_tilt))

            duration = sweep_duration(pan, tilt, target_pan, target_tilt,
                                      SEEK_MIN_DURATION, SEEK_MAX_DURATION)

            result = sweep_to(target_pan, target_tilt, duration, label="SEEKING  ")

            if result == 'quit':
                quit_requested = True
            elif result == 'locked':
                print(f"\n** LOCKED at light={light_level:.2f} **")
                mode = 'explore'

        elif mode == 'explore':
            # Pick a target near the current position, inside the explore zone
            # Use a small random offset from current, then clip to zone
            offset_pan  = random.uniform(-10, 10)
            offset_tilt = random.uniform(-10, 10)
            target_pan  = pan  + offset_pan
            target_tilt = tilt + offset_tilt

            # Constrain to explore zone around centre
            target_pan  = max(PAN_CENTRE  - EXPLORE_PAN_RANGE,
                              min(PAN_CENTRE  + EXPLORE_PAN_RANGE,  target_pan))
            target_tilt = max(TILT_CENTRE - EXPLORE_TILT_RANGE,
                              min(TILT_CENTRE + EXPLORE_TILT_RANGE, target_tilt))

            duration = sweep_duration(pan, tilt, target_pan, target_tilt,
                                      EXPLORE_MIN_DURATION, EXPLORE_MAX_DURATION)

            result = sweep_to(target_pan, target_tilt, duration, label="EXPLORING")

            if result == 'quit':
                quit_requested = True
                continue
            elif result == 'lost':
                print(f"\n** LOST LOCK at light={light_level:.2f} **")
                mode = 'seek'
                continue

            # Long thinking pause after move
            pause = random.uniform(EXPLORE_PAUSE_MIN, EXPLORE_PAUSE_MAX)
            result = hold_for(pause, label="HOLDING  ")

            if result == 'quit':
                quit_requested = True
            elif result == 'lost':
                print(f"\n** LOST LOCK at light={light_level:.2f} **")
                mode = 'seek'

finally:
    print("\nShutting down")
    disable_torque(PAN_ID)
    disable_torque(TILT_ID)
    portHandler.closePort()