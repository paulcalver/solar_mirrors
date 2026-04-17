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
ADDR_PROFILE_ACC = 108
ADDR_POS_P_GAIN  = 84
ADDR_POS_I_GAIN  = 82
ADDR_POS_D_GAIN  = 80

PAN_ID       = 1
TILT_ID      = 2

# ── Position limits ────────────────────────────────────────
PAN_CENTRE   = 1142
TILT_CENTRE  = 845

# Explore mode — small circles around a locked centre
EXPLORE_PAN_RANGE    = 15
EXPLORE_TILT_RANGE   = 22

# Seek mode — wandering noise-based drift
SEEK_PAN_RANGE       = 125
SEEK_TILT_RANGE      = 75
SEEK_EXPANSION_TIME  = 120.0
SEEK_START_FRACTION  = 0.3

# ── Motion control: continuous position streaming ──────────
# We command positions at 50Hz and let the servo's PID track the
# continuously-moving target. Profile Velocity = 0 (no cap) means
# the PID responds to each new target immediately. Profile Acceleration
# smooths the PID's response so there's no snap at each command.
UPDATE_RATE_HZ       = 50
UPDATE_PERIOD        = 1.0 / UPDATE_RATE_HZ   # 0.02s
PROFILE_ACCELERATION = 30   # lower = smoother easing

# ── Light level configuration ──────────────────────────────
SEEK_THRESHOLD    = 0.30
EXPLORE_THRESHOLD = 0.15

VOLTAGE_MIN    = 0.1
VOLTAGE_MAX    = 2.8
SAMPLE_WINDOW  = 8

light_level = 0.0
voltage_history = deque(maxlen=SAMPLE_WINDOW)

# ── ADC read timing ────────────────────────────────────────
# ADS1115 at data_rate=8 returns a new reading every ~125ms.
# We don't need to read every loop iteration (50Hz would just return
# the same value repeatedly). Read roughly every 200ms instead.
ADC_READ_INTERVAL = 0.2
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
        ads.data_rate = 8
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

def smooth_noise(t, offset=0.0):
    """Smooth pseudo-random drift using incommensurate sine frequencies.
    Returns a value roughly in [-1, 1] that varies continuously over time."""
    return (math.sin(t * 0.13 + offset) * 0.5 +
            math.sin(t * 0.07 + offset * 1.7) * 0.3 +
            math.sin(t * 0.03 + offset * 2.3) * 0.2)

def read_voltage():
    try:
        return ads_channel.voltage
    except OSError:
        return None

def update_light_level_from_adc():
    global light_level

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

# ── Enable torque and configure servos ────────────────────
enable_torque(PAN_ID)
enable_torque(TILT_ID)
set_pid_gains(PAN_ID,  p=1500, i=0, d=500)
set_pid_gains(TILT_ID, p=1500, i=0, d=500)

# Zero velocity cap = let PID respond freely to moving targets
set_velocity(PAN_ID,  0)
set_velocity(TILT_ID, 0)

# Acceleration profile smooths response to sudden target changes
set_acceleration(PAN_ID,  PROFILE_ACCELERATION)
set_acceleration(TILT_ID, PROFILE_ACCELERATION)

move_to(PAN_ID,  PAN_CENTRE)
move_to(TILT_ID, TILT_CENTRE)
time.sleep(1)

if args.simulate:
    print("SIMULATION MODE")
    print("Controls: h = high light  l = low light  0 = no light  q = quit")
else:
    print("LIVE MODE (reading from ADS1115)")
    print("Controls: q = quit")
print(f"Update rate: {UPDATE_RATE_HZ}Hz  Profile accel: {PROFILE_ACCELERATION}")
print("─" * 50)

# ── Main loop ──────────────────────────────────────────────
# One unified high-rate loop. Mode determines how the commanded
# position is calculated, but commands always stream at UPDATE_RATE_HZ
# so the servos are always tracking a continuously-moving target.

mode       = 'seek'
seek_time  = 0.0   # continuous time in seek mode
t          = 0.0   # continuous time in explore mode
loop_start = time.time()

try:
    while True:
        loop_time_start = time.time()

        # ── Update light level (throttled to ADC rate) ─────
        if not args.simulate:
            if loop_time_start - last_adc_read >= ADC_READ_INTERVAL:
                update_light_level_from_adc()
                last_adc_read = loop_time_start

        # ── Keypress check ─────────────────────────────────
        if check_keypress() == 'quit':
            break

        # ── Seek mode ──────────────────────────────────────
        if mode == 'seek':
            # Search radius slowly expands from SEEK_START_FRACTION to 1.0
            expansion = min(1.0, seek_time / SEEK_EXPANSION_TIME)
            radius_scale = SEEK_START_FRACTION + (1.0 - SEEK_START_FRACTION) * expansion

            pan_amp  = SEEK_PAN_RANGE  * radius_scale
            tilt_amp = SEEK_TILT_RANGE * radius_scale

            pan  = PAN_CENTRE  + pan_amp  * smooth_noise(seek_time, offset=0.0)
            tilt = TILT_CENTRE + tilt_amp * smooth_noise(seek_time, offset=100.0)

            pan  = max(512, min(1536, pan))
            tilt = max(768, min(1280, tilt))

            move_to(PAN_ID,  pan)
            move_to(TILT_ID, tilt)

            # Print at reduced rate to avoid flooding terminal
            if int(seek_time * 10) % 5 == 0:
                print(f"\rSEEKING   light={light_level:.2f}  "
                      f"radius={radius_scale:.2f}  "
                      f"pan={int(pan)}  tilt={int(tilt)}    ",
                      end='', flush=True)

            seek_time += UPDATE_PERIOD

            if light_level >= SEEK_THRESHOLD:
                print(f"\n** LOCKED at light={light_level:.2f} **")
                mode = 'explore'
                t    = 0.0

        # ── Explore mode ───────────────────────────────────
        elif mode == 'explore':
            amp_scale = 0.6 + 0.4 * light_level
            pan_amp   = EXPLORE_PAN_RANGE  * amp_scale
            tilt_amp  = EXPLORE_TILT_RANGE * amp_scale

            # Slow circle — one full loop every ~60s
            angular_speed = 0.10 + light_level * 0.05

            # Ramp-in over first 3 seconds to prevent jump on entry
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

            # Print at reduced rate
            if int(t * 10) % 5 == 0:
                print(f"\rEXPLORING light={light_level:.2f}  ramp={ramp:.2f}  "
                      f"pan={int(pan)}  tilt={int(tilt)}    ",
                      end='', flush=True)

            t += UPDATE_PERIOD

            if light_level < EXPLORE_THRESHOLD:
                print(f"\n** LOST LOCK at light={light_level:.2f} **")
                mode      = 'seek'
                seek_time = 0.0

        # ── Maintain loop timing ───────────────────────────
        # Sleep just long enough to hit UPDATE_PERIOD consistently,
        # compensating for how long the work above took.
        elapsed = time.time() - loop_time_start
        sleep_time = UPDATE_PERIOD - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

finally:
    print("\nShutting down")
    disable_torque(PAN_ID)
    disable_torque(TILT_ID)
    portHandler.closePort()