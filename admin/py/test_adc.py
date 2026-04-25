import board
import busio
import time
from adafruit_ads1x15.ads1115 import ADS1115
from adafruit_ads1x15.analog_in import AnalogIn

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS1115(i2c)
chan = AnalogIn(ads, 0)

while True:
    print(f"Voltage: {chan.voltage:.4f}V")
    time.sleep(0.5)