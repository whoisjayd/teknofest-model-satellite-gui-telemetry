from datetime import datetime

import board
import busio
import pytz
from adafruit_ds3231 import DS3231

# Initialize I2C interface and DS3231 RTC
i2c = busio.I2C(board.SCL, board.SDA)
rtc = DS3231(i2c)

# Get the current time with the specified timezone
now = datetime.now(pytz.timezone("Europe/Istanbul"))

# Convert to struct_time
t = now.timetuple()

# Set RTC datetime
rtc.datetime = t

# Print the time set
print("Time set to:", now.strftime("%d/%m/%Y , %H:%M:%S"))
