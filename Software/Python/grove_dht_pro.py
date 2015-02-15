# GrovePi + Grove Temperature & Humidity Sensor Pro
# http://www.seeedstudio.com/wiki/Grove_-_Temperature_and_Humidity_Sensor_Pro

import grovepi

# Connect the Grove Temperature & Humidity Sensor Pro to digital port D4
# SIG,NC,VCC,GND
sensor = 4

# There are 3 supported sensor modules:
#module = 0  # DHT11 blue
module = 1   # DHT22 white / AM2302 wired version of DHT22 in larger plastic shell
#module = 2  # DHT21 black / AM2301 wired version of DHT21 in larger plastic shell

# DHT11:
# http://www.seeedstudio.com/wiki/Grove_-_Temperature_and_Humidity_Sensor
# http://www.adafruit.com/products/386

# DHT22:
# http://www.seeedstudio.com/wiki/Grove_-_Temperature_and_Humidity_Sensor_Pro
# http://www.adafruit.com/products/385

# AM2302:
# http://www.adafruit.com/product/393

# Learn more about the DHT modules:
# https://learn.adafruit.com/dht

while True:
    try:
        [temp,humidity] = grovepi.dht(sensor,module)
        print "temperature =", temp, "*C, humidity =", humidity, "%"

    except KeyboardInterrupt:
        break
    except IOError:
        print "Error"
