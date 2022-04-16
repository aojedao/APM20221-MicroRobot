import Adafruit_BBIO.GPIO as GPIO

GPIO.setup("RED_LED",GPIO.OUT)
while True:
    GPIO.output("RED_LED",GPIO.HIGH)
GPIO.cleanup()
