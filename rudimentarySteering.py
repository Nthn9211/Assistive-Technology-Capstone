#  MBTechWorks.com 2016
#  Pulse Width Modulation (PWM) demo to cycle brightness of an LED

import RPi.GPIO as GPIO   # Import the GPIO library.
import time               # Import time library

GPIO.setmode(GPIO.BOARD)  # Set Pi to use pin number when referencing GPIO pins.
                          # Can use GPIO.setmode(GPIO.BCM) instead to use
                          # Broadcom SOC channel names.
#  MBTechWorks.com 2016
#  Pulse Width Modulation (PWM) demo to cycle brightness of an LED

GPIO.setup(16, GPIO.OUT)  # Set GPIO pin 16 to output mode.
GPIO.setup(18, GPIO.OUT)  # Set GPIO pin 18 to output mode.
bluePin = GPIO.PWM(18, 100)   # Initialize PWM on pwmPin 100Hz frequency
orangePin = GPIO.PWM(16, 100)   # Initialize PWM on pwmPin 100Hz frequency

# main loop of program
bluedc=80                              # set blue dc variable to 0 for 0%
orangedc=80
bluePin.start(bluedc)
orangePin.start(orangedc)
bluePin.ChangeDutyCycle(bluedc)
orangePin.ChangeDutyCycle(orangedc)
print("Press Ctrl+C to exit\n")
try:
  while True:                      # Loop until Ctl C is pressed to stop.
      FWBmovement = input("Press W to move forward, B to stop, S for back\n")
      if FWBmovement == 'w':
         bluedc = 96
         bluePin.ChangeDutyCycle(bluedc)
      elif FWBmovement == 'b':
         bluedc = 63
         bluePin.ChangeDutyCycle(bluedc)
      elif FWBmovement == 's':
         bluedc = 80
         bluePin.ChangeDutyCycle(bluedc)
      LRmovement  = input("Press A to go left, D to go Right, C to center\n")
      if LRmovement == 'd':
          orangedc = 96.3
          orangePin.ChangeDutyCycle(orangedc)
      elif LRmovement == 'a':
          orangedc = 63.3
          orangePin.ChangeDutyCycle(orangedc)
      elif LRmovement == 'c':
         orangedc = 80
         orangePin.ChangeDutyCycle(orangedc)
except KeyboardInterrupt:
  print("Ctl C pressed - ending program")

bluePin.stop()                         # stop PWM
orangePin.stop()                         # stop PWM
GPIO.cleanup()                     # resets GPIO ports used back to input modei to use pin number when referencing GPIO pins.