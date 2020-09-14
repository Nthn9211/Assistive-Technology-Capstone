import pygame
from pygame.locals import *

import RPi.GPIO as GPIO   # Import the GPIO library.
import time               # Import time library

GPIO.setmode(GPIO.BOARD)  # Set Pi to use pin number when referencing GPIO pins.
                          # Can use GPIO.setmode(GPIO.BCM) instead to use
                          # Broadcom SOC channel names.
#  MBTechWorks.com 2016
#  Pulse Width Modulation (PWM) demo to cycle brightness of an LED

GPIO.setup(16, GPIO.OUT)  # Set GPIO pin 16 to output mode. Blue wire
GPIO.setup(18, GPIO.OUT)  # Set GPIO pin 18 to output mode. Orange wire
GPIO.setup(11, GPIO.IN)  # Set GPIO pin 11 to input mode. Coordinate X white wire
GPIO.setup(13, GPIO.IN)  # Set GPIO pin 11 to input mode. Angular Z  black wire
bluePin = GPIO.PWM(18, 100)   # Initialize PWM on pwmPin 100Hz frequency
orangePin = GPIO.PWM(16, 100)   # Initialize PWM on pwmPin 100Hz frequency

bluedc=80                              # set blue dc variable to 0 for 0%
orangedc=80
bluePin.start(bluedc)
orangePin.start(orangedc)
bluePin.ChangeDutyCycle(bluedc)
orangePin.ChangeDutyCycle(orangedc)

def display(str):
    text = font.render(str, True, (255, 255, 255), (159, 182, 205))
    textRect = text.get_rect()
    textRect.centerx = screen.get_rect().centerx
    textRect.centery = screen.get_rect().centery

    screen.blit(text, textRect)
    pygame.display.update() #Updates display

pygame.init()
screen = pygame.display.set_mode( (640,480) )
pygame.display.set_caption('Use Arrow Keys for steering, Escape to exit') #Sets title of the display window
screen.fill((159, 182, 205))

font = pygame.font.Font(None, 17)

num = 0

linX = 0
angZ = .5

done = False
while not done:

    pygame.event.pump()
    keys = pygame.key.get_pressed()

    bluedc = ((0.16)*(linX**3)) + 0.78 #Conversion from linear input to exponential PWM output
    orangedc = ((0.16)*(angZ**3)) + 0.78

    bluePin.ChangeDutyCycle(bluedc) #Updating PWM
    orangePin.ChangeDutyCycle(orangedc)

    if keys[K_ESCAPE]:
        done = True

bluePin.stop()                     # stop PWM
orangePin.stop()                   # stop PWM
GPIO.cleanup()                     # resets GPIO ports used back to input modei to use pin number when referencing GPIO pins.