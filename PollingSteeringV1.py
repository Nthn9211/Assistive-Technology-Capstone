import pygame
from pygame.locals import *

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
    pygame.display.update()

pygame.init()
screen = pygame.display.set_mode( (640,480) )
pygame.display.set_caption('Use WASD for steering, Q to full stop, Escape to exit')
screen.fill((159, 182, 205))

font = pygame.font.Font(None, 17)

num = 0
done = False
while not done:
    display( str(num) )

    pygame.event.pump()
    keys = pygame.key.get_pressed()
    if keys[K_LEFT]:
        orangedc = 63.3
        orangePin.ChangeDutyCycle(orangedc)
    elif keys[K_RIGHT]:
        orangedc = 96.3
        orangePin.ChangeDutyCycle(orangedc)
    else:
        orangedc = 80
        orangePin.ChangeDutyCycle(orangedc)

    if keys[K_UP]:
        bluedc = 96
        bluePin.ChangeDutyCycle(bluedc)
    elif keys[K_DOWN]:
        bluedc = 63
        bluePin.ChangeDutyCycle(bluedc)
    else:
        bluedc = 80
        bluePin.ChangeDutyCycle(bluedc)

    if keys[K_ESCAPE]:
        done = True

bluePin.stop()                         # stop PWM
orangePin.stop()                         # stop PWM
GPIO.cleanup()                     # resets GPIO ports used back to input modei to use pin number when referencing GPIO pins.