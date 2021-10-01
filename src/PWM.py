#  MBTechWorks.com 2016
#  Pulse Width Modulation (PWM) demo to cycle brightness of an LED

import RPi.GPIO as GPIO   # Import the GPIO library.
import time, sys, getopt  # Import libraries

releaseServo = 4.8
blockServo = 3.2


n = len(sys.argv)

servoNumber = int(sys.argv[1])
state = sys.argv[2]

print(servoNumber)
print(state)

GPIO.setmode(GPIO.BOARD)  # Set Pi to use pin number when referencing GPIO pins.
                          # Can use GPIO.setmode(GPIO.BCM) instead to use                           # Broadcom SOC channel names.
GPIO.setup(servoNumber, GPIO.OUT)  # Set GPIO pin to output mode.
pwm = GPIO.PWM(servoNumber, 20)   # Initialize PWM on pwmPin 100Hz frequency

# main loop of program
if state == 'close':
  dutyCycle = blockServo
elif state == 'open':
  dutyCycle = releaseServo
  
pwm.start(dutyCycle)               # Start PWM with 0% duty cycle
time.sleep(0.5)

pwm.stop()                         # stop PWM
GPIO.cleanup()                     # resets GPIO ports used back to input mode

