#!/usr/bin/python

# http://makezine.com/projects/tutorial-raspberry-pi-gpio-pins-and-python/

#sudo apt-get update
#sudo apt-get install python-dev python-pip
#sudo pip install --upgrade distribute
#sudo pip install ipython
####sudo pip install --upgrade RPi.GPIO

import sys
import subprocess
import os
import glob

#gpio read
# 3.3VOLTS !!!!! to the gpio pins...
import RPi.GPIO as GPIO


def setup():
    pin_start_movie = 4
    pin_reset = 6
    pin_scrensaver = 8
    pin_go_to_zero_positions = 10

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin_start_movie, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(pin_reset, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(pin_screensaver, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(pin_go_to_zero_positions, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    GPIO.add_event_detect(pin_start_movie, GPIO.RISING)
    GPIO.add_event_detect(pin_reset, GPIO.RISING)
    GPIO.add_event_detect(pin_screensaver, GPIO.RISING)
    GPIO.add_event_detect(pin_go_to_zero_positions, GPIO.RISING)
	#GPIO.add_event_detect(4, GPIO.BOTH)
    GPIO.add_event_callback(pin_start_movie, movie_callback)
    GPIO.add_event_callback(pin_reset, reset_callback)
    GPIO.add_event_callback(pin_screensaver, screensaver_callback)
    GPIO.add_event_callback(pin_go_to_zero_positions, zero_callback)

    #GPIO.add_event_detect(channel, GPIO.RISING, callback=my_callback, bouncetime=200)
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
#GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
   
#def buttonPressed(value):
#    print('Scored! %s'%value)
   
#GPIO.add_event_detect(17, GPIO.FALLING, callback=lambda x: buttonPressed(50), bouncetime=2000)
#GPIO.add_event_detect(27, GPIO.FALLING, callback=lambda x: buttonPressed(150), bouncetime=2000)

def movie_callback():
    print 'MOVIE PUSHED!'
    infile = '/media/usb/movie.mp4'
    playmovie(infile=infile)
#    GPIO.output(25, GPIO.input(4))

def reset_callback():
    print 'RESET PUSHED!'

def screensaver_callback():
    print 'SCREENSAVER PUSHED!'
    infile = '/media/usb/movie.mp4'
    playmovie(infile=infile)

def zero_callback():
    print 'ZERO PUSHED!'

def playmovie(infile = '/media/usb/movie.mp4'):
  	a = subprocess.call( [ "omxplayer", "-o", "hdmi", infile])


if __name__ == "__main__":
    print('starting raspi stuff')
    setup()

