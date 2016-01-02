#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
import serial
import pexpect 
import time
from uuid import getnode as get_mac

#from pyomxplayer import OMXPlayer

#gpio read
# 3.3VOLTS !!!!! to the gpio pins...
import RPi.GPIO as GPIO

def setup_gpio(master=True):
    pin_start_movie1 = 4
    pin_start_movie2 = 5
    pin_start_movie3 = 6
    pin_start_movie4 = 7
    pin_pause = 8
    pin_stop = 9
    
    GPIO.setmode(GPIO.BCM)
    if master:
        mode=GPIO.OUT
        GPIO.setup(pin_start_movie1,mode)
        GPIO.setup(pin_start_movie2,mode)
        GPIO.setup(pin_start_movie3,mode)
        GPIO.setup(pin_start_movie4,mode)
        GPIO.setup(pin_pause,mode)
        GPIO.setup(pin_stop,mode)
    else:
	mode=GPIO.IN
        GPIO.setup(pin_start_movie1,mode, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(pin_start_movie2,mode, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(pin_start_movie3,mode, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(pin_start_movie4,mode, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(pin_pause,mode, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(pin_stop,mode, pull_up_down=GPIO.PUD_DOWN)

#    GPIO.output(25, GPIO.input(4))

    if slave:
        GPIO.add_event_detect(pin_start_movie1, GPIO.RISING)
        GPIO.add_event_detect(pin_start_movie2, GPIO.RISING)
        GPIO.add_event_detect(pin_start_movie3, GPIO.RISING)
        GPIO.add_event_detect(pin_start_movie4, GPIO.RISING)
        GPIO.add_event_detect(pin_pause, GPIO.RISING)
        GPIO.add_event_detect(pin_stop, GPIO.RISING)

        GPIO.add_event_callback(pin_start_movie1, slave_movie1_callback)
        GPIO.add_event_callback(pin_start_movie2, slave_movie2_callback)
        GPIO.add_event_callback(pin_start_movie3, slave_movie3_callback)
        GPIO.add_event_callback(pin_start_movie4, slave_movie4_callback)
        GPIO.add_event_callback(pin_pause, slave_pause_callback)
        GPIO.add_event_callback(pin_stop, slave_stop_callback)

global mov 
mov = None
def slave_movie1_callback():
    print 'movie1 slave'
    slave_movie = '1.mp4'
    global mov
    mov = pexpect.spawn("/usr/bin/omxplayer -o hdmi -s " + slave_movie)

def slave_movie2_callback():
    print 'movie2 slave'
    slave_movie = '2.mp4'
    global mov
    mov = pexpect.spawn("/usr/bin/omxplayer -o hdmi -s " + slave_movie)

def slave_movie3_callback():
    print 'movie3 slave'
    slave_movie = '3.mpeg'
    global mov 
    mov = pexpect.spawn("/usr/bin/omxplayer -o hdmi -s " + slave_movie)

def slave_movie4_callback():
    print 'movie4 slave'
    slave_movie = '4.ogv'
    global mov 
    mov = pexpect.spawn("/usr/bin/omxplayer -o hdmi -s " + slave_movie)

def slave_pause_callback():
    print 'slave attempting pause'
    global mov
    mov.send('p')

def slave_stop_callback():
    print 'slave attempting stop'
    global mov
    mov.send('q')


class movietime:
    def playmovie(self,infile = '/media/usb/movie.mp4'):
#omxplayer -o both --no-osd --audio_fifo 0.01 --video_fifo 0.01 9de7027baa3f.mp4 

#    infile = "9de7027baa3f.mp4"
        self.loop= OMXPlayer(infile,'-o local', start_playback=True, do_dict=False)
# - See more at: http://www.sundh.com/blog/2013/10/loop-videos-seamlessly-omxplayer/#sthash.3ku3xeZQ.dpuf
 #       return self
    #a = subprocess.call( [ "omxplayer", "-o", "both","--no-osd","--audio_fifo","0.01","--video_fifo","0.01", infile])

    def pausemovie(movie):
        self.loop1.toggle_pause()


def serialread():
    response = serialport.readlines(None)
    print("got: "+str(response))
    if len(response)>0:
        z = response[0]
        return(z)
    return None

def serialwrite(strn='hello'):
    print("sending: "+str(strn))
    serialport.write(strn)

def slaveloop():
    setup_gpio(master=False)
    while(1):
        sys.stdout.write('.')
	time.sleep(0.5)

def masterloop():
    setup_gpio(master=True)
    serialport = serial.Serial("/dev/ttyACM0", 9600, timeout=0.5)
    mov = movietime()
    while(1):
	read_string = serialread()
	if read_string is not None:
	    if 'start' in read_string:
                print('yay starting movie')
		if 'movie1' in read_string:
                    print('starting movie1')
                    a= pexpect.spawn("/usr/bin/omxplayer"+ " -o hdmi -s 1.mp4")
		if 'movie2' in read_string:
                    print('starting movie1')
                    a= pexpect.spawn("/usr/bin/omxplayer"+ " -o hdmi -s 2.mp4")
		if 'movie3' in read_string:
                    print('starting movie1')
                    a= pexpect.spawn("/usr/bin/omxplayer"+ " -o hdmi -s 3.mpeg")
		if 'movie4' in read_string:
                    print('starting movie1')
                    a= pexpect.spawn("/usr/bin/omxplayer"+ " -o hdmi -s 4.ogg")
	    if 'pause' in read_string:
                print('yay pausing movie')
                a.send('p')
	    if 'quit' in read_string:
                print('yay stopping movie')
                a.send('q')


#    setup()
#                movie = playmovie(infile = "9de7027baa3f.mp4")
#                mov.playmovie(infile = "9de7027baa3f.mp4")
#                mov.pausemovie()



if __name__ == "__main__":
    print('starting raspi stuff')
    mac = get_mac()
    if mac == 202481586470451
 	master = True
    else:
	master = False
    print('mac:'+str(mac) +' i am master='+str(master))
#    serialport = serial.Serial("/dev/ttyS0", 9600, timeout=0.5)
    if master:
	masterloop()
    else:
	slaveloop()
