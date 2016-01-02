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
#import pyserial

#from pyomxplayer import OMXPlayer

#gpio read
# 3.3VOLTS !!!!! to the gpio pins...
import RPi.GPIO as GPIO

def setup_gpio(master=True):
    pin_start_movie1 = 2
    pin_start_movie2 = 3
    pin_start_movie3 = 4
    pin_start_movie4 = 17
    pin_pause = 27
    pin_stop = 22
    
    GPIO.setmode(GPIO.BCM)
    if master:
        mode=GPIO.OUT
        GPIO.setup(pin_start_movie1,mode)
        GPIO.setup(pin_start_movie2,mode)
        GPIO.setup(pin_start_movie3,mode)
        GPIO.setup(pin_start_movie4,mode)
        GPIO.setup(pin_pause,mode)
        GPIO.setup(pin_stop,mode)

        GPIO.output(pin_start_movie1,False)
        GPIO.output(pin_start_movie2,False)
        GPIO.output(pin_start_movie3,False)
        GPIO.output(pin_start_movie4,False)
        GPIO.output(pin_pause,False)
        GPIO.output(pin_stop,False)

    else:
	mode=GPIO.IN
#        GPIO.setup(pin_start_movie1,mode, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(pin_start_movie1,mode)
        GPIO.setup(pin_start_movie2,mode)
        GPIO.setup(pin_start_movie3,mode)
        GPIO.setup(pin_start_movie4,mode)
        GPIO.setup(pin_pause,mode)
        GPIO.setup(pin_stop,mode)

#    GPIO.output(25, GPIO.input(4))

    if not master:
	edge=GPIO.BOTH
#        GPIO.add_event_detect(pin_start_movie1, edge)
#        GPIO.add_event_detect(pin_start_movie2, edge)
#        GPIO.add_event_detect(pin_start_movie3, edge)
#        GPIO.add_event_detect(pin_start_movie4, edge)
#        GPIO.add_event_detect(pin_pause, edge)
#        GPIO.add_event_detect(pin_stop, edge)

#        GPIO.add_event_callback(pin_start_movie1, slave_movie1_callback)
#        GPIO.add_event_callback(pin_start_movie2, slave_movie2_callback)
#        GPIO.add_event_callback(pin_start_movie3, slave_movie3_callback)
#        GPIO.add_event_callback(pin_start_movie4, slave_movie4_callback)
#        GPIO.add_event_callback(pin_pause, slave_pause_callback)
#        GPIO.add_event_callback(pin_stop, slave_stop_callback)

def check_pins():
    pin_start_movie1 = 2
    pin_start_movie2 = 3
    pin_start_movie3 = 4
    pin_start_movie4 = 17
    pin_pause = 27
    pin_stop = 22
#    GPIO.setup(10, GPIO.OUT) 
#    GPIO pin 8 is the input. 
#    GPIO.setup(8, GPIO.IN) 
# Initialise GPIO10 to high (true) so that the LED is off. 
#    GPIO.output(10, True) 
    if GPIO.input(pin_start_movie1): 
	print('pin 1 on')
	slave_movie1_callback()
    if GPIO.input(pin_start_movie2): 
	print('pin 2 on')
	slave_movie1_callback()
    if GPIO.input(pin_start_movie3): 
	print('pin 3 on')
	slave_movie1_callback()
    if GPIO.input(pin_start_movie4): 
	print('pin 4 on')
	slave_movie1_callback()
    if GPIO.input(pin_pause): 
	print('pin 5 on')
	slave_pause_callback()
    if GPIO.input(pin_stop): 
	print('pin 6 on')
	slave_stop_callback()



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


def serialread(serialport):
    serialport.flush()
    serialport.write('ears on bro')
    response = serialport.readlines(None)
#    response = serial.readlines(None)
    print("got: "+str(response))
    if len(response)>0:
        z = response[0]
	serialport.flush()
        return(z)
    print('got none string')
    serialport.flush()
    return None




def serialwrite(serialport,strn='hello'):
    print("sending: "+str(strn))
    serialport.write(strn)
#    serial.write(strn)

def slaveloop():
    setup_gpio(master=False)
    while(1):
        sys.stdout.write('.')
	check_pins()
	time.sleep(0.05)

def masterloop():
    pin_start_movie1 = 2
    pin_start_movie2 = 3
    pin_start_movie3 = 4
    pin_start_movie4 = 17
    pin_pause = 27
    pin_stop = 22
    minpause = 0.200
    setup_gpio(master=True)
    #ard = serial.Serial(port,9600,timeout=5)
    serialport = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
  #  mov = movietime()
    while(1):
	read_string = serialread(serialport)
	print('read string:'+str(read_string))
	if read_string is not None:
	    if 'start' in read_string:
                print('yay starting movie')
		if 'movie1' in read_string:
                    print('starting movie1')
                    a= pexpect.spawn("/usr/bin/omxplayer"+ " -o hdmi -s 1.mp4")
		    GPIO.output(pin_start_movie1,True)
		    time.sleep(minpause)		   
		    GPIO.output(pin_start_movie1,False)
		if 'movie2' in read_string:
                    print('starting movie2')
                    a= pexpect.spawn("/usr/bin/omxplayer"+ " -o hdmi -s 2.mp4")
		    GPIO.output(pin_start_movie1,True)
		    time.sleep(minpause)		   
		    GPIO.output(pin_start_movie1,False)
		if 'movie3' in read_string:
                    print('starting movie3')
                    a= pexpect.spawn("/usr/bin/omxplayer"+ " -o hdmi -s 3.mpeg")
		    GPIO.output(pin_start_movie1,True)
		    time.sleep(minpause)		   
		    GPIO.output(pin_start_movie1,False)
		if 'movie4' in read_string:
                    print('starting movie4')
                    a= pexpect.spawn("/usr/bin/omxplayer"+ " -o hdmi -s 4.ogg")
		    GPIO.output(pin_start_movie1,True)
		    time.sleep(minpause)		   
		    GPIO.output(pin_start_movie1,False)
	    if 'pause' in read_string:
                print('yay pausing movie')
                a.send('p')
		GPIO.output(pin_pause,True)
		time.sleep(minpause)		   
		GPIO.output(pin_pause,False)
	    if 'quit' in read_string:
                print('yay stopping movie')
                a.send('q')
		GPIO.output(pin_stop,True)
		time.sleep(minpause)		   
		GPIO.output(pin_stop,False)
	else:
	    print('read string dont make no sense bro')

#    setup()
#                movie = playmovie(infile = "9de7027baa3f.mp4")
#                mov.playmovie(infile = "9de7027baa3f.mp4")
#                mov.pausemovie()



if __name__ == "__main__":
    print('starting raspi stuff')
    mac = get_mac()
    if mac == 202481586470451:
 	master = True
    else:
	master = False
    print('mac:'+str(mac) +' i am master='+str(master))
#    serialport = serial.Serial("/dev/ttyS0", 9600, timeout=0.5)
    if master:
	masterloop()
    else:
	slaveloop()
