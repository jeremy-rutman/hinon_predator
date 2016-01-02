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
    
    if master:
        mode=GPIO.OUT
    else:
	mode=GPIO.IN

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin_start_movie1,mode, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(pin_start_movie2,mode, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(pin_start_movie3,mode, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(pin_start_movie4,mode, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(pin_pause,mode, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(pin_stop,mode, pull_up_down=GPIO.PUD_DOWN)

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
    strn = "string sending"
#    serialport.write(strn)
#    print("sending: "+str(strn))
    response = serialport.readlines(None)
    print("got: "+str(response))
   # print(response.type())
    if len(response)>0:
        z = response[0]
        return(z)
    return None

def slaveloop():
    setup_gpio()

def masterloop():
    serialport = serial.Serial("/dev/ttyACM0", 9600, timeout=0.5)
    mov = movietime()
    while(1):
	read_string = serialread()
	if read_string is not None:
	    if 'start' in read_string:
                print('yay starting movie')
		if 'movie1' in read_string:
                    print('starting movie1')
                    a= pexpect.spawn("/usr/bin/omxplayer"+ " -o hdmi -s 9de7027baa3f.mp4")
		if 'movie2' in read_string:
                    print('starting movie1')
                    a= pexpect.spawn("/usr/bin/omxplayer"+ " -o hdmi -s 9de7027baa3f.mp4")
		if 'movie3' in read_string:
                    print('starting movie1')
                    a= pexpect.spawn("/usr/bin/omxplayer"+ " -o hdmi -s 9de7027baa3f.mp4")
		if 'movie4' in read_string:
                    print('starting movie1')
                    a= pexpect.spawn("/usr/bin/omxplayer"+ " -o hdmi -s 9de7027baa3f.mp4")
	    if 'pause' in read_string:
                print('yay pausing movie')
                a.send('p')
	    if 'quit' in read_string:
                print('yay stopping movie')
                a.send('q')

a= pexpect.spawn("/usr/bin/omxplayer"+ " -o hdmi -s 9de7027baa3f.mp4")

a.send('q')

#    setup()
#                movie = playmovie(infile = "9de7027baa3f.mp4")
#                mov.playmovie(infile = "9de7027baa3f.mp4")
#                mov.pausemovie()



master = true
if __name__ == "__main__":
    print('starting raspi stuff')
#    serialport = serial.Serial("/dev/ttyS0", 9600, timeout=0.5)
    if master:
	masterloop()
    else:
	slaveloop()
