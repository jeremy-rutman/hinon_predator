
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
import logging

#import pyserial

#from pyomxplayer import OMXPlayer

#gpio read
# 3.3VOLTS !!!!! to the gpio pins...
import RPi.GPIO as GPIO

def setup_gpio(master=True):
    global pin_start_movie1
    global pin_start_movie2
    global pin_start_movie3
    global pin_start_movie4 
    global pin_pause
    global pin_stop

    GPIO.setmode(GPIO.BCM)
    if master:
        print('setting up pins as master')    
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
        print('setting up pins as slave')    
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
    global pin_start_movie1
    global pin_start_movie2
    global pin_start_movie3
    global pin_start_movie4 
    global pin_pause
    global pin_stop
#    GPIO.setup(10, GPIO.OUT) 
#    GPIO pin 8 is the input. 
#    GPIO.setup(8, GPIO.IN) 
# Initialise GPIO10 to high (true) so that the LED is off. 
#    GPIO.output(10, True) 
    if GPIO.input(pin_start_movie1): 
	print('pin 1('+str(pin_start_movie1)+')on')
	slave_movie1_callback()

    if GPIO.input(pin_start_movie2): 
	print('pin 2('+str(pin_start_movie2)+')on')
	slave_movie2_callback()

    if GPIO.input(pin_start_movie3): 
	print('pin 3('+str(pin_start_movie3)+')on')
	slave_movie3_callback()

    if GPIO.input(pin_start_movie4): 
	print('pin 4('+str(pin_start_movie4)+')on')
	slave_movie4_callback()

    if GPIO.input(pin_pause): 
	print('pin pause('+str(pin_pause)+')on')
	slave_pause_callback()

    if GPIO.input(pin_stop): 
	print('pin stop('+str(pin_stop)+')on')
	slave_stop_callback()



global mov 
mov = None
def slave_movie1_callback():
    print 'movie1 slave'
    slave_movie = '/home/pi/hinon_predator/5.mp4'
    global mov
    mov = pexpect.spawn("/usr/bin/omxplayer -o hdmi  " + slave_movie)

def slave_movie2_callback():
    print 'movie2 slave'
    slave_movie = '/home/pi/hinon_predator/6.mp4'
    global mov
    mov = pexpect.spawn("/usr/bin/omxplayer -o hdmi  " + slave_movie)

def slave_movie3_callback():
    print 'movie3 slave'
    slave_movie = '/home/pi/hinon_predator/7.mp4'
    global mov 
    mov = pexpect.spawn("/usr/bin/omxplayer -o hdmi " + slave_movie)

def slave_movie4_callback():
    print 'movie4 slave'
    slave_movie = '/home/pi/hinon_predator/8.mp4'
    global mov 
    mov = pexpect.spawn("/usr/bin/omxplayer -o hdmi " + slave_movie)

def slave_pause_callback():
    print 'slave attempting pause'
    global mov
    if mov is not None:
        mov.send('p')

def slave_stop_callback():
    print 'slave attempting stop'
    global mov
    if mov is not None:
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
#    serialport.flush()
#    serialport.write('ears on bro')
#    print("checking")
#    response = serialport.readlines(None)
    response = serialport.readline(None)  #read only single line
#    response = serial.readlines(None)
#    print("got: "+str(response))
    if len(response)>0:
	z=response
#        z = response[0]
#	serialport.flush()
#        print('got:'+str(response))
 	sys.stdout.flush()
        return(z)
    print('got none string')
#    serialport.flush()
    return None


def serialwrite(serialport,strn='hello'):
    print("sending: "+str(strn))
    serialport.write(strn)
#    serial.write(strn)

def slaveloop():
    setup_gpio(master=False)
    n=0
    print('starting slave loop')
    logging.info('starting slave loop')
    while(1):
	n=(n+1)%10
        if n==0:
	    sys.stdout.write('.')
	    sys.stdout.flush()
	    logging.info('.')            
	check_pins()
#	print('a')
	time.sleep(0.05)
	sys.stdout.flush()

def masterloop():
    global pin_start_movie1
    global pin_start_movie2
    global pin_start_movie3
    global pin_start_movie4 
    global pin_pause
    global pin_stop

    minpause = 0.2
    setup_gpio(master=True)
    #ard = serial.Serial(port,9600,timeout=5)
#    serialport = serial.Serial("/dev/ttyUSB0", 9600, timeout=0.5)
    serialport = serial.Serial("/dev/ttyACM0", 9600, timeout=0.5)
#    serialport = serial.Serial("/dev/ttyAMA0", 9600, timeout=0.5)
  #  mov = movietime()
    a = None
    while(1):
	sys.stdout.flush()
#        serialport.flush()
        GPIO.output(pin_start_movie1,False)
        GPIO.output(pin_start_movie2,False)
        GPIO.output(pin_start_movie3,False)
        GPIO.output(pin_start_movie4,False)
        GPIO.output(pin_pause,False)
        GPIO.output(pin_stop,False)

	read_string = serialread(serialport)
	print('read string:'+str(read_string))
        command = "/usr/bin/omxplayer -o hdmi "
        debug = True
        if debug is True:
            command = command + " --win \"100 100 300 300\"  " 
	if read_string is not None:
	    if 'start' in read_string:
                print('yay got start')
		if 'movie1' in read_string:
                    print('starting movie1')
		    GPIO.output(pin_start_movie1,True)
                    a= pexpect.spawn(command + " /home/pi/hinon_predator/1.mp4")
		    time.sleep(minpause)		   
		    GPIO.output(pin_start_movie1,False)
		if 'movie2' in read_string:
                    print('starting movie2')
		    GPIO.output(pin_start_movie2,True)
                    a= pexpect.spawn(command + " /home/pi/hinon_predator/2.mp4")
		    time.sleep(minpause)		   
		    GPIO.output(pin_start_movie2,False)
		if 'movie3' in read_string:
                    print('starting movie3')
		    GPIO.output(pin_start_movie3,True)
                    a= pexpect.spawn(command + " /home/pi/hinon_predator/3.mp4")
		    time.sleep(minpause)		   
		    GPIO.output(pin_start_movie3,False)
		if 'movie4' in read_string:
                    print('starting movie4')
		    GPIO.output(pin_start_movie4,True)
                    a= pexpect.spawn(command + " /home/pi/hinon_predator/4.mp4")
		    time.sleep(minpause)		   
		    GPIO.output(pin_start_movie4,False)
		if 'zeroing' in read_string:
                    print('zeroing, stop movie')
		    GPIO.output(pin_stop,True)
                    if(a):
			a.send('q')
		    time.sleep(minpause)		   
		    GPIO.output(pin_stop,False)
		if 'showoff' in read_string:
                    print('showoff, stop movie')
		    GPIO.output(pin_stop,True)
                    if(a):
			a.send('q')
		    time.sleep(minpause)		   
		    GPIO.output(pin_stop,False)
	    if 'pause' in read_string:
                print('yay pausing movie')
		if a:
		    GPIO.output(pin_pause,True)
                    a.send('p')
		    time.sleep(minpause)		   
		    GPIO.output(pin_pause,False)
	    if 'quit' in read_string:
                print('yay stopping movie')
		if a:
		    GPIO.output(pin_stop,True)
                    a.send('q')
		    time.sleep(minpause)		   
		    GPIO.output(pin_stop,False)
	else:
            pass
#	    print('read string dont make no sense bro')

#    setup()
#                movie = playmovie(infile = "9de7027baa3f.mp4")
#                mov.playmovie(infile = "9de7027baa3f.mp4")
#                mov.pausemovie()



global pin_start_movie1
global pin_start_movie2
global pin_start_movie3
global pin_start_movie4 
global pin_pause
global pin_stop
pin_start_movie1 = 4  #pin 7   see http://pinout.xyz
pin_start_movie2 = 17 #pin 11
pin_start_movie3 = 27 #pin 13
pin_start_movie4 = 22 #pin 15
pin_pause = 10   #pin 12
pin_stop = 9  #pin 16

logging.basicConfig(filename='movie.log',level=logging.CRITICAL)

#a= pexpect.spawn("sudo /usr/bin/omxplayer -o hdmi /home1.mp4")

if __name__ == "__main__":
    print('starting raspi stuff')
#    a= pexpect.spawn("sudo /usr/bin/omxplayer -o hdmi /home/pi/hinon_predator/3.mp3")
#    time.sleep(5)		   
#    a.send('p')
#    time.sleep(5)		   
#    a.send('p')
    logging.info('starting raspi stuff')
    mac = get_mac()
    if mac == 202481586470451: 
 	master = True
        print('found mac saying this is master pi')
    else:
	master = False
    with open('/home/pi/hinon_predator/whoami.txt','r') as f:
        data = f.read()
        print('data:'+str(data))
	if data is not None and 'master' in data:
            master = True
            print('found file saying this is master pi')
    print('mac:'+str(mac) +' i am master='+str(master))
    time.sleep(.1)
#    serialport = serial.Serial("/dev/ttyS0", 9600, timeout=0.5)
    if master:
	masterloop()
    else:
	slaveloop()
