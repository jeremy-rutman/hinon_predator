#include <PololuMaestro.h>
#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008
#include <SoftwareSerial.h>


//UDP comms setup
byte mac[] = {  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(185,184,183, 15);
unsigned int localPort = 5000;      // local port to listen on
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
char  ReplyBuffer[] = "acknowledged";       // a string to send back
EthernetUDP Udp;

//maestro 12ch servo controller setup
uint8_t MAESTRO_RX_PIN = 8;
uint8_t MAESTRO_TX_PIN = 9;
SoftwareSerial maestroSerial(MAESTRO_RX_PIN, MAESTRO_TX_PIN);  //10:RX   11:TX
MicroMaestro maestro(maestroSerial);

//servo upper/center/lower limits in quarters of a microsecond 

//                     0    1    2    3    4     5    6    7   8   9     10   11 
//                     arm roll pit  yaw  flp   ail  rud ele hood mtdy  mtdp  whl
int servoMins[] =    {4600,5800,5700,4000,6000,5000,5900,4000,4200,4300,3700,3900};
int servoCenters[] = {5900,6100,6100,4500,7000,6500,6350,6300,7200,5900,5822,6000};
int servoMaxes[] =   {6600,6400,6700,8000,7312,7100,6800,7800,7200,7800,6500,8100};
//SERVO NUMBERS - channel numbers out of the maestro servo controller and into hinon's system
uint8_t ROLL = 1;
uint8_t PIT = 2;  //pitch
uint8_t YAW = 3;
uint8_t FLP = 4; //was 4,5
uint8_t AIL = 5; //was 6,7 
uint8_t RUD = 6; //was 8    
uint8_t ELE = 7; // reversed
uint8_t HOOD = 8; //was 11   
uint8_t MTD_YA = 9;     //12 
uint8_t MTD_PI = 10; //13     
uint8_t WHL = 11; //on off

//numbers to refer to other parts of system 
uint8_t MOTOR = 12;
uint8_t LED = 13;
uint8_t ARM = 0;  //this is the servo for hinon's "servo Hbridge"
uint8_t ROLLYAW = 15;  //not being used  for simultaneous roll/yaw

//arduino pins for nonservo parts of system  running lowside nmos switches on arduino shield
uint8_t MOTOR_PIN = 3;      
uint8_t LED_PIN = 6;
uint8_t ARM_FWD_1 = 4;  //check these
uint8_t ARM_FWD_2 = 7;
uint8_t ARM_REV_1 = 5;
uint8_t ARM_REV_2 = 2;

 //status codes  what are we doing (current activity) and wht are we supposed to be doing (status is command from button or serial (udp))
uint8_t STATUS = 0; 
uint8_t RETURN_TO = 0; 
uint8_t CURRENT_ACTIVITY = 0;
uint8_t LEFTRIGHT = 0;

const uint8_t ZERO = 50;   //zeroing system (aka 'reset')
const uint8_t MOVIE = 51; 
const uint8_t SHOWOFF = 53;   //aka 'demo'
const uint8_t SCREENSAVER = 54; 
const uint8_t PAUSE = 55; 

const uint8_t WAITING = 0;   //waiting for command 
const uint8_t PSPEED = 50;

const uint8_t LEFT = 0;
const uint8_t RIGHT = 1;
const uint8_t OPEN = 1;
const uint8_t CLOSED = 0; 
const uint8_t FORWARD = 2;
const uint8_t REVERSE = 3; 

void setup()
{
  STATUS = 0;
  // Set the serial baud rate.
  Serial.begin(9600);
  Serial.println("doing setup");
  maestroSerial.begin(9600);
  
  // start the Ethernet and UDP:
//  Ethernet.begin(mac,ip,gateway,gateway,subnet);
  Ethernet.begin(mac,ip);
  Udp.begin(localPort);
//  server.begin(); 
 
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

 pinMode(ARM_FWD_1, OUTPUT);
 pinMode(ARM_FWD_2, OUTPUT);
 pinMode(ARM_REV_1, OUTPUT);
 pinMode(ARM_REV_2, OUTPUT);

  pinMode(A0,INPUT);  //these are arduino pin numbers for buttonpresses
  pinMode(A1,INPUT);  //these are arduino pin numbers for buttonpresses
  pinMode(A2,INPUT);  //these are arduino pin numbers for buttonpresses
  pinMode(A3,INPUT);
  pinMode(A4,INPUT);
  pinMode(A5,INPUT);

  
  zero_all();
}

int position_rel(int servonum, int relative_position)
{
   int SERVOCENTER = servoCenters[servonum]; 
   int SERVOMAX = servoMaxes[servonum]; 
   int SERVOMIN = servoMins[servonum]; 
   int qmicros = 0;//
//    Serial.print("servonum "); Serial.print(servonum);
   if(servonum>20) servonum=servonum-20;
//    Serial.print(" using servonum "); Serial.println(servonum);
   if (relative_position>100)
 {
   Serial.println("too high value in position_rel");
   return 0;
 }
   if (relative_position<0)
 {
   Serial.println("too low value in position_rel");
   return 0;
 }
 if(relative_position>50)
  {
    qmicros = SERVOCENTER+ (SERVOMAX-SERVOCENTER)*(float)(relative_position-50.0)/50.0;
  }
  else  //this is retarded, both cases can be taken care of in the line above
  {
    qmicros = SERVOCENTER-(SERVOCENTER-SERVOMIN)*(float)(50.0-relative_position)/50.0;    
  }
//  Serial.print(qmicros);     Serial.print(" micros to ");Serial.println(servonum);
  maestro.setTarget(servonum, qmicros);
  return(docheck());
  
}



int move_rel(int servonum,int current,int dest,int del)
{

  Serial.print("servo:");
  Serial.print(servonum); 
  Serial.print(" from:");
  Serial.print(current);
  Serial.print(" to:");
  Serial.print(dest);
  Serial.print(" delay:");
  Serial.println(del);
  int i;
  
  if (dest>current) 
  {
   for (int pulselen = current; pulselen < dest; pulselen+= 1) 
    { 
  //     Serial.print(pulselen); Serial.print(" length for channel ");Serial.println(servonum); 
      if(position_rel(servonum, pulselen))
      {
        return(1);
      }
      
      if(mydelay(del)) return 1;
    }
  }
  else
  {
      if(position_rel(servonum, current)) return(1);

   for (int pulselen = current; pulselen > dest; pulselen-= 1) 
    { 
    //   Serial.print(pulselein); Serial.print(" length for channel ");Serial.println(servonum); 
      if(position_rel(servonum, pulselen)) return(1);
      if(mydelay(del)) return 1;
    }    
  }
}

int zero_all() 
{
//   arm(FORWARD);
 //  mydelay(5000);
 //  arm(REVERSE);
 //mydelay(5000);
 //arm(CLOSED);
 //mydelay(5000);

  CURRENT_ACTIVITY= ZERO;
   arm(REVERSE);
  Serial.println("zeroing all");
  
      Serial.println("quit movie");
      Serial.println("quit movie");
Serial.flush();
    motor(0);
  for (uint16_t servonum = 1; servonum < 11; servonum++) 
    {
Serial.print("zeroing ");Serial.println(servonum);
      if(position_rel(servonum, 50)) return 1;
      if(mydelay(100)) return 1;
      if(docheck()) return 1;      
    }
  if(wheels(CLOSED)) return 1;
    //move_rel(WHL,0,100,10);
    motor(0);
    led(0);
     if(mydelay(11000)) return 1;
 //  arm(FORWARD);
 //   delay(5000);
    arm(CLOSED);
    Serial.println("done zeroing");
//    delay(500);
  CURRENT_ACTIVITY = WAITING;
}

 
uint8_t mydelay(unsigned long mymsec)
{
  unsigned long msec = (unsigned long) mymsec;
  int curStatus;
  int savedStatus;
  unsigned long orig_time = millis();  
  unsigned long cur_time = millis(); 
  
  while ((unsigned long)(cur_time-orig_time)< (unsigned long) msec)
  {
    //delay(200);
//    Serial.print("cur time ");Serial.print(cur_time);
 //   Serial.print(" orig time ");Serial.print(orig_time);
  //  Serial.print(" wait time ");Serial.print(msec);
   // Serial.print(" diff time ");Serial.println(cur_time-orig_time);
    if(curStatus = docheck())
   {
     delay(200);
     return 1; 
   }     
    cur_time = millis();
  }
  return 0;
}


//wheels up or down
uint8_t wheels(int command)
{
   int servonum = WHL;
   if (command==CLOSED)
    {
    Serial.println("wheels up (closed)");
        if(move_rel(servonum, 99,100,2)) return 1; 
        if(mydelay(200)) return 1;
    }
    else
    {
    Serial.println("wheels down (open)");
       if( move_rel(servonum, 2,0,2)) return 1; 
        if(mydelay(200)) return 1;
    }
}

//open or close cockpit hood
int  hood(int command)
{
  int servonum = HOOD;
  int SERVOMIN = servoMins[servonum]; 
  int SERVOCENTER = servoCenters[servonum]; 
  int SERVOMAX = servoMaxes[servonum] ;
  if (command == OPEN)
  {
    //yinnon   change here  move_rel(servonum,start,finish,delay(ms))
      if(move_rel(servonum,0,100,40)) return 1 ;
  }
  else   //close
  {
    //and here
      if(move_rel(servonum,100,0,40)) return 1;
  }
}

void motor(int pwm_level)
{
  int normalized_level;
 normalized_level = (int)((float)pwm_level*2.55);
  Serial.print("motor command, normalized level=");Serial.print(pwm_level);Serial.print(" ");Serial.println(normalized_level);
   analogWrite(MOTOR_PIN,normalized_level); 
}

void led(int pwm_level)
{
  int normalized_level;
 normalized_level = (int)((float)pwm_level*2.55);
  Serial.print("LED command normalized level=");Serial.print(pwm_level);Serial.print(" ");Serial.println(normalized_level);
   analogWrite(LED_PIN,normalized_level); 
}

int arm(int direction)
{
     if(direction==FORWARD)
   {
     Serial.println("arm forward");
      digitalWrite(ARM_FWD_1,HIGH);
     digitalWrite(ARM_FWD_2,HIGH);
      delay(100);
     digitalWrite(ARM_REV_1,LOW);
     digitalWrite(ARM_REV_2,LOW);

//      position_rel(ARM,100);
 //     digitalWrite(A0,HIGH);
 //     digitalWrite(A1,LOW);
   } 
   else if(direction==REVERSE)
   {
     Serial.println("arm reverse");
     digitalWrite(ARM_REV_1,HIGH);
     digitalWrite(ARM_REV_2,HIGH);
     delay(100);
     digitalWrite(ARM_FWD_1,LOW);
     digitalWrite(ARM_FWD_2,LOW);
//      digitalWrite(A0,LOW);
  //    digitalWrite(A1,HIGH);

  //    position_rel(ARM,0);
   } 
   else if(direction==CLOSED)
   {
     Serial.println("arm off");
     digitalWrite(ARM_REV_1,HIGH);
     digitalWrite(ARM_REV_2,HIGH);
     digitalWrite(ARM_FWD_1,HIGH);
     digitalWrite(ARM_FWD_2,HIGH);
 //     digitalWrite(A0,LOW);
//      digitalWrite(A1,LOW);
//      position_rel(ARM,50);
   } 
   return docheck();
}

//do everthing one by one
int showoff()
{
    Serial.println("quit movie");

  Serial.println("doing showoff");
 delay(100);
 CURRENT_ACTIVITY = SHOWOFF;
  int min=1;

  led(0);
  if(mydelay(1000)) return(1);
  if(docheck()) return(1);
  led(100);

  arm(FORWARD);
  if(mydelay(15000)) return(1);
  if(docheck()) return(1);

  arm(CLOSED);
  if(mydelay(2000)) return(1);
  if(docheck()) return(1);

  motor(0);
  if(mydelay(2000)) return(1);
  if(docheck()) return(1);
  motor(100);
  if(mydelay(2000)) return(1);
  if(docheck()) return(1);
 // motor(0);
  
  for(int servonum=1;servonum<3;servonum++)
  {
//    move_rel(servonum,50,51,100);
    if(move_rel(servonum,50,100,20)) return(1);
    if(move_rel(servonum,100,0,20)) return(1);
    if(move_rel(servonum,0,50,20)) return 1;
    if(mydelay(1000)) return(1);
 //   if (docheck() )return(1);
  }
  
    if(move_rel(YAW,50,100,40)) return 1;
    if(move_rel(YAW,100,0,40)) return 1;
   if( move_rel(YAW,0,50,40)) return 1;

    for(int servonum=4;servonum<11;servonum++)
  {
//    move_rel(servonum,50,51,100);
   if( move_rel(servonum,50,100,20))return(1);
    if(move_rel(servonum,100,0,20))return(1);
    if(move_rel(servonum,0,50,20))return(1);
    if(mydelay(1000)) return(1);
  }
  
   if( move_rel(WHL,50,100,10))return(1);  //wheels OPEN
    if(mydelay(5000)) return(1);
    if(move_rel(WHL,100, 0,10))return(1);   //wheels OPEN
    if(mydelay(5000)) return(1);
    
    motor(0);
  led(0);
  arm(REVERSE);
  if(mydelay(10000)) return(1);
  CURRENT_ACTIVITY = WAITING;
  Serial.println("done showoff");
}

int screensaver()
{
      Serial.println("doing screensaver");

      Serial.println("quit movie");
  RETURN_TO = WAITING;
  CURRENT_ACTIVITY = SCREENSAVER;
  if(LEFTRIGHT==LEFT) Serial.println("start movie3");
  else Serial.println("start movie4");

  if(wheels(OPEN)) return(1);
  motor(100);
  led(100);
  if(docheck()) return(1);
  led(100);

//takeoff
  if(move_rel(ELE,50,0,20)) return 1;
  if(move_rel(FLP,50,0,50)) return 1;
  if(move_rel(PIT,50,40,50)) return 1;
  if(mydelay(500)) return 1;
  if(arm(FORWARD)) return(1);
  if(mydelay(4500)) return 1;
  move_rel(PIT,40,15,50);
  if(wheels(CLOSED)) return(1);
  if(mydelay(2000)) return(1);

//straighten
  move_rel(ELE,0,50,20);
  move_rel(FLP,0,50,50);
  move_rel(PIT,35,50,50);

  if(mydelay(2000)) return(1);

//random movearound
  if(docheck()) return(1);
  if(move_rel(YAW,50,55,50)) return 1;
  if(move_rel(ROLL,50,40,100)) return 1;
  if(move_rel(FLP,50,0,50)) return 1;
  
  if(mydelay(2000)) return(1);

  if(docheck()) return(1);
  if(move_rel(YAW,55,45,50)) return 1;
  if(move_rel(ROLL,40,60,80)) return 1;
  if(move_rel(FLP,0,100,50)) return 1;
  
  
//landing
  move_rel(ELE,50,100,20);
  move_rel(PIT,50,45,50);
  if(mydelay(500)) return 1;
  if(arm(REVERSE)) return(1);
  if(mydelay(1000)) return 1;
  if(wheels(OPEN)) return(1);
  if(mydelay(3000)) return 1;
  move_rel(PIT,45,50,50);
  if(mydelay(3000)) return 1;
  move_rel(ELE,100,50,20);
  motor(0);
  led(0);
  if(mydelay(5000)) return 1;
// Alll this needs to finish by 20s

  for(int i=0;i<3;i++)
  {
    led(100);
    if(move_rel(ELE,50,0,100)) return 1;
    if(mydelay(1000)) return 1;
    if(move_rel(FLP,50,0,100)) return 1;
    if(mydelay(1000)) return 1;
  //  if(move_rel(PIT,50,40,50)) return 1;
  
    if(move_rel(ELE,0,100,100)) return 1;
    if(mydelay(1000)) return 1;
    if(move_rel(FLP,0,100,100)) return 1;
    if(mydelay(1000)) return 1;
  
  // 
    if(move_rel(ELE,100,50,100)) return 1;
    if(mydelay(1000)) return 1;
    if(move_rel(FLP,0,100,100)) return 1;
    led(0);
    if(mydelay(1000)) return 1;
  }

    if(mydelay(5000)) return 1;
    Serial.println('finished screensaver');
    Serial.println('quit movie');
    
    RETURN_TO=SCREENSAVER;
  
}
// ROLL PIT  YAW  FLP AIL RUD ELE HOOD  MTD_YA  MTD_PI  WHL = 11; //on off
// WHL = 11;MOTOR = 12; LED = 13;ARM = 14;


uint16_t index = 0;
 
unsigned long st_millis=0;
unsigned long t_millis=0;
unsigned long t_seconds=0;
unsigned long start_seconds=0;
   const int  PREDELAY = 0;
int movie1()
{
Serial.println('quit movie'); //stop any currently running movie
  STATUS = 0;
  if(LEFTRIGHT==LEFT) Serial.println("start movie1");
  else Serial.println("start movie2");
  CURRENT_ACTIVITY = MOVIE;
  RETURN_TO = WAITING;
  const uint8_t movielength = 95;  //number of actiofns in movie  warning if this isn't high enough, you are writing into random memory 
  int timeline[movielength];
  uint8_t tl_servos[movielength] ;
  uint8_t tl_args1[movielength] ;
  uint8_t tl_args2[movielength] ;
  uint8_t tl_args3[movielength] ;

//we define the movie actions by means of a timeline
//initialize movie timeline with all zeros. if this length is too short (less than numbner of movie commands) you are writing into unprotected memory ...
  int i=0;
  for(i=0;i<movielength;i++)
  {
      timeline[i]=0;    //when to do something
      tl_servos[i]=0;   //which servo/action to do 
      tl_args1[i]=0;    //initial position for servo  these are from 0 to 100, so 50 is in the middle or 'zeroed' position
      tl_args2[i]=0;   //final position for servo from 0 to 100
      tl_args3[i]=0;   //mSec to wait between subsequent servo steps (slows down servo movement)
  }

//  const int8_t INIT_DELAY = 5 ;
//  if (mydelay(INIT_DELAY*1000)) return;
  const int8_t EXTRA_TIME = 1;  //delay time for syncing movie to command from control computer (seconds)
  i=0;

//the timeline !

  //zero stuff out and wait 
//lower arm (its its not already down)
  timeline[i]=-2+EXTRA_TIME;
  tl_servos[i]=ARM;
  tl_args1[i]=REVERSE;
  i++;
//motor off
  timeline[i]=-2+EXTRA_TIME;
  tl_servos[i]=MOTOR;
  tl_args1[i]=0;
  i++;  
//wheels down
  timeline[i]=0+EXTRA_TIME;
  tl_servos[i]=WHL;
  tl_args1[i]=OPEN;
  i++;  
//LED on
  timeline[i]=0+EXTRA_TIME;
  tl_servos[i]=LED;
  tl_args1[i]=100;  //100% on - this is a PWM level from 0 to 100
  i++;  

//start your engines
  timeline[i]=13+EXTRA_TIME;
  tl_servos[i]=MOTOR;
  tl_args1[i]=100;   //100% on  this is a PWM level from 0 to 100
  i++;  
//takeoff
  timeline[i]=20+EXTRA_TIME;
  tl_servos[i]=ELE;  //elevator (back flap)
  tl_args1[i]=50;   //start position (50 is centered)
  tl_args2[i]=20;      //end position (0 is minimum) 
  tl_args3[i]=20;  //'slowness'  # of mSec pause between each servo step  (integer steps bet. start and end, so tot time taken (in mSec) is (startend)*slowness
  i++;   
  timeline[i]=20+EXTRA_TIME;
  tl_servos[i]=FLP;   //front main flaps
  tl_args1[i]=50;
  tl_args2[i]=0;
  tl_args3[i]=50;
  i++;   
  timeline[i]=24+EXTRA_TIME;
  tl_servos[i]=PIT;   //pitch  tilt airplane
  tl_args1[i]=50;
  tl_args2[i]=47;
  tl_args3[i]=50;
  i++;   
  timeline[i]=24+EXTRA_TIME;
  tl_servos[i]=ARM;
  tl_args1[i]=FORWARD;
  i++;   
  timeline[i]=30+EXTRA_TIME;
  tl_servos[i]=PIT;
  tl_args1[i]=47;
  tl_args2[i]=15;
  tl_args3[i]=50;
  i++;   

//wheels up
  timeline[i]=35+EXTRA_TIME;
  tl_servos[i]=WHL;
  tl_args1[i]=CLOSED;
  i++;   

//straighten out at max height
timeline[i]=42+EXTRA_TIME;
  tl_servos[i]=PIT;
  tl_args1[i]=15;
  tl_args2[i]=50;
  tl_args3[i]=50;
  i++;     
  timeline[i]=43+EXTRA_TIME;
  tl_servos[i]=FLP;
  tl_args1[i]=0;
  tl_args2[i]=50;
  tl_args3[i]=50;
  i++;     
  timeline[i]=44+EXTRA_TIME;
  tl_servos[i]=ELE;
  tl_args1[i]=0;
  tl_args2[i]=50;
  tl_args3[i]=50;
  i++;     

  timeline[i]=47+EXTRA_TIME;
  tl_servos[i]=ARM;
  tl_args1[i]=CLOSED;
  i++;   

//random roll
  timeline[i]=50+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=50;
  tl_args2[i]=40; 
  tl_args3[i]=30;
  i++;   

//roll back
  timeline[i]=52+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=40;
  tl_args2[i]=50;
  tl_args3[i]=30;
  i++;   
  
//camera on FIRST TARGET
timeline[i]=56+EXTRA_TIME;
  tl_servos[i]=MTD_YA;   //'mated yaw'  turn camera left/right
  tl_args1[i]=50;
  tl_args2[i]=70;
  tl_args3[i]=60;
  i++;     
  timeline[i]=58+EXTRA_TIME;
  tl_servos[i]=MTD_PI;  //'mated pitch'  turn camera up/down
  tl_args1[i]=70;
  tl_args2[i]=00;
  tl_args3[i]=30;
  i++;     
  
  
//camera on SECOND TARGET 
  timeline[i]=60+5+EXTRA_TIME;
  tl_servos[i]=MTD_YA;
  tl_args1[i]=50;
  tl_args2[i]=60;
  tl_args3[i]=20;
  i++;     
  timeline[i]=60+5+EXTRA_TIME;
  tl_servos[i]=MTD_PI;
  tl_args1[i]=70;
  tl_args2[i]=0;
  tl_args3[i]=30;
  i++;     

//camera on THIRD TARGET
timeline[i]=60+7+EXTRA_TIME;
  tl_servos[i]=MTD_YA;
  tl_args1[i]=50;
  tl_args2[i]=60;
  tl_args3[i]=20;
  i++;     
  timeline[i]=60+7+EXTRA_TIME;
  tl_servos[i]=MTD_PI;
  tl_args1[i]=70;
  tl_args2[i]=0;
  tl_args3[i]=30;
  i++;     

  
//random roll
  timeline[i]=60+9+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=50;
  tl_args2[i]=30;
  tl_args3[i]=100;
  i++;   
//roll back
  timeline[i]=60+12+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=30;
  tl_args2[i]=50;
  tl_args3[i]=100;
  i++;   


//random roll
  timeline[i]=60+15+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=50;
  tl_args2[i]=60;
  tl_args3[i]=100;
  i++;   
//roll back
  timeline[i]=60+17+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=60;
  tl_args2[i]=50;
  tl_args3[i]=100;
  i++;   
  
    //camera on FOURTH TARGET
  timeline[i]=60+21+EXTRA_TIME;
  tl_servos[i]=MTD_YA;
  tl_args1[i]=50;
  tl_args2[i]=60;
  tl_args3[i]=50;
  i++;     
  timeline[i]=60+22+EXTRA_TIME;
  tl_servos[i]=MTD_PI;
  tl_args1[i]=70;
  tl_args2[i]=0;
  tl_args3[i]=30;
  i++;     

  //random roll
  timeline[i]=60+23+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=50;
  tl_args2[i]=80;
  tl_args3[i]=80;
  i++;   
//roll back
  timeline[i]=60+17+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=80;
  tl_args2[i]=50;
  tl_args3[i]=80;
  i++;   
  


//random roll
  timeline[i]=60+26+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=50;
  tl_args2[i]=20;
  tl_args3[i]=100;
  i++;   
//roll back
  timeline[i]=60+28+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=20;
  tl_args2[i]=50;
  tl_args3[i]=100;
  i++;   
  
//camera on FIFTH TARGET
  timeline[i]=60+34+EXTRA_TIME;
  tl_servos[i]=MTD_YA;
  tl_args1[i]=50;
  tl_args2[i]=60;
  tl_args3[i]=40;
  i++;     
  timeline[i]=60+34+EXTRA_TIME;
  tl_servos[i]=MTD_PI;
  tl_args1[i]=70;
  tl_args2[i]=0;
  tl_args3[i]=350;
  i++;     

    //camera on 6th TARGET
  timeline[i]=60+44+EXTRA_TIME;
  tl_servos[i]=MTD_YA;
  tl_args1[i]=50;
  tl_args2[i]=60;
  tl_args3[i]=30;
  i++;     
  timeline[i]=60+44+EXTRA_TIME;
  tl_servos[i]=MTD_PI;
  tl_args1[i]=70;
  tl_args2[i]=0;
  tl_args3[i]=30;
  i++;     

//camera on 7th TARGET
  timeline[i]=60+52+EXTRA_TIME;
  tl_servos[i]=MTD_YA;
  tl_args1[i]=50;
  tl_args2[i]=40;
  tl_args3[i]=30;
  i++;     
  timeline[i]=60+52+EXTRA_TIME;
  tl_servos[i]=MTD_PI;
  tl_args1[i]=70;
  tl_args2[i]=0;
  tl_args3[i]=30;
  i++;     

//camera on 8th TARGET
  timeline[i]=60+55+EXTRA_TIME;
  tl_servos[i]=MTD_YA;
  tl_args1[i]=50;
  tl_args2[i]=40;
  tl_args3[i]=30;
  i++;     
  timeline[i]=60+55+EXTRA_TIME;
  tl_servos[i]=MTD_PI;
  tl_args1[i]=70;
  tl_args2[i]=0;
  tl_args3[i]=30;
  i++;     

//RANDOM ROLL
  timeline[i]=60+58+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=50;
  tl_args2[i]=45;
  tl_args3[i]=100;
  i++;     
  timeline[i]=120+1+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=45;
  tl_args2[i]=50;
  tl_args3[i]=100;
  i++;     

// slight descent over clouds  
  timeline[i]=120+10+EXTRA_TIME;
  tl_servos[i]=PIT;
  tl_args1[i]=50;
  tl_args2[i]=52;
  tl_args3[i]=30;
  i++;     
  timeline[i]=120+11+EXTRA_TIME;
  tl_servos[i]=ARM;
  tl_args1[i]=REVERSE;
  i++;     
  timeline[i]=120+13+EXTRA_TIME;
  tl_servos[i]=ARM;
  tl_args1[i]=CLOSED;
  i++;     

//RANDOM TILT
timeline[i]=120+20+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=50;
  tl_args2[i]=60;
  tl_args3[i]=80;
  i++;     
  timeline[i]=120+22+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=60;
  tl_args2[i]=50;
  tl_args3[i]=80;
  i++;     

//left turn
  timeline[i]=120+34+EXTRA_TIME;
  tl_servos[i]=AIL;
  tl_args1[i]=50;
  tl_args2[i]=30;
  tl_args3[i]=40;
  i++;
  timeline[i]=120+35+EXTRA_TIME;
  tl_servos[i]=YAW;
  tl_args1[i]=50;
  tl_args2[i]=45;
  tl_args3[i]=100;
  i++;     
  timeline[i]=120+36+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=50;
  tl_args2[i]=40;
  tl_args3[i]=100;
  i++;     

  timeline[i]=120+38+EXTRA_TIME;
  tl_servos[i]=AIL;
  tl_args1[i]=30;
  tl_args2[i]=50;
  tl_args3[i]=50;
  i++;     

//camera on target 9
 timeline[i]=120+40+EXTRA_TIME;
  tl_servos[i]=MTD_YA;
  tl_args1[i]=50;
  tl_args2[i]=40;
  tl_args3[i]=30;
  i++;     
  timeline[i]=120+40+EXTRA_TIME;
  tl_servos[i]=MTD_PI;
  tl_args1[i]=70;
  tl_args2[i]=0;
  tl_args3[i]=30;
  i++;     

//RANDOM TILT
timeline[i]=120+50+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=40;
  tl_args2[i]=45;
  tl_args3[i]=80;
  i++;     
  timeline[i]=120+52+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=45;
  tl_args2[i]=50;
  tl_args3[i]=80;
  i++;     

//RANDOM PITCH
  timeline[i]=120+56+EXTRA_TIME;
  tl_servos[i]=PIT;
  tl_args1[i]=50;
  tl_args2[i]=15;
  tl_args3[i]=50;
  i++;   

//RANDOM PITCH
  timeline[i]=180+EXTRA_TIME;
  tl_servos[i]=PIT;
  tl_args1[i]=15;
  tl_args2[i]=50;
  tl_args3[i]=50;
  i++;   
 
 //camera on target 10
timeline[i]=180+7+EXTRA_TIME;
  tl_servos[i]=MTD_YA;
  tl_args1[i]=50;
  tl_args2[i]=40;
  tl_args3[i]=30;
  i++;     
  timeline[i]=180+7+EXTRA_TIME;
  tl_servos[i]=MTD_PI;
  tl_args1[i]=70;
  tl_args2[i]=0;
  tl_args3[i]=30;
  i++;     

//camera on target 11
timeline[i]=180+9+EXTRA_TIME;
  tl_servos[i]=MTD_YA;
  tl_args1[i]=50;
  tl_args2[i]=40;
  tl_args3[i]=30;
  i++;     
  timeline[i]=180+9+EXTRA_TIME;
  tl_servos[i]=MTD_PI;
  tl_args1[i]=70;
  tl_args2[i]=0;
  tl_args3[i]=50;
  i++;     

//RANDOM TILT
timeline[i]=180+10+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=50;
  tl_args2[i]=80;
  tl_args3[i]=80;
  i++;     
  timeline[i]=180+12+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=80;
  tl_args2[i]=50;
  tl_args3[i]=120;
  i++;     

//camera on target 12
  timeline[i]=180+13+EXTRA_TIME;
  tl_servos[i]=MTD_YA;
  tl_args1[i]=50;
  tl_args2[i]=40;
  tl_args3[i]=30;
  i++;     
  timeline[i]=180+13+EXTRA_TIME;
  tl_servos[i]=MTD_PI;
  tl_args1[i]=70;
  tl_args2[i]=0;
  tl_args3[i]=30;
  i++;     
 
//RANDOM TILT
timeline[i]=180+20+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=50;
  tl_args2[i]=60;
  tl_args3[i]=80;
  i++;     
  timeline[i]=180+25+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=60;
  tl_args2[i]=40;
  tl_args3[i]=120;
  i++;     
  timeline[i]=180+30+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=40;
  tl_args2[i]=50;
  tl_args3[i]=80;
  i++;     


//Random roll
  timeline[i]=180+35+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=45;
  tl_args2[i]=55;
  tl_args3[i]=70;
  i++;
  timeline[i]=180+40+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=55;
  tl_args2[i]=50;
  tl_args3[i]=100;
  i++;

  //camera on target 13
  timeline[i]=180+45+EXTRA_TIME;
  tl_servos[i]=MTD_YA;
  tl_args1[i]=50;
  tl_args2[i]=40;
  tl_args3[i]=30;
  i++;     
  timeline[i]=180+45+EXTRA_TIME;
  tl_servos[i]=MTD_PI;
    tl_args1[i]=100;
  tl_args2[i]=0;
  tl_args3[i]=30;
  i++;     

//random rooll
  timeline[i]=180+46+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=50;
  tl_args2[i]=55;
  tl_args3[i]=50;
  i++;     
  timeline[i]=180+48+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=55;
  tl_args2[i]=50;
  tl_args3[i]=50;
  i++;     


  //camera on target 14
  timeline[i]=180+50+EXTRA_TIME;
  tl_servos[i]=MTD_YA;
  tl_args1[i]=50;
  tl_args2[i]=40;
  tl_args3[i]=30;
  i++;     
  timeline[i]=180+50+EXTRA_TIME;
  tl_servos[i]=MTD_PI;
    tl_args1[i]=70;
  tl_args2[i]=0;
  tl_args3[i]=30;
  i++;     

//STRAIGHTEN 
  timeline[i]=231+EXTRA_TIME;
  tl_servos[i]=YAW;
  tl_args1[i]=45;
  tl_args2[i]=50;
  tl_args3[i]=30;
  i++;     
  timeline[i]=231+EXTRA_TIME;
  tl_servos[i]=AIL;
  tl_args1[i]=30;
  tl_args2[i]=50;
  tl_args3[i]=30;
  i++;     
  timeline[i]=231+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=60;
  tl_args2[i]=50;
  tl_args3[i]=30;
  i++;     


//camera on target 15
  timeline[i]=237+EXTRA_TIME;
  tl_servos[i]=MTD_YA;
  tl_args1[i]=50;
  tl_args2[i]=40;
  tl_args3[i]=30;
  i++;     
  timeline[i]=237+EXTRA_TIME;
  tl_servos[i]=MTD_PI;
   tl_args1[i]=70;
  tl_args2[i]=0;
  tl_args3[i]=20;
 i++;     

    timeline[i]=240+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=50;
  tl_args2[i]=45;
  tl_args3[i]=80;
  i++;     
    timeline[i]=241+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=45;
  tl_args2[i]=50;
  tl_args3[i]=80;
  i++;     



  
//LEFT TURN HARD
 timeline[i]=245+EXTRA_TIME;
  tl_servos[i]=FLP;
  tl_args1[i]=50;
  tl_args2[i]=0;
  tl_args3[i]=50;
  i++;     
  timeline[i]=246+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=50;
  tl_args2[i]=25;
  tl_args3[i]=80;
  i++;     
  timeline[i]=247+EXTRA_TIME;
  tl_servos[i]=YAW;
  tl_args1[i]=50;
  tl_args2[i]=45;
  tl_args3[i]=80;
  i++;     

//camera on target 17
timeline[i]=250+EXTRA_TIME;
  tl_servos[i]=MTD_YA;
  tl_args1[i]=50;
  tl_args2[i]=60;
  tl_args3[i]=30;
  i++;     
  timeline[i]=250+EXTRA_TIME;
  tl_servos[i]=MTD_PI;
   tl_args1[i]=70;
  tl_args2[i]=0;
  tl_args3[i]=20;
 i++;     


//random roll
  timeline[i]=246+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=25;
  tl_args2[i]=45;
  tl_args3[i]=100;
  i++;     

  
  //STRAIGHTEN
 timeline[i]=260+EXTRA_TIME;
  tl_servos[i]=FLP;
  tl_args1[i]=0;
  tl_args2[i]=50;
  tl_args3[i]=50;
  i++;     
  timeline[i]=261+EXTRA_TIME;
  tl_servos[i]=ROLL;
  tl_args1[i]=45;
  tl_args2[i]=50;
  tl_args3[i]=80;
  i++;       
  timeline[i]=262+EXTRA_TIME;
  tl_servos[i]=YAW;
  tl_args1[i]=55;
  tl_args2[i]=50;
  tl_args3[i]=80;
  i++;     
  
//LANDING

  timeline[i]=295+EXTRA_TIME;
  tl_servos[i]=WHL;
 // position_rel(WHL,100);
  tl_args1[i]=OPEN;
  i++;     

  timeline[i]=300+EXTRA_TIME;
  tl_servos[i]=ARM;
  tl_args1[i]=REVERSE;
  i++;     

  //motor off
  timeline[i]=308+EXTRA_TIME;
  tl_servos[i]=MOTOR;
  tl_args1[i]=0;
  i++;  

  timeline[i]=310+EXTRA_TIME;
  tl_servos[i]=HOOD;
  tl_args1[i]=100;
  tl_args2[i]=0;
  tl_args3[i]=20;
  i++;     
    

  timeline[i]=335+EXTRA_TIME;
  tl_servos[i]=HOOD;
  tl_args1[i]=0;
  tl_args2[i]=100;
  tl_args3[i]=20;
  i++;     
   

  timeline[i]=336+EXTRA_TIME;
  tl_servos[i]=ARM;
  tl_args1[i]=CLOSED;
  i++;     


//Serial.println("movie timeline");
//    for(i=0;i<movielength;i++)
//  {
//     Serial.print("i:");
//     Serial.print(i);
//     Serial.print(" servo:");
//     Serial.print(tl_servos[i]);
//     Serial.print(" arg1:");
//     Serial.print(tl_args1[i]);
//     Serial.print(" arg2:");
//     Serial.print(tl_args2[i]);
//     Serial.print(" arg3:");
//     Serial.println(tl_args3[i]);
//  }


//delay(100);
 // ;
 index = 0 ;
    t_millis=millis();
    start_seconds =(int)( t_millis/1000);

  int max_index = sizeof(timeline)/sizeof(timeline[0]);
    Serial.print("movie time, max index ");Serial.println(max_index);
  while(index<max_index)
  {
      if(docheck())return(1);

    t_millis=millis();
    t_seconds =(int)( t_millis/1000);
    int dt = (int)t_seconds-(int)start_seconds;
    Serial.print("curtime:"); Serial.print(dt);
    Serial.print(" nexttime:"); Serial.print((int)timeline[index]-(int)PREDELAY);
    Serial.print(" planned :"); Serial.println((int)timeline[index]);
 
    if ((int)dt>=(int)((int)timeline[index]-(int)PREDELAY))
    {
        if(docheck())return(1);

      Serial.print("doing command on servo "); Serial.print(tl_servos[index]);Serial.print(" index:");Serial.println(index);
      Serial.flush();
      if(mydelay(20)) return (1);
      if (tl_servos[index]==LED)
      {
        led(tl_args1[index]);
      }    
      else if (tl_servos[index]==MOTOR)
      {
        motor(tl_args1[index]);
      }    
      else if(tl_servos[index]==ARM)
      {
        arm(tl_args1[index]);
      }
      else if(tl_servos[index]==WHL)
      {
        if(wheels(tl_args1[index])) return 1;
      }
      else if(tl_servos[index]==ROLLYAW)
      {
        index++;
        if(wheels(tl_args1[index])) return 1;
      }
      else 
      {
        if(tl_servos[index]>20)
        {
        if(position_rel(tl_servos[index],tl_args1[index])) return 1;
        }
        else
        { if(move_rel(tl_servos[index],tl_args1[index],tl_args2[index],tl_args3[index])) return 1;
        }

      }
      index++;
    }

  }

   if(mydelay(7000)) return(1);
   Serial.println('finished mov');
  Serial.println('quit movie');
   RETURN_TO = MOVIE;
 //  zero_all();

//    delay(100000);
// ROLL PIT  YAW  FLP AIL RUD ELE HOOD  MTD_YA  MTD_PI  WHL = 11; //on off
}

uint8_t docheck()
{
// delay(50);
 buttoncheck();
// STATUS = 0;
 if (STATUS) 
 {
 Serial.print(STATUS);
  Serial.print("is status, activity= ");Serial.println(CURRENT_ACTIVITY);
 return STATUS;
 }
 
 udpcheck();
 if (STATUS) 
 {
 Serial.print(STATUS);
  Serial.print("=status, activity= ");Serial.println(CURRENT_ACTIVITY);
 }
 return STATUS;
  
}

int buttoncheck()
{
//  delay(100);
  STATUS = 0;
 // Serial.print("activity ");Serial.print(CURRENT_ACTIVITY);
//  Serial.print(" status ");Serial.print(STATUS);
//  Serial.print(" A2 ");Serial.print(analogRead(A2));
//  Serial.print(" A3 ");Serial.print(analogRead(A3));
//  Serial.print(" A4 ");Serial.print(analogRead(A4));
//  Serial.print(" A5 ");Serial.println(analogRead(A5));

  if(analogRead(A0)>512) 
  {
    LEFTRIGHT = LEFT;
  }
  else LEFTRIGHT=RIGHT;

  while(analogRead(A1)>512) 
  {
    STATUS = PAUSE;
    delay(100);
  }

  while(analogRead(A2)>512) 
  {
    STATUS = ZERO;
    CURRENT_ACTIVITY=ZERO;
    delay(100);
  }
  while(analogRead(A5)>512) 
  {
    STATUS = MOVIE; 
    CURRENT_ACTIVITY=MOVIE;
    delay(100);
  }
  while(analogRead(A4)>512) 
  {
    STATUS = SCREENSAVER;
    CURRENT_ACTIVITY=SCREENSAVER;
   delay(100);
  } 
  while(analogRead(A3)>512) 
  {
    STATUS = SHOWOFF; 
    CURRENT_ACTIVITY=SHOWOFF;
    delay(100);
  }
  return STATUS;
}

int udpcheck()
{
  STATUS = 0;
//\  Serial.println("listening");
  // if there's data available, read a packet
  uint8_t got1=0;
  uint8_t got2=0;
   char *token;
   char string1[20];
   char string2[20];
   int packetSize = Udp.parsePacket();
  //Udp.write(ReplyBuffer);
   int SERVOCENTER = servoCenters[0]; 
   int SERVOMAX = servoMaxes[0]; 
   int SERVOMIN = servoMins[0]; 
   strcpy(string1,"\n");
   strcpy(string2,"\n");
  if(packetSize)
  {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = Udp.remoteIP();
    for (int i =0; i < 4; i++)
    {
      Serial.print(remote[i], DEC);
      if (i < 3)
      {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    Udp.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);
    Serial.println("Contents:");
    Serial.println(packetBuffer);

    token = strtok(packetBuffer," ");
   
   /* walk through other tokens */
   if( token != NULL ) 
   {
     Serial.print( "string1:");Serial.println(token );
     strcpy(string1,token);
     token = strtok(NULL, " ");
     got1 = 1;
     if( token != NULL ) 
     {
     Serial.print( "string2:");Serial.println(token );
       strcpy(string2,token);
       got2 = 1;
     }
   }  
   if(got1)
   {
    if (strcmp(packetBuffer,"move")==0)
    {
      STATUS = MOVIE;
         Serial.println("got movie from udp");
      return STATUS;
      //movie1();
    }
    if (strcmp(packetBuffer,"zero")==0)
    {
        STATUS = ZERO;      
         Serial.println("got zero from udp");
      return STATUS;
//zero_all();
    }
    if (strcmp(packetBuffer,"ssav")==0)
    {
        STATUS = SCREENSAVER;
         Serial.println("got screensaver from udp");
      return STATUS;
//      minishowoff();
    }
    if (strcmp(packetBuffer,"demo")==0)
    {
         STATUS = SHOWOFF;
         Serial.println("got demo from udp");
      return STATUS;
//       showoff();
    }
    strcpy(packetBuffer,"");
     
   }
  }
}
void pause()
{
 Serial.println("pause"); 
 mydelay(100);
}
void loop() 
{
Serial.println("mainloop");
  
  if (!STATUS) mydelay(150); 

if (RETURN_TO == SCREENSAVER) STATUS = SCREENSAVER;
if (RETURN_TO == MOVIE) STATUS = MOVIE;
  
  switch (STATUS) 
  {
     case ZERO:
      STATUS = 0;
      Serial.println("got ZERO command");
      zero_all();
      CURRENT_ACTIVITY=WAITING;
      break;
    case MOVIE:
      STATUS = 0;
      Serial.println("got MOVi command");
      movie1();
   //   zero_all();
      CURRENT_ACTIVITY=WAITING;
      break;
    case SCREENSAVER:
      STATUS = 0;
      Serial.println("got SCREENSAVER command");
      screensaver();
      CURRENT_ACTIVITY=WAITING;
      break;
    case SHOWOFF:
      STATUS = 0;
      Serial.println("got SHOWOFF command");
      showoff();
      CURRENT_ACTIVITY=WAITING;
      break;
    case PAUSE:
      STATUS = 0;
      Serial.println("got paus command");
      pause();
      CURRENT_ACTIVITY=WAITING;
      break;
    default:
     STATUS = docheck();
  }
  
//  mydelay((unsigned long)20);
}

