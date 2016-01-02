/*
  Serial Call and Response in ASCII
 http://www.arduino.cc/en/Tutorial/SerialCallResponseASCII
 */

int firstSensor = 0;    // first analog sensor
int secondSensor = 0;   // second analog sensor
int thirdSensor = 0;    // digital sensor
int inByte = 0;         // incoming serial byte

int led = 13;

void setup()
{
  // start serial port at 9600 bps and wait for port to open:
  Serial.begin(9600);
  // give it a name:
  pinMode(led, OUTPUT);    

// the setup routine runs once when you press reset:        
  // initialize the digital pin as an output.
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  //pinMode(2, INPUT);   // digital sensor is on digital pin 2
  establishContact();  // send a byte to establish contact until receiver responds 
}

void loop()
{
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {
    // get incoming byte:
    inByte = Serial.read();
    Serial.println("start movie1");
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(100);               // wait for a second

    delay(5000);
    Serial.println("start movie2");
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(100);               // wait for a second
    delay(5000);
    Serial.println("start movie3");
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(100);               // wait for a second
    delay(5000);
    Serial.println("start movie4");
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(100);               // wait for a second
    delay(5000);
  }
}

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.println("0,0,0");   // send an initial string
    delay(300);
  }
}


