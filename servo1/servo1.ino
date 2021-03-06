/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <String.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  225 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  350 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  4*SERVOMIN // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  4*SERVOMAX // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define MAX_X 600
#define MAX_Y 400
#define MAX_Z 90
#define MIN_X 0
#define MIN_Y 0
#define MIN_Z 40


// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);

  
//  Serial.println("8 channel Servo test!");

  pwm.begin();
  // In theory the internal oscillator is 25MHz but it really isn't
  // that precise. You can 'calibrate' by tweaking this number till
  // you get the frequency you're expecting!
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
//  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
//  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
//  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);  
}

int yaw_old = 0;
int pitch_old = 0;
int gripper_old = 0;
  int x = 0;
  int y = 0;
  int z = 0;

 
void loop() {
  char serial[20];
 // String test = "100,75,100";
//    Serial.println("loop");

 while(Serial.available() == 0) {
 }
  char c[2];
  Serial.readBytes(serial, 12);
   c[0] = serial[0];
   c[1] = serial[1];
  c[2] = serial[2];
  
  x = (c[0]-'0')*100 + (c[1]-'0')*10 + (c[2]-'0')*1;
  y = (serial[4]-'0')*100 + (serial[5]-'0')*10 + (serial[6]-'0')*1;
  z = (serial[8]-'0')*100 + (serial[9]-'0')*10 + (serial[10]-'0')*1;
//    Serial.println("hello");

//  Serial.println(x);
//    Serial.println(y);
//  Serial.println(z);
   int yaw = 200+((400-200))*((double)(x)/(MAX_X-MIN_X));
 int pitch = SERVOMIN+((SERVOMAX-SERVOMIN)*3)*((double)(y)/(MAX_Y-MIN_Y));
 int gripper = SERVOMIN+(SERVOMAX-SERVOMIN)*((double)(z)/(MAX_Z-MIN_Z));
 
// Serial.println(gripper);
   Serial.println(yaw);
// Serial.println(pitch);
 
//  Serial.println(serial);
 
//
//  Serial.println(x);
//  Serial.println(y);
//  Serial.println(z);
//  
//  Get hand position
//  x = getX();
//  y = getY();
//  z = getZ();

//int x = 100;
//int y = 50;
//int z = 50;
//
// int yaw = SERVOMIN+(SERVOMAX-SERVOMIN)*((double)(x)/(MAX_X-MIN_X));
// int pitch = SERVOMIN+(SERVOMAX-SERVOMIN)*((double)(y)/(MAX_Y-MIN_Y));
// int gripper = SERVOMIN+(SERVOMAX-SERVOMIN)*((double)(z)/(MAX_Z-MIN_Z));
// 
// Serial.println(gripper);
//   Serial.println(yaw);
// Serial.println(pitch);
//  Serial.println(x);
//    Serial.println(y);
//  Serial.println(z);


  // Drive each servo one at a time using setPWM()
  //Serial.println(servonum);
//  
    
    servonum = 5;
    pwm.setPWM(servonum, 0, gripper);

    servonum = 0;
    pwm.setPWM(servonum, 0, yaw);

    servonum = 2;
    pwm.setPWM(servonum, 0, pitch);

  
    servonum = 1;
    pwm.setPWM(servonum, 0, SERVOMIN+(SERVOMAX-SERVOMIN)*0.5);
  
    servonum = 3;
    pwm.setPWM(servonum, 0, SERVOMIN+(SERVOMAX-SERVOMIN)*0.5);

  
    yaw_old = yaw;
    pitch_old = pitch;
    gripper_old = gripper;
}

/*
  delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
  }

  delay(500);

  // Drive each servo one at a time using writeMicroseconds(), it's not precise due to calculation rounding!
  // The writeMicroseconds() function is used to mimic the Arduino Servo library writeMicroseconds() behavior. 
//
//  uint16_t microsec = USMIN;
//  pwm.writeMicroseconds(servonum, 75);
   
  for (uint16_t microsec = USMIN; microsec < USMAX; microsec++) {
    pwm.writeMicroseconds(servonum, microsec);
  }

  delay(500);

 
  for (uint16_t microsec = USMAX; microsec > USMIN; microsec--) {
    pwm.writeMicroseconds(servonum, microsec);
  }

  delay(500);

  servonum++;
  if (servonum > 7) servonum = 0; // Testing the first 8 servo channels */
