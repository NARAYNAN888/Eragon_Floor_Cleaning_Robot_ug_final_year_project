
// Before uploading the code you have to install the necessary library//
//AFMotor Library https://learn.adafruit.com/adafruit-motor-shield/library-install //
//NewPing Library https://github.com/livetronic/Arduino-NewPing// 
//Servo Library https://github.com/arduino-libraries/Servo.git //
// To Install the libraries go to sketch >> Include Library >> Add .ZIP File >> Select the Downloaded ZIP files From the Above links //

#include <AFMotor.h>  
#include <NewPing.h>
#include <Servo.h>


#define TRIG_PIN A0 
#define ECHO_PIN A1 
#define MAX_DISTANCE 200 
#define MAX_SPEED 170 // sets speed of DC  motors
#define MAX_SPEED_OFFSET 20
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); 
AF_DCMotor motor1(1, MOTOR12_1KHZ); 
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4,
Servo myServo;

boolean goesForward=false;
int distance = 100;
int speedSet = 0;
//mop purpose
int r_motor_n = 10; //PWM control Right Motor +
int r_motor_p = 11; //PWM control Right Motor -
int l_motor_p = 9; //PWM control Left Motor -
int l_motor_n = 6; //PWM control Left Motor +
int pump = 4;
int mop = 5;
int myServo = ;
int speedy = 255;
int incomingByte = 0; // for incoming serial data

void setup()
{
myServo.attach(3);
myServo.write(0);  
pinMode(r_motor_n, OUTPUT); //Set control pins to be outputs
pinMode(r_motor_p, OUTPUT);
pinMode(l_motor_p, OUTPUT);
pinMode(l_motor_n, OUTPUT);
pinMode(pump, OUTPUT);
pinMode(mop, OUTPUT);
digitalWrite(r_motor_n, LOW); //set both motors off for start-up
digitalWrite(r_motor_p, LOW);
digitalWrite(l_motor_p, LOW);
digitalWrite(l_motor_n, LOW);
digitalWrite(pump, LOW);
digitalWrite(mop, LOW);

myservo.attach(10);  
myservo.write(115); 
delay(2000);
distance = readPing();
delay(100);
distance = readPing();
delay(100);
distance = readPing();
delay(100);
distance = readPing();
delay(100);
Serial.begin(9600);

}


void loop()
{

if (Serial.available() > 0) 
{
incomingByte = Serial.read();
}

switch(incomingByte)
{

case 'S': // control to stop the robot
digitalWrite(r_motor_n, LOW); 
digitalWrite(r_motor_p, LOW);
digitalWrite(l_motor_p, LOW);
digitalWrite(l_motor_n, LOW);
Serial.println("Stop");
incomingByte='*';
break;

case 'R': //control for right
analogWrite(r_motor_n, speedy); 
digitalWrite(r_motor_p, LOW);
analogWrite(l_motor_p, speedy);
digitalWrite(l_motor_n, LOW);
Serial.println("right");
incomingByte='*';
break;


case 'L': //control for left
analogWrite(r_motor_n, LOW); 
digitalWrite(r_motor_p, speedy);
analogWrite(l_motor_p, LOW);
digitalWrite(l_motor_n, speedy);
Serial.println("right");
incomingByte='*';
break;


case 'F': //control for forward
analogWrite(r_motor_n, speedy); 
digitalWrite(r_motor_p, LOW);
analogWrite(l_motor_p, LOW);
digitalWrite(l_motor_n, speedy);
Serial.println("right");
incomingByte='*';
break;


case 'B': //control for backward
analogWrite(r_motor_n, LOW); 
digitalWrite(r_motor_p, speedy);
analogWrite(l_motor_p, speedy);
digitalWrite(l_motor_n, LOW);
Serial.println("right");
incomingByte='*';
break;


case 'P': // pump on
digitalWrite(pump, HIGH);
Serial.println("pump on");
incomingByte='*';
break;

case 'p': // pump off
digitalWrite(pump, LOW); 
Serial.println("pump off");
incomingByte='*';
break;

case 'M':
digitalWrite(mop, HIGH); // mopper on
Serial.println("mopper on");
incomingByte='*';
break;

case 'm':
digitalWrite(mop, LOW); // mopper off
Serial.println("mopper off");
incomingByte='*';
break;
//
//case 'U': // roller up
//myServo.write(0);
//Serial.println("roller up");
//incomingByte='*';
//break;
//
//case 'u': // roller down
//myServo.write(135);
//Serial.println("roller down");
//incomingByte='*';
//break;


case '1':
speedy = 155;
Serial.println("speed= 10");
incomingByte='*';
break;

case '2':
speedy = 185;
Serial.println("speed= 25");
incomingByte='*';
break;

case '3':
speedy = 215;
Serial.println("speed= 75");
incomingByte='*';
break;

case '4':
speedy = 255;
Serial.println("speed= 100");
incomingByte='*';
break;

delay(5000);
}

//vaccume purpose
int distanceR = 0;
 int distanceL =  0;
 delay(40);
 
 if(distance<=15)
 {
  moveStop();
  delay(100);
  moveBackward();
  delay(300);
  moveStop();
  delay(200);
  distanceR = lookRight();
  delay(200);
  distanceL = lookLeft();
  delay(200);

  if(distanceR>=distanceL)
  {
    turnRight();
    moveStop();
  }else
  {
    turnLeft();
    moveStop();
  }
 }else
 {
  moveForward();
 }
 distance = readPing();
}

int lookRight()
{
    myservo.write(50); 
    delay(500);
    int distance = readPing();
    delay(100);
    myservo.write(115); 
    return distance;
}

int lookLeft()
{
    myservo.write(170); 
    delay(500);
    int distance = readPing();
    delay(100);
    myservo.write(115); 
    return distance;
    delay(100);
}

int readPing() { 
  delay(70);
  int cm = sonar.ping_cm();
  if(cm==0)
  {
    cm = 250;
  }
  return cm;
}

void moveStop() {
  motor1.run(RELEASE); 
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  } 
  
void moveForward() {

 if(!goesForward)
  {
    goesForward=true;
    motor1.run(FORWARD);      
    motor2.run(FORWARD);
    motor3.run(FORWARD); 
    motor4.run(FORWARD);     
   for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) // slowly bring the speed up to avoid loading down the batteries too quickly
   {
    motor1.setSpeed(speedSet);
    motor2.setSpeed(speedSet);
    motor3.setSpeed(speedSet);
    motor4.setSpeed(speedSet);
    delay(5);
   }
  }
}

void moveBackward() {
    goesForward=false;
    motor1.run(BACKWARD);      
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);  
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) // slowly bring the speed up to avoid loading down the batteries too quickly
  {
    motor1.setSpeed(speedSet);
    motor2.setSpeed(speedSet);
    motor3.setSpeed(speedSet);
    motor4.setSpeed(speedSet);
    delay(5);
  }
}  

void turnRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);     
  delay(500);
  motor1.run(FORWARD);      
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);      
} 
 
void turnLeft() {
  motor1.run(BACKWARD);     
  motor2.run(BACKWARD);  
  motor3.run(FORWARD);
  motor4.run(FORWARD);   
  delay(500);
  motor1.run(FORWARD);     
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}
