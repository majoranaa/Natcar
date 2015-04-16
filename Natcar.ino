// CURRENT VERSION

#include <Servo.h>
#define SERVO 21

#define LEFT_MAX 55
#define RIGHT_MAX 145
#define OFFSET 30
#define MIDPNT 100
#define MIDDLE 60
#define MAX_SPEED 120
#define MIN_SPEED 40
#define THRESH 200
#define UP_THRESH 21
#define DOWN_THRESH 22
#define ACCEL .8
#define DECCEL 10
#define LIGHT_THRESH 2

// debugging
#define DEBUG_CAM
#define DEBUG_CONT
#define DEBUG_MOTOR
//#define DEBUG_STOP

// define pins
const int brightness = 20; //A0; A6 camera input
const int SI = 0; //1; to camera - ask for image
const int clk = 1; //4; to camera - clock
const int md_1 = 23; // A9 motor driver 1
const int md_2 = 22; // A8 motor driver 2

// define PID coefficients
const float KP = 0.7;
const float KD = 0; // this should be negative
const float KI = 0.6;//.0001;
float accum,de;
unsigned long last_time, now;

// construct servo controller
Servo myServo;

// index variables
int i;

// READ VALUES FROM CAMERA globals
int maxVal, maxPos;
int mid, val, output;
int rightEdge, rightFound;
int error, lastErr;
int last_mid;
int count;

// MOTOR globals
float motor_speed;

void setup()
{
  // initialize pins
  myServo.attach(SERVO);
  Serial.begin(9600);
  pinMode(brightness, INPUT);
  pinMode(SI, OUTPUT);
  pinMode(clk, OUTPUT);
  pinMode(md_1, OUTPUT);
  pinMode(md_2, OUTPUT);
  
  // initialize vars
  maxVal = 0;
  maxPos = 1;
  rightEdge = 128;
  rightFound = 0;
  error = 0;
  lastErr = 0;
  val = 0;
  last_mid = MIDDLE;
  accum = 0;
  de = 0;
  last_time = millis();
  count = 0;
  
  motor_speed = MAX_SPEED/2;
}

void stop() {
  analogWrite(md_2, 0);
  analogWrite(md_1, 0);
  while(1) {}
}

void loop()
{
  // READ VALUES FROM CAMERA
  maxPos = 1;
  maxVal = 0;
  rightEdge = 128;
  rightFound = 0;
  
  digitalWrite(SI,HIGH);
  digitalWrite(clk,HIGH);
  digitalWrite(SI,LOW);
  digitalWrite(clk,LOW);
  for (i = 1; i <= 128; i++)
  {
    digitalWrite(clk, HIGH);
    val = analogRead(brightness);
    val =(val*9.)/1024.;
    digitalWrite(clk,LOW);

    if (val > maxVal) {
      maxVal = val;
      maxPos = i;
      rightFound = 0;
    } else if (val < maxVal && !rightFound) {
      rightEdge = i;
      rightFound = 1;
    }
    
    #ifdef DEBUG_CAM
    Serial.print(val);
    #endif
  }
  
  Serial.println();
  mid = (rightEdge-1 + maxPos)/2;
  
  #ifdef DEBUG_CAM
  for (i = 1; i < rightEdge; i++) {
     if (i == maxPos || i == rightEdge-1) {
       Serial.print("|");
     } else if (i == mid) {
       Serial.print("*");
     } else {
       Serial.print(" ");
     }
  }
  Serial.println();
  #endif
  
  // CONTROL ALGORITHM - PID
  error = mid - MIDDLE;
  now = millis();
  de = ((float)error)*(now-last_time)/1000;
  if (((de<0)&&(accum>0))||((de>0)&&(accum<0))) accum = de;
  else accum += de;
  output = KP*error + KD*(((float)(error - lastErr))/(((float)(now-last_time))/1000)) + KI*accum; // /(now/3000);
  
  #ifdef DEBUG_CONT
  Serial.print("Output: ");
  Serial.println(output);
  Serial.print("Accum: ");
  Serial.println(accum/(now/3000));
  Serial.print("Diff: ");
  Serial.println(((float)(error-lastErr))/((float)(now-last_time)/1000));
  #endif

  lastErr = error;
  last_time = now;
  
  // SET SPEED
  if (error < UP_THRESH && error > -UP_THRESH) motor_speed = (motor_speed>=MAX_SPEED)?MAX_SPEED:motor_speed+ACCEL;
  if (error > DOWN_THRESH || error < -DOWN_THRESH) motor_speed = (motor_speed<=MIN_SPEED)?MIN_SPEED:motor_speed-DECCEL;
  
  if (maxVal < LIGHT_THRESH) {
    mid = last_mid;
    output = (mid<MIDDLE)?-OFFSET:((mid>MIDDLE)?OFFSET:0);
    Serial.print("Off the line with output: ");
    Serial.println(output);
    motor_speed = MIN_SPEED;
    count++;
    lastErr = 0;
    #ifdef DEBUG_STOP
    if (count > THRESH) {
      stop();
    }
    #endif
  }
  
  last_mid = mid;
  
  // SET SERVO
  //myServo.write((((output + MIDPNT)>RIGHT_MAX)?RIGHT_MAX:(((output+MIDPNT)<LEFT_MAX)?LEFT_MAX:output+MIDPNT))+(motor_speed/10));
  myServo.write(((output + MIDPNT)>RIGHT_MAX)?RIGHT_MAX:(((output+MIDPNT)<LEFT_MAX)?LEFT_MAX:output+MIDPNT));
  //myServo.write(MIDPNT);

  // WRITE SPEED
  analogWrite(md_2, motor_speed);
  analogWrite(md_1, 0);
  
  #ifdef DEBUG_MOTOR
  Serial.print("Motor speed: ");
  Serial.println(motor_speed);
  Serial.print("ERROR: ");
  Serial.println(error);
  #endif
  
  /*#ifdef DEBUG_STOP
  if (millis() > 8000) {
    analogWrite(md_1, 0);
    analogWrite(md_2,0);
  }
  #endif*/
  
  delay(5);
}
  
